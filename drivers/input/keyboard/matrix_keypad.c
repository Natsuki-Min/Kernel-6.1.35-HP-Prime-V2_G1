// SPDX-License-Identifier: GPL-2.0-only
/*
 *  GPIO driven matrix keyboard driver
 *
 *  Copyright (c) 2008 Marek Vasut <marek.vasut@gmail.com>
 *
 *  Based on corgikbd.c
 *
 *  FN key support added.
 *
 *  Device Tree support enhanced to include Fn-key configuration.
 *
 *  Device Tree properties:
 *  - row-gpios:          GPIO specifiers for matrix rows.
 *  - col-gpios:          GPIO specifiers for matrix columns.
 *  - linux,keymap:       Standard matrix keymap definition.
 *  - debounce-delay-ms:  Debounce interval in milliseconds.
 *  - col-scan-delay-us:  Delay between column scans in microseconds.
 *  - linux,no-autorepeat:Disable autorepeat.
 *  - wakeup-source:      Device can wake up the system.
 *  - drive-inactive-cols:Drive inactive columns to a specific level instead
 *                       of Hi-Z.
 *  - gpio-activelow:     Use active-low logic for GPIOs.
 *
 *  Optional Fn-key support properties:
 *  - linux,fn-key-position: A <row col> tuple specifying the Fn key location.
 *  - linux,fn-keymap:    Keymap to be used when the Fn key is held down.
 */

#include <linux/types.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/input/matrix_keypad.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_platform.h>

struct matrix_keypad {
	const struct matrix_keypad_platform_data *pdata;
	struct input_dev *input_dev;
	unsigned int row_shift;

	DECLARE_BITMAP(disabled_gpios, MATRIX_MAX_ROWS);

	uint32_t last_key_state[MATRIX_MAX_COLS];
	struct delayed_work work;
	spinlock_t lock;
	bool scan_pending;
	bool stopped;
	bool gpio_all_disabled;

	/* for Fn key support */
	bool fn_key_down;
	unsigned short *fn_keycodes;
	unsigned short *active_keycodes;
	int keymap_size;
};

/*
 * NOTE: If drive_inactive_cols is false, then the GPIO has to be put into
 * HiZ when de-activated to cause minmal side effect when scanning other
 * columns. In that case it is configured here to be input, otherwise it is
 * driven with the inactive value.
 */
static void __activate_col(const struct matrix_keypad_platform_data *pdata,
			   int col, bool on)
{
	bool level_on = !pdata->active_low;

	if (on) {
		gpio_direction_output(pdata->col_gpios[col], level_on);
	} else {
		gpio_set_value_cansleep(pdata->col_gpios[col], !level_on);
		if (!pdata->drive_inactive_cols)
			gpio_direction_input(pdata->col_gpios[col]);
	}
}

static void activate_col(const struct matrix_keypad_platform_data *pdata,
			 int col, bool on)
{
	__activate_col(pdata, col, on);

	if (on && pdata->col_scan_delay_us)
		udelay(pdata->col_scan_delay_us);
}

static void activate_all_cols(const struct matrix_keypad_platform_data *pdata,
			      bool on)
{
	int col;

	for (col = 0; col < pdata->num_col_gpios; col++)
		__activate_col(pdata, col, on);
}

static bool row_asserted(const struct matrix_keypad_platform_data *pdata,
			 int row)
{
	return gpio_get_value_cansleep(pdata->row_gpios[row]) ?
			!pdata->active_low : pdata->active_low;
}

static void enable_row_irqs(struct matrix_keypad *keypad)
{
	const struct matrix_keypad_platform_data *pdata = keypad->pdata;
	int i;

	if (pdata->clustered_irq > 0)
		enable_irq(pdata->clustered_irq);
	else {
		for (i = 0; i < pdata->num_row_gpios; i++)
			enable_irq(gpio_to_irq(pdata->row_gpios[i]));
	}
}

static void disable_row_irqs(struct matrix_keypad *keypad)
{
	const struct matrix_keypad_platform_data *pdata = keypad->pdata;
	int i;

	if (pdata->clustered_irq > 0)
		disable_irq_nosync(pdata->clustered_irq);
	else {
		for (i = 0; i < pdata->num_row_gpios; i++)
			disable_irq_nosync(gpio_to_irq(pdata->row_gpios[i]));
	}
}

/*
 * This gets the keys from keyboard and reports it to input subsystem
 */
static void matrix_keypad_scan(struct work_struct *work)
{
	struct matrix_keypad *keypad =
		container_of(work, struct matrix_keypad, work.work);
	struct input_dev *input_dev = keypad->input_dev;
	const struct matrix_keypad_platform_data *pdata = keypad->pdata;
	uint32_t new_state[MATRIX_MAX_COLS];
	bool fn_state_changed = false;
	bool new_fn_state;
	int row, col;

	/* Part 1: Scan the hardware to get current key state */
	activate_all_cols(pdata, false);

	memset(new_state, 0, sizeof(new_state));

	/* assert each column and read the row status out */
	for (col = 0; col < pdata->num_col_gpios; col++) {
		activate_col(pdata, col, true);
		for (row = 0; row < pdata->num_row_gpios; row++)
			if (row_asserted(pdata, row))
				new_state[col] |= (1 << row);
		activate_col(pdata, col, false);
	}

	/* Part 2: Handle Fn key state change if Fn is defined */
	if (pdata->fn_keymap_data) {
		new_fn_state =
			(new_state[pdata->fn_col] & (1 << pdata->fn_row)) != 0;

		if (keypad->fn_key_down != new_fn_state) {
			keypad->fn_key_down = new_fn_state;
			fn_state_changed = true;
		}
	}

	/* Part 3: Process all keys and report events */
	for (col = 0; col < pdata->num_col_gpios; col++) {
		for (row = 0; row < pdata->num_row_gpios; row++) {
			uint32_t mask = 1 << row;
			int code;
			bool is_down;
			bool was_down;

			/* Skip the Fn key itself; its state is handled above */
			if (pdata->fn_keymap_data &&
			    row == pdata->fn_row && col == pdata->fn_col) {
				continue;
			}

			is_down = (new_state[col] & mask) != 0;
			was_down = (keypad->last_key_state[col] & mask) != 0;

			/*
			 * Continue if key state is stable, unless the Fn
			 * modifier changed while the key is pressed.
			 */
			if (is_down == was_down && !(fn_state_changed && is_down))
				continue;

			code = MATRIX_SCAN_CODE(row, col, keypad->row_shift);

			/* Report the scan code for raw access */
			input_event(input_dev, EV_MSC, MSC_SCAN, code);

			/*
			 * If the key was previously down, we must report a
			 * release event for it before reporting a new press.
			 */
			if (was_down) {
				input_report_key(input_dev,
						 keypad->active_keycodes[code],
						 0);
			}

			/* If the key is currently down, report a press event. */
			if (is_down) {
				const unsigned short *keycodes = input_dev->keycode;
				unsigned short keycode;

				if (keypad->fn_key_down && keypad->fn_keycodes)
					keycode = keypad->fn_keycodes[code];
				else
					keycode = keycodes[code];

				keypad->active_keycodes[code] = keycode;

				input_report_key(input_dev, keycode, 1);
			}
		}
	}

	input_sync(input_dev);

	/* Part 4: Save state and re-enable hardware for interrupts */
	memcpy(keypad->last_key_state, new_state, sizeof(new_state));
	activate_all_cols(pdata, true);

	spin_lock_irq(&keypad->lock);
	keypad->scan_pending = false;
	enable_row_irqs(keypad);
	spin_unlock_irq(&keypad->lock);
}

static irqreturn_t matrix_keypad_interrupt(int irq, void *id)
{
	struct matrix_keypad *keypad = id;
	unsigned long flags;

	spin_lock_irqsave(&keypad->lock, flags);

	/*
	 * See if another IRQ beaten us to it and scheduled the
	 * scan already. In that case we should not try to
	 * disable IRQs again.
	 */
	if (unlikely(keypad->scan_pending || keypad->stopped))
		goto out;

	disable_row_irqs(keypad);
	keypad->scan_pending = true;
	schedule_delayed_work(&keypad->work,
		msecs_to_jiffies(keypad->pdata->debounce_ms));

out:
	spin_unlock_irqrestore(&keypad->lock, flags);
	return IRQ_HANDLED;
}

static int matrix_keypad_start(struct input_dev *dev)
{
	struct matrix_keypad *keypad = input_get_drvdata(dev);

	keypad->stopped = false;
	mb();

	/*
	 * Schedule an immediate key scan to capture current key state;
	 * columns will be activated and IRQs be enabled after the scan.
	 */
	schedule_delayed_work(&keypad->work, 0);

	return 0;
}

static void matrix_keypad_stop(struct input_dev *dev)
{
	struct matrix_keypad *keypad = input_get_drvdata(dev);

	spin_lock_irq(&keypad->lock);
	keypad->stopped = true;
	spin_unlock_irq(&keypad->lock);

	flush_delayed_work(&keypad->work);
	/*
	 * matrix_keypad_scan() will leave IRQs enabled;
	 * we should disable them now.
	 */
	disable_row_irqs(keypad);
}

#ifdef CONFIG_PM_SLEEP
static void matrix_keypad_enable_wakeup(struct matrix_keypad *keypad)
{
	const struct matrix_keypad_platform_data *pdata = keypad->pdata;
	unsigned int gpio;
	int i;

	if (pdata->clustered_irq > 0) {
		if (enable_irq_wake(pdata->clustered_irq) == 0)
			keypad->gpio_all_disabled = true;
	} else {

		for (i = 0; i < pdata->num_row_gpios; i++) {
			if (!test_bit(i, keypad->disabled_gpios)) {
				gpio = pdata->row_gpios[i];
				if (enable_irq_wake(gpio_to_irq(gpio)) == 0){
					__set_bit(i, keypad->disabled_gpios);}
			}
		}
	}
}

static void matrix_keypad_disable_wakeup(struct matrix_keypad *keypad)
{
	const struct matrix_keypad_platform_data *pdata = keypad->pdata;
	unsigned int gpio;
	int i;

	if (pdata->clustered_irq > 0) {
		if (keypad->gpio_all_disabled) {
			disable_irq_wake(pdata->clustered_irq);
			keypad->gpio_all_disabled = false;
		}
	} else {
		for (i = 0; i < pdata->num_row_gpios; i++) {
			if (test_and_clear_bit(i, keypad->disabled_gpios)) {
				gpio = pdata->row_gpios[i];
				disable_irq_wake(gpio_to_irq(gpio));
			}
		}
	}
}

static int matrix_keypad_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct matrix_keypad *keypad = platform_get_drvdata(pdev);

	matrix_keypad_stop(keypad->input_dev);
	if (device_may_wakeup(&pdev->dev)){
		matrix_keypad_enable_wakeup(keypad);
	}
	return 0;
}

static int matrix_keypad_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct matrix_keypad *keypad = platform_get_drvdata(pdev);

	if (device_may_wakeup(&pdev->dev))
		matrix_keypad_disable_wakeup(keypad);

	matrix_keypad_start(keypad->input_dev);

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(matrix_keypad_pm_ops,
			 matrix_keypad_suspend, matrix_keypad_resume);

static int matrix_keypad_init_gpio(struct platform_device *pdev,
				   struct matrix_keypad *keypad)
{
	const struct matrix_keypad_platform_data *pdata = keypad->pdata;
	int i, err;

	/* initialized strobe lines as outputs, activated */
	for (i = 0; i < pdata->num_col_gpios; i++) {
		err = gpio_request(pdata->col_gpios[i], "matrix_kbd_col");
		if (err) {
			dev_err(&pdev->dev,
				"failed to request GPIO%d for COL%d\n",
				pdata->col_gpios[i], i);
			goto err_free_cols;
		}

		gpio_direction_output(pdata->col_gpios[i], !pdata->active_low);
	}

	for (i = 0; i < pdata->num_row_gpios; i++) {
		err = gpio_request(pdata->row_gpios[i], "matrix_kbd_row");
		if (err) {
			dev_err(&pdev->dev,
				"failed to request GPIO%d for ROW%d\n",
				pdata->row_gpios[i], i);
			goto err_free_rows;
		}

		gpio_direction_input(pdata->row_gpios[i]);
	}

	if (pdata->clustered_irq > 0) {
		err = request_any_context_irq(pdata->clustered_irq,
				matrix_keypad_interrupt,
				pdata->clustered_irq_flags,
				"matrix-keypad", keypad);
		if (err < 0) {
			dev_err(&pdev->dev,
				"Unable to acquire clustered interrupt\n");
			goto err_free_rows;
		}
	} else {
		for (i = 0; i < pdata->num_row_gpios; i++) {
			err = request_any_context_irq(
					gpio_to_irq(pdata->row_gpios[i]),
					matrix_keypad_interrupt,
					IRQF_TRIGGER_RISING |
					IRQF_TRIGGER_FALLING,
					"matrix-keypad", keypad);
			if (err < 0) {
				dev_err(&pdev->dev,
					"Unable to acquire interrupt for GPIO line %i\n",
					pdata->row_gpios[i]);
				goto err_free_irqs;
			}
		}
	}

	/* initialized as disabled - enabled by input->open */
	disable_row_irqs(keypad);
	return 0;

err_free_irqs:
	while (--i >= 0)
		free_irq(gpio_to_irq(pdata->row_gpios[i]), keypad);
	i = pdata->num_row_gpios;
err_free_rows:
	while (--i >= 0)
		gpio_free(pdata->row_gpios[i]);
	i = pdata->num_col_gpios;
err_free_cols:
	while (--i >= 0)
		gpio_free(pdata->col_gpios[i]);

	return err;
}

static void matrix_keypad_free_gpio(struct matrix_keypad *keypad)
{
	const struct matrix_keypad_platform_data *pdata = keypad->pdata;
	int i;

	if (pdata->clustered_irq > 0) {
		free_irq(pdata->clustered_irq, keypad);
	} else {
		for (i = 0; i < pdata->num_row_gpios; i++)
			free_irq(gpio_to_irq(pdata->row_gpios[i]), keypad);
	}

	for (i = 0; i < pdata->num_row_gpios; i++)
		gpio_free(pdata->row_gpios[i]);

	for (i = 0; i < pdata->num_col_gpios; i++)
		gpio_free(pdata->col_gpios[i]);
}

#ifdef CONFIG_OF
static struct matrix_keypad_platform_data *
matrix_keypad_parse_dt(struct device *dev)
{
	struct matrix_keypad_platform_data *pdata;
	struct device_node *np = dev->of_node;
	unsigned int *gpios;
	int ret, i, nrow, ncol;
	u32 fn_pos[2]; /* MODIFICATION: Array to hold fn key position */

	if (!np) {
		dev_err(dev, "device lacks DT data\n");
		return ERR_PTR(-ENODEV);
	}

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
		dev_err(dev, "could not allocate memory for platform data\n");
		return ERR_PTR(-ENOMEM);
	}

	/* MODIFICATION: Parse the Fn key position */
	if (of_property_read_u32_array(np, "linux,fn-key-position", fn_pos, 2) == 0) {
		pdata->fn_row = fn_pos[0];
		pdata->fn_col = fn_pos[1];
		/*
		 * Set fn_keymap_data to a non-NULL value to signal its presence
		 * to the probe function. The actual keymap will be parsed from
		 * the 'linux,fn-keymap' property.
		 */
		pdata->fn_keymap_data = (void *)1;
	}

	pdata->num_row_gpios = nrow = gpiod_count(dev, "row");
	pdata->num_col_gpios = ncol = gpiod_count(dev, "col");
	if (nrow < 0 || ncol < 0) {
		dev_err(dev, "number of keypad rows/columns not specified\n");
		return ERR_PTR(-EINVAL);
	}

	if (of_get_property(np, "linux,no-autorepeat", NULL))
		pdata->no_autorepeat = true;

	pdata->wakeup = of_property_read_bool(np, "wakeup-source") ||
			of_property_read_bool(np, "linux,wakeup"); /* legacy */
	
	if (of_get_property(np, "gpio-activelow", NULL))
		pdata->active_low = true;

	pdata->drive_inactive_cols =
		of_property_read_bool(np, "drive-inactive-cols");

	of_property_read_u32(np, "debounce-delay-ms", &pdata->debounce_ms);
	of_property_read_u32(np, "col-scan-delay-us",
						&pdata->col_scan_delay_us);

	gpios = devm_kcalloc(dev,
			     pdata->num_row_gpios + pdata->num_col_gpios,
			     sizeof(unsigned int),
			     GFP_KERNEL);
	if (!gpios) {
		dev_err(dev, "could not allocate memory for gpios\n");
		return ERR_PTR(-ENOMEM);
	}

	for (i = 0; i < nrow; i++) {
		ret = of_get_named_gpio(np, "row-gpios", i);
		if (ret < 0)
			return ERR_PTR(ret);
		gpios[i] = ret;
	}

	for (i = 0; i < ncol; i++) {
		ret = of_get_named_gpio(np, "col-gpios", i);
		if (ret < 0)
			return ERR_PTR(ret);
		gpios[nrow + i] = ret;
	}

	pdata->row_gpios = gpios;
	pdata->col_gpios = &gpios[pdata->num_row_gpios];

	return pdata;
}
#else
static inline struct matrix_keypad_platform_data *
matrix_keypad_parse_dt(struct device *dev)
{
	dev_err(dev, "no platform data defined\n");

	return ERR_PTR(-EINVAL);
}
#endif
static int matrix_keypad_probe(struct platform_device *pdev)
{
	const struct matrix_keypad_platform_data *pdata;
	struct matrix_keypad *keypad;
	struct input_dev *input_dev;
	/* MODIFICATION: Add of_node pointer and keymap name for DT */
	struct device_node *np = pdev->dev.of_node;
	const char *keymap_name = NULL;
	int err;

	pdata = dev_get_platdata(&pdev->dev);
	if (!pdata) {
		pdata = matrix_keypad_parse_dt(&pdev->dev);
		if (IS_ERR(pdata))
			return PTR_ERR(pdata);
		/* MODIFICATION: For DT, the keymap is parsed from a property */
		keymap_name = "linux,keymap";
	} else if (!pdata->keymap_data) {
		dev_err(&pdev->dev, "no keymap data defined\n");
		return -EINVAL;
	}

	keypad = kzalloc(sizeof(struct matrix_keypad), GFP_KERNEL);
	input_dev = input_allocate_device();
	if (!keypad || !input_dev) {
		err = -ENOMEM;
		goto err_free_mem;
	}

	keypad->input_dev = input_dev;
	keypad->pdata = pdata;
	keypad->row_shift = get_count_order(pdata->num_col_gpios);
	keypad->keymap_size = pdata->num_row_gpios << keypad->row_shift;
	keypad->stopped = true;
	INIT_DELAYED_WORK(&keypad->work, matrix_keypad_scan);
	spin_lock_init(&keypad->lock);

	input_dev->name		= pdev->name;
	input_dev->id.bustype	= BUS_HOST;
	input_dev->dev.parent	= &pdev->dev;
	input_dev->open		= matrix_keypad_start;
	input_dev->close	= matrix_keypad_stop;

	/*
	 * Build the Fn keymap first, if it exists.
	 */
	if (pdata->fn_keymap_data) {
		if (pdata->fn_row >= pdata->num_row_gpios ||
		    pdata->fn_col >= pdata->num_col_gpios) {
			dev_err(&pdev->dev, "Fn key position out of bounds\n");
			err = -EINVAL;
			goto err_free_mem;
		}

		keypad->fn_keycodes = kcalloc(keypad->keymap_size,
					      sizeof(unsigned short),
					      GFP_KERNEL);
		if (!keypad->fn_keycodes) {
			err = -ENOMEM;
			goto err_free_mem;
		}

		/* MODIFICATION: Use property name for DT, data for platform data */
		err = matrix_keypad_build_keymap(np ? NULL : pdata->fn_keymap_data,
						 np ? "linux,fn-keymap" : NULL,
						 pdata->num_row_gpios,
						 pdata->num_col_gpios,
						 keypad->fn_keycodes,
						 input_dev);
		if (err) {
			dev_err(&pdev->dev, "failed to build Fn keymap\n");
			goto err_free_fn_keycodes;
		}

		keypad->active_keycodes = kcalloc(keypad->keymap_size,
						  sizeof(unsigned short),
						  GFP_KERNEL);
		if (!keypad->active_keycodes) {
			err = -ENOMEM;
			goto err_free_fn_keycodes;
		}
	}

	/*
	 * Build the main keymap last. This ensures input_dev->keycode points
	 * to the main keymap when we are done.
	 */
	/* MODIFICATION: Use property name for DT, data for platform data */
	err = matrix_keypad_build_keymap(np ? NULL : pdata->keymap_data,
					 keymap_name,
					 pdata->num_row_gpios,
					 pdata->num_col_gpios,
					 NULL, input_dev);
	if (err) {
		dev_err(&pdev->dev, "failed to build keymap\n");
		goto err_free_active_keycodes;
	}


	if (!pdata->no_autorepeat)
		__set_bit(EV_REP, input_dev->evbit);
	input_set_capability(input_dev, EV_MSC, MSC_SCAN);
	input_set_drvdata(input_dev, keypad);

	err = matrix_keypad_init_gpio(pdev, keypad);
	if (err)
		goto err_free_active_keycodes;

	err = input_register_device(keypad->input_dev);
	if (err)
		goto err_free_gpio;

	device_init_wakeup(&pdev->dev, pdata->wakeup);
	platform_set_drvdata(pdev, keypad);

	return 0;

err_free_gpio:
	matrix_keypad_free_gpio(keypad);
err_free_active_keycodes:
	kfree(keypad->active_keycodes);
err_free_fn_keycodes:
	kfree(keypad->fn_keycodes);
err_free_mem:
	input_free_device(input_dev);
	kfree(keypad);
	return err;
}

static int matrix_keypad_remove(struct platform_device *pdev)
{
	struct matrix_keypad *keypad = platform_get_drvdata(pdev);

	matrix_keypad_free_gpio(keypad);
	input_unregister_device(keypad->input_dev);
	kfree(keypad->fn_keycodes);
	kfree(keypad->active_keycodes);
	kfree(keypad);

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id matrix_keypad_dt_match[] = {
	{ .compatible = "gpio-matrix-keypad" },
	{ }
};
MODULE_DEVICE_TABLE(of, matrix_keypad_dt_match);
#endif

static struct platform_driver matrix_keypad_driver = {
	.probe		= matrix_keypad_probe,
	.remove		= matrix_keypad_remove,
	.driver		= {
		.name	= "matrix-keypad",
		.pm	= &matrix_keypad_pm_ops,
		.of_match_table = of_match_ptr(matrix_keypad_dt_match),
	},
};
module_platform_driver(matrix_keypad_driver);

MODULE_AUTHOR("Marek Vasut <marek.vasut@gmail.com>");
MODULE_DESCRIPTION("GPIO Driven Matrix Keypad Driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:matrix-keypad");
