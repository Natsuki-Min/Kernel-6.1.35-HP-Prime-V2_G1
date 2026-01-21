/*
 *  Driver for Goodix Touchscreens
 *
 *  Copyright (c) 2014 Red Hat Inc.
 *  Copyright (c) 2015 K. Merker <merker@debian.org>
 *
 *  This code is based on gt9xx.c authored by andrew@goodix.com:
 *
 *  2010 - 2012 Goodix Technology.
 */

/*
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; version 2 of the License.
 */

#include <linux/kernel.h>
#include <linux/dmi.h>
#include <linux/firmware.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/acpi.h>
#include <linux/input/goodix.h>
#include <linux/of.h>
#include <linux/of_gpio.h> /* 添加头文件支持设备树GPIO解析 */

#include <asm/unaligned.h>


#define GOODIX_GPIO_INT_NAME		"goodix_irq"
#define GOODIX_GPIO_RST_NAME		"goodix_reset"

#define GOODIX_MAX_HEIGHT		4096
#define GOODIX_MAX_WIDTH		4096
#define GOODIX_INT_TRIGGER		1
#define GOODIX_CONTACT_SIZE		8
#define GOODIX_MAX_CONTACTS		10

#define GOODIX_CONFIG_MIN_LENGTH	186
#define GOODIX_CONFIG_MAX_LENGTH	240
#define GOODIX_CONFIG_911_LENGTH	186
#define GOODIX_CONFIG_967_LENGTH	228

/* Register defines */
#define GOODIX_REG_COMMAND		0x8040
#define GOODIX_CMD_SCREEN_OFF		0x05

#define GOODIX_READ_COOR_ADDR		0x814E
#define GOODIX_REG_CONFIG_DATA		0x8047
#define GOODIX_REG_ID			0x8140

#define RESOLUTION_LOC		1
#define MAX_CONTACTS_LOC	5
#define TRIGGER_LOC		6

struct goodix_ts_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct goodix_ts_platform_data pdat;
	const struct goodix_chip_data *chip;
	int abs_x_max;
	int abs_y_max;
	bool swapped_x_y;
	bool inverted_x;
	bool inverted_y;
	unsigned int max_touch_num;
	unsigned int int_trigger_type;
	int cfg_len;
	int gpiod_int;
	int gpiod_rst;
	u16 id;
	u16 version;
	char *cfg_name;
	u8 config[GOODIX_CONFIG_MAX_LENGTH];
	struct completion firmware_loading_complete;
	unsigned long irq_flags;
};
struct goodix_chip_data {
	u16 config_addr;
	int config_len;
	int (*check_config)(struct goodix_ts_data *ts, const u8 *cfg, int len);
	void (*calc_config_checksum)(struct goodix_ts_data *ts);
};


static const unsigned long goodix_irq_flags[] = {
	IRQ_TYPE_EDGE_RISING,
	IRQ_TYPE_EDGE_FALLING,
	IRQ_TYPE_LEVEL_LOW,
	IRQ_TYPE_LEVEL_HIGH,
};

/**
 * goodix_i2c_read - read data from a register of the i2c slave device.
 *
 * @client: i2c device.
 * @reg: the register to read from.
 * @buf: raw write data buffer.
 * @len: length of the buffer to write
 */
int goodix_i2c_read(struct i2c_client *client,
			   u16 reg, u8 *buf, int len)
{
	struct i2c_msg msgs[2];
	u16 wbuf = cpu_to_be16(reg);
	int ret;

	msgs[0].flags = 0;
	msgs[0].addr  = client->addr;
	msgs[0].len   = 2;
	msgs[0].buf   = (u8 *)&wbuf;

	msgs[1].flags = I2C_M_RD;
	msgs[1].addr  = client->addr;
	msgs[1].len   = len;
	msgs[1].buf   = buf;

	ret = i2c_transfer(client->adapter, msgs, 2);
	return ret < 0 ? ret : (ret != ARRAY_SIZE(msgs) ? -EIO : 0);
}

/**
 * goodix_i2c_write - write data to a register of the i2c slave device.
 *
 * @client: i2c device.
 * @reg: the register to write to.
 * @buf: raw data buffer to write.
 * @len: length of the buffer to write
 */
int goodix_i2c_write(struct i2c_client *client, u16 reg, const u8 *buf,
			    unsigned len)
{
	u8 *addr_buf;
	struct i2c_msg msg;
	int ret;

	addr_buf = kmalloc(len + 2, GFP_KERNEL);
	if (!addr_buf)
		return -ENOMEM;

	addr_buf[0] = reg >> 8;
	addr_buf[1] = reg & 0xFF;
	memcpy(&addr_buf[2], buf, len);

	msg.flags = 0;
	msg.addr = client->addr;
	msg.buf = addr_buf;
	msg.len = len + 2;

	ret = i2c_transfer(client->adapter, &msg, 1);
	kfree(addr_buf);
	return ret < 0 ? ret : (ret != 1 ? -EIO : 0);
}

int goodix_i2c_write_u8(struct i2c_client *client, u16 reg, u8 value)
{
	return goodix_i2c_write(client, reg, &value, sizeof(value));
}

static int goodix_get_cfg_len(u16 id)
{
	switch (id) {
	case 911:
	case 9271:
	case 9110:
	case 927:
	case 928:
		return GOODIX_CONFIG_911_LENGTH;

	case 912:
	case 967:
		return GOODIX_CONFIG_967_LENGTH;

	default:
		return GOODIX_CONFIG_MAX_LENGTH;
	}
}

static int goodix_ts_read_input_report(struct goodix_ts_data *ts, u8 *data)
{
	int touch_num;
	int error;

	error = goodix_i2c_read(ts->client, GOODIX_READ_COOR_ADDR, data,
				GOODIX_CONTACT_SIZE + 1);
	if (error) {
		dev_err(&ts->client->dev, "I2C transfer error: %d\n", error);
		return error;
	}

	if (!(data[0] & 0x80))
		return -EAGAIN;

	touch_num = data[0] & 0x0f;
	if (touch_num > ts->max_touch_num)
		return -EPROTO;

	if (touch_num > 1) {
		data += 1 + GOODIX_CONTACT_SIZE;
		error = goodix_i2c_read(ts->client,
					GOODIX_READ_COOR_ADDR +
						1 + GOODIX_CONTACT_SIZE,
					data,
					GOODIX_CONTACT_SIZE * (touch_num - 1));
		if (error)
			return error;
	}

	return touch_num;
}

static void goodix_ts_report_touch(struct goodix_ts_data *ts, u8 *coor_data)
{
	int input_x = get_unaligned_le16(&coor_data[1]);
	int input_y = get_unaligned_le16(&coor_data[3]);
	int input_w = get_unaligned_le16(&coor_data[5]);

	/* Inversions have to happen before axis swapping */
	if (ts->inverted_x)
		input_x = ts->abs_x_max - input_x;
	if (ts->inverted_y)
		input_y = ts->abs_y_max - input_y;
	if (ts->swapped_x_y)
		swap(input_x, input_y);

	if (ts->pdat.multitouch) {
		input_report_abs(ts->input_dev, ABS_MT_POSITION_X, input_x);
		input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, input_y);
		input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, input_w);
		input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, input_w);
	}

	input_event(ts->input_dev, EV_ABS, ABS_X, input_x);
	input_event(ts->input_dev, EV_ABS, ABS_Y, input_y);
	input_event(ts->input_dev, EV_KEY, BTN_TOUCH, 1);
	input_report_abs(ts->input_dev, ABS_PRESSURE, 1);
}

/**
 * goodix_process_events - Process incoming events
 *
 * @ts: our goodix_ts_data pointer
 *
 * Called when the IRQ is triggered. Read the current device state, and push
 * the input events to the user space.
 */
static void goodix_process_events(struct goodix_ts_data *ts)
{
	u8  point_data[1 + GOODIX_CONTACT_SIZE * GOODIX_MAX_CONTACTS];
	int touch_num;
	int i;

	touch_num = goodix_ts_read_input_report(ts, point_data);
	if (touch_num < 0)
		return;

	if (!touch_num) {
		input_event(ts->input_dev, EV_KEY, BTN_TOUCH, 0);
		input_report_abs(ts->input_dev, ABS_PRESSURE, 0);
	}

	for (i = 0; i < touch_num; i++)
		goodix_ts_report_touch(ts,
				&point_data[1 + GOODIX_CONTACT_SIZE * i]);

	input_sync(ts->input_dev);
}

/**
 * goodix_ts_irq_handler - The IRQ handler
 *
 * @irq: interrupt number.
 * @dev_id: private data pointer.
 */
static irqreturn_t goodix_ts_irq_handler(int irq, void *dev_id)
{
	struct goodix_ts_data *ts = dev_id;

	goodix_process_events(ts);
	if (goodix_i2c_write_u8(ts->client, GOODIX_READ_COOR_ADDR, 0) < 0)
		dev_err(&ts->client->dev, "I2C write end_cmd error\n");

	return IRQ_HANDLED;
}

static void goodix_free_irq(struct goodix_ts_data *ts)
{
	devm_free_irq(&ts->client->dev, ts->client->irq, ts);
}

static int goodix_request_irq(struct goodix_ts_data *ts)
{
	dev_info(&ts->client->dev, "REQUESTING IRQ %d\n", ts->client->irq);
	return devm_request_threaded_irq(&ts->client->dev, ts->client->irq,
					 NULL, goodix_ts_irq_handler,
					 ts->irq_flags, ts->client->name, ts);
}

int goodix_int_sync(struct goodix_ts_data *ts)
{
	int error;

	error = gpio_direction_output(ts->gpiod_int, 0);
	if (error)
		return error;

	msleep(50);				/* T5: 50ms */

	/* 复位完成后，确保中断脚设回输入模式，以便接收中断 */
	error = gpio_direction_input(ts->gpiod_int);
	if (error)
		return error;

	return 0;
}

/**
 * goodix_reset - Reset device during power on
 *
 * @ts: goodix_ts_data pointer
 */
static int goodix_reset(struct goodix_ts_data *ts)
{
	int error;

	/* begin select I2C slave addr */
	error = gpio_direction_output(ts->gpiod_rst, 0);
	if (error)
		return error;

	msleep(20);				/* T2: > 10ms */

	/* HIGH: 0x28/0x29, LOW: 0xBA/0xBB */
	/* 设置中断脚的电平来决定I2C地址 */
	error = gpio_direction_output(ts->gpiod_int, ts->client->addr == 0x14);
	if (error)
		return error;

	usleep_range(100, 2000);		/* T3: > 100us */

	error = gpio_direction_output(ts->gpiod_rst, 1);
	if (error)
		return error;

	usleep_range(6000, 10000);		/* T4: > 5ms */

	/* end select I2C slave addr */
	/* 复位结束后，复位脚设为输入（或高电平） */
	error = gpio_direction_input(ts->gpiod_rst);
	if (error)
		return error;

	/* 
	 * 重要：同步中断脚状态。
	 * 在 goodix_int_sync 里，会将 gpiod_int 设回 input 模式。
	 * 这对于后续的 gpio_to_irq 至关重要。
	 */
	error = goodix_int_sync(ts);
	if (error)
		return error;

	return 0;
}

/**
 * goodix_get_gpio_config - Get GPIO config from ACPI/DT
 *
 * @ts: goodix_ts_data pointer
 */
static int goodix_get_gpio_config(struct goodix_ts_data *ts)
{
	int error;
	struct device *dev;
	struct device_node *np;

	if (!ts->client)
		return -EINVAL;
	dev = &ts->client->dev;
	np = dev->of_node;

	ts->gpiod_rst = -1;
	ts->gpiod_int = -1;

	/* 1. 尝试从设备树获取 GPIO */
	if (np) {
		ts->gpiod_rst = of_get_named_gpio(np, "reset-gpios", 0);
		if (!gpio_is_valid(ts->gpiod_rst))
			ts->gpiod_rst = of_get_named_gpio(np, "goodix,reset-gpio", 0);

		ts->gpiod_int = of_get_named_gpio(np, "irq-gpios", 0);
		if (!gpio_is_valid(ts->gpiod_int))
			ts->gpiod_int = of_get_named_gpio(np, "goodix,irq-gpio", 0);
	}
	/* 2. 兼容旧的 Platform Data */
	else if (ts->pdat.pin_rst || ts->pdat.pin_int) {
		ts->gpiod_rst = ts->pdat.pin_rst;
		ts->gpiod_int = ts->pdat.pin_int;
	}

	/* 3. 申请 Reset GPIO */
	if (gpio_is_valid(ts->gpiod_rst)) {
		error = devm_gpio_request(dev, ts->gpiod_rst, GOODIX_GPIO_RST_NAME);
		if (error) {
			dev_err(dev, "Failed to request Reset GPIO %d: %d\n", 
				ts->gpiod_rst, error);
			return error;
		}
		/* 初始化为复位状态 */
		gpio_direction_output(ts->gpiod_rst, 0);
	}

	/* 4. 申请 IRQ GPIO */
	if (gpio_is_valid(ts->gpiod_int)) {
		error = devm_gpio_request(dev, ts->gpiod_int, GOODIX_GPIO_INT_NAME);
		if (error) {
			dev_err(dev, "Failed to request INT GPIO %d: %d\n", 
				ts->gpiod_int, error);
			return error;
		}
		/* 
		 * 必须先设置为 Input。
		 * 对于 S3C2416，通常 request_irq 会负责把引脚配成 EINT，
		 * 但先设为 Input 是标准的 GPIO 用法。
		 */
		gpio_direction_input(ts->gpiod_int);
	} else {
		dev_err(dev, "Invalid INT GPIO!\n");
		return -EINVAL;
	}

	return 0;
}

/**
 * goodix_read_config - Read the embedded configuration of the panel
 *
 * @ts: our goodix_ts_data pointer
 *
 * Must be called during probe
 */
static void goodix_read_config(struct goodix_ts_data *ts)
{
	u8 config[GOODIX_CONFIG_MAX_LENGTH];
	int error;

	error = goodix_i2c_read(ts->client, GOODIX_REG_CONFIG_DATA,
				config, ts->cfg_len);
	if (error) {
		dev_warn(&ts->client->dev,
			 "Error reading config (%d), using defaults\n",
			 error);
		ts->abs_x_max = GOODIX_MAX_WIDTH;
		ts->abs_y_max = GOODIX_MAX_HEIGHT;
		if (ts->swapped_x_y)
			swap(ts->abs_x_max, ts->abs_y_max);
		ts->int_trigger_type = GOODIX_INT_TRIGGER;
		ts->max_touch_num = GOODIX_MAX_CONTACTS;
		return;
	}

	ts->abs_x_max = get_unaligned_le16(&config[RESOLUTION_LOC]);
	ts->abs_y_max = get_unaligned_le16(&config[RESOLUTION_LOC + 2]);
	if (ts->swapped_x_y)
		swap(ts->abs_x_max, ts->abs_y_max);
	ts->int_trigger_type = config[TRIGGER_LOC] & 0x03;
	ts->max_touch_num = config[MAX_CONTACTS_LOC] & 0x0f;
	if (!ts->abs_x_max || !ts->abs_y_max || !ts->max_touch_num) {
		dev_err(&ts->client->dev,
			"Invalid config, using defaults\n");
		ts->abs_x_max = GOODIX_MAX_WIDTH;
		ts->abs_y_max = GOODIX_MAX_HEIGHT;
		if (ts->swapped_x_y)
			swap(ts->abs_x_max, ts->abs_y_max);
		ts->max_touch_num = GOODIX_MAX_CONTACTS;
	}
}

/**
 * goodix_read_version - Read goodix touchscreen version
 *
 * @ts: our goodix_ts_data pointer
 */
static int goodix_read_version(struct goodix_ts_data *ts)
{
	int error;
	u8 buf[6];
	char id_str[5];

	error = goodix_i2c_read(ts->client, GOODIX_REG_ID, buf, sizeof(buf));
	if (error) {
		dev_err(&ts->client->dev, "read version failed: %d\n", error);
		return error;
	}

	memcpy(id_str, buf, 4);
	id_str[4] = 0;
	ts->id = simple_strtoul(id_str, NULL, 10);

	ts->version = get_unaligned_le16(&buf[4]);

	dev_info(&ts->client->dev, "ID %d, version: %04x\n", ts->id,
		 ts->version);

	return 0;
}

/**
 * goodix_i2c_test - I2C test function to check if the device answers.
 *
 * @client: the i2c client
 */
static int goodix_i2c_test(struct i2c_client *client)
{
	int retry = 0;
	int error;
	u8 test;

	while (retry++ < 2) {
		error = goodix_i2c_read(client, GOODIX_REG_CONFIG_DATA,
					&test, 1);
		if (!error)
			return 0;

		dev_err(&client->dev, "i2c test failed attempt %d: %d\n",
			retry, error);
		msleep(20);
	}

	return error;
}


/**
 * goodix_check_cfg - Checks if config fw is valid
 *
 * @ts: goodix_ts_data pointer
 * @cfg: firmware config data
 * @len: config data length
 */
static int goodix_check_cfg(struct goodix_ts_data *ts, const u8 *cfg, int len)
{
	if (len < GOODIX_CONFIG_MIN_LENGTH ||
	    len > GOODIX_CONFIG_MAX_LENGTH) {
		dev_err(&ts->client->dev,
			"The length of the config fw is not correct");
		return -EINVAL;
	}

	return ts->chip->check_config(ts, cfg, len);
}

/**
 * goodix_send_cfg - Write fw config to device
 *
 * @ts: goodix_ts_data pointer
 * @cfg: config firmware to write to device
 * @len: config data length
 */
int goodix_send_cfg(struct goodix_ts_data *ts, const u8 *cfg, int len)
{
	int error;

	error = goodix_check_cfg(ts, cfg, len);
	if (error)
		return error;

	error = goodix_i2c_write(ts->client, ts->chip->config_addr, cfg, len);
	if (error)
		return error;

	dev_dbg(&ts->client->dev, "Config sent successfully.");

	/* Let the firmware reconfigure itself, so sleep for 10ms */
	usleep_range(10000, 11000);

	return 0;
}


static int goodix_check_cfg_8(struct goodix_ts_data *ts, const u8 *cfg, int len)
{
	int i, raw_cfg_len = len - 2;
	u8 check_sum = 0;

	for (i = 0; i < raw_cfg_len; i++)
		check_sum += cfg[i];
	check_sum = (~check_sum) + 1;
	if (check_sum != cfg[raw_cfg_len]) {
		dev_err(&ts->client->dev,
			"The checksum of the config fw is not correct");
		return -EINVAL;
	}

	if (cfg[raw_cfg_len + 1] != 1) {
		dev_err(&ts->client->dev,
			"Config fw must have Config_Fresh register set");
		return -EINVAL;
	}

	return 0;
}

static void goodix_calc_cfg_checksum_8(struct goodix_ts_data *ts)
{
	int i, raw_cfg_len = ts->chip->config_len - 2;
	u8 check_sum = 0;

	for (i = 0; i < raw_cfg_len; i++)
		check_sum += ts->config[i];
	check_sum = (~check_sum) + 1;

	ts->config[raw_cfg_len] = check_sum;
	ts->config[raw_cfg_len + 1] = 1; /* Set "config_fresh" bit */
}

static int goodix_check_cfg_16(struct goodix_ts_data *ts, const u8 *cfg,
			       int len)
{
	int i, raw_cfg_len = len - 3;
	u16 check_sum = 0;

	for (i = 0; i < raw_cfg_len; i += 2)
		check_sum += get_unaligned_be16(&cfg[i]);
	check_sum = (~check_sum) + 1;
	if (check_sum != get_unaligned_be16(&cfg[raw_cfg_len])) {
		dev_err(&ts->client->dev,
			"The checksum of the config fw is not correct");
		return -EINVAL;
	}

	if (cfg[raw_cfg_len + 2] != 1) {
		dev_err(&ts->client->dev,
			"Config fw must have Config_Fresh register set");
		return -EINVAL;
	}

	return 0;
}

static void goodix_calc_cfg_checksum_16(struct goodix_ts_data *ts)
{
	int i, raw_cfg_len = ts->chip->config_len - 3;
	u16 check_sum = 0;

	for (i = 0; i < raw_cfg_len; i += 2)
		check_sum += get_unaligned_be16(&ts->config[i]);
	check_sum = (~check_sum) + 1;

	put_unaligned_be16(check_sum, &ts->config[raw_cfg_len]);
	ts->config[raw_cfg_len + 2] = 1; /* Set "config_fresh" bit */
}


/**
 * goodix_reset_no_int_sync - Reset device, leaving interrupt line in output mode
 *
 * @ts: goodix_ts_data pointer
 */
int goodix_reset_no_int_sync(struct goodix_ts_data *ts)
{
	int error;

	/* begin select I2C slave addr */
	error = gpio_direction_output(ts->gpiod_rst, 0);
	if (error)
		goto error;

	msleep(20);				/* T2: > 10ms */

	/* HIGH: 0x28/0x29, LOW: 0xBA/0xBB */
	error = gpio_direction_output(ts->gpiod_int, ts->client->addr == 0x14);
	if (error)
		goto error;

	usleep_range(100, 2000);		/* T3: > 100us */

	error = gpio_direction_output(ts->gpiod_rst, 1);
	if (error)
		goto error;

	usleep_range(6000, 10000);		/* T4: > 5ms */

	/*
	 * Put the reset pin back in to input / high-impedance mode to save
	 * power. Only do this in the non ACPI case since some ACPI boards
	 * don't have a pull-up, so there the reset pin must stay active-high.
	 */
	//if (ts->irq_pin_access_method == IRQ_PIN_ACCESS_GPIO) 
	{
		error = gpio_direction_input(ts->gpiod_rst);
		if (error)
			goto error;
	}

	return 0;

error:
	dev_err(&ts->client->dev, "Controller reset failed.\n");
	return error;
}


/**
 * goodix_request_input_dev - Allocate, populate and register the input device
 *
 * @ts: our goodix_ts_data pointer
 *
 * Must be called during probe
 */
static int goodix_request_input_dev(struct goodix_ts_data *ts)
{
	int error;

	ts->input_dev = input_allocate_device();
	if (!ts->input_dev) {
		dev_err(&ts->client->dev, "Failed to allocate input device.");
		return -ENOMEM;
	}

	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X,
			     0, ts->abs_x_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y,
			     0, ts->abs_y_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);

	__set_bit(EV_ABS, ts->input_dev->evbit);
	__set_bit(EV_KEY, ts->input_dev->evbit);
	__set_bit(BTN_TOUCH, ts->input_dev->keybit);
	__set_bit(ABS_X, ts->input_dev->absbit);
	__set_bit(ABS_Y, ts->input_dev->absbit);
	__set_bit(ABS_PRESSURE, ts->input_dev->absbit);
	__set_bit(EV_SYN, ts->input_dev->evbit);

	ts->input_dev->name = "Goodix Capacitive TouchScreen";
	ts->input_dev->phys = "input/ts";
	ts->input_dev->id.bustype = BUS_I2C;
	ts->input_dev->id.vendor = 0x0416;
	ts->input_dev->id.product = ts->id;
	ts->input_dev->id.version = ts->version;

	error = input_register_device(ts->input_dev);
	if (error) {
		dev_err(&ts->client->dev,
			"Failed to register input device: %d", error);
		return error;
	}

	return 0;
}

/**
 * goodix_configure_dev - Finish device initialization
 *
 * @ts: our goodix_ts_data pointer
 *
 * Must be called from probe to finish initialization of the device.
 * Contains the common initialization code for both devices that
 * declare gpio pins and devices that do not. It is either called
 * directly from probe or from request_firmware_wait callback.
 */
static int goodix_configure_dev(struct goodix_ts_data *ts)
{
	int error;

	/* 设备树属性解析 */
	ts->swapped_x_y = of_property_read_bool(ts->client->dev.of_node, "touchscreen-swapped-x-y");
	ts->inverted_x = of_property_read_bool(ts->client->dev.of_node, "touchscreen-inverted-x");
	ts->inverted_y = of_property_read_bool(ts->client->dev.of_node, "touchscreen-inverted-y");

	goodix_read_config(ts);

	error = goodix_request_input_dev(ts);
	if (error)
		return error;

	ts->irq_flags = goodix_irq_flags[ts->int_trigger_type] | IRQF_ONESHOT;
	
	/* 
	 * 关键点：动态转换 GPIO 到 IRQ 
	 * 类似于 matrix_keypad，我们在驱动里做这个映射，
	 * 而不是依赖 DTS 里的 interrupts 属性。
	 */
	if (gpio_is_valid(ts->gpiod_int)) {
		ts->client->irq = gpio_to_irq(ts->gpiod_int);
		if (ts->client->irq < 0) {
			dev_err(&ts->client->dev, "Unable to map GPIO %d to IRQ\n", ts->gpiod_int);
			return ts->client->irq;
		}
	} else {
		dev_err(&ts->client->dev, "Invalid GPIO for IRQ\n");
		return -EINVAL;
	}

	error = goodix_request_irq(ts);
	if (error) {
		dev_err(&ts->client->dev, "request IRQ failed: %d\n", error);
		return error;
	}

	return 0;
}

static int goodix_ts_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct goodix_ts_data *ts;
	int error;

	dev_dbg(&client->dev, "I2C Address: 0x%02x\n", client->addr);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "I2C check functionality failed.\n");
		return -ENXIO;
	}

	ts = devm_kzalloc(&client->dev, sizeof(*ts), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;

	ts->client = client;
	i2c_set_clientdata(client, ts);
	init_completion(&ts->firmware_loading_complete);

	if (client->dev.platform_data)
		memcpy(&ts->pdat, client->dev.platform_data, sizeof(ts->pdat));

	/* 获取 GPIO */
	error = goodix_get_gpio_config(ts);
	if (error) {
		dev_warn(&client->dev, "Get GPIO config error\n");
		return error;
	}

	if (gpio_is_valid(ts->gpiod_rst)) {
		/* reset the controller */
		error = goodix_reset(ts);
		if (error) {
			dev_err(&client->dev, "Controller reset failed.\n");
			return error;
		}
	}

	error = goodix_i2c_test(client);
	if (error) {
		dev_err(&client->dev, "I2C communication failure: %d\n", error);
		return error;
	}

	error = goodix_read_version(ts);
	if (error) {
		dev_err(&client->dev, "Read version failed.\n");
		return error;
	}

	ts->cfg_len = goodix_get_cfg_len(ts->id);

	error = goodix_configure_dev(ts);
	if (error) {
		dev_err(&client->dev,
			"Failed to configure device %d\n", error);
		return error;
	}

	return 0;
}

static void goodix_ts_remove(struct i2c_client *client)
{
	struct goodix_ts_data *ts = i2c_get_clientdata(client);

	if (ts->gpiod_int && ts->gpiod_rst)
		wait_for_completion(&ts->firmware_loading_complete);
 
}

static int __maybe_unused goodix_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct goodix_ts_data *ts = i2c_get_clientdata(client);
	int error;

	/* We need gpio pins to suspend/resume */
	if (!ts->gpiod_int || !ts->gpiod_rst)
		return 0;

	wait_for_completion(&ts->firmware_loading_complete);

	/* Free IRQ as IRQ pin is used as output in the suspend sequence */
	goodix_free_irq(ts);

	/* Output LOW on the INT pin for 5 ms */
	error = gpio_direction_output(ts->gpiod_int, 0);
	if (error) {
		goodix_request_irq(ts);
		return error;
	}

	usleep_range(5000, 6000);

	error = goodix_i2c_write_u8(ts->client, GOODIX_REG_COMMAND,
				    GOODIX_CMD_SCREEN_OFF);
	if (error) {
		dev_err(&ts->client->dev, "Screen off command failed\n");
		gpio_direction_input(ts->gpiod_int);
		goodix_request_irq(ts);
		return -EAGAIN;
	}

	/*
	 * The datasheet specifies that the interval between sending screen-off
	 * command and wake-up should be longer than 58 ms. To avoid waking up
	 * sooner, delay 58ms here.
	 */
	msleep(58);
	return 0;
}

static int __maybe_unused goodix_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct goodix_ts_data *ts = i2c_get_clientdata(client);
	int error;

	if (!ts->gpiod_int || !ts->gpiod_rst)
		return 0;

	/*
	 * Exit sleep mode by outputting HIGH level to INT pin
	 * for 2ms~5ms.
	 */
	error = gpio_direction_output(ts->gpiod_int, 1);
	if (error)
		return error;

	usleep_range(2000, 5000);

	error = goodix_int_sync(ts);
	if (error)
		return error;
	
	/* 恢复 IRQ */
	if (gpio_is_valid(ts->gpiod_int)) 
		ts->client->irq = gpio_to_irq(ts->gpiod_int);

	error = goodix_request_irq(ts);
	if (error)
		return error;

	return 0;
}

static SIMPLE_DEV_PM_OPS(goodix_pm_ops, goodix_suspend, goodix_resume);

static const struct i2c_device_id goodix_ts_id[] = {
	{ "GDIX1001:00", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, goodix_ts_id);

#ifdef CONFIG_ACPI
static const struct acpi_device_id goodix_acpi_match[] = {
	{ "GDIX1001", 0 },
	{ }
};
MODULE_DEVICE_TABLE(acpi, goodix_acpi_match);
#endif

#ifdef CONFIG_OF
static const struct of_device_id goodix_of_match[] = {
	{ .compatible = "goodix,gt911" },
	{ .compatible = "goodix,gt9110" },
	{ .compatible = "goodix,gt912" },
	{ .compatible = "goodix,gt927" },
	{ .compatible = "goodix,gt9271" },
	{ .compatible = "goodix,gt928" },
	{ .compatible = "goodix,gt967" },
	{ }
};
MODULE_DEVICE_TABLE(of, goodix_of_match);
#endif

static struct i2c_driver goodix_ts_driver = {
	.probe = goodix_ts_probe,
	.remove = goodix_ts_remove,
	.id_table = goodix_ts_id,
	.driver = {
		.name = "Goodix-TS",
		.pm = &goodix_pm_ops,
		.of_match_table = of_match_ptr(goodix_of_match), /* 必须添加这个! */
	},
};

static int __init goodix_init(void)
{
	return i2c_add_driver(&goodix_ts_driver);
}

static void __exit goodix_exit(void)
{
	i2c_del_driver(&goodix_ts_driver);
}

module_init(goodix_init);
module_exit(goodix_exit);

MODULE_AUTHOR("Benjamin Tissoires <benjamin.tissoires@gmail.com>");
MODULE_AUTHOR("Bastien Nocera <hadess@hadess.net>");
MODULE_DESCRIPTION("Goodix touchscreen driver");
MODULE_LICENSE("GPL v2");