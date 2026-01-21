/*
 * Novatek NT11002 TouchScreen Driver - Advanced Version
 * 
 * Features:
 * 1. Robust Probe: Checks I2C ACK only (ignores 0xFF data).
 * 2. Debug Interface: /sys/.../raw_access for firmware dumping.
 * 3. MT Protocol B support with ID fix.
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/sysfs.h>

#define NOVATEK_TS_NAME     "novatek_nt11002"

#define MAX_FINGER_NUM      4
#define I2C_XFER_LEN        12

struct novatek_ts_data {
    struct i2c_client *client;
    struct input_dev *input_dev;
    struct gpio_desc *reset_gpio;
    struct gpio_desc *irq_gpio;
    u32 max_x;
    u32 max_y;
    int irq;
    struct timer_list release_timer;
    /* For raw access interface */
    struct mutex sysfs_mutex;
    u8 sysfs_reg_addr; /* Stored register address for subsequent reads */
};
static void novatek_timer_callback(struct timer_list *t)
{
    struct novatek_ts_data *ts = from_timer(ts, t, release_timer);
    struct input_dev *input = ts->input_dev;

    // 定时器这里顺序无所谓，因为是要全部释放，但为了保持一致：
    input_mt_sync_frame(input); // 开启新的一帧
    input_mt_drop_unused(input); // 这一帧里谁都没Report，所以全扔掉
    input_sync(input);
}
/* --- Core I2C Helper --- */
static int novatek_i2c_read(struct i2c_client *client, u8 *buf, int len)
{
    // 很多触摸屏不需要写寄存器地址就能直接读，或者地址是 0。
    // 旧驱动是写了 0x00 的。
    u8 reg_addr = 0x00;
    struct i2c_msg msgs[2] = {
        {
            .addr = client->addr,
            .flags = 0, // Write
            .len = 1,
            .buf = &reg_addr,
        },
        {
            .addr = client->addr,
            .flags = I2C_M_RD, // Read
            .len = len,
            .buf = buf,
        },
    };
    
    int ret = i2c_transfer(client->adapter, msgs, 2);
    // 成功应该返回 2 (2个msg都处理了)
    if (ret != 2)
        return -EIO;

    return 0;
}

/* --- Hardware Control --- */
static void novatek_reset(struct novatek_ts_data *ts)
{
    if (!ts->reset_gpio)
        return;

    gpiod_set_value_cansleep(ts->reset_gpio, 1);
    msleep(50);
    gpiod_set_value_cansleep(ts->reset_gpio, 0);
    msleep(150); 
}

/* --- Interrupt Handler --- */
static void novatek_release_handler(struct timer_list *t)
{
    struct novatek_ts_data *ts = from_timer(ts, t, release_timer);
    struct input_dev *input = ts->input_dev;

    input_mt_sync_frame(input);
    input_mt_drop_unused(input); // 没人认领，全部释放
    input_sync(input);
}

static irqreturn_t novatek_ts_handler(int irq, void *dev_id)
{
    struct novatek_ts_data *ts = dev_id;
    struct input_dev *input = ts->input_dev;
    u8 buf[24] = {0}; 
    int i, ret;
    
    // 1. 喂狗
    mod_timer(&ts->release_timer, jiffies + msecs_to_jiffies(100));

    ret = novatek_i2c_read(ts->client, buf, sizeof(buf));
    if (ret < 0) {
        dev_err_ratelimited(&ts->client->dev, "I2C read failed\n");
        return IRQ_HANDLED;
    }

    // ==========================================================
    // 【关键修复】: 必须先 Sync 告诉内核新的一帧开始了
    // 这样后面 Report 的 Slot 才会获得最新的帧号
    // ==========================================================
    input_mt_sync_frame(input);

    // 2. 遍历数据
    for (i = 0; i < MAX_FINGER_NUM; i++) {
        int pos = i * 6;
        
        if (buf[pos] == 0xff) {
            break;
        }

        u8 id = (buf[pos] >> 3) & 0x07; 
        u8 status = buf[pos] & 0x03;

        if (id < 1 || id > MAX_FINGER_NUM) continue;
        int slot_id = id - 1;

        // 0x01: Down, 0x02
        bool active = (status == 0x01 || status == 0x02 );

        input_mt_slot(input, slot_id);
        input_mt_report_slot_state(input, MT_TOOL_FINGER, active);

        if (active) {
            u8 x_high = buf[pos + 1];
            u8 y_high = buf[pos + 2];
            u8 low_byte = buf[pos + 3];

            int x = (x_high << 4) | ((low_byte & 0xF0) >> 4);
            int y = (y_high << 4) | (low_byte & 0x0F);

            if (x >= ts->max_x) x = ts->max_x - 1;
            if (y >= ts->max_y) y = ts->max_y - 1;

            input_report_abs(input, ABS_MT_POSITION_X, x);
            input_report_abs(input, ABS_MT_POSITION_Y, y);
        }
    }

    // 3. 清理掉那些这一帧没有 Report 过的 Slot
    // 因为我们在循环前 Sync 了，所以循环里 Report 过的都是“新”的，不会被清理
    // 只有循环里没处理的（因为break跳过或者之前的Ghost）才会被丢弃
    input_mt_drop_unused(input);

    input_sync(input);

    return IRQ_HANDLED;
}
/* --- Sysfs Debug Interface (Ported from original ntp_flash logic) --- */
/*
 * This allows you to perform raw I2C R/W from userspace to dump firmware.
 * Path: /sys/bus/i2c/devices/X-00XX/raw_access
 * 
 * Logic:
 * 1. Write to file: Sets the register address (1st byte) and optionally writes data.
 * 2. Read from file: Reads data from the previously set register address.
 * 
 * Note: It disables the Touch IRQ during transfer to avoid collision.
 */
static ssize_t novatek_sysfs_read(struct file *filp, struct kobject *kobj,
                                  struct bin_attribute *bin_attr,
                                  char *buf, loff_t off, size_t count)
{
    struct device *dev = kobj_to_dev(kobj);
    struct i2c_client *client = to_i2c_client(dev);
    struct novatek_ts_data *ts = i2c_get_clientdata(client);
    struct i2c_msg msgs[2];
    int ret;

    mutex_lock(&ts->sysfs_mutex);
    disable_irq(ts->irq);

    /* Phase 1: Write Register Address */
    msgs[0].addr = client->addr;
    msgs[0].flags = 0;
    msgs[0].len = 1;
    msgs[0].buf = &ts->sysfs_reg_addr;

    /* Phase 2: Read Data */
    msgs[1].addr = client->addr;
    msgs[1].flags = I2C_M_RD;
    msgs[1].len = count;
    msgs[1].buf = buf;

    ret = i2c_transfer(client->adapter, msgs, 2);

    enable_irq(ts->irq);
    mutex_unlock(&ts->sysfs_mutex);

    if (ret != 2)
        return -EIO;

    return count;
}

static ssize_t novatek_sysfs_write(struct file *filp, struct kobject *kobj,
                                   struct bin_attribute *bin_attr,
                                   char *buf, loff_t off, size_t count)
{
    struct device *dev = kobj_to_dev(kobj);
    struct i2c_client *client = to_i2c_client(dev);
    struct novatek_ts_data *ts = i2c_get_clientdata(client);
    struct i2c_msg msg;
    int ret;

    if (count < 1) return -EINVAL;

    mutex_lock(&ts->sysfs_mutex);
    disable_irq(ts->irq);

    /* Store the first byte as the "Current Register" for future reads */
    ts->sysfs_reg_addr = buf[0];

    /* Perform the write (Register + Data) */
    msg.addr = client->addr;
    msg.flags = 0;
    msg.len = count;
    msg.buf = buf;

    ret = i2c_transfer(client->adapter, &msg, 1);

    enable_irq(ts->irq);
    mutex_unlock(&ts->sysfs_mutex);

    if (ret != 1)
        return -EIO;

    return count;
}

static struct bin_attribute raw_access_attr = {
    .attr = {
        .name = "raw_access",
        .mode = 0660,
    },
    .read = novatek_sysfs_read,
    .write = novatek_sysfs_write,
    .size = 0, /* Unlimited size */
};

/* --- Probe --- */
static int novatek_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    struct device *dev = &client->dev;
    struct novatek_ts_data *ts;
    int error;
    int irq_num;
    u8 dummy_buf = 0;
    struct i2c_msg dummy_msg;
    int retry = 3;
    bool ack_received = false;

    dev_info(dev, "Probing Novatek TS at addr 0x%02x\n", client->addr);

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
        return -ENODEV;

    ts = devm_kzalloc(dev, sizeof(*ts), GFP_KERNEL);
    if (!ts)
        return -ENOMEM;

    ts->client = client;
    mutex_init(&ts->sysfs_mutex);
    i2c_set_clientdata(client, ts);

    /* GPIO & IRQ Setup */
    ts->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
    if (IS_ERR(ts->reset_gpio)) return PTR_ERR(ts->reset_gpio);

    ts->irq_gpio = devm_gpiod_get(dev, "irq", GPIOD_IN);
    if (IS_ERR(ts->irq_gpio)) return PTR_ERR(ts->irq_gpio);

    irq_num = gpiod_to_irq(ts->irq_gpio);
    if (irq_num < 0) return irq_num;
    ts->irq = irq_num;

    /* DT Properties */
    if (of_property_read_u32(dev->of_node, "touchscreen-size-x", &ts->max_x))
        ts->max_x = 1280;
    if (of_property_read_u32(dev->of_node, "touchscreen-size-y", &ts->max_y))
        ts->max_y = 800;

    /* Hardware Reset */
    novatek_reset(ts);

    /* 
     * --- Connection Check (ACK Only) ---
     * Perform a 1-byte read. We don't care if data is 0xFF.
     * We ONLY check if i2c_transfer returns 1 (meaning ACK received).
     */
    dummy_msg.addr = client->addr;
    dummy_msg.flags = I2C_M_RD;
    dummy_msg.len = 1;
    dummy_msg.buf = &dummy_buf;

    while (retry--) {
        error = i2c_transfer(client->adapter, &dummy_msg, 1);
        if (error == 1) {
            ack_received = true;
            break;
        }
        msleep(20);
    }

    if (!ack_received) {
        dev_err(dev, "I2C Address 0x%02x not responding (NACK). Check connection.\n", client->addr);
        return -ENXIO;
    }
    
    dev_info(dev, "Device found (ACK received). Registering...\n");
    timer_setup(&ts->release_timer, novatek_timer_callback, 0);
    /* Input Device Setup */
    ts->input_dev = devm_input_allocate_device(dev);
    if (!ts->input_dev) return -ENOMEM;

    ts->input_dev->name = "Novatek NT11002 TouchScreen";
    ts->input_dev->id.bustype = BUS_I2C;
    
    input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, ts->max_x-1, 0, 0);
    input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, ts->max_y-1, 0, 0);
    
    error = input_mt_init_slots(ts->input_dev, MAX_FINGER_NUM, INPUT_MT_DIRECT);
    if (error) return error;

    error = input_register_device(ts->input_dev);
    if (error) return error;

    /* IRQ Request */
    error = devm_request_threaded_irq(dev, irq_num, NULL, novatek_ts_handler,
                                      IRQF_ONESHOT | IRQF_TRIGGER_LOW,
                                      NOVATEK_TS_NAME, ts);
    if (error) {
        dev_err(dev, "Failed to request IRQ\n");
        return error;
    }

    /* Register Sysfs Raw Access */
    error = sysfs_create_bin_file(&dev->kobj, &raw_access_attr);
    if (error) {
        dev_warn(dev, "Failed to create debug sysfs entry\n");
    }

    return 0;
}

static void novatek_remove(struct i2c_client *client)
{
    struct device *dev = &client->dev;
    struct novatek_ts_data *ts = i2c_get_clientdata(client);
    del_timer_sync(&ts->release_timer);
    sysfs_remove_bin_file(&dev->kobj, &raw_access_attr);
    return ;
}

static const struct of_device_id novatek_dt_match[] = {
    { .compatible = "novatek,nt11002" },
    { }
};
MODULE_DEVICE_TABLE(of, novatek_dt_match);

static const struct i2c_device_id novatek_id_match[] = {
    { NOVATEK_TS_NAME, 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, novatek_id_match);

static struct i2c_driver novatek_ts_driver = {
    .driver = {
        .name = NOVATEK_TS_NAME,
        .of_match_table = novatek_dt_match,
    },
    .probe = novatek_probe,
    .remove = novatek_remove,
    .id_table = novatek_id_match,
};

module_i2c_driver(novatek_ts_driver);

MODULE_LICENSE("GPL");