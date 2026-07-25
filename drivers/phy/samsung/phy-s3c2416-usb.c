#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/phy/phy.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/pm.h>
#include <linux/pm_wakeirq.h>
#include <linux/usb/role.h>
#include <linux/pinctrl/pinconf-generic.h>


#define S3C2443_PWRCFG      0x00 /* Offset from 0x4c000060 */
#define S3C2443_PHYCTRL     0x20
#define S3C2443_PHYPWR      0x24
#define S3C2443_URSTCON     0x28
#define S3C2443_UCLKCON     0x2C

/* Register bits */
#define PWRCFG_USBPHY       BIT(4)
#define MISCCR_SEL_SUSPND   BIT(12)
#define HSUDC_RESET         (BIT(2) | BIT(0))

/* PHYCTRL Bits */
#define PHYCTRL_DOWNSTREAM_PORT  BIT(0)
#define PHYCTRL_EXT_CLK_OSC      BIT(2)
#define PHYCTRL_CLK_SEL_12M      (0x2 << 3)
#define PHYCTRL_CLK_SEL_24M      (0x3 << 3)
#define PHYCTRL_CLK_SEL_48M      (0x0 << 3)

struct s3c2416_usb_phy {
    struct device *dev;
    void __iomem *base;
    struct regmap *misc_regmap;
    
    struct gpio_desc *usbd_phy_en;
    struct gpio_desc *id;
    struct gpio_desc *vbus_det;
    struct gpio_desc *vbus_boost;
    struct gpio_desc *vbus_5v;

    u32 refclk_freq;
    bool is_osc;
    bool is_host_mode;
    int id_irq;
    int vbus_irq;
    bool id_wake_enabled;   
    bool vbus_wake_enabled; 
    #if IS_ENABLED(CONFIG_USB_ROLE_SWITCH)
    struct usb_role_switch *role_sw;
    enum usb_role forced_role;
    #endif
};

static void s3c2416_phy_set_role(struct s3c2416_usb_phy *phy)
{
    u32 phyctrl = readl(phy->base + S3C2443_PHYCTRL);
    u32 uclkcon = readl(phy->base + S3C2443_UCLKCON);
    int id_val = gpiod_get_value_cansleep(phy->id);
    int vbus_val = gpiod_get_value_cansleep(phy->vbus_det);
    bool is_host;

#if IS_ENABLED(CONFIG_USB_ROLE_SWITCH)
    /* If manually forced from sysfs, override the hardware pins */
    if (phy->forced_role == USB_ROLE_HOST)
        is_host = true;
    else if (phy->forced_role == USB_ROLE_DEVICE)
        is_host = false;
    else
        is_host = (id_val == 0); /* Auto mode */
#else
    is_host = (id_val == 0);
#endif

    if (is_host) {
        /* ID grounded = Host Mode */
        phy->is_host_mode = true;
        if (phy->usbd_phy_en)
            gpiod_set_value_cansleep(phy->usbd_phy_en, 0);
        phyctrl |= PHYCTRL_DOWNSTREAM_PORT; 
        uclkcon &= ~BIT(31); /* Disable VBUS detect pullup */

        /* Enable 5V Boost & Output */
        if (phy->vbus_boost) gpiod_set_value_cansleep(phy->vbus_boost, 1);
        if (phy->vbus_5v) gpiod_set_value_cansleep(phy->vbus_5v, 1);
        
        dev_info(phy->dev, "Switched to HOST mode\n");
    } else {
        /* ID floating = Device Mode */
        phy->is_host_mode = false;
        if (phy->usbd_phy_en)
            gpiod_set_value_cansleep(phy->usbd_phy_en, 1);
        phyctrl &= ~PHYCTRL_DOWNSTREAM_PORT;
        
        if (vbus_val || (IS_ENABLED(CONFIG_USB_ROLE_SWITCH) && phy->forced_role == USB_ROLE_DEVICE)) 
            uclkcon |= BIT(31);
        else 
            uclkcon &= ~BIT(31);

        /* Disable 5V Boost & Output */
        if (phy->vbus_boost) gpiod_set_value_cansleep(phy->vbus_boost, 0);
        if (phy->vbus_5v) gpiod_set_value_cansleep(phy->vbus_5v, 0);
        
        dev_info(phy->dev, "Switched to DEVICE mode (VBUS %s)\n", vbus_val ? "connected" : "disconnected");
    }

    writel(phyctrl, phy->base + S3C2443_PHYCTRL);
    writel(uclkcon, phy->base + S3C2443_UCLKCON);
}

static irqreturn_t s3c2416_usb_phy_irq(int irq, void *data)
{
    struct s3c2416_usb_phy *phy = data;
    s3c2416_phy_set_role(phy);
    return IRQ_HANDLED;
}

static int s3c2416_usb_phy_power_on(struct phy *p)
{
    struct s3c2416_usb_phy *phy = phy_get_drvdata(p);
    u32 cfg, phyctrl;

    /* 1. Clear Suspend bit in MISCCR */
    regmap_update_bits(phy->misc_regmap, 0x80, MISCCR_SEL_SUSPND, 0);

    /* 2. Enable GPH14 PHY enable pin */
    if (phy->usbd_phy_en)
        gpiod_set_value_cansleep(phy->usbd_phy_en, 1);

    /* 3. Turn on USBPHY bit in PWRCFG */
    cfg = readl(phy->base + S3C2443_PWRCFG);
    writel(cfg | PWRCFG_USBPHY, phy->base + S3C2443_PWRCFG);
    udelay(5);

    /* 4. Configure PHYCTRL (Clock source and Freq) */
    phyctrl = 0;
    if (phy->is_osc) phyctrl |= PHYCTRL_EXT_CLK_OSC;
    
    if (phy->refclk_freq == 12000000) phyctrl |= PHYCTRL_CLK_SEL_12M;
    else if (phy->refclk_freq == 24000000) phyctrl |= PHYCTRL_CLK_SEL_24M;
    else phyctrl |= PHYCTRL_CLK_SEL_48M; /* 48M is 0x00 */
    
    writel(phyctrl, phy->base + S3C2443_PHYCTRL);

    /* 5. Magic PHYPWR value */
    writel(0x80000024, phy->base + S3C2443_PHYPWR);

    /* 6. Enable Clocks in UCLKCON */
    cfg = readl(phy->base + S3C2443_UCLKCON);
    cfg |= BIT(2) | BIT(1); /* FUNC_CLK_EN and HOST_CLK_EN */
    writel(cfg, phy->base + S3C2443_UCLKCON);
    udelay(5);

    /* 7. Reset Sequence */
    cfg = readl(phy->base + S3C2443_URSTCON);
    writel(cfg | HSUDC_RESET, phy->base + S3C2443_URSTCON);
    mdelay(1);
    writel(cfg & ~HSUDC_RESET, phy->base + S3C2443_URSTCON);
    udelay(5);

    /* Check role immediately on power up */
    s3c2416_phy_set_role(phy);

    return 0;
}

static int s3c2416_usb_phy_power_off(struct phy *p)
{
    struct s3c2416_usb_phy *phy = phy_get_drvdata(p);
    u32 cfg;

    /* Turn off USBPHY bit in PWRCFG */
    cfg = readl(phy->base + S3C2443_PWRCFG);
    writel(cfg & ~PWRCFG_USBPHY, phy->base + S3C2443_PWRCFG);

    /* Disable PHY enable pin */
    if (phy->usbd_phy_en)
        gpiod_set_value_cansleep(phy->usbd_phy_en, 0);

    /* Set Suspend bit in MISCCR */
    regmap_update_bits(phy->misc_regmap, 0x80, MISCCR_SEL_SUSPND, MISCCR_SEL_SUSPND);

    return 0;
}

static const struct phy_ops s3c2416_usb_phy_ops = {
    .power_on   = s3c2416_usb_phy_power_on,
    .power_off  = s3c2416_usb_phy_power_off,
    .owner      = THIS_MODULE,
};
#ifdef CONFIG_PM_SLEEP
static int s3c2416_usb_phy_suspend(struct device *dev)
{
    struct s3c2416_usb_phy *phy = dev_get_drvdata(dev);

    if (device_may_wakeup(dev)) {
        /* Only mark as enabled if enable_irq_wake returns 0 (success) */
        if (phy->id_irq && enable_irq_wake(phy->id_irq) == 0)
            phy->id_wake_enabled = true;
            
        if (phy->vbus_irq && enable_irq_wake(phy->vbus_irq) == 0)
            phy->vbus_wake_enabled = true;
    }

    return 0;
}

static int s3c2416_usb_phy_resume(struct device *dev)
{
    struct s3c2416_usb_phy *phy = dev_get_drvdata(dev);

    if (device_may_wakeup(dev)) {
        /* Only disable if we successfully enabled it earlier */
        if (phy->id_wake_enabled) {
            disable_irq_wake(phy->id_irq);
            phy->id_wake_enabled = false;
        }
        if (phy->vbus_wake_enabled) {
            disable_irq_wake(phy->vbus_irq);
            phy->vbus_wake_enabled = false;
        }
    }

    /* Force a role check just in case the cable changed while asleep */
    s3c2416_phy_set_role(phy);

    return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(s3c2416_usb_phy_pm_ops, 
                         s3c2416_usb_phy_suspend, 
                         s3c2416_usb_phy_resume);
#if IS_ENABLED(CONFIG_USB_ROLE_SWITCH)
static int s3c2416_usb_role_set(struct usb_role_switch *sw, enum usb_role role)
{
    struct s3c2416_usb_phy *phy = usb_role_switch_get_drvdata(sw);

    phy->forced_role = role;
    if (phy->id) {
        if (role == USB_ROLE_HOST) {
            unsigned long config = pinconf_to_config_packed(PIN_CONFIG_BIAS_PULL_DOWN, 0);
            gpiod_set_config(phy->id, config);
        } else {
            unsigned long config = pinconf_to_config_packed(PIN_CONFIG_BIAS_PULL_UP, 0);
            gpiod_set_config(phy->id, config);
        }
    }
    s3c2416_phy_set_role(phy);

    return 0;
}

static enum usb_role s3c2416_usb_role_get(struct usb_role_switch *sw)
{
    struct s3c2416_usb_phy *phy = usb_role_switch_get_drvdata(sw);

    if (phy->forced_role != USB_ROLE_NONE)
        return phy->forced_role;

    return phy->is_host_mode ? USB_ROLE_HOST : USB_ROLE_DEVICE;
}
#endif
static int s3c2416_usb_phy_probe(struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    struct s3c2416_usb_phy *phy;
    struct phy *generic_phy;
    struct phy_provider *phy_provider;
    int id_irq, vbus_irq;

    phy = devm_kzalloc(dev, sizeof(*phy), GFP_KERNEL);
    if (!phy) return -ENOMEM;
    phy->dev = dev;

    platform_set_drvdata(pdev, phy);

    phy->base = devm_platform_ioremap_resource(pdev, 0);
    if (IS_ERR(phy->base)) return PTR_ERR(phy->base);

    phy->misc_regmap = syscon_regmap_lookup_by_phandle(dev->of_node, "samsung,syscon-misc");
    if (IS_ERR(phy->misc_regmap)) {
        dev_err(dev, "Failed to find misc syscon\n");
        return PTR_ERR(phy->misc_regmap);
    }

    /* DT Configuration */
    of_property_read_u32(dev->of_node, "samsung,refclk-freq", &phy->refclk_freq);
    phy->is_osc = of_property_read_bool(dev->of_node, "samsung,refclk-osc");

    /* Acquire GPIOs */
    phy->usbd_phy_en = devm_gpiod_get_optional(dev, "usbd-phy-enable", GPIOD_OUT_LOW);
    phy->id = devm_gpiod_get_optional(dev, "id", GPIOD_IN);
    phy->vbus_det = devm_gpiod_get_optional(dev, "vbus-det", GPIOD_IN);
    phy->vbus_boost = devm_gpiod_get_optional(dev, "vbus-boost", GPIOD_OUT_LOW);
    phy->vbus_5v = devm_gpiod_get_optional(dev, "vbus-5v", GPIOD_OUT_LOW);

    /* Setup Interrupts for ID and VBUS */
    if (phy->id) {
        phy->id_irq = gpiod_to_irq(phy->id);
        devm_request_threaded_irq(dev, phy->id_irq, NULL, s3c2416_usb_phy_irq, 
                                  IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_ONESHOT, 
                                  "usb-id", phy);
        /* Allow this IRQ to wake the system */
        device_init_wakeup(dev, true);
    }
    
    if (phy->vbus_det) {
        phy->vbus_irq = gpiod_to_irq(phy->vbus_det);
        devm_request_threaded_irq(dev, phy->vbus_irq, NULL, s3c2416_usb_phy_irq, 
                                  IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_ONESHOT, 
                                  "usb-vbus", phy);
        device_init_wakeup(dev, true);
    }
#if IS_ENABLED(CONFIG_USB_ROLE_SWITCH)
    {
        struct usb_role_switch_desc role_desc = {
            .name = dev_name(dev),
            .fwnode = dev_fwnode(dev),
            .set = s3c2416_usb_role_set,
            .get = s3c2416_usb_role_get,
            .allow_userspace_control = true,
            .driver_data = phy,
        };

        phy->forced_role = USB_ROLE_NONE;
        phy->role_sw = usb_role_switch_register(dev, &role_desc);
        if (IS_ERR(phy->role_sw)) {
            dev_warn(dev, "Failed to register USB role switch\n");
            phy->role_sw = NULL;
        } else {
            /* Note: We use devm_add_action_or_reset so it cleans up automatically */
            devm_add_action_or_reset(dev, (void (*)(void *))usb_role_switch_unregister, phy->role_sw);
        }
    }
#endif
    /* Register Generic PHY */
    generic_phy = devm_phy_create(dev, dev->of_node, &s3c2416_usb_phy_ops);
    if (IS_ERR(generic_phy)) return PTR_ERR(generic_phy);
    
    phy_set_drvdata(generic_phy, phy);

    phy_provider = devm_of_phy_provider_register(dev, of_phy_simple_xlate);
    return PTR_ERR_OR_ZERO(phy_provider);
}

static const struct of_device_id s3c2416_usb_phy_of_match[] = {
    { .compatible = "samsung,s3c2416-usb-phy" },
    { },
};
MODULE_DEVICE_TABLE(of, s3c2416_usb_phy_of_match);

static struct platform_driver s3c2416_usb_phy_driver = {
    .probe = s3c2416_usb_phy_probe,
    .driver = {
        .name = "s3c2416-usb-phy",
        .of_match_table = s3c2416_usb_phy_of_match,
        .pm = &s3c2416_usb_phy_pm_ops, 
    },
};
module_platform_driver(s3c2416_usb_phy_driver);

MODULE_AUTHOR("Custom");
MODULE_DESCRIPTION("Samsung S3C2416/2443 USB PHY Driver");
MODULE_LICENSE("GPL v2");