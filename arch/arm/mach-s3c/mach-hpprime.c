// SPDX-License-Identifier: GPL-2.0
// 

#include "irqs-s3c24xx.h"
#include "linux/input-event-codes.h"
#include "linux/mtd/nand.h"
#include "linux/printk.h"
#include "linux/sizes.h"
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/timer.h>
#include <linux/init.h>
#include <linux/serial_core.h>
#include <linux/serial_s3c.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/mtd/partitions.h>
#include <linux/gpio.h>
#include <linux/fb.h>
#include <linux/delay.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>
#include <linux/irqchip.h>
#include <video/samsung_fimd.h>
#include <asm/irq.h>
#include <asm/mach-types.h>

#include "hardware-s3c24xx.h"
#include "regs-gpio.h"
#include "regs-s3c2443-clock.h"
#include "gpio-samsung.h"

#include <linux/platform_data/leds-s3c24xx.h>
#include <linux/platform_data/i2c-s3c2410.h>

#include "gpio-cfg.h"
#include "devs.h"
#include "cpu.h"
#include <linux/platform_data/mtd-nand-s3c2410.h>
#include "sdhci.h"
#include <linux/platform_data/usb-s3c2410_udc.h>
#include <linux/platform_data/s3c-hsudc.h>
#include <linux/platform_data/simplefb.h>

#include "fb.h"

#include "gpio-cfg.h"
#include "devs.h"
#include "pm.h"

#include "s3c24xx.h"
#include "common-smdk-s3c24xx.h"
 
#include <linux/input.h>
#include <linux/input/goodix.h>
#include <linux/input/matrix_keypad.h>
#include <linux/gpio.h>
#include <linux/gpio_keys.h>

static struct map_desc smdk2416_iodesc[] __initdata = {
	/* ISA IO Space map (memory space selected by A24) */

	{
		.virtual	= (u32)S3C24XX_VA_ISA_BYTE,
		.pfn		= __phys_to_pfn(S3C2410_CS2),
		.length		= 0x10000,
		.type		= MT_DEVICE,
	}, 
	{
		.virtual	= (u32)S3C24XX_VA_ISA_BYTE + 0x10000,
		.pfn		= __phys_to_pfn(S3C2410_CS2 + (1<<24)),
		.length		= SZ_4M,
		.type		= MT_DEVICE,
	}, 
};

#define UCON (S3C2410_UCON_DEFAULT	| \
		S3C2440_UCON_PCLK	| \
		S3C2443_UCON_RXERR_IRQEN)

#define ULCON (S3C2410_LCON_CS8 | S3C2410_LCON_PNONE)

#define UFCON (S3C2410_UFCON_RXTRIG8	| \
		S3C2410_UFCON_FIFOMODE	| \
		S3C2440_UFCON_TXTRIG16)

static struct s3c2410_uartcfg smdk2416_uartcfgs[] __initdata = {
	[0] = {
		.hwport	     = 0,
		.flags	     = 0,
		.ucon	     = UCON,
		.ulcon	     = ULCON,
		.ufcon	     = UFCON,
	}
};
/*
static void smdk2416_hsudc_gpio_init(void)
{
	printk("<smdk2416_hsudc_gpio_init>\n"); 
	
	s3c_gpio_setpull(S3C2410_GPH(14), S3C_GPIO_PULL_UP); 
	s3c_gpio_cfgpin(S3C2410_GPH(14), S3C_GPIO_SFN(1)); 
	
	gpio_direction_output(S3C2410_GPH(14), 1);
	s3c2410_modify_misccr(S3C2416_MISCCR_SEL_SUSPND, 0);
}

static void smdk2416_hsudc_gpio_uninit(void)
{
	//printk("<smdk2416_hsudc_gpio_uninit>\n");

	s3c2410_modify_misccr(S3C2416_MISCCR_SEL_SUSPND, 1);
	//gpio_direction_output(S3C2410_GPH(14), 0);
	//gpio_direction_input(S3C2410_GPH(14));
	s3c_gpio_setpull(S3C2410_GPH(14), S3C_GPIO_PULL_NONE);
	s3c_gpio_cfgpin(S3C2410_GPH(14), S3C_GPIO_SFN(0)); 

	printk("<smdk2416_hsudc_gpio_uninit>\n");
}*/
// S3C2416 GPIO 物理基地址 (参考手册)


static void smdk2416_hsudc_gpio_init(void)
{

    //printk("<smdk2416_hsudc_gpio_init> \n"); 
	writel(readl(S3C2410_GPHDAT)|(1L<<14),S3C2410_GPHDAT);
	s3c2410_modify_misccr(S3C2416_MISCCR_SEL_SUSPND, 0);
}

static void smdk2416_hsudc_gpio_uninit(void)
{
	writel(readl(S3C2410_GPHDAT)&~(1L<<14),S3C2410_GPHDAT);
	s3c2410_modify_misccr(S3C2416_MISCCR_SEL_SUSPND, 1);
    // 同样建议暂时注释掉旧 API，防止卸载驱动时崩溃
    // 或者实现类似的直接寄存器操作将 GPH14 拉低
   // printk("<smdk2416_hsudc_gpio_uninit>\n");
}

static struct s3c24xx_hsudc_platdata smdk2416_hsudc_platdata = {
	.epnum = 9,
	.gpio_init = smdk2416_hsudc_gpio_init,
	.gpio_uninit = smdk2416_hsudc_gpio_uninit,
};
/*
static struct s3c_fb_pd_win smdk2416_fb_win[] = {
	[0] = {
		.default_bpp	= 16,
		.max_bpp	= 32,
		.xres           = 320,
		.yres           = 240,
	},
};
 

static struct fb_videomode smdk2416_lcd_timing = {
	//.pixclock = 4,
	.refresh = 60,
	
	.left_margin = 64+1, //VIDTCON1_HBPD-1     0x401100
	.right_margin = 17+1, //VIDTCON1_HFPD-1   0x401100
	.hsync_len = 0+1, //VIDTCON1_HSPW-1
	
	.upper_margin = 17+1, //VIDTCON0_VBPD-1    0x110300
	.lower_margin = 3+1, //VIDTCON0_VFPD-1    0x110300
	.vsync_len = 0+1, //VIDTCON0_VSPW-1
	
	.xres = 320,
	.yres = 240,
};


static void s3c2416_fb_gpio_setup_24bpp(void)
{ 
 	//unsigned int gpio;
//
	//for (gpio = S3C2410_GPC(1); gpio <= S3C2410_GPC(4); gpio++) {
	//	s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(2));
	//	s3c_gpio_setpull(gpio, S3C_GPIO_PULL_NONE);
	//}
//
	//for (gpio = S3C2410_GPC(8); gpio <= S3C2410_GPC(15); gpio++) {
	//	s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(2));
	//	s3c_gpio_setpull(gpio, S3C_GPIO_PULL_NONE);
	//}
//
	//for (gpio = S3C2410_GPD(8); gpio <= S3C2410_GPD(15); gpio++) {
	//	s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(2));
	//	s3c_gpio_setpull(gpio, S3C_GPIO_PULL_NONE);
	//}
}

static struct s3c_fb_platdata smdk2416_fb_platdata = {
	.win[0]		= &smdk2416_fb_win[0],
	.vtiming	= &smdk2416_lcd_timing,
	.setup_gpio	= s3c2416_fb_gpio_setup_24bpp,
	.vidcon0	= VIDCON0_VIDOUT_RGB | VIDCON0_PNRMODE_SERIAL_RGB | (1 << 5),
};*/
 
/*
static struct mtd_partition smdk_default_nand_part[] = {
	[0] = {
		.name	= "HP Boot Code",
		.offset	= 0,
		.size	= SZ_256K,
		.mask_flags = MTD_WRITEABLE,
	},
	[1] = {
		.name	= "Linux Bootloader",
		.offset	= SZ_256K,
		.size	= SZ_512K,
	},
	[2] = {
		.name	= "kernel",
		.offset	= SZ_1M,
		.size	= SZ_8M,
	},
	[3] = {
		.name	= "rootfs",
		.offset = SZ_1M + SZ_8M,
		.size	= (54) * SZ_1M,
	},
	[4] = {
		.name	= "data",
		.offset = MTDPART_OFS_APPEND,
		.size	= 112 * SZ_1M,
	},
	[5] = {
		.name	= "swap",
		.offset = MTDPART_OFS_APPEND,
		.size	= SZ_64M,
	},
	[6] = {
		.name	= "reserved",
		.offset = SZ_256M - SZ_32K,
		.size	= SZ_32K,
		.mask_flags = MTD_WRITEABLE,
	}
};


static struct s3c2410_nand_set smdk_nand_sets[] = {
	[0] = {
		.name		= "NAND",
		.nr_chips	= 1,
		.nr_partitions	= ARRAY_SIZE(smdk_default_nand_part),
		.partitions	= smdk_default_nand_part,
	},
};

static struct s3c2410_platform_nand smdk_nand_info = {
	.tacls		= 20,
	.twrph0		= 30,
	.twrph1		= 20,
	.nr_sets	= ARRAY_SIZE(smdk_nand_sets),
	.sets		= smdk_nand_sets,
	.engine_type	= NAND_ECC_ENGINE_TYPE_ON_HOST,
};
*/
 
//static uint8_t fb_ram[ 320*240*4 ]  __attribute__ ((aligned (PAGE_SIZE)));
//#define fb_ram ((u32)S3C24XX_VA_ISA_BYTE + 0x10000 + SZ_4M)
//#define fb_ram (0x30000000 + SZ_32M - SZ_512K)

/* simple-framebuffer */
/*static struct resource simplefb_resources[] __initdata = { 
	//DEFINE_RES_MEM((uint32_t)&fb_ram[0], 320*240*2), 
	DEFINE_RES_MEM(fb_ram, SZ_512K), 

};
static struct simplefb_platform_data  simplefb_pdata __initdata = {
	.width = 320,
	.height = 240,
	.stride = 320 * 4, //bytes_per_line
	.format = "x8r8g8b8",
};

static struct platform_device_info simplefb_info __initdata = {
	.parent		= &platform_bus,
	.name		= "simple-framebuffer",
	.id		= -1,
	.res		= simplefb_resources,
	.num_res	= ARRAY_SIZE(simplefb_resources),
	.data		= &simplefb_pdata,
	.size_data	= sizeof(simplefb_pdata),
};*/

static struct platform_device *smdk2416_devices[] __initdata = {
	//&s3c_device_fb,
	//&s3c_device_nand,
	//&s3c_device_wdt,
	//&hp_keyboard,
	//&s3c_device_ohci,
	//&s3c_device_i2c0, 
	//&s3c_device_usb_hsudc,
	&s3c2443_device_dma,
};

 struct goodix_ts_platform_data ts_dat =
 {
	.pin_int = S3C2410_GPF(2),
	.pin_rst = S3C2410_GPF(0),
	.multitouch = 3,
 };

static struct i2c_board_info i2c_devs0[] __initdata = {
	{ 
			I2C_BOARD_INFO("GDIX1001:00", 0x5D),
			.irq = IRQ_EINT2,
			.platform_data = &ts_dat
	},
};

static void __init smdk2416_init_time(void)
{
	s3c2416_init_clocks(12000000);
	s3c24xx_timer_init();
}

static void __init smdk2416_map_io(void)
{
s3c24xx_init_io(NULL, 0);
//	s3c24xx_init_io(smdk2416_iodesc, ARRAY_SIZE(smdk2416_iodesc));
//	s3c24xx_init_uarts(smdk2416_uartcfgs, ARRAY_SIZE(smdk2416_uartcfgs));
//	s3c24xx_set_timer_source(S3C24XX_PWM3, S3C24XX_PWM4);
}

static void powercut(void)
{
	u32 cfg;

	cfg = readl(S3C2443_PWRCFG) & ~S3C2443_PWRCFG_USBPHY;
	writel(cfg, S3C2443_PWRCFG);

	writel(S3C2443_PHYPWR_FSUSPEND, S3C2443_PHYPWR);

	cfg = readl(S3C2443_UCLKCON) & ~S3C2443_UCLKCON_FUNC_CLKEN;
	writel(cfg, S3C2443_UCLKCON);
	//smdk2416_hsudc_gpio_uninit();
}
static irqreturn_t
prime_wake_interrupt(int irq, void *ignored)
{
	return IRQ_HANDLED;
}

static void prime_init_pm(void)
{
	int ret = 0;

	ret = request_irq(IRQ_EINT8, &prime_wake_interrupt,
				IRQF_TRIGGER_FALLING | IRQF_SHARED,
				"HPprime_wakeup", &prime_wake_interrupt);
	if (ret != 0) {
		printk(KERN_ERR "HPPrime: no wakeup irq, %d?\n", ret);
	} else {
		enable_irq_wake(IRQ_EINT8);
		/* configure the suspend/resume status pin */
		/*s3c_gpio_cfgpin(S3C2410_GPF(2), S3C2410_GPIO_OUTPUT);
		s3c_gpio_setpull(S3C2410_GPF(2), S3C_GPIO_PULL_UP);*/
	}
}
static void __init smdk2416_machine_init(void)
{
	//s3c_i2c0_set_platdata(NULL);

	struct device_node *dma_np;
    dma_np = of_find_node_by_path("/dma-controller@4b000000"); // 或者用 compatible 查找
    if (dma_np) {
        s3c2443_device_dma.dev.of_node = dma_np;
    } else {
        pr_err("Failed to find DMA node in Device Tree\n");
    }//TODO:We must port it full!!!!!

	//s3c_fb_set_platdata(&smdk2416_fb_platdata);
	
	
	//s3c_nand_set_platdata(&smdk_nand_info);

	//s3c_sdhci0_set_platdata(&smdk2416_hsmmc0_pdata);
	//s3c_sdhci1_set_platdata(&smdk2416_hsmmc1_pdata);

	/*s3c24xx_hsudc_set_platdata(&smdk2416_hsudc_platdata);
	struct device_node *hsudc_np;
     hsudc_np = of_find_node_by_path("/hsudc@49800000");
    if (hsudc_np) {
        printk(KERN_INFO "S3C2416: Found HSUDC DT node, binding context...\n");
        s3c_device_usb_hsudc.dev.of_node = hsudc_np;
    } else {
        printk(KERN_ERR "S3C2416: Failed to find HSUDC DT node!\n");
    }*/
	
 
	  
	//i2c_register_board_info(0, i2c_devs0, ARRAY_SIZE(i2c_devs0));

	 platform_add_devices(smdk2416_devices, ARRAY_SIZE(smdk2416_devices));
	//platform_device_register_full(&simplefb_info);
	
	//pm_power_off = powercut;
	
	s3c_pm_init(); 
	//prime_init_pm();
}
static const char *const s3c2416_dt_compat[] __initconst = {
	"samsung,s3c2416",
	//"samsung,HPPrime",
	NULL
};
DT_MACHINE_START(HPPRIMEV2, "HPPRIMEV2")
	/* Maintainer: Repeerc <repeerc@qq.com> */
	.atag_offset	= 0x100,
	.nr_irqs	= NR_IRQS_S3C2416,
	.dt_compat	= s3c2416_dt_compat,
	//.init_irq	= s3c2416_init_irq,
	.init_irq	= irqchip_init,
	.map_io		= smdk2416_map_io,
	.init_machine	= smdk2416_machine_init,
	//.init_time	= smdk2416_init_time,
MACHINE_END
