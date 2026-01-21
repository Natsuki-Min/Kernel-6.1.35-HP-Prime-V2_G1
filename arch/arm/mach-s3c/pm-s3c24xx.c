// SPDX-License-Identifier: GPL-2.0+
//
// Copyright (c) 2004-2006 Simtec Electronics
//	Ben Dooks <ben@simtec.co.uk>
//
// S3C24XX Power Manager (Suspend-To-RAM) support
//
// See Documentation/arm/samsung-s3c24xx/suspend.rst for more information
//
// Parts based on arch/arm/mach-pxa/pm.c
//
// Thanks to Dimitry Andric for debugging

#include <linux/init.h>
#include <linux/suspend.h>
#include <linux/errno.h>
#include <linux/time.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/serial_core.h>
#include <linux/serial_s3c.h>
#include <linux/io.h>

#include "regs-clock.h"
#include "regs-gpio.h"
#include "regs-irq.h"
#include "gpio-samsung.h"

#include <asm/mach/time.h>

#include "gpio-cfg.h"
#include "pm.h"

#include "regs-mem-s3c24xx.h"

#define PFX "s3c24xx-pm: "

#ifdef CONFIG_PM_SLEEP
static struct sleep_save core_save[] = {
	/* we restore the timings here, with the proviso that the board
	 * brings the system up in an slower, or equal frequency setting
	 * to the original system.
	 *
	 * if we cannot guarantee this, then things are going to go very
	 * wrong here, as we modify the refresh and both pll settings.
	 */

	SAVE_ITEM(S3C2410_BWSCON),
	SAVE_ITEM(S3C2410_BANKCON0),
	SAVE_ITEM(S3C2410_BANKCON1),
	SAVE_ITEM(S3C2410_BANKCON2),
	SAVE_ITEM(S3C2410_BANKCON3),
	SAVE_ITEM(S3C2410_BANKCON4),
	SAVE_ITEM(S3C2410_BANKCON5),
};
#endif

/* s3c_pm_check_resume_pin
 *
 * check to see if the pin is configured correctly for sleep mode, and
 * make any necessary adjustments if it is not
*/

static void s3c_pm_check_resume_pin(unsigned int pin, unsigned int irqoffs)
{
	unsigned long irqstate;
	unsigned long pinstate;
	int irq = gpio_to_irq(pin);

	if (irqoffs < 4)
		irqstate = s3c_irqwake_intmask & (1L<<irqoffs);
	else
		irqstate = s3c_irqwake_eintmask & (1L<<irqoffs);

	pinstate = s3c_gpio_getcfg(pin);

	if (!irqstate) {
		if (pinstate == S3C2410_GPIO_IRQ)
			S3C_PMDBG("Leaving IRQ %d (pin %d) as is\n", irq, pin);
	} else {
		if (pinstate == S3C2410_GPIO_IRQ) {
			S3C_PMDBG("Disabling IRQ %d (pin %d)\n", irq, pin);
			s3c_gpio_cfgpin(pin, S3C2410_GPIO_INPUT);
		}
	}
}

/* s3c_pm_configure_extint
 *
 * configure all external interrupt pins
*/

void s3c_pm_configure_extint(void)
{
	int pin;

	/* 
	 * Device Tree 模式下的寄存器修正 
	 * 目的: 将配置为 EINT 但未开启唤醒的引脚设为 Input，防止漏电
	 * 保护: 绝对不修改配置为 Output 的引脚
	 */
	if (of_have_populated_dt()) {
		void __iomem *gpio_base;
		unsigned long con, con_new;
		unsigned long mask;
		unsigned int cfg;
		int i;

		/* 映射 GPIO 基地址 (S3C24XX 默认为 0x56000000) */
		gpio_base = ioremap(0x56000000, 0x100);
		if (!gpio_base) {
			pr_err(PFX "Failed to ioremap GPIO base for PM\n");
			return;
		}

		/* ==============================================
		 * 处理 GPF (EINT0 - EINT7)
		 * 寄存器: 0x50
		 * ============================================== */
		con = __raw_readl(gpio_base + 0x50);
		con_new = con;

		for (i = 0; i <= 7; i++) {
			/* 读取当前引脚的2位配置: 00=In, 01=Out, 10=EINT */
			cfg = (con >> (i * 2)) & 0x3;

			/* 重点: 只处理当前是 EINT (0x2) 的引脚，忽略 Output (0x1) */
			if (cfg == 0x2) {
				/* 
				 * 获取唤醒掩码位 
				 * S3C24XX 中，Mask 为 0 表示使能唤醒，1 表示屏蔽(不唤醒)
				 */
				if (i < 4)
					mask = s3c_irqwake_intmask & (1L << i); /* EINT0-3 在 intmask */
				else
					mask = s3c_irqwake_eintmask & (1L << i); /* EINT4-7 在 eintmask */

				/* 如果 Mask 存在(非0)，说明不需要唤醒 -> 强制改为 Input */
				if (mask) {
					con_new &= ~(3 << (i * 2));
				}
			}
		}
		/* 只有变化了才写入 */
		if (con != con_new)
			__raw_writel(con_new, gpio_base + 0x50);

		/* ==============================================
		 * 处理 GPG (EINT8 - EINT15...)
		 * 寄存器: 0x60
		 * ============================================== */
		con = __raw_readl(gpio_base + 0x60);
		con_new = con;

		for (i = 0; i <= 7; i++) {
			cfg = (con >> (i * 2)) & 0x3;

			/* 只处理 EINT */
			if (cfg == 0x2) {
				/* GPG0 对应 EINT8，所以位偏移是 i + 8 */
				mask = s3c_irqwake_eintmask & (1L << (i + 8));

				/* 如果被 Mask (不唤醒) -> 改为 Input */
				if (mask) {
					con_new &= ~(3 << (i * 2));
				}
			}
		}
		if (con != con_new)
			__raw_writel(con_new, gpio_base + 0x60);

		iounmap(gpio_base);
		return;
	}

	/* Legacy Non-DT 模式保持原样 */
	for (pin = S3C2410_GPF(0); pin <= S3C2410_GPF(7); pin++) {
		s3c_pm_check_resume_pin(pin, pin - S3C2410_GPF(0));
	}

	for (pin = S3C2410_GPG(0); pin <= S3C2410_GPG(7); pin++) {
		s3c_pm_check_resume_pin(pin, (pin - S3C2410_GPG(0))+8);
	}
}

#ifdef CONFIG_PM_SLEEP
void s3c_pm_restore_core(void)
{
	s3c_pm_do_restore_core(core_save, ARRAY_SIZE(core_save));
}

void s3c_pm_save_core(void)
{
	s3c_pm_do_save(core_save, ARRAY_SIZE(core_save));
}
#endif
