/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (c) 2004 Simtec Electronics
 *	Ben Dooks <ben@simtec.co.uk>
 *
 * S3C2410 Power Manager (Suspend-To-RAM) support
 *
 * Based on PXA/SA1100 sleep code by:
 *	Nicolas Pitre, (c) 2002 Monta Vista Software Inc
 *	Cliff Brake, (c) 2001
 */

#include <linux/linkage.h>
#include <linux/serial_s3c.h>
#include <asm/assembler.h>
#include "map.h"

#include "regs-gpio.h"
#include "regs-clock.h"

/*
 * S3C24XX_DEBUG_RESUME is dangerous if your bootloader does not
 * reset the UART configuration, only enable if you really need this!
 */
//#define S3C24XX_DEBUG_RESUME

	.text

	/* sleep magic, to allow the bootloader to check for an valid
	 * image to resume to. Must be the first word before the
	 * s3c_cpu_resume entry.
	*/

	.word	0x2bedf00d

	/* s3c_cpu_resume
	 *
	 * resume code entry for bootloader to call
	*/

ENTRY(s3c_cpu_resume)
	mov	r0, #PSR_I_BIT | PSR_F_BIT | SVC_MODE
	msr	cpsr_c, r0

	@@ load UART to allow us to print the two characters for
	@@ resume debug

	mov	r2, #S3C24XX_PA_UART & 0xff000000
	orr	r2, r2, #S3C24XX_PA_UART & 0xff000

#if 0
	/* Set GPC5 as output and output high */
	@mov	r14, #S3C24XX_PA_GPIO
	mov	r14, 0x56000000
	/* Configure GPC5 as output (bits [11:10] = 01) */
	ldr	r12, [ r14, #0x20 ]      /* GPCCON address 0x56000020 */
	bic	r12, r12, #3<<10          /* Clear bits [11:10] for GPC5 */
	orr	r12, r12, #1<<10           /* Set bits [11:10] = 01 (output) */
	str	r12, [ r14, #0x20 ]        /* Write back to GPCCON */
	
	/* Set GPC5 output high */
	ldr	r12, [ r14, #0x24 ]        /* GPCDAT address 0x56000024 */
	orr	r12, r12, #1<<5             /* Set bit 5 to 1 (output high) */
	str	r12, [ r14, #0x24 ]        /* Write back to GPCDAT */
#endif

#ifdef S3C24XX_DEBUG_RESUME
	mov	r3, #'L'
	strb	r3, [ r2, #S3C2410_UTXH ]
1001:
	ldrb	r14, [ r2, #S3C2410_UTRSTAT ]
	tst	r14, #S3C2410_UTRSTAT_TXE
	beq	1001b
#endif /* S3C24XX_DEBUG_RESUME */

	b	cpu_resume
