/*
 * linux/arch/mips/jz4740/board-pavo.c
 *
 * JZ4740 PAVO board setup routines.
 *
 * Copyright (c) 2006-2007  Ingenic Semiconductor Inc.
 * Author: <lhhuang@ingenic.cn>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/sched.h>
#include <linux/ioport.h>
#include <linux/mm.h>
#include <linux/console.h>
#include <linux/delay.h>

#include <asm/cpu.h>
#include <asm/bootinfo.h>
#include <asm/mipsregs.h>
#include <asm/reboot.h>

#include <asm/jzsoc.h>

extern void (*jz_timer_callback)(void);

static void dancing(void)
{
	static unsigned int count = 0;

	count ++;
	count &= 1;
	if (count)
		__gpio_set_pin(GPIO_LED_EN);
	else
		__gpio_clear_pin(GPIO_LED_EN);
}

static void pavo_timer_callback(void)
{
	static unsigned long count = 0;

	if ((++count) % 50 == 0) {
		dancing();
		count = 0;
	}
}

static void __init board_cpm_setup(void)
{
	/* Stop unused module clocks here.
	 * We have started all module clocks at arch/mips/jz4740/setup.c.
	 */
}

static void __init board_gpio_setup(void)
{
	/*
	 * Most of the GPIO pins should have been initialized by the boot-loader
	 */

	/*
	 * Initialize MSC pins
	 */
	__gpio_as_msc();

	/*
	 * Initialize LCD pins
	 */
	__gpio_as_lcd_18bit();

	/*
	 * Initialize SSI pins
	 */
	__gpio_as_ssi();

	/*
	 * Initialize I2C pins
	 */
	__gpio_as_i2c();

	/*
	 * Initialize Other pins
	 */
	__gpio_as_output(GPIO_SD_VCC_EN_N);
	__gpio_clear_pin(GPIO_SD_VCC_EN_N);

	__gpio_as_input(GPIO_SD_CD_N);
	__gpio_disable_pull(GPIO_SD_CD_N);

	__gpio_as_input(GPIO_SD_WP);
	__gpio_disable_pull(GPIO_SD_WP);

	__gpio_as_input(GPIO_DC_DETE_N);
	__gpio_as_input(GPIO_CHARG_STAT_N);
	__gpio_as_input(GPIO_USB_DETE);

	__gpio_as_output(GPIO_DISP_OFF_N);

	__gpio_as_output(GPIO_LED_EN);

#define RD_N_PIN (32 + 29)
#define WE_N_PIN (32 + 30)
#define CS4_PIN (32 + 28)
#define CS3_PIN (32 + 27) 
	unsigned int reg;
	__gpio_as_func0(CS4_PIN);
	__gpio_as_func0(RD_N_PIN);
	__gpio_as_func0(WE_N_PIN);

	reg = REG_EMC_SMCR4;
	reg = (reg & (~EMC_SMCR_BW_MASK)) | EMC_SMCR_BW_16BIT;
	REG_EMC_SMCR4 = reg;	
	
	__gpio_as_input(CS3_PIN);
	__gpio_as_irq_rise_edge(CS3_PIN);

#define BELL_PIN  (32*3 + 15) 
	__gpio_as_output(BELL_PIN);
	__gpio_clear_pin(BELL_PIN);

}

void __init jz_board_setup(void)
{
	printk("JZ4740 PAVO board setup\n");

	board_cpm_setup();
	board_gpio_setup();

	jz_timer_callback = pavo_timer_callback;
}
