/*
 *  linux/include/asm-mips/mach-jz4750/board-apus.h
 *
 *  JZ4750-based APUS board ver 1.x definition.
 *
 *  Copyright (C) 2008 Ingenic Semiconductor Inc.
 *
 *  Author: <cwjia@ingenic.cn>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_JZ4750_APUS_H__
#define __ASM_JZ4750_APUS_H__

/*====================================================================== 
 * Frequencies of on-board oscillators
 */
#define JZ_EXTAL		24000000  /* Main extal freq: 24 MHz */
#define JZ_EXTAL2		32768     /* RTC extal freq: 32.768 KHz */

/*====================================================================== 
 * GPIO
 */
#define GPIO_DISP_OFF_N         (32*4+25) /* GPE25 */
#define GPIO_SD0_VCC_EN_N	(32*2+10) /* GPC10 */
#define GPIO_SD0_CD_N		(32*2+11) /* GPC11 */
#define GPIO_SD0_WP		(32*2+12) /* GPC12 */
#define GPIO_SD1_VCC_EN_N	(32*2+13) /* GPC13 */
#define GPIO_SD1_CD_N		(32*2+14) /* GPC14 */
#define GPIO_USB_DETE		(32*2+11) /* GPC15 */
#define GPIO_DC_DETE_N		(32*2+8)  /* GPC8 */
#define GPIO_CHARG_STAT_N	(32*2+9)  /* GPC9 */
#define GPIO_LCD_VCC_EN_N	(32*3+30) /* GPC10 */
#define GPIO_LCD_PWM   		(32*4+24) /* GPE24 */
#define GPIO_UDC_HOTPLUG	GPIO_USB_DETE

#define CFG_PBAT_DIV            4

/*====================================================================== 
 * LCD backlight
 */
#define LCD_PWM_CHN 4    /* pwm channel */

#define LCD_MAX_BACKLIGHT		100
#define LCD_MIN_BACKLIGHT		1
#define LCD_DEFAULT_BACKLIGHT		80

/* LCD Backlight PWM Control - River. */
#define HAVE_LCD_PWM_CONTROL	1

#ifdef HAVE_LCD_PWM_CONTROL
static inline void __lcd_pwm_set_backlight_level(int n)
{
	__tcu_stop_counter(LCD_PWM_CHN);
	
	__tcu_set_pwm_output_shutdown_abrupt(LCD_PWM_CHN);
	__tcu_disable_pwm_output(LCD_PWM_CHN);

	__tcu_set_count(LCD_PWM_CHN, 0);
	__tcu_set_full_data(LCD_PWM_CHN, LCD_MAX_BACKLIGHT + 1);
	__tcu_set_half_data(LCD_PWM_CHN, n);

	__tcu_enable_pwm_output(LCD_PWM_CHN);
	__tcu_start_counter(LCD_PWM_CHN);

	return;
}

static inline void __lcd_pwm_start(void)
{
	__gpio_as_pwm(4);

	__tcu_stop_counter(LCD_PWM_CHN);
	
	__tcu_select_extalclk(LCD_PWM_CHN);
	__tcu_select_clk_div4(LCD_PWM_CHN);
	__tcu_init_pwm_output_high(LCD_PWM_CHN);

	__lcd_pwm_set_backlight_level(LCD_DEFAULT_BACKLIGHT);

	return;
}

static inline void __lcd_pwm_stop(void)
{
	__tcu_stop_counter(LCD_PWM_CHN);

	__tcu_set_pwm_output_shutdown_abrupt(LCD_PWM_CHN);
	__tcu_disable_pwm_output(LCD_PWM_CHN);

	return;
}

#define __lcd_set_backlight_level(n) __lcd_pwm_set_backlight_level(n)

#else /* Old GPIO Control */
/* 100 level: 0,1,...,100 */
#define __lcd_set_backlight_level(n)	\
do {					\
	__gpio_as_output(GPIO_LCD_PWM);	\
	__gpio_set_pin(GPIO_LCD_PWM);	\
} while (0)
#endif

#define __lcd_close_backlight()		\
do {					\
	__gpio_as_output(GPIO_LCD_PWM);	\
	__gpio_clear_pin(GPIO_LCD_PWM);	\
} while (0)

/*====================================================================== 
 * MMC/SD
 */

#define MSC0_WP_PIN		GPIO_SD0_WP
#define MSC0_HOTPLUG_PIN	GPIO_SD0_CD_N
#define MSC0_HOTPLUG_IRQ	(IRQ_GPIO_0 + GPIO_SD0_CD_N)

#define MSC1_WP_PIN		GPIO_SD1_WP
#define MSC1_HOTPLUG_PIN	GPIO_SD1_CD_N
#define MSC1_HOTPLUG_IRQ	(IRQ_GPIO_0 + GPIO_SD1_CD_N)

#define __msc0_init_io()			\
do {						\
	__gpio_as_output(GPIO_SD0_VCC_EN_N);	\
	__gpio_as_input(GPIO_SD0_CD_N);		\
} while (0)

#define __msc0_enable_power()			\
do {						\
	__gpio_clear_pin(GPIO_SD0_VCC_EN_N);	\
} while (0)

#define __msc0_disable_power()			\
do {						\
	__gpio_set_pin(GPIO_SD0_VCC_EN_N);	\
} while (0)

#define __msc0_card_detected(s)			\
({						\
	int detected = 1;			\
	if (__gpio_get_pin(GPIO_SD0_CD_N))	\
		detected = 0;			\
	detected;				\
})

#define __msc1_init_io()			\
do {						\
	__gpio_as_output(GPIO_SD1_VCC_EN_N);	\
	__gpio_as_input(GPIO_SD1_CD_N);		\
} while (0)

#define __msc1_enable_power()			\
do {						\
	__gpio_clear_pin(GPIO_SD1_VCC_EN_N);	\
} while (0)

#define __msc1_disable_power()			\
do {						\
	__gpio_set_pin(GPIO_SD1_VCC_EN_N);	\
} while (0)

#define __msc1_card_detected(s)			\
({						\
	int detected = 0;			\
	if (__gpio_get_pin(GPIO_SD1_CD_N))	\
		detected = 1;			\
	detected;				\
})

#endif /* __ASM_JZ4750_APUS_H__ */
