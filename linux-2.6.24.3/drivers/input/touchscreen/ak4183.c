/*
 * drivers/input/touchscreen/ak4183.c
 *
 * ak4183.c i2c driver for volans board
 * 
 * Touch screen driver for AK4183.
 *
 * Routine transform_to_screen_x() and transform_to_screen_y() come from
 * drivers/input/touchscreen/jz_ts.c
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 */

#include <linux/module.h>
#include <linux/version.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/miscdevice.h>
#include <linux/delay.h>
#include <linux/poll.h>
#include <linux/string.h>
#include <linux/pm.h>
#include <linux/pm_legacy.h>
#include <linux/input.h>

#include <asm/uaccess.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <asm/jzsoc.h>

#include "ak4183.h"

#define DEBUG 0

#if DEBUG
#	define warn(fmt, args...) printk(KERN_WARNING fmt, ##args)
#	define info(fmt, args...) printk(KERN_INFO fmt, ##args)
#	define dbg(fmt, args...) printk(KERN_DEBUG fmt, ##args)
#else
#	define warn(fmt, args...)
#	define info(fmt, args...)
#	define dbg(fmt, args...)
#endif
#define err(fmt, args...) printk(KERN_ERR fmt, ##args)

struct	ak4183		*ts;
static unsigned int	slave_addr = 0x48;

/*
 * ak4183 control command
 * BIT	Name	Function
 *  7	 S	Start Bit 0: Sleep mode, 1: Accelerate and Axis
 * 6-4	A2-A0	Channel Selection Bits
 *  3	X1	Don't care
 *  2	PD0	Power down
 *  1	Mode	Resolution of A/D converter. 0: 12bit, 1: 8bit
 *  0	X2	Don't care
 */
static inline void ak4183_write(u8 val)
{
	i2c_write(slave_addr, &val, 0, 1);
}

static inline unsigned int ak4183_read(void)
{
	unsigned char val = 0;

	i2c_read(slave_addr, &val, 0, 1);
	return val;
}

static unsigned int ak4183_adc_read(u8 cmd)
{
	unsigned char val = 0;

	i2c_write(slave_addr, &cmd, 0, 1);
	i2c_read(slave_addr, &val, 0, 1);

	return val;
}

static inline unsigned int ak4183_read_xpos(void)
{
	return ak4183_adc_read(0xC2);//X-axis,0xC0 for 12bit,0xC2 for 8bit
}

static inline unsigned int ak4183_read_pressure(void)
{
	unsigned int z1, z2, xpos, pressure=0;//300 Om
	//Z1 pressure
	z1 = ak4183_adc_read(0xE2);//0xE0 for 12bit,0xE2 for 8bit
	if(z1>0)
	{
		//Z2 pressure
		z2 = ak4183_adc_read(0xF2);//0xF0 for 12bit,0xF2 for 8bit
		if(z2>z1)
		{
			xpos = ak4183_read_xpos();
			//pressure = (300*xpos*(z2-z1))/(4096*z1);
			pressure = (300*xpos*(z2-z1))/(256*z1);
		}
	}
	
	return pressure;
}

static inline unsigned int ak4183_read_ypos(void)
{
	return ak4183_adc_read(0xD2);//Y-axis,0xD0 for 12bit,0xD2 for 8bit
}

static int is_pen_down(void)
{
	unsigned int p;
	p = ak4183_read_pressure();
	return (p > 100) ? 1 : 0;
}

static unsigned long transform_to_screen_x(unsigned long x)
{       
	if (x < TSMINX) x = TSMINX;
	if (x > TSMAXX) x = TSMAXX;

	return SCREEN_X - (x - TSMINX) * SCREEN_X / (TSMAXX - TSMINX);
}

static unsigned long transform_to_screen_y(unsigned long y)
{
	if (y < TSMINY) y = TSMINY;
	if (y > TSMAXY) y = TSMAXY;

	return (y - TSMINY) * SCREEN_Y / (TSMAXY - TSMINY);
}

static irqreturn_t ak4183_irq(int irq, void *dev_id)
{
	schedule_delayed_work(&ts->work, 1);

	return IRQ_HANDLED;
}

static unsigned int get_xaxis(void)
{
	int i, j;
	unsigned int x_raw[SAMPLES];
	unsigned int tmp, x = 0;

	memset(x_raw, 0, sizeof(unsigned int)*SAMPLES);

	ak4183_write(0x82);
	for (i = 0,j = 0; i < SAMPLES; i++) {
		tmp = ak4183_read();
		if((tmp < TSMINX) || (tmp > TSMAXX))
			continue;
		x_raw[j++] = tmp;
	}
	if (j > 4) {
		for (i = 0; i < j; i++) {
			x += x_raw[i];
		}
		x /= j;

		return x;
	}

	return 0xbadbabe;
}

static unsigned int get_yaxis(void)
{
	int i, j;
	unsigned int y_raw[SAMPLES];
	unsigned int tmp, y = 0;

	memset(y_raw, 0, sizeof(unsigned int)*SAMPLES);

	ak4183_write(0x92);
	for (i = 0,j = 0; i < SAMPLES; i++) {
		tmp = ak4183_read();
		if((tmp < TSMINY) || (tmp > TSMAXY))
			continue;
		y_raw[j++] = tmp;
	}
	if (j > 4) {
		for (i = 0; i < j; i++) {
			y += y_raw[i];
		}
		y /= j;

		return y;
	}

	return 0xbadbabe;
}

static void report_event(struct work_struct *work)
{
	int x = 0, y = 0, z1 = 0, z2 = 0;

sample_again:
	if (__gpio_get_pin(TS_PIN)) {
		goto fake_touch;
	} else {
		x = get_xaxis();
		y = get_yaxis();
		z1 = ak4183_adc_read(0xE2);
		z2 = ak4183_adc_read(0xF2);
		if ((x == 0xbadbabe) || (y == 0xbadbabe)) {
			mdelay(10);
			goto sample_again;	
		}
		x = transform_to_screen_x(x);
		y = transform_to_screen_y(y);
		input_report_abs(ts->input, ABS_X, x);
		input_report_abs(ts->input, ABS_Y, y);
		input_report_abs(ts->input, ABS_PRESSURE, z1);
		input_report_key(ts->input, BTN_TOUCH, 1);
		input_sync(ts->input);
	}

	while(!__gpio_get_pin(TS_PIN));
	input_report_abs(ts->input, ABS_PRESSURE, 0);
	input_report_key(ts->input, BTN_TOUCH, 0);
	input_sync(ts->input);

fake_touch:
	return;
}

/*
 * Module init and exit
 */
static int __init ak4183_init(void)
{
	struct	input_dev	*input_dev;
	int			err = -1;

	__gpio_as_i2c();
	__gpio_as_irq_fall_edge(TS_PIN);

	i2c_open();

	i2c_setclk(100000);	/* 100KHz */

	ts = kmalloc(sizeof(struct ak4183), GFP_KERNEL);
	memset(ts, 0, sizeof(struct ak4183));

	input_dev = input_allocate_device();
	if (!ts || !input_dev) {
		err = -ENOMEM;
		goto err_free_mem;
	}

	strcpy(ts->phys, "input/ts0");

	input_dev->name = "ak4183ts";
	input_dev->phys = ts->phys;

	set_bit(EV_ABS, input_dev->evbit);
	set_bit(ABS_X, input_dev->absbit);
	set_bit(ABS_Y, input_dev->absbit);
	set_bit(ABS_PRESSURE, input_dev->absbit);
	set_bit(EV_KEY, input_dev->evbit);
	set_bit(BTN_TOUCH, input_dev->keybit);
	
	input_set_abs_params(input_dev, ABS_X, 0, SCREEN_X + 1, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, 0, SCREEN_Y + 1, 0, 0);
	input_set_abs_params(input_dev, ABS_PRESSURE, 0, PRESS_Z + 1, 0, 0);

	input_set_drvdata(input_dev, ts);

	err = input_register_device(input_dev);
	if (err < 0) {
		printk("Register ak4183 touch screen driver failed");
		return err;
	}

	ts->input = input_dev;
	spin_lock_init(&ts->lock);
	INIT_DELAYED_WORK(&ts->work, report_event);

	err = request_irq(TS_IRQ, ak4183_irq, IRQF_DISABLED, AK4183_NAME, ts);
	if (err) {
		printk("unable to get touch screen IRQ %d", TS_IRQ);
		goto err_free_irq;
	}

	return 0;

err_free_irq:
	free_irq(TS_IRQ, ts);

err_free_mem:
	input_free_device(input_dev);
	kfree(ts);
	return err;
}

static void __exit ak4183_exit(void)
{
	i2c_close();
	free_irq(TS_IRQ, ts);
	input_unregister_device(ts->input);
	cancel_delayed_work_sync(&ts->work);
}

MODULE_AUTHOR("Ross<fdbai@ingenic.cn>");
MODULE_DESCRIPTION("AK4183 touch screen driver");
MODULE_LICENSE("GPL");

module_init(ak4183_init);
module_exit(ak4183_exit);
