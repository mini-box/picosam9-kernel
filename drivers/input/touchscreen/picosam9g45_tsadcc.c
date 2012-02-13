/*
 *  picoSAM9G45  Touch Screen Driver
 *
 *  Copyright (c) 2008 ATMEL
 *  Copyright (c) 2008 Dan Liang
 *  Copyright (c) 2008 TimeSys Corporation
 *  Copyright (c) 2008 Justin Waters
 *  Copyright (c) 2011 Nicu Pavel
 *
 *  Based on touchscreen code from Atmel Corporation.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
#include <linux/init.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <mach/board.h>
#include <mach/cpu.h>

#include "atmel_tsadcc.h"

#define cpu_has_9x5_adc() (cpu_is_at91sam9x5())

#define ADC_DEFAULT_CLOCK	100000

#define ZTHRESHOLD		3200

#define COUNT_MAX		20

/* the maximum X axis for touchscreen before the buttons stripe */
#define MAX_X		1010
/* added for ADC on picoPC */
#define ATMEL_TSADCC_TSAMOD_INTERLEAVE_MODE	(0x2)	/* Interleave Mode */
#define ATMEL_TSADCC_CDR6	0x48	/* Channel Data 6 */
#define ATMEL_TSADCC_CDR7	0x4C	/* Channel Data 7 */

struct analogInputInfo {
    int chan;
    int port;
    int value;
    int valid;
    unsigned long last_update;
};

static struct analogInputInfo analogInputMap[] = {
    [0] = { .chan = 6, .port = ATMEL_TSADCC_CDR6, .value = 0, .last_update = 0, .valid = 0 },
    [1] = { .chan = 7, .port = ATMEL_TSADCC_CDR7, .value = 0, .last_update = 0, .valid = 0 },
    [2] = { .chan = 5, .port = ATMEL_TSADCC_CDR5, .value = 0, .last_update = 0, .valid = 0 },
    [3] = { .chan = 4, .port = ATMEL_TSADCC_CDR4, .value = 0, .last_update = 0, .valid = 0 },
};


static const int buttonEvents[] = {
	BTN_TOUCH,
};

static const int keyEvents[] = {
	85, KEY_HOME, KEY_MENU, 
	KEY_BACK, 88,
};

#ifdef CONFIG_TOUCHSCREEN_PICOSAM9G45_TSADCC_CALIBRATE
int calibrated;	
int tx1;		
int ty1;		
int tz1;		
int tx2;		
int ty2;		
int tz2;		
int rawX;
int rawY;
int ts;

module_param(tx1, int, 0664);
module_param(ty1, int, 0664);
module_param(tz1, int, 0664);
module_param(tx2, int, 0664);
module_param(ty2, int, 0664);
module_param(tz2, int, 0664);
module_param(rawX, int, 0664);
module_param(rawY, int, 0664);
module_param(ts, int, 0664);
module_param(calibrated, int, 0664);

#endif

struct picosam9g45_tsadcc {
	struct input_dev	*input;
	char			phys[32];
	struct clk		*clk;
	int			irq;
	unsigned int		prev_absx;
	unsigned int		prev_absy;
	unsigned int		prev_absz;
};

/* Not thread safe */
static struct semaphore	adc_lock;
static void __iomem		*tsc_base;
static unsigned int		trigger_period;

static void __iomem		*tsc_base;
static unsigned int		trigger_period;

#define atmel_tsadcc_read(reg)		__raw_readl(tsc_base + (reg))
#define atmel_tsadcc_write(reg, val)	__raw_writel((val), tsc_base + (reg))

static unsigned int do_filter(unsigned int val[], int count, int needed) {
	int i, j;
	int max_delta, max_delta_index;
	unsigned int average;

	if (needed == 0)
		return val[0];

	for (i = count; i > needed; i--) {
		average = 0;
		for (j = 0; j < i; j++)
			average += val[j];
		average /= i;

		max_delta = 0;
		max_delta_index = -1;
		for (j = 0; j < i; j++) {
			if (abs(val[j] - average) > max_delta) {
				max_delta = abs(val[j] - average);
				max_delta_index = j;
			}
		}

		if (max_delta_index < 0)
			return average;

		if (max_delta_index < i - 1)
			for (j = 0; j < i - max_delta_index; j++)
				val[max_delta_index + j] = val[max_delta_index + j + 1];
	}

	average = 0;
	for (i = 0; i < needed; i++)
		average += val[i];
	average /= needed;

	return average;
}

#ifdef CONFIG_TOUCHSCREEN_PICOSAM9G45_TSADCC_CALIBRATE
static void do_calibrate(int *x,int *y){
	int cal_x,cal_y;
	rawX = *x;
	rawY = *y;
	cal_x = *x;
	cal_y = *y;
		 
	if(calibrated && ts)
	{
		cal_x = (rawX*tx1 + rawY*ty1 + tz1)/ts ;
		cal_y = (rawX*tx2 + rawY*ty2 + tz2)/ts ;
	}
	*x = cal_x;
	*y = cal_y;
}

#else
static void do_calibrate(int * x, int * y){}
#endif

static irqreturn_t picosam9g45_tsadcc_interrupt(int irq, void *dev)
{
	struct picosam9g45_tsadcc	*ts_dev = (struct picosam9g45_tsadcc *)dev;
	struct input_dev	*input_dev = ts_dev->input;
	struct at91_tsadcc_data *pdata = input_dev->dev.parent->platform_data;
	static int count = 0;

	unsigned int status;
	unsigned int reg;
	unsigned int x, y;
	unsigned int z1, z2;
	unsigned int Rxp = 1;
	unsigned int factor = 1000;

	static unsigned int point_buffer_x[COUNT_MAX];
	static unsigned int point_buffer_y[COUNT_MAX];

	status = atmel_tsadcc_read(ATMEL_TSADCC_SR);
	status &= atmel_tsadcc_read(ATMEL_TSADCC_IMR);

	if (status & ATMEL_TSADCC_NOCNT) {
		/* Contact lost */
		if (cpu_has_9x5_adc()) {
			/* 9X5 using TSMR to set PENDBC time */
			reg = atmel_tsadcc_read(ATMEL_TSADCC_TSMR) | ((pdata->pendet_debounce << 28) & ATMEL_TSADCC_PENDBC);
			atmel_tsadcc_write(ATMEL_TSADCC_TSMR, reg);
		} else {
			reg = atmel_tsadcc_read(ATMEL_TSADCC_MR) | ATMEL_TSADCC_PENDBC;
			atmel_tsadcc_write(ATMEL_TSADCC_MR, reg);
		}
		atmel_tsadcc_write(ATMEL_TSADCC_TRGR, ATMEL_TSADCC_TRGMOD_NONE);
		atmel_tsadcc_write(ATMEL_TSADCC_IDR,
				   ATMEL_TSADCC_CONVERSION_END | ATMEL_TSADCC_NOCNT);
		atmel_tsadcc_write(ATMEL_TSADCC_IER, ATMEL_TSADCC_PENCNT);

		input_report_key(input_dev, BTN_TOUCH, 0);
		input_report_abs(input_dev, ABS_PRESSURE, 0);
#ifdef CONFIG_TOUCHSCREEN_PICOSAM9G45_TSADCC_BUTTONS
		input_report_key(input_dev, 85,0);
		input_report_key(input_dev, 88, 0);
		input_report_key(input_dev, KEY_HOME, 0);
		input_report_key(input_dev, KEY_BACK, 0);
		input_report_key(input_dev, KEY_MENU, 0);
#endif

		input_sync(input_dev);

	} else if (status & ATMEL_TSADCC_PENCNT) {
		/* Pen detected */
		if (cpu_has_9x5_adc()) {
			reg = atmel_tsadcc_read(ATMEL_TSADCC_TSMR);
			reg &= ~ATMEL_TSADCC_PENDBC;
			atmel_tsadcc_write(ATMEL_TSADCC_TSMR, reg);
		} else {
			reg = atmel_tsadcc_read(ATMEL_TSADCC_MR);
			reg &= ~ATMEL_TSADCC_PENDBC;
			atmel_tsadcc_write(ATMEL_TSADCC_MR, reg);
		}

		atmel_tsadcc_write(ATMEL_TSADCC_IDR, ATMEL_TSADCC_PENCNT);
		atmel_tsadcc_write(ATMEL_TSADCC_IER,
				   ATMEL_TSADCC_CONVERSION_END | ATMEL_TSADCC_NOCNT);
		if (cpu_has_9x5_adc()) {
		    atmel_tsadcc_write(ATMEL_TSADCC_TRGR,
				   ATMEL_TSADCC_TRGMOD_PERIOD | (0x00D0 << 16));
		}else{
		    atmel_tsadcc_write(ATMEL_TSADCC_TRGR,
				   ATMEL_TSADCC_TRGMOD_PERIOD | (trigger_period << 16));
		}

              count = 0;

	} else if ((status & ATMEL_TSADCC_CONVERSION_END) == ATMEL_TSADCC_CONVERSION_END) {
		/* Conversion finished */
		
		/* make new measurement */
		if (cpu_has_9x5_adc()) {
			unsigned int xscale, yscale;
			
			/* calculate position */
			reg = atmel_tsadcc_read(ATMEL_TSADCC_XPOSR);
			ts_dev->prev_absx = (reg & ATMEL_TSADCC_XPOS) << 10;
			xscale = (reg & ATMEL_TSADCC_XSCALE) >> 16;
			ts_dev->prev_absx /= xscale ? xscale: 1;

			reg = atmel_tsadcc_read(ATMEL_TSADCC_YPOSR);
			ts_dev->prev_absy = (reg & ATMEL_TSADCC_YPOS) << 10;
			yscale = (reg & ATMEL_TSADCC_YSCALE) >> 16;
			ts_dev->prev_absy /= yscale ? yscale: 1 << 10;

			/* calculate the pressure */
			reg = atmel_tsadcc_read(ATMEL_TSADCC_PRESSR);
			z1 = reg & ATMEL_TSADCC_PRESSR_Z1;
			z2 = (reg & ATMEL_TSADCC_PRESSR_Z2) >> 16;

			if (z1 != 0)
				ts_dev->prev_absz = Rxp * (ts_dev->prev_absx * factor / 1024) * (z2 * factor / z1 - factor) / factor;
			else
				ts_dev->prev_absz = 0;

		} else {
			ts_dev->prev_absx = atmel_tsadcc_read(ATMEL_TSADCC_CDR3) << 10;
			ts_dev->prev_absx /= atmel_tsadcc_read(ATMEL_TSADCC_CDR2);

			ts_dev->prev_absy = atmel_tsadcc_read(ATMEL_TSADCC_CDR1) << 10;
			ts_dev->prev_absy /= atmel_tsadcc_read(ATMEL_TSADCC_CDR0);
		}
		if (count < COUNT_MAX) {
			point_buffer_x[count] = ts_dev->prev_absx;
			point_buffer_y[count] = ts_dev->prev_absy;
			count++;
		} else {
			count = 0;
			
			x =  do_filter(point_buffer_x, COUNT_MAX, COUNT_MAX * 3 / 4);
			y =  do_filter(point_buffer_y, COUNT_MAX, COUNT_MAX * 3 / 4);
#ifdef CONFIG_TOUCHSCREEN_PICOSAM9G45_TSADCC_BUTTONS
			if (y > 820)
			{
				if (x < 200 && x > 50)
					input_report_key(input_dev, 85, 1);
				if (x < 400 && x > 200)
					input_report_key(input_dev, KEY_BACK, 1);
				if (x < 600 && x > 400)
					input_report_key(input_dev, KEY_HOME, 1);
				if (x < 820 && x > 600)
					input_report_key(input_dev, KEY_MENU, 1);
				if (x < 1000 && x > 820)
					input_report_key(input_dev, 88, 1);
			}
			else
#endif
			{
				do_calibrate(&x,&y);
				input_report_abs(input_dev, ABS_X, x);
				input_report_abs(input_dev, ABS_Y, y);
				input_report_key(input_dev, BTN_TOUCH, 1);
				input_report_abs(input_dev, ABS_PRESSURE, 7500);
			}
			input_sync(input_dev);
		}
	}
	return IRQ_HANDLED;
}

static ssize_t picosam9g45_adc_read(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t status = 0;
	int ani = -1;
	int sreg;

	if(down_interruptible(&adc_lock))
		return -ERESTARTSYS;

	if(strlen(attr->attr.name)==4 && strncmp(attr->attr.name,"ani",3)==0) {
		ani = attr->attr.name[3]-'0';
	}

	if(ani < 0 || ani > 3){
		return -EIO;
	}
	/* 1 second update time */
	if (time_after(jiffies, analogInputMap[ani].last_update + HZ) ||
	    !analogInputMap[ani].valid)
	{
		atmel_tsadcc_write(ATMEL_TSADCC_CR, ATMEL_TSADCC_START);
		/*  Wait till ADC finishes converting the channel */
		for(sreg=0; !(sreg & ATMEL_TSADCC_EOC(analogInputMap[ani].chan)); sreg=atmel_tsadcc_read(ATMEL_TSADCC_SR))
			cpu_relax();

		analogInputMap[ani].value = atmel_tsadcc_read(analogInputMap[ani].port);
		analogInputMap[ani].last_update = jiffies;
		analogInputMap[ani].valid = 1;
	}
	status = sprintf(buf, "%d\n", analogInputMap[ani].value);
	up(&adc_lock);
	return status;
}

static struct platform_device adc_dev = {
    .name = "picosam9g45_adc",
    .id = -1,
};

static DEVICE_ATTR(ani0, 0444, picosam9g45_adc_read, NULL);
static DEVICE_ATTR(ani1, 0444, picosam9g45_adc_read, NULL);
static DEVICE_ATTR(ani2, 0444, picosam9g45_adc_read, NULL);
static DEVICE_ATTR(ani3, 0444, picosam9g45_adc_read, NULL);

static const struct attribute *adc_attrs[] = {
    &dev_attr_ani0.attr,
    &dev_attr_ani1.attr,
    &dev_attr_ani2.attr,
    &dev_attr_ani3.attr,
    NULL
};

static const struct attribute_group adc_attr_group = {
    .attrs =  (struct attribute **) adc_attrs
};


/*
 * The functions for inserting/removing us as a module.
 */

static int __devinit picosam9g45_tsadcc_probe(struct platform_device *pdev)
{
	struct picosam9g45_tsadcc	*ts_dev;
	struct input_dev	*input_dev;
	struct resource		*res;
	struct at91_tsadcc_data *pdata = pdev->dev.platform_data;
	int		err = 0;
	int i;
	unsigned int	prsc;
	unsigned int	reg;
	unsigned int	startup_time;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "no mmio resource defined.\n");
		return -ENXIO;
	}

	/* Allocate memory for device */
	ts_dev = kzalloc(sizeof(struct picosam9g45_tsadcc), GFP_KERNEL);
	if (!ts_dev) {
		dev_err(&pdev->dev, "failed to allocate memory.\n");
		return -ENOMEM;
	}
	platform_set_drvdata(pdev, ts_dev);

	input_dev = input_allocate_device();
	if (!input_dev) {
		dev_err(&pdev->dev, "failed to allocate input device.\n");
		err = -EBUSY;
		goto err_free_mem;
	}

	ts_dev->irq = platform_get_irq(pdev, 0);
	if (ts_dev->irq < 0) {
		dev_err(&pdev->dev, "no irq ID is designated.\n");
		err = -ENODEV;
		goto err_free_dev;
	}

	if (!request_mem_region(res->start, resource_size(res),
				"atmel tsadcc regs")) {
		dev_err(&pdev->dev, "resources is unavailable.\n");
		err = -EBUSY;
		goto err_free_dev;
	}

	tsc_base = ioremap(res->start, resource_size(res));
	if (!tsc_base) {
		dev_err(&pdev->dev, "failed to map registers.\n");
		err = -ENOMEM;
		goto err_release_mem;
	}

	err = request_irq(ts_dev->irq, picosam9g45_tsadcc_interrupt, IRQF_DISABLED,
			pdev->dev.driver->name, ts_dev);
	if (err) {
		dev_err(&pdev->dev, "failed to allocate irq.\n");
		goto err_unmap_regs;
	}

	ts_dev->clk = clk_get(&pdev->dev, "tsc_clk");
	if (IS_ERR(ts_dev->clk)) {
		dev_err(&pdev->dev, "failed to get ts_clk\n");
		err = PTR_ERR(ts_dev->clk);
		goto err_free_irq;
	}
	
	sema_init(&adc_lock, 1);

	err = platform_device_register(&adc_dev);
	if (err) {
	    goto err_free_irq;
	}

	err = sysfs_create_group(&adc_dev.dev.kobj, &adc_attr_group);
	if (err) {
	    goto err_free_irq;
	}

	ts_dev->input = input_dev;

	snprintf(ts_dev->phys, sizeof(ts_dev->phys),
		 "%s/input0", dev_name(&pdev->dev));

	input_dev->name = "pico-touch";
	input_dev->phys = ts_dev->phys;
	input_dev->dev.parent = &pdev->dev;

#ifdef CONFIG_TOUCHSCREEN_PICOSAM9G45_TSADCC_CALIBRATE
    calibrated = 0;
#endif
	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(EV_ABS, input_dev->evbit);
	
	for (i = 0; i < ARRAY_SIZE(buttonEvents); ++i)
		__set_bit(buttonEvents[i], input_dev->keybit);
		
	for (i = 0; i < ARRAY_SIZE(keyEvents); ++i)
		__set_bit(keyEvents[i], input_dev->keybit);

	
	input_set_abs_params(input_dev, ABS_X, 0, 0x3FF, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, 0, 0x3FF, 0, 0);
	input_set_abs_params(input_dev, ABS_PRESSURE, 0, 15000, 0, 0);

	input_set_capability(input_dev, EV_KEY, BTN_TOUCH);
	
	device_init_wakeup(&pdev->dev, 1);

	/* clk_enable() always returns 0, no need to check it */
	clk_enable(ts_dev->clk);

	prsc = clk_get_rate(ts_dev->clk);
	dev_info(&pdev->dev, "Master clock is set at: %d Hz\n", prsc);

    	if (!pdata)
		goto err_fail;

	if (!pdata->adc_clock)
		pdata->adc_clock = ADC_DEFAULT_CLOCK;

	prsc = (prsc / (2 * pdata->adc_clock)) - 1;

	trigger_period = pdata->adc_clock / (200 * COUNT_MAX) -1;
	if (trigger_period < 1)
		trigger_period = 1;
	startup_time = (60 * pdata->adc_clock) / (8 * 1000000) - 1;
	if (startup_time < 1)
		startup_time = 1;

	/* saturate if this value is too high */
	if (cpu_is_at91sam9rl()) {
		if (prsc > PRESCALER_VAL(ATMEL_TSADCC_PRESCAL))
			prsc = PRESCALER_VAL(ATMEL_TSADCC_PRESCAL);
	} else {
		if (prsc > PRESCALER_VAL(ATMEL_TSADCC_EPRESCAL))
			prsc = PRESCALER_VAL(ATMEL_TSADCC_EPRESCAL);
	}

	dev_info(&pdev->dev, "Prescaler is set at: %d\n", prsc);

	if (cpu_has_9x5_adc()) {
		reg = 	((0x00 << 5) & ATMEL_TSADCC_SLEEP)	|	/* no Sleep Mode */
			((0x00 << 6) & ATMEL_TSADCC_FWUP)	|	/* no Fast Wake Up needed */
			(prsc << 8)				|
            ((0x4 << 16) & ATMEL_TSADCC_STARTUP)	|
			((pdata->ts_sample_hold_time << 24) & ATMEL_TSADCC_TRACKTIM);
	} else {
		reg = ATMEL_TSADCC_TSAMOD_INTERLEAVE_MODE		|
			((0x00 << 5) & ATMEL_TSADCC_SLEEP)	|	/* Normal Mode */
			((0x01 << 6) & ATMEL_TSADCC_PENDET)	|	/* Enable Pen Detect */
			(prsc << 8)				|
			((startup_time << 16) & ATMEL_TSADCC_STARTUP)	|
			((pdata->pendet_debounce << 28) & ATMEL_TSADCC_PENDBC);
	}

	atmel_tsadcc_write(ATMEL_TSADCC_CR, ATMEL_TSADCC_SWRST);
	atmel_tsadcc_write(ATMEL_TSADCC_MR, reg);
	atmel_tsadcc_write(ATMEL_TSADCC_TRGR, ATMEL_TSADCC_TRGMOD_NONE);

	if (cpu_has_9x5_adc()) {
		atmel_tsadcc_write(ATMEL_TSADCC_TSMR,
					ATMEL_TSADCC_TSMODE_4WIRE_PRESS	|
					((pdata->filtering_average << 4) & ATMEL_TSADCC_TSAV) |	/* Touchscreen average */
					ATMEL_TSADCC_NOTSDMA		|
					ATMEL_TSADCC_PENDET_ENA		|
					((pdata->pendet_debounce << 28) & ATMEL_TSADCC_PENDBC) |
					(0x3 << 8));				/* Touchscreen freq */
	} else {
		atmel_tsadcc_write(ATMEL_TSADCC_TSR,
			(pdata->ts_sample_hold_time << 24) & ATMEL_TSADCC_TSSHTIM);
	}

	atmel_tsadcc_read(ATMEL_TSADCC_SR);
	atmel_tsadcc_write(ATMEL_TSADCC_IER, ATMEL_TSADCC_PENCNT);

	/* Enable all channels */
	atmel_tsadcc_write(ATMEL_TSADCC_CHER, 0xFF);

	/* All went ok, so register to the input system */
	err = input_register_device(input_dev);
	if (err)
		goto err_fail;

	return 0;

err_fail:
	clk_disable(ts_dev->clk);
	clk_put(ts_dev->clk);
err_free_irq:
	free_irq(ts_dev->irq, ts_dev);
err_unmap_regs:
	iounmap(tsc_base);
err_release_mem:
	release_mem_region(res->start, resource_size(res));
err_free_dev:
	input_free_device(ts_dev->input);
err_free_mem:
	kfree(ts_dev);
	return err;
}

static int __devexit picosam9g45_tsadcc_remove(struct platform_device *pdev)
{
	struct picosam9g45_tsadcc *ts_dev = dev_get_drvdata(&pdev->dev);
	struct resource *res;

	free_irq(ts_dev->irq, ts_dev);

	input_unregister_device(ts_dev->input);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	iounmap(tsc_base);
	release_mem_region(res->start, resource_size(res));

	clk_disable(ts_dev->clk);
	clk_put(ts_dev->clk);

	kfree(ts_dev);

	return 0;
}

static struct platform_driver picosam9g45_tsadcc_driver = {
	.probe		= picosam9g45_tsadcc_probe,
	.remove		= __devexit_p(picosam9g45_tsadcc_remove),
	.driver		= {
		.name	= "picosam9g45_tsadcc",
	},
};

static int __init picosam9g45_tsadcc_init(void)
{
	return platform_driver_register(&picosam9g45_tsadcc_driver);
}

static void __exit picosam9g45_tsadcc_exit(void)
{
	platform_driver_unregister(&picosam9g45_tsadcc_driver);
}

module_init(picosam9g45_tsadcc_init);
module_exit(picosam9g45_tsadcc_exit);


MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("picoSAM9G45 TouchScreen Driver");
MODULE_AUTHOR("Nicu Pavel <npavel@mini-box.com>");

