/*
 *  Atmel Touch Screen Driver
 *
 *  Copyright (c) 2008 ATMEL
 *  Copyright (c) 2008 Dan Liang
 *  Copyright (c) 2008 TimeSys Corporation
 *  Copyright (c) 2008 Justin Waters
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

#if defined(CONFIG_MACH_AT91SAM9G45EKES)
	#define COUNT_MAX		1
#else
	#define COUNT_MAX		20
#endif

#define ANDROID_CALIBRATION

#if defined(ANDROID_CALIBRATION)
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

struct atmel_tsadcc {
	struct input_dev	*input;
	char			phys[32];
	struct clk		*clk;
	int			irq;
	unsigned int		prev_absx;
	unsigned int		prev_absy;
	unsigned int		prev_absz;   
};

static void __iomem		*tsc_base;
static unsigned int		trigger_period;

#define atmel_tsadcc_read(reg)		__raw_readl(tsc_base + (reg))
#define atmel_tsadcc_write(reg, val)	__raw_writel((val), tsc_base + (reg))

#if defined(CONFIG_MACH_AT91SAM9M10G45EK) || defined(CONFIG_MACH_AT91SAM9G45EKES) || defined(CONFIG_MACH_AT91SAM9M10EKES)
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
#endif

#if defined(ANDROID_CALIBRATION)
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

static irqreturn_t atmel_tsadcc_interrupt(int irq, void *dev)
{
	struct atmel_tsadcc	*ts_dev = (struct atmel_tsadcc *)dev;
	struct input_dev	*input_dev = ts_dev->input;
	struct at91_tsadcc_data *pdata = input_dev->dev.parent->platform_data;
	static int count = 0;

	unsigned int status;
	unsigned int reg;
	unsigned int x, y;
	unsigned int z1, z2;
	unsigned int Rxp = 1;
	unsigned int factor = 1000;

#if defined(CONFIG_MACH_AT91SAM9M10G45EK) || defined(CONFIG_MACH_AT91SAM9G45EKES) || defined(CONFIG_MACH_AT91SAM9M10EKES)
	static unsigned int point_buffer_x[COUNT_MAX];
	static unsigned int point_buffer_y[COUNT_MAX];
#endif

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

#if defined(CONFIG_MACH_AT91SAM9M10G45EK) || defined(CONFIG_MACH_AT91SAM9G45EKES) || defined(CONFIG_MACH_AT91SAM9M10EKES)
		if (count < COUNT_MAX) {
			point_buffer_x[count] = ts_dev->prev_absx;
			point_buffer_y[count] = ts_dev->prev_absy;
			count++;
		} else {
			
			count = 0;
			x =  do_filter(point_buffer_x, COUNT_MAX, COUNT_MAX * 3 / 4);
			y =  do_filter(point_buffer_y, COUNT_MAX, COUNT_MAX * 3 / 4);

			do_calibrate(&x,&y);
			input_report_abs(input_dev, ABS_X, x);
			input_report_abs(input_dev, ABS_Y, y);
			input_report_key(input_dev, BTN_TOUCH, 1);
			input_report_abs(input_dev, ABS_PRESSURE, 7500);
			input_sync(input_dev);
		}
#endif

#if defined(CONFIG_MACH_AT91SAM9X5EK)
				/* report measurement to input layer */
		if (ts_dev->prev_absz < ZTHRESHOLD) {
			dev_dbg(&input_dev->dev,
					"x = %d, y = %d, pressure = %d\n",
					ts_dev->prev_absx, ts_dev->prev_absy,
					ts_dev->prev_absz);
			x = ts_dev->prev_absx;
			y = ts_dev->prev_absy; 
			
			do_calibrate(&x,&y);		
			input_report_abs(input_dev, ABS_X, x);			
			input_report_abs(input_dev, ABS_Y, y);
			if (cpu_has_9x5_adc())
				input_report_abs(input_dev, ABS_PRESSURE, ts_dev->prev_absz);
			input_report_key(input_dev, BTN_TOUCH, 1);
			input_sync(input_dev);
		} else {
			dev_dbg(&input_dev->dev,
					"pressure too low: not reporting\n");
		}
#endif        
	}

	return IRQ_HANDLED;
}

/*
 * The functions for inserting/removing us as a module.
 */

static int __devinit atmel_tsadcc_probe(struct platform_device *pdev)
{
	struct atmel_tsadcc	*ts_dev;
	struct input_dev	*input_dev;
	struct resource		*res;
	struct at91_tsadcc_data *pdata = pdev->dev.platform_data;
	int		err = 0;
	unsigned int	prsc;
	unsigned int	reg;
       unsigned int	startup_time;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "no mmio resource defined.\n");
		return -ENXIO;
	}

	/* Allocate memory for device */
	ts_dev = kzalloc(sizeof(struct atmel_tsadcc), GFP_KERNEL);
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

	err = request_irq(ts_dev->irq, atmel_tsadcc_interrupt, IRQF_DISABLED,
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

	ts_dev->input = input_dev;

	snprintf(ts_dev->phys, sizeof(ts_dev->phys),
		 "%s/input0", dev_name(&pdev->dev));

	input_dev->name = "atmel touch screen controller";
	input_dev->phys = ts_dev->phys;
	input_dev->dev.parent = &pdev->dev;

#if defined(ANDROID_CALIBRATION)
    calibrated = 0;
#endif
	__set_bit(EV_ABS, input_dev->evbit);
	input_set_abs_params(input_dev, ABS_X, 0, 0x3FF, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, 0, 0x3FF, 0, 0);
	input_set_abs_params(input_dev, ABS_PRESSURE, 0, 15000, 0, 0);

	input_set_capability(input_dev, EV_KEY, BTN_TOUCH);

	/* clk_enable() always returns 0, no need to check it */
	clk_enable(ts_dev->clk);

	prsc = clk_get_rate(ts_dev->clk);
	dev_info(&pdev->dev, "Master clock is set at: %d Hz\n", prsc);

    	if (!pdata)
		goto err_fail;

	if (!pdata->adc_clock)
		pdata->adc_clock = ADC_DEFAULT_CLOCK;

	prsc = (prsc / (2 * pdata->adc_clock)) - 1;

#if defined(CONFIG_MACH_AT91SAM9M10G45EK)
	trigger_period = pdata->adc_clock / (200 * COUNT_MAX) -1;
	if (trigger_period < 1)
		trigger_period = 1;
	startup_time = (60 * pdata->adc_clock) / (8 * 1000000) - 1;
	if (startup_time < 1)
		startup_time = 1;
#else
	trigger_period =  0x0FFF;
	startup_time = 0x26;
#endif    

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
		reg = ATMEL_TSADCC_TSAMOD_TS_ONLY_MODE		|
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

static int __devexit atmel_tsadcc_remove(struct platform_device *pdev)
{
	struct atmel_tsadcc *ts_dev = dev_get_drvdata(&pdev->dev);
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

static struct platform_driver atmel_tsadcc_driver = {
	.probe		= atmel_tsadcc_probe,
	.remove		= __devexit_p(atmel_tsadcc_remove),
	.driver		= {
		.name	= "atmel_tsadcc",
	},
};

static int __init atmel_tsadcc_init(void)
{
	return platform_driver_register(&atmel_tsadcc_driver);
}

static void __exit atmel_tsadcc_exit(void)
{
	platform_driver_unregister(&atmel_tsadcc_driver);
}

module_init(atmel_tsadcc_init);
module_exit(atmel_tsadcc_exit);


MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Atmel TouchScreen Driver");
MODULE_AUTHOR("Dan Liang <dan.liang@atmel.com>");

