/*
 *  Touchscreen driver for picoSAM9G45 based on MStar chipset
 *
 *  Copyright (c) 2011 Honmax Corp. Laijinchuang
 *  Copyright (c) 2011 Nicu Pavel <npavel@mini-box.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/types.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/timer.h>
#include <linux/i2c-dev.h>
#include <linux/slab.h>

#include <asm/gpio.h>
#include <asm/irq.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/mach-types.h>
#include "touchscreen_msg_pub.h"


#define MS_TS_MSG20XX_DEBUG 1
#define MS_TS_MSG20XX_TRACE 0

#if (MS_TS_MSG20XX_DEBUG == 1)
#define DBG(fmt, arg...) pr_info(fmt, ##arg)
#else
#define DBG(fmt, arg...)
#endif

#if (MS_TS_MSG20XX_TRACE == 1)
#define TRACE(fmt, arg...) pr_info(fmt, ##arg)
#else
#define TRACE(fmt, arg...)
#endif

#define MS_TS_MSG20XX_X_MIN   0
#define MS_TS_MSG20XX_X_MAX   2047
#define MS_TS_MSG20XX_Y_MIN   0
#define MS_TS_MSG20XX_Y_MAX   2047

#define MS_TS_MSG20XX_EXTINT_GPIO_NUM   23

#define MS_TS_I2C_DRIVER_NAME "ms-msg20xx"
static struct workqueue_struct *mstar_wq;

#define I2C_MINORS	256
struct i2c_dev {
	struct list_head list;
	struct i2c_adapter *adap;
	struct device *dev;
};
static struct class *i2c_dev_class;
static LIST_HEAD( i2c_dev_list);
static DEFINE_SPINLOCK( i2c_dev_list_lock);

/* Data type */
struct ms_ts_msg20xx_drvdata {
  struct input_dev *input;
  struct input_dev *input_key;
  MdlTouchScreenInfo_t tsInfo;
  int extint_pin;
  int irq; 
  unsigned short addr;
  struct i2c_client *client;
  struct work_struct work;
};

enum {
	TOUCHSCREEN_PREES=0,
	TOUCH_RELEASE,
	TOUCHBUTTON_PREES,
	TOUCH_ERROR,
};

#define KEY_MAX_CNT 5
u32 key_map[3]={ KEY_HOME, KEY_MENU, KEY_BACK };
u16 key_cnt[3]={ 0 };
bool keypress_flag[3]={ false, false, false };

struct my_timer {
	struct timer_list timer;
	struct ms_ts_msg20xx_drvdata *tdata;
};
struct my_timer button_up_timer;

#if 0
static struct i2c_dev *i2c_dev_get_by_minor(unsigned index)
{
	struct i2c_dev *i2c_dev;

	spin_lock(&i2c_dev_list_lock);
	list_for_each_entry(i2c_dev, &i2c_dev_list, list) {
		if (i2c_dev->adap->nr == index)
			goto found;
	}
	i2c_dev = NULL;
found:
	spin_unlock(&i2c_dev_list_lock);
	return i2c_dev;
}
#endif 

static struct i2c_dev *get_free_i2c_dev(struct i2c_adapter *adap)
{
	struct i2c_dev *i2c_dev;

	if (adap->nr >= I2C_MINORS) {
		printk(KERN_ERR "i2c-dev: Out of device minors (%d)\n",
		       adap->nr);
		return ERR_PTR(-ENODEV);
	}

	i2c_dev = kzalloc(sizeof(*i2c_dev), GFP_KERNEL);
	if (!i2c_dev)
		return ERR_PTR(-ENOMEM);
	i2c_dev->adap = adap;

	spin_lock(&i2c_dev_list_lock);
	list_add_tail(&i2c_dev->list, &i2c_dev_list);
	spin_unlock(&i2c_dev_list_lock);
	return i2c_dev;
}

static void return_i2c_dev(struct i2c_dev *i2c_dev)
{
	spin_lock(&i2c_dev_list_lock);
	list_del(&i2c_dev->list);
	spin_unlock(&i2c_dev_list_lock);
	kfree(i2c_dev);
}

/*Function as i2c_master_send */
static int i2c_read_bytes(struct i2c_client *client, uint8_t *buf, int len)
{
	struct i2c_msg msgs[2];
	int ret = -1;

	TRACE("i2c_read_bytes\n");
	msgs[0].flags=I2C_M_RD;
	msgs[0].addr=client->addr;
	msgs[0].len=len;
	msgs[0].buf=&buf[0];
	ret = i2c_transfer(client->adapter,msgs, 1);
	return ret;
}

/*Function as i2c_master_send */
static int i2c_write_bytes(struct i2c_client *client,uint8_t *data,int len)
{
	struct i2c_msg msg;
	int ret = -1;

	TRACE("i2c_write_bytes\n");
	msg.flags=!I2C_M_RD;
	msg.addr=client->addr;
	msg.len=len;
	msg.buf=data;
	
	ret = i2c_transfer(client->adapter,&msg, 1);
	return ret;
}

static int checksum(u8 *buf, u8 len)
{
	s32 i=0, sumdata=0;
	if(len<=0)
		return -1;
	for(i=0;i<len-1;i++)
	{
		sumdata +=buf[i];
	}
	if((u8)((-sumdata) & 0xff) !=buf[len-1])
		return -1;
	return 0;
}


static u8  check_keytype(u8 *buf, u8 len)
{
	if(buf[0]!=0x52 || len<8)
		return TOUCH_ERROR;
	if(buf[1]==0xff && buf[2]==0xff && buf[3]==0xff && buf[4]==0x00 && buf[6]==0xff)
	{
		if(buf[5]!=0x00)
			return TOUCHBUTTON_PREES;
		return TOUCH_RELEASE;
	}
	return TOUCHSCREEN_PREES;
}


/*
static void check_button_up_event(unsigned long data)
{
	u8 i,keyvalue;
	DBG("enter %s\n",__FUNCTION__);
	if(button_up_timer.tdata==NULL)
	{
		DBG("button_up_timer.tdata is null\n");
		return;
	}
	keyvalue=button_up_timer.tdata->tsInfo.nKeyValue;
	for(i=0;i<3;i++)
	{
		if(keyvalue&(0x01<<i))
		{
			input_report_key(button_up_timer.tdata->input_key, key_map[i], 1);
			input_report_key(button_up_timer.tdata->input_key, key_map[i], 0);
			input_sync(button_up_timer.tdata->input_key);
			DBG("report %d button\n",i);
		}
	}
	 button_up_timer.tdata->tsInfo.nOldKeyValue=0;
}
*/

static void check_button_up_event(unsigned long data)
{
	keypress_flag[0]=false;
	keypress_flag[1]=false;
	keypress_flag[2]=false;
	key_cnt[0]=0;
	key_cnt[1]=0;
	key_cnt[2]=0;
	button_up_timer.tdata->tsInfo.nOldKeyValue=0;
}

void start_button_up_timer(void)
{	
        mod_timer(&button_up_timer.timer,(jiffies + HZ*10/30));
}

static void mstar_ts_report_data(struct ms_ts_msg20xx_drvdata *tdata)
{
	int i;
	if(tdata->tsInfo.nKeyMode==TOUCHSCREEN_PREES || tdata->tsInfo.nKeyMode==TOUCH_RELEASE)
	{
		for(i=0;i<tdata->tsInfo.nFingerNum;i++)
		{
			if(tdata->tsInfo.nKeyMode==TOUCHSCREEN_PREES)
			{
				input_report_abs(tdata->input, ABS_MT_TOUCH_MAJOR, 1);
			}
			else if(tdata->tsInfo.nKeyMode==TOUCH_RELEASE)
			{
				input_report_abs(tdata->input, ABS_MT_TOUCH_MAJOR, 0);
			}
			input_report_abs(tdata->input, ABS_MT_POSITION_X, (int)tdata->tsInfo.atPoint[i].Row);
			input_report_abs(tdata->input, ABS_MT_POSITION_Y, (int)tdata->tsInfo.atPoint[i].Col);
			input_mt_sync(tdata->input);
		}
		input_sync(tdata->input);
	}
	else if(tdata->tsInfo.nKeyMode==TOUCHBUTTON_PREES)
	{
		start_button_up_timer();
		for(i=0;i<3;i++)
		{
			if(tdata->tsInfo.nKeyValue & (0x01<<i) )
			{
				key_cnt[i]++;
				if((key_cnt[i]>KEY_MAX_CNT) && (keypress_flag[i]==false))
				{
					input_report_key(tdata->input_key, key_map[i], 1);
					input_report_key(tdata->input_key, key_map[i], 0);
					input_sync(tdata->input_key);
					keypress_flag[i]=true;
				}
			}
		}
		tdata->tsInfo.nOldKeyValue=tdata->tsInfo.nKeyValue;
	}
	TRACE("report finish\n");
}

static void mstar_ts_work_func(struct work_struct *work)
{
	struct ms_ts_msg20xx_drvdata *tdata = container_of(work,	struct ms_ts_msg20xx_drvdata,	work);
	u8 buf[8]={0};
	int ret,i;
	u16 pointer_x[2]={0}, pointer_y[2]={0},finger=0,button;
	TRACE("enter %s\n",__FUNCTION__);
	ret=i2c_read_bytes(tdata->client,buf,8);
	if(ret<0)
	{
		DBG("touchscreen: read i2c fail \n");
	}
	else
	{
		TRACE("read data: ");
		for(i=0;i<8;i++)
		{
			TRACE("%x ",buf[i]);
		}
		TRACE("\n");
		if(buf[0]==0x52 && !checksum(buf,8))
		{
			switch(check_keytype(buf,8))
			{
				case TOUCHSCREEN_PREES:
					pointer_x[0]=(((u16)(buf[1] & 0xf0))<<4)|(u16)buf[2];
					pointer_y[0]=(((u16)(buf[1] & 0x0f))<<8)|(u16)buf[3];
					pointer_x[1]=((u16)(buf[4] & 0xf0)<<4)|(u16)buf[5];
					pointer_y[1]=(((u16)buf[4] & 0x0f)<<8)|(u16)buf[6];
					if(pointer_x[1]==0 && pointer_y[1]==0)
						finger=1;
					else
						finger=2;
					pointer_x[1] +=pointer_x[0];
					if(pointer_y[1] & 0x0800)
					{
						pointer_y[1]=(((~(pointer_y[1]&0x7ff)) & 0x7ff) +1);
						pointer_y[1]=pointer_y[0]-pointer_y[1];
					}
					else
						pointer_y[1]+=pointer_y[0];
					pointer_y[0]=MS_TS_MSG20XX_Y_MAX-pointer_y[0];
					pointer_y[1]=MS_TS_MSG20XX_Y_MAX-pointer_y[1];
					tdata->tsInfo.nKeyMode=TOUCHSCREEN_PREES;
					tdata->tsInfo.nFingerNum=finger;
					tdata->tsInfo.atPoint[0].Row=pointer_x[0];
					tdata->tsInfo.atPoint[0].Col=pointer_y[0];
					tdata->tsInfo.atPoint[1].Row=pointer_x[1];
					tdata->tsInfo.atPoint[1].Col=pointer_y[1];
					TRACE("finger: %d,  p1[%d  %d]   p2[%d  %d]\n",finger,pointer_x[0],pointer_y[0],pointer_x[1],pointer_y[1]);
					break;
				case TOUCHBUTTON_PREES:
					tdata->tsInfo.nKeyMode=TOUCHBUTTON_PREES;
					button=buf[5];
					tdata->tsInfo.nKeyValue=button;
					DBG("button: %d\n",button);
					break;
				case TOUCH_RELEASE:
					tdata->tsInfo.nKeyMode=TOUCH_RELEASE;
					TRACE("release\n");
					break;
				case TOUCH_ERROR:
					tdata->tsInfo.nKeyMode=TOUCH_ERROR;
					DBG("error data\n");
					break;
			}
		}	
		mstar_ts_report_data(tdata);
	}
	
	enable_irq(tdata->irq);
}


irqreturn_t ms_ts_msg20xx_isr(int irq, void *dev_id)
{
	struct ms_ts_msg20xx_drvdata *tdata = dev_id;
	TRACE("*** %s ***\n", __FUNCTION__);
	if (at91_get_gpio_value(irq_to_gpio(irq)) == 0 )
	{
		/* IRQ is triggered by FALLING code here */
		disable_irq_nosync(tdata->client->irq);
		queue_work(mstar_wq, &tdata->work);
	}
	return IRQ_HANDLED;
}


static int ms_ts_msg20xx_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct ms_ts_msg20xx_drvdata *tdata;
	struct input_dev *input_dev;
	struct input_dev *input_key;
	struct device *dev;	
	struct i2c_dev *i2c_dev;
	int ret;
	
	TRACE("*** %s ***\n", __FUNCTION__);
	dev_dbg(&client->dev,"Initstall touch driver.\n");
	/* Check I2C function */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) 
	{
		dev_err(&client->dev, "Must have I2C_FUNC_I2C.\n");
		return -ENODEV;
	}

	tdata = kzalloc(sizeof(*tdata) + MAX_TOUCH_FINGER * sizeof(*tdata->tsInfo.atPoint), GFP_KERNEL);
	if (!tdata)
		return -ENOMEM;

	input_dev = input_allocate_device();
	input_key=input_allocate_device();
	if (!input_dev) {
		ret = -ENOMEM;
		pr_err("*** input device allocation failed ***\n");
		goto err1;
	}
	if (!input_key) {
		ret = -ENOMEM;
		pr_err("*** input device allocation failed ***\n");
		goto err1;
	}
	input_dev->name = "ms-msg20xx-1";
	input_dev->dev.parent =& client->dev;
	input_dev->id.bustype = BUS_I2C;
	input_dev->id.vendor = 0x0001;
	input_dev->id.product = 0x0001;
	input_dev->id.version = 0x0100;

	input_key->name="ms-msg20xx-2";
	input_key->dev.parent =& client->dev;
	input_key->id.bustype = BUS_I2C;
	input_key->id.vendor = 0x0001;
	input_key->id.product = 0x0001;
	input_key->id.version = 0x0100;

	tdata->input = input_dev;
	tdata->input_key=input_key;
	tdata->extint_pin = irq_to_gpio(client->irq);
	tdata->irq=client->irq;
	tdata->addr=client->addr;
	tdata->client=client;

    /* report the absolute axis event only */
	set_bit(EV_ABS, input_dev->evbit);
	set_bit(ABS_MT_POSITION_X, input_dev->absbit);   	 
	set_bit(ABS_MT_POSITION_Y, input_dev->absbit);   
	set_bit(ABS_MT_TOUCH_MAJOR, input_dev->absbit);   	 
	set_bit(ABS_MT_WIDTH_MAJOR, input_dev->absbit);
	input_set_abs_params(input_dev, ABS_X, MS_TS_MSG20XX_X_MIN, MS_TS_MSG20XX_X_MAX, 0, 0);	
	input_set_abs_params(input_dev, ABS_Y, MS_TS_MSG20XX_Y_MIN, MS_TS_MSG20XX_Y_MAX, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_X, MS_TS_MSG20XX_X_MIN,MS_TS_MSG20XX_X_MAX, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, MS_TS_MSG20XX_Y_MIN,MS_TS_MSG20XX_Y_MAX, 0, 0);
	/* just be used to indicate the finger-release */
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_drvdata(input_dev, tdata);

	set_bit(EV_KEY, input_key->evbit);
	set_bit(key_map[0], input_key->keybit);
	set_bit(key_map[1],input_key->keybit);	
	set_bit(key_map[2], input_key->keybit);
	input_key->keycodesize = 3;	
	input_key->keycodemax = 3;	
	input_set_drvdata(input_key, tdata);

	ret = input_register_device(input_dev);
	if (ret < 0)
	{
		input_free_device(input_dev);
		pr_err("*** Unable to register ms-touchscreen input device ***\n");
		goto err1;
	}

	ret = input_register_device(input_key);
	if (ret < 0)
	{
		input_free_device(input_key);
		pr_err("*** Unable to register ms-touchscreen input device ***\n");
		goto err1;
	}
	/* request the external interrupt gpio pin */
	/* set the pin as input */
	if(at91_set_gpio_input(tdata->extint_pin, 0))
	{
		DBG(KERN_DEBUG "Could not set PA27 for GPIO input. \n");
		goto err2;
	}
	if(at91_set_deglitch(tdata->extint_pin,1))
	{
		DBG(KERN_DEBUG "Could not set PA27 for GPIO input. \n");
		goto err2;
	}

	INIT_WORK(&tdata->work, mstar_ts_work_func);
	init_timer(&button_up_timer.timer);
	button_up_timer.timer.function=check_button_up_event;
	button_up_timer.tdata=tdata;
	
	/* request an irq and register the isr */
	ret = request_irq(tdata->irq, ms_ts_msg20xx_isr,IRQF_TRIGGER_FALLING|IRQF_TRIGGER_RISING,"ms_msg20xx", tdata);	    
	if (ret) 
	{
		pr_err("*** ms-ts-msg20xx: Unable to claim irq %d; error %d ***\n",	tdata->irq, ret);
		goto err2;
	}

	DBG("*** ms touchscreen registered ***\n");
	i2c_dev = get_free_i2c_dev(client->adapter);	
	if (IS_ERR(i2c_dev))
	{		
		ret = PTR_ERR(i2c_dev);	
		goto err3;
	}

	dev = device_create(i2c_dev_class,&client->adapter->dev, MKDEV(I2C_MAJOR, client->adapter->nr), NULL, "msg20-%d", client->adapter->nr);	
	if (IS_ERR(dev))
	{
		ret = PTR_ERR(dev);
		DBG("create msg20 dev error\n");
		goto err3;
	}
	return 0;
err3:
	free_irq(tdata->irq, input_dev);
err2:
	input_unregister_device(input_dev);
err1:
	kfree(tdata);
	return ret;
}

static int __devexit ms_ts_msg20xx_remove(struct platform_device *pdev)
{
	struct ms_ts_msg20xx_drvdata *tdata = platform_get_drvdata(pdev);
	struct input_dev *input_dev = tdata->input;
	struct input_dev *input_key = tdata->input_key;
	struct i2c_dev *i2c_dev;
	int ret=0;

	free_irq(gpio_to_irq(tdata->extint_pin), input_dev);
	gpio_free(tdata->extint_pin);
	i2c_dev = get_free_i2c_dev(tdata->client->adapter);
	if (IS_ERR(i2c_dev))
	{
		ret= PTR_ERR(i2c_dev);
	}
	else
	{
		return_i2c_dev(i2c_dev);
		device_destroy(i2c_dev_class, MKDEV(I2C_MAJOR, tdata->client->adapter->nr));
	}
	input_unregister_device(input_dev);
	input_unregister_device(input_key);
	kfree(tdata);
	return 0;
}

static const struct i2c_device_id ms_ts_msg20xx_i2c_id[] = {
	{ MS_TS_I2C_DRIVER_NAME, 0 }, { }
};

static struct i2c_driver ms_ts_msg20xx_driver =
{
	.id_table	= ms_ts_msg20xx_i2c_id,
	.probe		= ms_ts_msg20xx_probe,
	.remove		= __devexit_p(ms_ts_msg20xx_remove),
	.driver		= {
				.name	= MS_TS_I2C_DRIVER_NAME,
				.owner =THIS_MODULE,
			},
};

static ssize_t mstar_write(struct file *file, char __user *buf,size_t count, loff_t *ppos)
{
	struct i2c_client *client;
	char *tmp;
	static int ret=0;
	client = file->private_data;

	tmp = kmalloc(count,GFP_KERNEL);
	if (tmp==NULL)
		return -ENOMEM;
	if (copy_from_user(tmp,buf,count))
	{
		DBG("copy_from_user error\n");
		kfree(tmp);
		return -EFAULT;
	}
	ret = i2c_master_send(client,tmp,count);
	mdelay(100);
	if(ret!=count)	
	{			
		printk("Unable to write to i2c page for TP!\n");
	}
	kfree(tmp);
	return ret;
}

static int mstar_open(struct inode *inode, struct file *file)
{
	file->private_data = button_up_timer.tdata->client;
	return 0;
}

static int mstar_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}

static const struct file_operations mstar_i2c_ts_fops ={
	.owner = THIS_MODULE,
	.write = mstar_write,
	.open = mstar_open,
	.release = mstar_release,
};

static int __init ms_ts_msg20xx_init(void)
{
	int ret;
	DBG("MStar touchscreen driver init\n");
	mstar_wq = create_workqueue("mstar_wq");
	if (!mstar_wq) 
	{
		DBG(KERN_ALERT "creat workqueue faiked\n");
		return -ENOMEM;
		
	}
	
	ret = register_chrdev(I2C_MAJOR,"msg20",&mstar_i2c_ts_fops);
	if(ret)
	{
		printk(KERN_ERR "%s:register chrdev failed\n",__FILE__);
		return ret;
	}

	i2c_dev_class = class_create(THIS_MODULE, "mstar_i2c_dev");
	if (IS_ERR(i2c_dev_class))
	{		
		ret = PTR_ERR(i2c_dev_class);
		class_destroy(i2c_dev_class);
	}
	
	ret= i2c_add_driver(&ms_ts_msg20xx_driver);
	if(ret)
	{
		DBG("MStar touchscreen driver init failed\n");
	}
	return ret;
}

static void __exit ms_ts_msg20xx_exit(void)
{
	i2c_del_driver(&ms_ts_msg20xx_driver);
	class_destroy(i2c_dev_class);
	unregister_chrdev(I2C_MAJOR,"msg20");
	if (mstar_wq)
		destroy_workqueue(mstar_wq);
}

module_init(ms_ts_msg20xx_init);
module_exit(ms_ts_msg20xx_exit);
MODULE_LICENSE("GPL");
