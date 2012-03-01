/* 
 * drivers/input/touchscreen/ft5x0x_ts.c
 *
 * FocalTech ft5x0x TouchScreen driver. 
 *
 * Copyright (c) 2010  Focal tech Ltd.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * VERSION      	DATE			AUTHOR
 *    1.0		  2010-01-05			WenFS
 *
 * note: only support mulititouch	Wenfs 2010-10-01
 */


#undef DEBUG

#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/earlysuspend.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <mach/gpio.h>

#include "ft5x06_ts.h"

//#define BABBAGE_TP_RESET (2*32 + 13) /*GPIO_3_13*/
#define BABBAGE_TP_RESET (3*32+25)	//modify for v1.2 by yangzhiwen at 20111025
#define BABBAGE_TP_ENABLE (2*32 + 13)
static struct i2c_client *this_client;
//static struct ft5x0x_ts_platform_data *pdata;

#define CONFIG_FT5X0X_MULTITOUCH 1
#define FT5X0X_PRINT_DEBUG_MSG 0
struct ts_event {
	u16	x1;
	u16	y1;
	u16	x2;
	u16	y2;
	u16	x3;
	u16	y3;
	u16	x4;
	u16	y4;
	u16	x5;
	u16	y5;
	u16	pressure;
    u8  touch_point;
};

struct ft5x0x_ts_data {
	struct input_dev	*input_dev;
	struct input_dev	*input_dev_key;
	struct input_dev	*input_media_key;
	struct ts_event		event;
	struct work_struct 	pen_event_work;
	struct workqueue_struct *ts_workqueue;
	struct early_suspend	early_suspend;
};

/***********************************************************************************************
Name	:	ft5x0x_i2c_rxdata 

Input	:	*rxdata
                     *length

Output	:	ret

function	:	

***********************************************************************************************/
static int ft5x0x_i2c_rxdata(char *rxdata, int length)
{
	int ret;

	struct i2c_msg msgs[] = {
		{
			.addr	= this_client->addr,
			.flags	= 0,
			.len	= 1,
			.buf	= rxdata,
		},
		{
			.addr	= this_client->addr,
			.flags	= I2C_M_RD,
			.len	= length,
			.buf	= rxdata,
		},
	};

    //msleep(1);
	ret = i2c_transfer(this_client->adapter, msgs, 2);
	if (ret < 0)
		pr_err("msg %s i2c read error: %d\n", __func__, ret);
	
	return ret;
}
/***********************************************************************************************
Name	:	 

Input	:	
                     

Output	:	

function	:	

***********************************************************************************************/
static int ft5x0x_i2c_txdata(char *txdata, int length)
{
	int ret;

	struct i2c_msg msg[] = {
		{
			.addr	= this_client->addr,
			.flags	= 0,
			.len	= length,
			.buf	= txdata,
		},
	};

   	//msleep(1);
	ret = i2c_transfer(this_client->adapter, msg, 1);
	if (ret < 0)
		pr_err("%s i2c write error: %d\n", __func__, ret);

	return ret;
}
/***********************************************************************************************
Name	:	 ft5x0x_write_reg

Input	:	addr -- address
                     para -- parameter

Output	:	

function	:	write register of ft5x0x

***********************************************************************************************/
#ifdef FT5X06TS_WRITE_REGS
static int ft5x0x_write_reg(u8 addr, u8 para)
{
    u8 buf[3];
    int ret = -1;

    buf[0] = addr;
    buf[1] = para;
    ret = ft5x0x_i2c_txdata(buf, 2);
    if (ret < 0) {
        pr_err("write reg failed! %#x ret: %d", buf[0], ret);
        return -1;
    }
    
    return 0;
}
#endif

/***********************************************************************************************
Name	:	ft5x0x_read_reg 

Input	:	addr
                     pdata

Output	:	

function	:	read register of ft5x0x

***********************************************************************************************/
static int ft5x0x_read_reg(u8 addr, u8 *pdata)
{
	int ret;
	u8 buf[2] = {0};
	struct i2c_msg msgs[] = {
		{
			.addr	= this_client->addr,
			.flags	= 0,
			.len	= 1,
			.buf	= buf,
		},
		{
			.addr	= this_client->addr,
			.flags	= I2C_M_RD,
			.len	= 1,
			.buf	= buf,
		},
	};
	buf[0] = addr;
	ret = i2c_transfer(this_client->adapter, msgs, 2);
	if (ret < 0)
		pr_err("msg %s i2c read error: %d\n", __func__, ret);

	*pdata = buf[0];
	return ret;
  
}


/***********************************************************************************************
Name	:	 ft5x0x_read_fw_ver

Input	:	 void
                     

Output	:	 firmware version 	

function	:	 read TP firmware version

***********************************************************************************************/
static unsigned char ft5x0x_read_fw_ver(void)
{
	unsigned char ver;
	ft5x0x_read_reg(FT5X0X_REG_FIRMID, &ver);
	return(ver);
}


#define CONFIG_SUPPORT_FTS_CTP_UPG


#ifdef CONFIG_SUPPORT_FTS_CTP_UPG

typedef enum
{
    ERR_OK,
    ERR_MODE,
    ERR_READID,
    ERR_ERASE,
    ERR_STATUS,
    ERR_ECC,
    ERR_DL_ERASE_FAIL,
    ERR_DL_PROGRAM_FAIL,
    ERR_DL_VERIFY_FAIL
}E_UPGRADE_ERR_TYPE;

typedef unsigned char         FTS_BYTE;     //8 bit
typedef unsigned short        FTS_WORD;    //16 bit
typedef unsigned int          FTS_DWRD;    //16 bit
typedef unsigned char         FTS_BOOL;    //8 bit

#define FTS_NULL                0x0
#define FTS_TRUE                0x01
#define FTS_FALSE              0x0

#define I2C_CTPM_ADDRESS       0xFF //no reference!


void delay_qt_ms(unsigned long  w_ms)
{
    unsigned long i;
    unsigned long j;

    for (i = 0; i < w_ms; i++)
    {
        for (j = 0; j < 1000; j++)
        {
            udelay(1);
        }
    }
}


/*
[function]: 
    callback: read data from ctpm by i2c interface,implemented by special user;
[parameters]:
    bt_ctpm_addr[in]    :the address of the ctpm;
    pbt_buf[out]        :data buffer;
    dw_lenth[in]        :the length of the data buffer;
[return]:
    FTS_TRUE     :success;
    FTS_FALSE    :fail;
*/
FTS_BOOL i2c_read_interface(FTS_BYTE bt_ctpm_addr, FTS_BYTE* pbt_buf, FTS_DWRD dw_lenth)
{
    int ret;
    
    ret=i2c_master_recv(this_client, pbt_buf, dw_lenth);

    if(ret<=0)
    {
        printk("[TSP]i2c_read_interface error\n");
        return FTS_FALSE;
    }
  
    return FTS_TRUE;
}

/*
[function]: 
    callback: write data to ctpm by i2c interface,implemented by special user;
[parameters]:
    bt_ctpm_addr[in]    :the address of the ctpm;
    pbt_buf[in]        :data buffer;
    dw_lenth[in]        :the length of the data buffer;
[return]:
    FTS_TRUE     :success;
    FTS_FALSE    :fail;
*/
FTS_BOOL i2c_write_interface(FTS_BYTE bt_ctpm_addr, FTS_BYTE* pbt_buf, FTS_DWRD dw_lenth)
{
    int ret;
    ret=i2c_master_send(this_client, pbt_buf, dw_lenth);
    if(ret<=0)
    {
        printk("[TSP]i2c_write_interface error line = %d, ret = %d\n", __LINE__, ret);
        return FTS_FALSE;
    }

    return FTS_TRUE;
}

/*
[function]: 
    send a command to ctpm.
[parameters]:
    btcmd[in]        :command code;
    btPara1[in]    :parameter 1;    
    btPara2[in]    :parameter 2;    
    btPara3[in]    :parameter 3;    
    num[in]        :the valid input parameter numbers, if only command code needed and no parameters followed,then the num is 1;    
[return]:
    FTS_TRUE    :success;
    FTS_FALSE    :io fail;
*/
FTS_BOOL cmd_write(FTS_BYTE btcmd,FTS_BYTE btPara1,FTS_BYTE btPara2,FTS_BYTE btPara3,FTS_BYTE num)
{
    FTS_BYTE write_cmd[4] = {0};

    write_cmd[0] = btcmd;
    write_cmd[1] = btPara1;
    write_cmd[2] = btPara2;
    write_cmd[3] = btPara3;
    return i2c_write_interface(I2C_CTPM_ADDRESS, write_cmd, num);
}

/*
[function]: 
    write data to ctpm , the destination address is 0.
[parameters]:
    pbt_buf[in]    :point to data buffer;
    bt_len[in]        :the data numbers;    
[return]:
    FTS_TRUE    :success;
    FTS_FALSE    :io fail;
*/
FTS_BOOL byte_write(FTS_BYTE* pbt_buf, FTS_DWRD dw_len)
{
    
    return i2c_write_interface(I2C_CTPM_ADDRESS, pbt_buf, dw_len);
}

/*
[function]: 
    read out data from ctpm,the destination address is 0.
[parameters]:
    pbt_buf[out]    :point to data buffer;
    bt_len[in]        :the data numbers;    
[return]:
    FTS_TRUE    :success;
    FTS_FALSE    :io fail;
*/
FTS_BOOL byte_read(FTS_BYTE* pbt_buf, FTS_BYTE bt_len)
{
    return i2c_read_interface(I2C_CTPM_ADDRESS, pbt_buf, bt_len);
}


/*
[function]: 
    burn the FW to ctpm.
[parameters]:(ref. SPEC)
    pbt_buf[in]    :point to Head+FW ;
    dw_lenth[in]:the length of the FW + 6(the Head length);    
    bt_ecc[in]    :the ECC of the FW
[return]:
    ERR_OK        :no error;
    ERR_MODE    :fail to switch to UPDATE mode;
    ERR_READID    :read id fail;
    ERR_ERASE    :erase chip fail;
    ERR_STATUS    :status error;
    ERR_ECC        :ecc error.
*/


#define    FTS_PACKET_LENGTH        128

static unsigned char CTPM_FW[]=
{
#define FT_UPDATE_FIRMWARE_FILENAME "ft070101_app_1118.i"
};

E_UPGRADE_ERR_TYPE  fts_ctpm_fw_upgrade(FTS_BYTE* pbt_buf, FTS_DWRD dw_lenth)
{
    FTS_BYTE reg_val[2] = {0};
    FTS_DWRD i = 0;

    FTS_DWRD  packet_number;
    FTS_DWRD  j;
    FTS_DWRD  temp;
    FTS_DWRD  lenght;
    FTS_BYTE  packet_buf[FTS_PACKET_LENGTH + 6];
    FTS_BYTE  auc_i2c_write_buf[10];
    FTS_BYTE bt_ecc;
    int      i_ret;

    /*********Step 1:Reset  CTPM *****/
	gpio_set_value(BABBAGE_TP_RESET,0);
    /*write 0xaa to register 0xfc*/
//    ft5x0x_write_reg(0xfc,0xaa);
    delay_qt_ms(50);
     /*write 0x55 to register 0xfc*/
//    ft5x0x_write_reg(0xfc,0x55);
	gpio_set_value(BABBAGE_TP_RESET,1);
    printk("[TSP] Step 1: Reset CTPM test\n");
   
    delay_qt_ms(30);   


    /*********Step 2:Enter upgrade mode *****/
    auc_i2c_write_buf[0] = 0x55;
    auc_i2c_write_buf[1] = 0xaa;
    do
    {
        i ++;
        i_ret = ft5x0x_i2c_txdata(auc_i2c_write_buf, 2);
        delay_qt_ms(5);
    }while(i_ret <= 0 && i < 5 );

    /*********Step 3:check READ-ID***********************/        
    cmd_write(0x90,0x00,0x00,0x00,4);
    byte_read(reg_val,2);
    if (reg_val[0] == 0x79 && reg_val[1] == 0x3)
    {
        printk("[TSP] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",reg_val[0],reg_val[1]);
    }
    else
    {
        return ERR_READID;
        //i_is_new_protocol = 1;
    }

     /*********Step 4:erase app*******************************/
    cmd_write(0x61,0x00,0x00,0x00,1);
   
    delay_qt_ms(1500);
    printk("[TSP] Step 4: erase. \n");

    /*********Step 5:write firmware(FW) to ctpm flash*********/
    bt_ecc = 0;
    printk("[TSP] Step 5: start upgrade. \n");
    dw_lenth = dw_lenth - 8;
    packet_number = (dw_lenth) / FTS_PACKET_LENGTH;
    packet_buf[0] = 0xbf;
    packet_buf[1] = 0x00;
    for (j=0;j<packet_number;j++)
    {
        temp = j * FTS_PACKET_LENGTH;
        packet_buf[2] = (FTS_BYTE)(temp>>8);
        packet_buf[3] = (FTS_BYTE)temp;
        lenght = FTS_PACKET_LENGTH;
        packet_buf[4] = (FTS_BYTE)(lenght>>8);
        packet_buf[5] = (FTS_BYTE)lenght;

        for (i=0;i<FTS_PACKET_LENGTH;i++)
        {
            packet_buf[6+i] = pbt_buf[j*FTS_PACKET_LENGTH + i]; 
            bt_ecc ^= packet_buf[6+i];
        }
        
        byte_write(&packet_buf[0],FTS_PACKET_LENGTH + 6);
        delay_qt_ms(FTS_PACKET_LENGTH/6 + 1);
        if ((j * FTS_PACKET_LENGTH % 1024) == 0)
        {
              printk("[TSP] upgrade the 0x%x th byte.\n", ((unsigned int)j) * FTS_PACKET_LENGTH);
        }
    }

    if ((dw_lenth) % FTS_PACKET_LENGTH > 0)
    {
        temp = packet_number * FTS_PACKET_LENGTH;
        packet_buf[2] = (FTS_BYTE)(temp>>8);
        packet_buf[3] = (FTS_BYTE)temp;

        temp = (dw_lenth) % FTS_PACKET_LENGTH;
        packet_buf[4] = (FTS_BYTE)(temp>>8);
        packet_buf[5] = (FTS_BYTE)temp;

        for (i=0;i<temp;i++)
        {
            packet_buf[6+i] = pbt_buf[ packet_number*FTS_PACKET_LENGTH + i]; 
            bt_ecc ^= packet_buf[6+i];
        }

        byte_write(&packet_buf[0],temp+6);    
        delay_qt_ms(20);
    }

    //send the last six byte
    for (i = 0; i<6; i++)
    {
        temp = 0x6ffa + i;
        packet_buf[2] = (FTS_BYTE)(temp>>8);
        packet_buf[3] = (FTS_BYTE)temp;
        temp =1;
        packet_buf[4] = (FTS_BYTE)(temp>>8);
        packet_buf[5] = (FTS_BYTE)temp;
        packet_buf[6] = pbt_buf[ dw_lenth + i]; 
        bt_ecc ^= packet_buf[6];

        byte_write(&packet_buf[0],7);  
        delay_qt_ms(20);
    }

    /*********Step 6: read out checksum***********************/
    /*send the opration head*/
    cmd_write(0xcc,0x00,0x00,0x00,1);
    byte_read(reg_val,1);
    printk("[TSP] Step 6:  ecc read 0x%x, new firmware 0x%x. \n", reg_val[0], bt_ecc);
    if(reg_val[0] != bt_ecc)
    {
        return ERR_ECC;
    }

    /*********Step 7: reset the new FW***********************/
    cmd_write(0x07,0x00,0x00,0x00,1);

    return ERR_OK;
}


int fts_ctpm_fw_upgrade_with_i_file(void)
{
   FTS_BYTE*     pbt_buf = FTS_NULL;
   int i_ret;

	printk("[FILE]%s\n",FT_UPDATE_FIRMWARE_FILENAME);
    //=========FW upgrade========================*/
   pbt_buf = CTPM_FW;
   /*call the upgrade function*/
   i_ret =  fts_ctpm_fw_upgrade(pbt_buf,sizeof(CTPM_FW));
   if (i_ret != 0)
   {
       //error handling ...
       //TBD
       printk("upgrade error\n");
   }

   return i_ret;
}

unsigned char fts_ctpm_get_upg_ver(void)
{
    unsigned int ui_sz;
    ui_sz = sizeof(CTPM_FW);
    if (ui_sz > 2)
    {
        return CTPM_FW[ui_sz - 2];
    }
    else
    {
        //TBD, error handling?
        return 0xff; //default value
    }
}

#endif


/***********************************************************************************************
Name	:	 

Input	:	
                     

Output	:	

function	:	

***********************************************************************************************/
static void ft5x0x_ts_release(void)
{
	struct ft5x0x_ts_data *data = NULL;

	data = i2c_get_clientdata(this_client);
	if ((data == NULL) || (data->input_dev == NULL))
		return;
#ifdef CONFIG_FT5X0X_MULTITOUCH	
	input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, 0);
#else
	input_report_abs(data->input_dev, ABS_PRESSURE, 0);
	input_report_key(data->input_dev, BTN_TOUCH, 0);
#endif
	input_sync(data->input_dev);
}

/*add by ljc for softkey--2012-01-17*/
struct my_timer{
	struct timer_list timer;
	struct ft5x0x_ts_data *tdata;
} ;
struct my_timer ft_button_up_timer;

static int report_key_value(int index);
static void check_button_up_event(unsigned long data)
{
	report_key_value(-1); // -1 means that one key have release
}

void ft_start_button_up_timer(void)
{	
        mod_timer(&ft_button_up_timer.timer,(jiffies + HZ*10/40));
}


typedef struct
{
	int x;
	int y;
	u8 cnt;
	bool key_press;
	int key_value;
}key_value_t;

#define MENU 139
#define HOME 102
#define BACK 158
#define SEARCH 217
#define PAUSE_PLAY 201

#if MAX_BUTTON_NUMS==THREE_BUTTONS
static  key_value_t button[]={
	{.x=1120, .y=703, .cnt=0, .key_press=false, .key_value=MENU}, //menu
	{.x=1184, .y=703, .cnt=0, .key_press=false, .key_value=HOME},//home
	{.x=1279, .y=703, .cnt=0, .key_press=false, .key_value=BACK},//back
};
#elif MAX_BUTTON_NUMS==FIVE_BUTTONS
static  key_value_t button[]={
	{.x=1120, .y=672, .cnt=0, .key_press=false, .key_value=HOME}, //home
	{.x=1120, .y=767, .cnt=0, .key_press=false, .key_value=MENU},//menu
	{.x=1184, .y=672, .cnt=0, .key_press=false, .key_value=BACK},//back
	{.x=1184, .y=767, .cnt=0, .key_press=false, .key_value= PAUSE_PLAY},//pause_play
	{.x=1279, .y=767, .cnt=0, .key_press=false, .key_value= SEARCH},//search
};
#endif
static int report_key_value(int index)
{
	int i=0;
	struct ft5x0x_ts_data *data = NULL;
	data = i2c_get_clientdata(this_client);
	
	if ((data == NULL) || (data->input_dev_key == NULL) || (data->input_media_key == NULL))
	    return 0;
	
	if(index>=0 && index<sizeof(button)/sizeof(button[0]))
	{
		button[index].cnt++;
		if(button[index].cnt>4)
		{
			if(!button[index].key_press)
			{
				button[index].key_press=true;
				if(button[index].key_value!=PAUSE_PLAY)
				{
					input_report_key(data->input_dev_key, button[index].key_value, 1);
					if(button[index].key_value!=HOME)
						input_report_key(data->input_dev_key, button[index].key_value, 0);
				}
				else
				{
					input_report_key(data->input_media_key, button[index].key_value, 1);
					input_report_key(data->input_media_key, button[index].key_value, 0);
				}
					
			}
			button[index].cnt=4;
		}
		for(i=0;i<sizeof(button)/sizeof(button[0]);i++)
		{
			if(i!=index)
			{
				button[i].cnt=0;
				button[i].key_press=false;
			}
		}
	}
	else if(index==-1) // -1 means that one key have release
	{
		for(i=0;i<sizeof(button)/sizeof(button[0]);i++)
		{
			if(button[i].key_press)
			{
				if(button[i].key_value!=PAUSE_PLAY) //PANIC: button[index].key_value
				{
					input_report_key(data->input_dev_key, button[i].key_value, 0);
					input_sync(data->input_dev_key);
				}
				else
				{
					input_report_key(data->input_media_key, button[i].key_value, 0);
					input_sync(data->input_media_key);
				}
			}
			button[i].cnt=0;
			button[i].key_press=false;
		}
	}
	return 0;
}
/*add the end ----2012-01-17*/

/*
static void ft5x0x_exchange_x_y(u16 *pX, u16 *pY)
{
	int	postion_temp = *pX;
	 *pX = *pY;
	if(postion_temp<10)
		postion_temp=25;
	*pY = SCREEN_MAX_Y-postion_temp;
	
}
*/

static int ft5x0x_getbutton_index( key_value_t key_value)
{
	int i;
	for(i=0;i<sizeof(button)/sizeof(button[0]);i++)
	{
		if(abs(key_value.x - button[i].x)<10 && abs(key_value.y - button[i].y)<10)
		{
			return i;
		}
	}
	return -1;
}

static int ft5x0x_read_data(void)
{
	struct ft5x0x_ts_data *data = NULL; 
	struct ts_event *event = NULL;
//	u8 buf[14] = {0};
	u8 buf[32] = {0};
	int ret = -1;
	key_value_t key_value;
	int button_index = -1;

	data =  i2c_get_clientdata(this_client);
	if (data == NULL) {
	    dev_dbg(&this_client->dev, "Invalid i2c client data\n");
	    return 0;
	}
	
	event = &data->event;
	
	if (event == NULL) {
	    dev_dbg(&this_client->dev, "No event interface !");
	    return 0;
	}

#ifdef CONFIG_FT5X0X_MULTITOUCH
//	ret = ft5x0x_i2c_rxdata(buf, 13);
	ret = ft5x0x_i2c_rxdata(buf, 31);
#else
	ret = ft5x0x_i2c_rxdata(buf, 7);

#endif
	

    
	if (ret < 0) 
	{
		pr_err("%s read_data i2c_rxdata failed: %d\n", __func__, ret);
		return ret;
	}
	
	dev_dbg(&this_client->dev, "%s: Received data from i2c_rxdata\n", __func__);

	memset(event, 0, sizeof(struct ts_event));
//	event->touch_point = buf[2] & 0x03;// 0000 0011
	event->touch_point = buf[2] & 0x07;// 000 0111
	dev_dbg(&this_client->dev, "Handling touch_point event: %d\n", event->touch_point);
	
	if (event->touch_point == 1)
	{
		key_value.x=(s16)(buf[3] & 0x0F)<<8 | (s16)buf[4];
		key_value.y=(s16)(buf[5] & 0x0F)<<8 | (s16)buf[6];
	
		dev_dbg(&this_client->dev, "single touch(%d, %d)\n", key_value.x, key_value.y);
	
		if (key_value.x > 1070 && key_value.y > 610)
		{
			button_index = ft5x0x_getbutton_index(key_value);
			dev_dbg(&this_client->dev, "button_index: %d\n", button_index);
			if(button_index != -1)
			{
				report_key_value(button_index);
				event->touch_point = 0;
				ft_start_button_up_timer();
				dev_vdbg(&this_client->dev, "Started button up timer for button: %d\n", button_index);
			}
		}
	}

	if (event->touch_point == 0) 
	{
		ft5x0x_ts_release();
		return 1; 
	}

#ifdef CONFIG_FT5X0X_MULTITOUCH
    switch (event->touch_point) {
		case 5:
			event->x5 = (s16)(buf[0x1b] & 0x0F)<<8 | (s16)buf[0x1c];
			event->y5 = (s16)(buf[0x1d] & 0x0F)<<8 | (s16)buf[0x1e];
			if(event->x5>1070 || event->y5>610)
				return -1;
			//ft5x0x_exchange_x_y(&event->x5, &event->y5);
		case 4:
			event->x4 = (s16)(buf[0x15] & 0x0F)<<8 | (s16)buf[0x16];
			event->y4 = (s16)(buf[0x17] & 0x0F)<<8 | (s16)buf[0x18];
			if(event->x4>1070 || event->y4>610)
				return -1;
			//ft5x0x_exchange_x_y(&event->x4, &event->y4);
		case 3:
			event->x3 = (s16)(buf[0x0f] & 0x0F)<<8 | (s16)buf[0x10];
			event->y3 = (s16)(buf[0x11] & 0x0F)<<8 | (s16)buf[0x12];
			if(event->x3>1070 || event->y3>610)
				return -1;
			//ft5x0x_exchange_x_y(&event->x3, &event->y3);
		case 2:
			event->x2 = (s16)(buf[9] & 0x0F)<<8 | (s16)buf[10];
			event->y2 = (s16)(buf[11] & 0x0F)<<8 | (s16)buf[12];
			if(event->x2>1070 || event->y2>610)
				return -1;
			//ft5x0x_exchange_x_y(&event->x2, &event->y2);
		case 1:
			event->x1 = (s16)(buf[3] & 0x0F)<<8 | (s16)buf[4];
			event->y1 = (s16)(buf[5] & 0x0F)<<8 | (s16)buf[6];
			if(event->x1 > 1070 || event->y1 > 610)
				return -1;
			#if FT5X0X_PRINT_DEBUG_MSG
			printk("orig(x1,y1):(%d,%d)\n",event->x1,event->y1);
			#endif
			//ft5x0x_exchange_x_y(&event->x1, &event->y1);
			#if FT5X0X_PRINT_DEBUG_MSG
			//printk("new(x1,y1):(%d,%d)\n",event->x1,event->y1);
			#endif
			if (event->x1>SCREEN_MAX_X)	
				event->x1=SCREEN_MAX_X;
            break;
		default:
		{
			dev_vdbg(&this_client->dev, "Unknown touch_point\n");
			return -1;
		}
	}
#else
    if (event->touch_point == 1) {
	event->x1 = (s16)(buf[3] & 0x0F)<<8 | (s16)buf[4];
	event->y1 = (s16)(buf[5] & 0x0F)<<8 | (s16)buf[6];
    }
#endif
	event->pressure = 250;
	dev_dbg(&this_client->dev, "%s: Handled read_data\n", __func__);
	#if FT5X0X_PRINT_DEBUG_MSG
	dev_dbg(&this_client->dev, "%s: 1:%d %d 2:%d %d \n", __func__,
		event->x1, event->y1, event->x2, event->y2);
	//printk("%d (%d, %d), (%d, %d)\n", event->touch_point, event->x1, event->y1, event->x2, event->y2);
	#endif
	
    return 0;
}
/***********************************************************************************************
Name	:	 

Input	:	
                     

Output	:	

function	:	

***********************************************************************************************/
static void ft5x0x_report_value(void)
{
	struct ft5x0x_ts_data *data = i2c_get_clientdata(this_client);
	struct ts_event *event = &data->event;
	//u8 uVersion;

	dev_dbg(&this_client->dev, "==ft5x0x_report_value =\n");
#ifdef CONFIG_FT5X0X_MULTITOUCH
	switch(event->touch_point) {
		case 5:
			input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, event->pressure);
			input_report_abs(data->input_dev, ABS_MT_POSITION_X, event->x5);
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y, event->y5);
			input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR, 1);
			input_mt_sync(data->input_dev);
			//printk("===x2 = %d,y2 = %d ====\n",event->x2,event->y2);
		case 4:
			input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, event->pressure);
			input_report_abs(data->input_dev, ABS_MT_POSITION_X, event->x4);
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y, event->y4);
			input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR, 1);
			input_mt_sync(data->input_dev);
			//printk("===x2 = %d,y2 = %d ====\n",event->x2,event->y2);
		case 3:
			input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, event->pressure);
			input_report_abs(data->input_dev, ABS_MT_POSITION_X, event->x3);
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y, event->y3);
			input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR, 1);
			input_mt_sync(data->input_dev);
			//printk("===x2 = %d,y2 = %d ====\n",event->x2,event->y2);
		case 2:
			input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, event->pressure);
			input_report_abs(data->input_dev, ABS_MT_POSITION_X, event->x2);
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y, event->y2);
			input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR, 1);
			input_mt_sync(data->input_dev);
			//printk("===x2 = %d,y2 = %d ====\n",event->x2,event->y2);
		case 1:
			input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, event->pressure);
			input_report_abs(data->input_dev, ABS_MT_POSITION_X, event->x1);
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y, event->y1);
			input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR, 1);
			input_mt_sync(data->input_dev);
			//printk("===x1 = %d,y1 = %d ====\n",event->x1,event->y1);

		default:
			//printk("==touch_point default =\n");
			break;
	}
#else	/* CONFIG_FT5X0X_MULTITOUCH*/
	if (event->touch_point == 1) {
		input_report_abs(data->input_dev, ABS_X, event->x1);
		input_report_abs(data->input_dev, ABS_Y, event->y1);
		input_report_abs(data->input_dev, ABS_PRESSURE, event->pressure);
	}
	input_report_key(data->input_dev, BTN_TOUCH, 1);
#endif	/* CONFIG_FT5X0X_MULTITOUCH*/
	input_sync(data->input_dev);

	dev_dbg(&this_client->dev, "%s: 1:%d %d 2:%d %d \n", __func__,
		event->x1, event->y1, event->x2, event->y2);
}	/*end ft5x0x_report_value*/
/***********************************************************************************************
Name	:	 

Input	:	
                     

Output	:	

function	:	

***********************************************************************************************/
static void ft5x0x_ts_pen_irq_work(struct work_struct *work)
{
	int ret = -1;
	#if FT5X0X_PRINT_DEBUG_MSG
	dev_vdbg(&this_client->dev, "=====Workqueue=====\n");
	#endif
	ret = ft5x0x_read_data();
	
	if (ret == 0) {
		ft5x0x_report_value();
	}
	else {
	    dev_vdbg(&this_client->dev, "data package read error\n");
	}
	enable_irq(this_client->irq);
}
/***********************************************************************************************
Name	:	 

Input	:	
                     

Output	:	

function	:	

***********************************************************************************************/
static irqreturn_t ft5x0x_ts_interrupt(int irq, void *dev_id)
{
	struct ft5x0x_ts_data *ft5x0x_ts = dev_id;
//    	disable_irq(this_client->irq);		
	disable_irq_nosync(this_client->irq);
//	printk("==int=\n");
	if (!work_pending(&ft5x0x_ts->pen_event_work)) {
		queue_work(ft5x0x_ts->ts_workqueue, &ft5x0x_ts->pen_event_work);
	}

	return IRQ_HANDLED;
}
#ifdef CONFIG_HAS_EARLYSUSPEND
/***********************************************************************************************
Name	:	 

Input	:	
                     

Output	:	

function	:	

***********************************************************************************************/
static void ft5x0x_ts_suspend(struct early_suspend *handler)
{
	struct ft5x0x_ts_data *ts;
	ts =  container_of(handler, struct ft5x0x_ts_data, early_suspend);

	dev_dbg(&this_client->dev, "==ft5x0x_ts_suspend=\n");	
}
/***********************************************************************************************
Name	:	 

Input	:	
                     

Output	:	

function	:	

***********************************************************************************************/
static void ft5x0x_ts_resume(struct early_suspend *handler)
{
	/*u8 uc_reg_value;*/
	dev_dbg(&this_client->dev, "==ft5x0x_ts_resume=\n");
}
#endif  //CONFIG_HAS_EARLYSUSPEND
/***********************************************************************************************
Name	:	 

Input	:	
                     

Output	:	

function	:	

***********************************************************************************************/
static int ft5x0x_detect_tp(struct i2c_client *client)
{
	int ret;
	struct i2c_msg msg[1];
	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 0;
	msg[0].buf = 0;
	ret = i2c_transfer(client->adapter, msg, 1);
	if(ret < 0)
	{
		msleep(50);
		ret = i2c_transfer(client->adapter, msg, 1);
	}
	return ret;
}


static int 
ft5x0x_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct ft5x0x_ts_data *ft5x0x_ts;
	struct input_dev *input_dev, *input_dev_key, *input_media_key;
	int err = 0;
	int gpio_irq;
	int ret;
	int i;
	unsigned char uc_reg_value; 
	
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}

	ret=ft5x0x_detect_tp(client);
	if(ret<0)
	{
		printk("ft5x0x_tp not found\n");
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}
	
	ft5x0x_ts = kzalloc(sizeof(*ft5x0x_ts), GFP_KERNEL);
	if (!ft5x0x_ts)	{
		err = -ENOMEM;
		goto exit_alloc_data_failed;
	}

	i2c_set_clientdata(client, ft5x0x_ts);
	this_client = client;
	printk(KERN_DEBUG "ft5x0x I2C Address: 0x%02X\n",client->addr);

	INIT_WORK(&ft5x0x_ts->pen_event_work, ft5x0x_ts_pen_irq_work);

	ft5x0x_ts->ts_workqueue = create_singlethread_workqueue(dev_name(&client->dev));
	if (!ft5x0x_ts->ts_workqueue) {
		err = -ESRCH;
		goto exit_create_singlethread;
	}


   	
//	pdata = client->dev.platform_data;
//	if (pdata == NULL) {
//		dev_err(&client->dev, "%s: platform data is null\n", __func__);
//		goto exit_platform_data_null;
//	}
	
//	printk("==request_irq=\n");
//	err = request_irq(client->irq, ft5x0x_ts_interrupt, IRQF_DISABLED, "ft5x0x_ts", ft5x0x_ts);

	gpio_irq = irq_to_gpio(client->irq);
	//ret = gpio_request(gpio_irq, "gpio_irq");	// GPIO_xxx for interrupt mode
	
       /* set the pin as input */
    	if(at91_set_gpio_input(gpio_irq, 0))
	{
		printk(KERN_DEBUG "Could not set PA27 for GPIO input. \n");
		//goto err2;
	}
	if(at91_set_deglitch(gpio_irq,1))
	{
		printk(KERN_DEBUG "Could not set PA27 for GPIO input. \n");
		//goto err2;
	}
	
/*	gpio_direction_input(gpio_irq);
	if (ret < 0)
	{
		printk(KERN_ERR " Failed to gpio_request\n");
		printk(KERN_ERR "Fail : gpio_request was called before this driver call\n");
	}*/
	
	err = request_irq(client->irq, ft5x0x_ts_interrupt, IRQF_TRIGGER_FALLING|IRQF_TRIGGER_RISING, "ft5x0x_ts", ft5x0x_ts);
	if (err < 0) {
		dev_err(&client->dev, "ft5x0x_probe: request irq failed\n");
		goto exit_irq_request_failed;
	}

	disable_irq(client->irq);

//	printk("==input_allocate_device=\n");
	input_dev = input_allocate_device();
	if (!input_dev) {
		err = -ENOMEM;
		dev_err(&client->dev, "failed to allocate input device\n");
		goto exit_input_dev_alloc_failed;
	}
	
	ft5x0x_ts->input_dev = input_dev;
	/*add by ljc for softkey----2011-09-29*/
	input_dev_key = input_allocate_device();
	if (!input_dev_key) {
		err = -ENOMEM;
		dev_err(&client->dev, "failed to allocate input device\n");
		goto exit_input_register_device_failed;
	}
	
	ft5x0x_ts->input_dev_key = input_dev_key;
	/*add the end----2011-09-29*/

	/*add by ljc for media_key----2012-01-17*/
	input_media_key = input_allocate_device();
	if (!input_media_key) {
		err = -ENOMEM;
		dev_err(&client->dev, "failed to allocate input device\n");
		goto exit_input_key_register_device_failed;
	}
	
	ft5x0x_ts->input_media_key = input_media_key;
	/*add the end----2012-01-17*/

#ifdef CONFIG_FT5X0X_MULTITOUCH
	set_bit(ABS_MT_TOUCH_MAJOR, input_dev->absbit);
	set_bit(ABS_MT_POSITION_X, input_dev->absbit);
	set_bit(ABS_MT_POSITION_Y, input_dev->absbit);
	set_bit(ABS_MT_WIDTH_MAJOR, input_dev->absbit);

	input_set_abs_params(input_dev,
			     ABS_MT_POSITION_X, SCREEN_MIN_X, SCREEN_MAX_X, 0, 0);
	input_set_abs_params(input_dev,
			     ABS_MT_POSITION_Y, SCREEN_MIN_Y, SCREEN_MAX_Y, 0, 0);
	input_set_abs_params(input_dev,
			     ABS_MT_TOUCH_MAJOR, 0, PRESS_MAX, 0, 0);
	input_set_abs_params(input_dev,
			     ABS_MT_WIDTH_MAJOR, 0, 20, 0, 0);
#else
	set_bit(ABS_X, input_dev->absbit);
	set_bit(ABS_Y, input_dev->absbit);
	set_bit(ABS_PRESSURE, input_dev->absbit);
	set_bit(BTN_TOUCH, input_dev->keybit);

	input_set_abs_params(input_dev, ABS_X, SCREEN_MIN_X, SCREEN_MAX_X, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, SCREEN_MIN_Y, SCREEN_MAX_Y, 0, 0);
	input_set_abs_params(input_dev,
			     ABS_PRESSURE, 0, PRESS_MAX, 0 , 0);
#endif

	set_bit(EV_ABS, input_dev->evbit);
	//set_bit(EV_KEY, input_dev->evbit);

	input_dev->name		= FT5X0X_NAME;		//dev_name(&client->dev)
	err = input_register_device(input_dev);
	if (err) {
		dev_err(&client->dev,
		"ft5x0x_ts_probe: failed to register input device: %s\n",
		dev_name(&client->dev));
		goto exit_input_key_register_device_failed;
	}

	/*add by ljc for softkey----2012-01-17*/
	ft5x0x_ts->input_dev_key->name = "ft5x0x_ts_key";
	ft5x0x_ts->input_dev_key->keycodesize = sizeof(button)/sizeof(button[0]);
	ft5x0x_ts->input_dev_key->keycodemax = sizeof(button)/sizeof(button[0]);	
	ft5x0x_ts->input_dev_key->id.bustype = BUS_I2C;
	ret = input_register_device(ft5x0x_ts->input_dev_key);
	if (ret)
	{
		printk(KERN_ERR "ft5x0x_ts_probe: Unable to register input device\n");
		goto exit_input_key_register_device_failed;
	}
	set_bit(EV_KEY, ft5x0x_ts->input_dev_key->evbit);
	for(i=0;i<sizeof(button)/sizeof(button[0]);i++)
	{
		set_bit(button[i].key_value, ft5x0x_ts->input_dev_key->keybit);
	}
	/*add the end----2012-01-17*/

	/*add by ljc for media_key----2012-01-17*/
	ft5x0x_ts->input_media_key->name = "AVRCP";
	ft5x0x_ts->input_media_key->keycodesize = 1;
	ft5x0x_ts->input_media_key->keycodemax = 1;	
	ft5x0x_ts->input_media_key->id.bustype = BUS_I2C;
	ret = input_register_device(ft5x0x_ts->input_media_key);
	if (ret)
	{
		printk(KERN_ERR "ft5x0x_ts_probe: Unable to register input device\n");
		goto exit_input_media_register_device_failed;
	}
	set_bit(EV_KEY, ft5x0x_ts->input_media_key->evbit);
	set_bit(PAUSE_PLAY, ft5x0x_ts->input_media_key->keybit);
	/*add the end----2012-01-17*/

#ifdef CONFIG_HAS_EARLYSUSPEND
	printk("ft5x06_ts: registered early suspend\n");
	ft5x0x_ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ft5x0x_ts->early_suspend.suspend = ft5x0x_ts_suspend;
	ft5x0x_ts->early_suspend.resume	= ft5x0x_ts_resume;
	register_early_suspend(&ft5x0x_ts->early_suspend);
#endif

    msleep(50);
    //get some register information
    uc_reg_value = ft5x0x_read_fw_ver();
    printk("%s: Firmware version = 0x%x\n", "ft5x06_ts", uc_reg_value);

//    fts_ctpm_fw_upgrade_with_i_file();


    
//wake the CTPM
//	__gpio_as_output(GPIO_FT5X0X_WAKE);		
//	__gpio_clear_pin(GPIO_FT5X0X_WAKE);		//set wake = 0,base on system
//	 msleep(100);
//	__gpio_set_pin(GPIO_FT5X0X_WAKE);			//set wake = 1,base on system
//	msleep(100);
//	ft5x0x_set_reg(0x88, 0x05); //5, 6,7,8
//	ft5x0x_set_reg(0x80, 30);
//	msleep(50);
//    	enable_irq(this_client->irq);


	init_timer(&ft_button_up_timer.timer);
	ft_button_up_timer.timer.function=check_button_up_event;
	ft_button_up_timer.tdata=ft5x0x_ts;


	enable_irq(this_client->irq);

	return 0;
	
exit_input_media_register_device_failed:
	input_free_device(input_media_key);
exit_input_key_register_device_failed:
	input_free_device(input_dev_key);
exit_input_register_device_failed:
	input_free_device(input_dev);
exit_input_dev_alloc_failed:
//	free_irq(client->irq, ft5x0x_ts);
	free_irq(client->irq, ft5x0x_ts);
exit_irq_request_failed:
//exit_platform_data_null:
	cancel_work_sync(&ft5x0x_ts->pen_event_work);
	destroy_workqueue(ft5x0x_ts->ts_workqueue);
exit_create_singlethread:
	printk("==singlethread error =\n");
	i2c_set_clientdata(client, NULL);
	kfree(ft5x0x_ts);
exit_alloc_data_failed:
exit_check_functionality_failed:
	return err;
}
/***********************************************************************************************
Name	:	 

Input	:	
                     

Output	:	

function	:	

***********************************************************************************************/
static int __devexit ft5x0x_ts_remove(struct i2c_client *client)
{
	struct ft5x0x_ts_data *ft5x0x_ts = i2c_get_clientdata(client);
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&ft5x0x_ts->early_suspend);
#endif
//	free_irq(client->irq, ft5x0x_ts);
	free_irq(client->irq, ft5x0x_ts);
	input_unregister_device(ft5x0x_ts->input_media_key);
	input_unregister_device(ft5x0x_ts->input_dev);
	input_unregister_device(ft5x0x_ts->input_dev_key);
	kfree(ft5x0x_ts);
	cancel_work_sync(&ft5x0x_ts->pen_event_work);
	destroy_workqueue(ft5x0x_ts->ts_workqueue);
	i2c_set_clientdata(client, NULL);
	return 0;
}

static const struct i2c_device_id ft5x0x_ts_id[] = {
	{ FT5X0X_NAME, 0 },{ }
};


MODULE_DEVICE_TABLE(i2c, ft5x0x_ts_id);

static struct i2c_driver ft5x0x_ts_driver = {
	.probe		= ft5x0x_ts_probe,
	.remove		= __devexit_p(ft5x0x_ts_remove),
	.id_table	= ft5x0x_ts_id,
	.driver	= {
		.name	= FT5X0X_NAME,
		.owner	= THIS_MODULE,
	},
};

/***********************************************************************************************
Name	:	 

Input	:	
                     

Output	:	

function	:	

***********************************************************************************************/
static int __init ft5x0x_ts_init(void)
{
	int ret;
	ret = i2c_add_driver(&ft5x0x_ts_driver);
	return ret;
}

/***********************************************************************************************
Name	:	 

Input	:	
                     

Output	:	

function	:	

***********************************************************************************************/
static void __exit ft5x0x_ts_exit(void)
{
	i2c_del_driver(&ft5x0x_ts_driver);
}

module_init(ft5x0x_ts_init);
module_exit(ft5x0x_ts_exit);

MODULE_AUTHOR("<wenfs@Focaltech-systems.com>");
MODULE_DESCRIPTION("FocalTech ft5x0x TouchScreen driver");
MODULE_LICENSE("GPL");
