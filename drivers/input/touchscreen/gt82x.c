/*
 * 
 * Copyright (C) 2011 Goodix, Inc.
 * 
 * Author: Scott
 * Date: 2012.01.05
 */
 

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/io.h>
//#include <mach/gpio.h>
//#include <plat/gpio-cfg.h>
#include <linux/irq.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/of_gpio.h>
#include "gt82x.h"

#define READ_TOUCH_ADDR_H   0x0F
#define READ_TOUCH_ADDR_L   0x40
#define READ_KEY_ADDR_H     0x0F
#define READ_KEY_ADDR_L     0x41
#define READ_COOR_ADDR_H    0x0F
#define READ_COOR_ADDR_L    0x42
#define RESOLUTION_LOC      71
#define TRIGGER_LOC         66

#define GOODIX_I2C_NAME "gt82x_ts"

static struct workqueue_struct *goodix_wq;
static const char *goodix_ts_name = "Goodix Capacitive TouchScreen";

static s32 goodix_ts_remove(struct i2c_client *);

#ifdef CONFIG_HAS_EARLYSUSPEND
static void goodix_ts_early_suspend(struct early_suspend *h);
static void goodix_ts_late_resume(struct early_suspend *h);
#endif

#ifdef CREATE_WR_NODE
extern s32 init_wr_node(struct i2c_client*);
extern void uninit_wr_node(void);
#endif

#ifdef AUTO_UPDATE_GUITAR
extern s32 init_update_proc(struct goodix_ts_data *);
#else
static void guitar_reset(s32);
#endif

//*************************Touchkey Surpport Part*****************************
//#define HAVE_TOUCH_KEY
#ifdef HAVE_TOUCH_KEY
    const u16 touch_key_array[]={
                                      KEY_MENU,             //MENU
                                      KEY_BACK,             //BACK
                                      KEY_HOME,             //HOME
                                      KEY_SEARCH,           //SEARCH
                                     }; 
    #define MAX_KEY_NUM     (sizeof(touch_key_array)/sizeof(touch_key_array[0]))
#endif

struct goodix_i2c_rmi_platform_data {
    uint32_t version;    /* Use this entry for panels with */
    //reservation
};

#if 1
//#define TOUCH_MAX_HEIGHT     7680
//#define TOUCH_MAX_WIDTH      5120
#define TOUCH_MAX_HEIGHT     600
#define TOUCH_MAX_WIDTH      1024
#else
#define AUTO_SET
u16 TOUCH_MAX_HEIGHT;
u16 TOUCH_MAX_WIDTH;
#endif


/*******************************************************	
*********************************************************/
/*Function as i2c_master_send */
static s32 i2c_read_bytes(struct i2c_client *client, u8 *buf, s32 len)
{
    struct i2c_msg msgs[2];
    s32 ret=-1;

    //发送写地址
    msgs[0].flags=!I2C_M_RD; //写消息
    msgs[0].addr=client->addr;
    msgs[0].len=2;
    msgs[0].buf=&buf[0];
    //接收数据
    msgs[1].flags=I2C_M_RD;//读消息
    msgs[1].addr=client->addr;
    msgs[1].len=len - ADDR_LENGTH;
    msgs[1].buf=&buf[2];

    ret=i2c_transfer(client->adapter,msgs, 2);

    return ret;
}

/*******************************************************	
*******************************************************/
/*Function as i2c_master_send */
static s32 i2c_write_bytes(struct i2c_client *client,u8 *data,s32 len)
{
    struct i2c_msg msg;
    s32 ret=-1;
    
    //发送设备地址
    msg.flags=!I2C_M_RD;//写消息
    msg.addr=client->addr;
    msg.len=len;
    msg.buf=data;        

    ret=i2c_transfer(client->adapter,&msg, 1);

    return ret;
}

/*******************************************************
*******************************************************/
static s32 i2c_pre_cmd(struct goodix_ts_data *ts)
{
    s32 ret;
    u8 pre_cmd_data[2]={0x0f, 0xff};

    ret=i2c_write_bytes(ts->client,pre_cmd_data,2);
    return ret;//*/
}

/*******************************************************
*******************************************************/
static s32 i2c_end_cmd(struct goodix_ts_data *ts)
{
    s32 ret;
    u8 end_cmd_data[2]={0x80, 0x00};    

    ret=i2c_write_bytes(ts->client,end_cmd_data,2);
    return ret;//*/
}

/*******************************************************
*******************************************************/
s32 goodix_init_panel(struct goodix_ts_data *ts, u8 send)
{
    s32 ret = -1;
    u8 config_info[]=
    {
//        0x0F,0x80,/*config address*/
        0x30,0x64,/*config address*/
#if 0       
/*F80*/ 0x00,0x0F,0x01,0x10,0x02,0x11,0x03,0x12,
        0x04,0x13,0x05,0x14,0x06,0x15,0x07,0x16,
/*F90*/ 0x08,0x17,0x09,0x18,0x0A,0x19,0x0B,0x1A,
        0x0C,0x1B,0xFF,0x1C,0x0E,0x1D,0x00,0x0A,
/*FA0*/ 0x01,0x0B,0x02,0x0C,0x03,0x0D,0x04,0x0E,
        0x05,0x0F,0x06,0x10,0x07,0x11,0xFF,0x12,
/*FB0*/ 0x09,0x13,0x0F,0x03,0x80,0x88,0x90,0x10,
        0x00,0x00,0x06,0x00,0x00,0x0E,0x48,0x34,
/*FC0*/ 0x4C,0x03,0x01,0x05,0x00,TOUCH_MAX_WIDTH>>8,TOUCH_MAX_WIDTH&0xff,TOUCH_MAX_HEIGHT>>8,
        TOUCH_MAX_HEIGHT&0xff,0x5A,0x5A,0x46,0x46,0x00,0x00,0x05,
/*FD0*/ 0x19,0x05,0x14,0x10,0x00,0x04,0x00,0x0f,
        0x10,0x30,0x50,0x70,0x38,0x28,0x60,0x20,
/*FE0*/ 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01
 #endif 
        0x16,0x08,0x17,0x09,0x18,0x0A,0x19,0x0B,
        0x1A,0x0C,0x1B,0x0D,0x1C,0x15,0x06,0x14,
        0x05,0x13,0x04,0x12,0x03,0x11,0x02,0x10,
        0x01,0x0F,0x00,0x07,0xFF,0x0E,0x02,0x0C,
        0x03,0x0D,0x04,0x0E,0x05,0x0F,0x06,0x10,
        0x07,0x11,0x08,0x12,0x09,0x13,0xFF,0x12,
        0x09,0x33,0x0F,0x03,0x80,0x88,0x90,0x14,
        0x00,0x00,0x00,0x00,0x00,0x06,0x48,0x34,
        0x05,0x03,0x00,0x05,0x00,0x14,0x00,0x20,
        0x00,0x5A,0x5A,0x46,0x46,0x00,0x00,0x13,
        0x19,0x05,0x14,0x10,0x00,0x04,0x00,0x00,
        0x70,0x78,0x80,0x00,0x32,0x28,0x30,0x20,
        0x00,0x00,0x00,0x00,0x0D,0x40,0x40,0x00,
        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01
    };
#ifdef AUTO_SET
    TOUCH_MAX_WIDTH  = ((config_info[RESOLUTION_LOC] << 8)|config_info[RESOLUTION_LOC + 1]);
    TOUCH_MAX_HEIGHT = ((config_info[RESOLUTION_LOC + 2] << 8)|config_info[RESOLUTION_LOC + 3]);

    DEBUG_MSG("TOUCH_MAX_WIDTH  : %d\n", (s32)TOUCH_MAX_WIDTH);
    DEBUG_MSG("TOUCH_MAX_HEIGHT : %d\n", (s32)TOUCH_MAX_HEIGHT);
#else
    config_info[RESOLUTION_LOC]     = TOUCH_MAX_WIDTH >> 8;
    config_info[RESOLUTION_LOC + 1] = TOUCH_MAX_WIDTH & 0xff;
    config_info[RESOLUTION_LOC + 2] = TOUCH_MAX_HEIGHT >> 8;
    config_info[RESOLUTION_LOC + 3] = TOUCH_MAX_HEIGHT & 0xff;
#endif
    DEBUG_MSG("TOUCH_MAX_WIDTH  : %d\n", (s32)TOUCH_MAX_WIDTH);
    DEBUG_MSG("TOUCH_MAX_HEIGHT : %d\n", (s32)TOUCH_MAX_HEIGHT);

    if (INT_TRIGGER == GT_IRQ_FALLING)
    {
        config_info[TRIGGER_LOC] &= 0xf7; 
    }
    else if (INT_TRIGGER == GT_IRQ_RISING)
    {
        config_info[TRIGGER_LOC] |= 0x08;
    }

    if (send)
    {
        ret=i2c_write_bytes(ts->client,config_info, (sizeof(config_info)/sizeof(config_info[0])));
        if (ret <= 0)
        {
            return fail;
        }
        i2c_end_cmd(ts);
        msleep(10);
    }
    return success;
}

static s32 touch_num(u8 value, s32 max)
{
    s32 tmp = 0;

    while((tmp < max) && value)
    {
        if ((value & 0x01) == 1)
        {
            tmp++;
        }
        value = value >> 1;
    }

    return tmp;
}

/*******************************************************	
********************************************************/
static void goodix_ts_work_func(struct work_struct *work)
{
    u8 finger = 0;
    u8 chk_sum = 0;
    u8 key = 0;
    static u8 last_key = 0;
    u16 X_value;
    u16 Y_value;
    u32 count = 0;
    u32 position = 0;
    s32 ret = -1;
    s32 tmp = 0;
    s32 i;
    u8 *coor_point;
    u8 touch_data[2 + 2 + 5*MAX_FINGER_NUM + 1] = {READ_TOUCH_ADDR_H,READ_TOUCH_ADDR_L,0, 0};
    static u8 finger_last[MAX_FINGER_NUM+1]={0};        //上次触摸按键的手指索引
    u8 finger_current[MAX_FINGER_NUM+1] = {0};        //当前触摸按键的手指索引

    struct goodix_ts_data *ts = container_of(work, struct goodix_ts_data, work);
    
#ifndef INT_PORT
COORDINATE_POLL:
#endif
    if( tmp > 9)
    {
        dev_info(&(ts->client->dev), "Because of transfer error,touchscreen stop working.\n");
        goto XFER_ERROR ;
    }

    //建议将数据一次性读取完
    ret=i2c_read_bytes(ts->client, touch_data,sizeof(touch_data)/sizeof(touch_data[0])); 
    i2c_end_cmd(ts);
    if(ret <= 0) 
    {
        dev_err(&(ts->client->dev),"I2C transfer error. Number:%d\n ", ret);
        ts->bad_data = 1;
        tmp ++;
#ifndef INT_PORT
        goto COORDINATE_POLL;
#else
        goto XFER_ERROR;
#endif
    }

    if(ts->bad_data)
    {
        //TODO:Is sending config once again (to reset the chip) useful?    
        ts->bad_data = 0;
        msleep(20);
    }

    if((touch_data[2]&0xC0)!=0x80)
    {
        goto DATA_NO_READY;        
    }

    key = touch_data[3]&0x0f; // 1, 2, 4, 8
    if (key == 0x0f)
    {
        if (fail == goodix_init_panel(ts, 1))
        {
/**/        DEBUG_COOR("Reload config failed!\n");
        }
        else
        {   
            DEBUG_COOR("Reload config successfully!\n");
        }
        goto XFER_ERROR;
    }

    finger = (u8)touch_num(touch_data[2]&0x1f, MAX_FINGER_NUM);

/**/DEBUG_COOR("touch num:%x\n", finger);

    for (i = 1;i < MAX_FINGER_NUM + 1; i++)        
    {
        finger_current[i] = !!(touch_data[2] & (0x01<<(i-1)));
    }

#ifdef DEBUG_COORD
/**/for (i = 0; i < finger*5+4; i++)
/**/{  
/**/    DEBUG_COOR("%5x", touch_data[i]);
/**/}
/**/DEBUG_COOR("\n");
#endif 

    //检验校验和    
    coor_point = &touch_data[4];
    chk_sum = 0;
    for ( i = 0; i < 5*finger; i++)
    {
        chk_sum += coor_point[i];
/**/    DEBUG_COOR("%5x", coor_point[i]);
    }
/**/DEBUG_COOR("\ncheck sum:%x\n", chk_sum);
/**/DEBUG_COOR("check sum byte:%x\n", coor_point[5*finger]);
    if (chk_sum != coor_point[5*finger])
    {
        goto XFER_ERROR;
    }

    //发送坐标//
    if (finger)
    {
        for(i = 0, position=1;position < MAX_FINGER_NUM+1; position++)
        {
            if(finger_current[position])
            {     
                X_value = coor_point[i] << 8;
                X_value = X_value | coor_point[i + 1];

                Y_value = coor_point[i + 2] << 8;
                Y_value = Y_value | coor_point[i + 3];
                
                input_report_key(ts->input_dev, BTN_TOUCH, 1);
                input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, position - 1);
                input_report_abs(ts->input_dev, ABS_MT_POSITION_X, Y_value);  //can change x-y!!!
                input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, X_value);
                input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR,15);
                input_mt_sync(ts->input_dev);
                i += 5;

    /**/        DEBUG_COOR("X:%d\n", (s32)X_value);
    /**/        DEBUG_COOR("Y:%d\n", (s32)Y_value);
            }
        }
    }
    else
    {
        input_report_key(ts->input_dev, BTN_TOUCH, 0);
        input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
        input_mt_sync(ts->input_dev);
    }

#ifdef HAVE_TOUCH_KEY
#ifdef DEBUG_COORD
/**/for (i = 0; i < 4; i++)
/**/{
/**/    DEBUG_COOR("key:%4x   ", !!(key&(0x01<<i)));
/**/}
/**/DEBUG_COOR("\n");
#endif

    if((last_key != 0) || (key != 0))
    {
        for(count = 0; count < MAX_KEY_NUM; count++)
        {
            input_report_key(ts->input_dev, touch_key_array[count], !!(key&(0x01<<count)));    
        }
    }
    last_key = key;
#endif

    input_sync(ts->input_dev);
    
    for(position=1;position<MAX_FINGER_NUM+1; position++)
    {
        finger_last[position] = finger_current[position];
    }

DATA_NO_READY:
XFER_ERROR:
    if(ts->irq_is_disable == 1)
    {
        ts->irq_is_disable = 0;
        enable_irq(ts->client->irq);
    }
}

/*******************************************************	
********************************************************/
static enum hrtimer_restart goodix_ts_timer_func(struct hrtimer *timer)
{
    struct goodix_ts_data *ts = container_of(timer, struct goodix_ts_data, timer);

    queue_work(goodix_wq, &ts->work);
    hrtimer_start(&ts->timer, ktime_set(0, (POLL_TIME+6)*1000000), HRTIMER_MODE_REL);

    return HRTIMER_NORESTART;
}

/*******************************************************	
********************************************************/
static irqreturn_t goodix_ts_irq_handler(s32 irq, void *dev_id)
{
    struct goodix_ts_data *ts = (struct goodix_ts_data*)dev_id;
printk("%s:%s:%d\n",__FILE__,__FUNCTION__,__LINE__);

    if (ts->use_irq)
    {
        if (!ts->irq_is_disable)
        {
            disable_irq_nosync(ts->client->irq);
            ts->irq_is_disable = 1;
        }
        
        queue_work(goodix_wq, &ts->work);
    }
    return IRQ_HANDLED;
}

/*******************************************************	
********************************************************/
//#if defined(INT_PORT)
static s32 goodix_ts_power(struct goodix_ts_data * ts, s32 on)
{
    s32 ret = -1;

    u8 i2c_control_buf[3] = {0x0f,0xf2,0xc0};        //suspend cmd

    if(ts == NULL || !ts->use_irq)
        return -2;

    switch(on)
    {
    case 0:
        ret = i2c_write_bytes(ts->client, i2c_control_buf, 3);
        return ret;

    case 1:
//johncao	
/*	
        GPIO_DIRECTION_OUTPUT(INT_PORT, 0);
        GPIO_SET_VALUE(INT_PORT, 0);
        msleep(5);
        GPIO_SET_VALUE(INT_PORT, 1);
        msleep(5);
        GPIO_DIRECTION_INPUT(INT_PORT);
        GPIO_PULL_UPDOWN(INT_PORT, 0);
        GPIO_CFG_PIN(INT_PORT, INT_CFG);

        msleep(10);
*/	
        return success;

    default:
        DEBUG_MSG(KERN_DEBUG "%s: Cant't support this command.", goodix_ts_name);
        return -EINVAL;
    }

}

static s32 init_input_dev(struct goodix_ts_data *ts)
{
    s32 i;
    s32 ret = 0;

    ts->input_dev = input_allocate_device();
    if (ts->input_dev == NULL)
    {
        dev_dbg(&ts->client->dev,"goodix_ts_probe: Failed to allocate input device\n");
        return fail;
    }

    ts->input_dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
    ts->input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
    ts->input_dev->absbit[0] = BIT(ABS_X) | BIT(ABS_Y) | BIT(ABS_PRESSURE);// absolute coor (x,y)

#ifdef HAVE_TOUCH_KEY
    for(i = 0; i < MAX_KEY_NUM; i++)
    {
        input_set_capability(ts->input_dev, EV_KEY, touch_key_array[i]);
    }
#endif

    if (fail==goodix_init_panel(ts, 0))
    {
	    printk("*****************goodix_init_panel init  failed\n");
	    return -1;
    }
    
#ifdef GOODIX_MULTI_TOUCH
    input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);
    input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
    input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, TOUCH_MAX_HEIGHT, 0, 0);
    input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, TOUCH_MAX_WIDTH, 0, 0);
    input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID, 0, 5, 0, 0);
#else
    input_set_abs_params(ts->input_dev, ABS_X, 0, TOUCH_MAX_HEIGHT, 0, 0);
    input_set_abs_params(ts->input_dev, ABS_Y, 0, TOUCH_MAX_WIDTH, 0, 0);
    input_set_abs_params(ts->input_dev, ABS_PRESSURE, 0, 255, 0, 0);
#endif    

    memcpy(ts->phys, "input/ts", 8);
    ts->input_dev->name = goodix_ts_name;
    ts->input_dev->phys = ts->phys;
    ts->input_dev->id.bustype = BUS_I2C;
    ts->input_dev->id.vendor = 0xDEAD;
    ts->input_dev->id.product = 0xBEEF;
    ts->input_dev->id.version = 10427;    //screen firmware version

    ret = input_register_device(ts->input_dev);
    if (ret) 
    {
        dev_err(&ts->client->dev,"Probe: Unable to register %s input device\n", ts->input_dev->name);
        input_free_device(ts->input_dev);
        return fail;
    }
    DEBUG_MSG("Register input device successfully!\n");

    return success;
}

static s32 set_pins(struct goodix_ts_data *ts)
{
    s32 ret = -1;
//johncao
/*

    ts->client->irq=TS_INT;        //If not defined in client
    if (ts->client->irq)
    {
        ret = GPIO_REQUEST(INT_PORT, "TS_INT");    //Request IO
        if (ret < 0) 
        {
            dev_err(&ts->client->dev, "Failed to request GPIO:%d, ERRNO:%d\n",(s32)INT_PORT,ret);
            goto err_gpio_request_failed;
        }
        DEBUG_MSG("Request int port successfully!\n");
        
        GPIO_DIRECTION_INPUT(INT_PORT);
        GPIO_PULL_UPDOWN(INT_PORT, 0);
        GPIO_CFG_PIN(INT_PORT, INT_CFG);        //Set IO port function    

        ret = request_irq(ts->client->irq, goodix_ts_irq_handler, INT_TRIGGER,
                          ts->client->name, ts);
        if (ret != 0) 
        {
            dev_err(&ts->client->dev,"Cannot allocate ts INT!ERRNO:%d\n", ret);
            GPIO_DIRECTION_INPUT(INT_PORT);
            GPIO_FREE(INT_PORT);
            goto err_gpio_request_failed;
        }
        else 
        {
            disable_irq(ts->client->irq);
            ts->use_irq = 1;
            ts->irq_is_disable = 1;
            dev_dbg(&ts->client->dev, "Reques EIRQ %d successed on GPIO:%d\n", TS_INT, INT_PORT);
        }
    }

err_gpio_request_failed:
    if (!ts->use_irq) 
    {
        hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
        ts->timer.function = goodix_ts_timer_func;
        hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
        DEBUG_MSG("Use timer!\n");
    }

    ret = GPIO_REQUEST(RESET_PORT, "TS_RESET");    //Request IO
    if (ret < 0) 
    {
        dev_err(&ts->client->dev, "Failed to request GPIO:%d, ERRNO:%d\n",(s32)RESET_PORT,ret);
    }
    else
    {
        ts->use_reset = 1;
        GPIO_DIRECTION_INPUT(RESET_PORT);
        GPIO_PULL_UPDOWN(RESET_PORT, 0);
    }

    dev_info(&ts->client->dev,"Start %s in %s mode\n", 
              ts->input_dev->name, ts->use_irq ? "interrupt" : "polling");
*/
    ret = request_irq(ts->client->irq, goodix_ts_irq_handler, INT_TRIGGER,
                         ts->client->name, ts);
    return ts->use_irq;
}

#define READ_VERSION
#ifdef READ_VERSION
static s32 goodix_ts_version(struct goodix_ts_data *ts)
{
    u8 buf[8];

    buf[0] = 0x0f;
    buf[1] = 0x7d;
    
    i2c_read_bytes(ts->client, buf, 5);
    i2c_end_cmd(ts);

    NOTICE("PID:%02x, VID:%02x%02x\n", buf[2], buf[3], buf[4]);

    return success;
}
#endif
static int gt82x_power_on(void)
{

}


/*******************************************************	
********************************************************/
static s32 goodix_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    s32 ret = 0;
    s32 retry=0;
    struct device_node *np = client->dev.of_node;
    struct goodix_ts_data *ts = NULL;
    struct goodix_i2c_rmi_platform_data *pdata = NULL;
    int gpio_int,gpio_rst;
	if (!np)
	{
		
printk("%s:%s:%d\n",__FILE__,__FUNCTION__,__LINE__);
		return -ENODEV;
	}
#if 1
	gpio_int = of_get_named_gpio(np, "gpio_int", 0);
printk("*******************gpio_int:%d\n",gpio_int);
	if (!gpio_is_valid(gpio_int))
	{
printk("%s:%s:%d\n",__FILE__,__FUNCTION__,__LINE__);
		return -ENODEV;
	}
#endif	
	gpio_rst = of_get_named_gpio(np, "gpio_rst", 0);
printk("*******************gpio_rst:%d\n",gpio_rst);
	if (!gpio_is_valid(gpio_rst))
	{
printk("%s:%s:%d\n",__FILE__,__FUNCTION__,__LINE__);
		return -ENODEV;
	}
#if 0
	ret = gpio_request_one(gpio_int, GPIOF_OUT_INIT_LOW,"gpio_int");
	if (ret < 0) 
	{
printk("%s:%s:%d\n",__FILE__,__FUNCTION__,__LINE__);
	  return ret;
	}
	gpio_set_value(gpio_int, 0);
	msleep(5);
	gpio_set_value(gpio_int, 1);
	msleep(5);
	gpio_set_value(gpio_int, 0);
	msleep(5);
	gpio_direction_input(gpio_int);

#endif
	ret = gpio_request_one(gpio_rst, GPIOF_OUT_INIT_LOW,"gpio_rst");
	
	if (ret < 0) 
	{
printk("%s:%s:%d\n",__FILE__,__FUNCTION__,__LINE__);
//	  return ret;
	}
//	gpio_direction_output(gpio_rst, 0);
	gpio_set_value(gpio_rst, 0);
	msleep(500);

printk("%s:%s:%d\n",__FILE__,__FUNCTION__,__LINE__);
    NOTICE("Start to install Goodix Capacitive TouchScreen driver.\n");
    NOTICE("*DRIVER INFORMATION\n");
    NOTICE("**RELEASE DATE:%s.\n", RELEASE_DATE);
    NOTICE("**COMPILE TIME:%s, %s.\n", __DATE__, __TIME__);

    //Check I2C function
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) 
    {
        dev_err(&client->dev, "Must have I2C_FUNC_I2C.\n");
        return -ENODEV;
    }

printk("%s:%s:%d\n",__FILE__,__FUNCTION__,__LINE__);
    ts = kzalloc(sizeof(*ts), GFP_KERNEL);
    if (ts == NULL)
    {
        return -ENOMEM;
    }

printk("%s:%s:%d\n",__FILE__,__FUNCTION__,__LINE__);
    INIT_WORK(&ts->work, goodix_ts_work_func);        //init work_struct
    ts->client = client;
    ts->power = goodix_ts_power;
    ts->bad_data = 0;
    ts->use_irq = 1;
    ts->use_reset = 0;
    ts->irq_is_disable = 0;
    i2c_set_clientdata(client, ts);
    pdata = client->dev.platform_data;

printk("%s:%s:%d\n",__FILE__,__FUNCTION__,__LINE__);
    if (fail == init_input_dev(ts))
    {
        return -1;
    }

//    set_pins(ts);
	ret = request_threaded_irq(client->irq, NULL, goodix_ts_irq_handler,
	     IRQF_TRIGGER_FALLING | IRQF_ONESHOT, "gt82x_ts", ts);	     
	 if (ret != 0)
	 {
printk("%s:%s:%d\n",__FILE__,__FUNCTION__,__LINE__);
		return -1;
	 } 

#ifdef CONFIG_HAS_EARLYSUSPEND
    ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
    ts->early_suspend.suspend = goodix_ts_early_suspend;
    ts->early_suspend.resume = goodix_ts_late_resume;
    register_early_suspend(&ts->early_suspend);
#endif

#ifdef CREATE_WR_NODE
//johncao    
//    init_wr_node(client);
#endif

#ifdef AUTO_UPDATE_GUITAR
    if (0xff == init_update_proc(ts))
    {
        NOTICE("Need update!\n");
        return 0;
    }
#else
    msleep(5);
//    guitar_reset(10);
#endif

printk("%s:%s:%d\n",__FILE__,__FUNCTION__,__LINE__);
    //Test I2C connection.    
    DEBUG_MSG("Testing I2C connection...\n");
//    for(retry = 0;retry < 3; retry++)
    while(1)            //For debug use!
    {
        ret = i2c_pre_cmd(ts);
        if (ret > 0)
            break;
        msleep(20);
printk("%s:%s:%d\n",__FILE__,__FUNCTION__,__LINE__);
    }
printk("%s:%s:%d\n",__FILE__,__FUNCTION__,__LINE__);
    if(ret <= 0)
    {
        dev_err(&client->dev, "Warnning: I2C communication might be ERROR!\n");
        WARNING("I2C test failed. I2C addr:%x\n", client->addr);
        goodix_ts_remove(ts->client);
        return -1;
    }

printk("%s:%s:%d\n",__FILE__,__FUNCTION__,__LINE__);
    //Send config
    for (retry = 0; retry < 3; retry++)
    {
        if (success == goodix_init_panel(ts, 1))
        {
            NOTICE("Initialize successfully!\n");
            break;
        }
    }
printk("%s:%s:%d\n",__FILE__,__FUNCTION__,__LINE__);
    if (retry >= 3)
    {
        ts->bad_data=1;
        WARNING("Initialize failed!\n");
        goodix_ts_remove(ts->client);
        return -1;
    }

printk("%s:%s:%d\n",__FILE__,__FUNCTION__,__LINE__);
    //Enable interrupt
	
    if(ts->use_irq && ts->irq_is_disable == 1)
    {
        ts->irq_is_disable = 0;
        enable_irq(client->irq);
    }

#ifdef READ_VERSION
    goodix_ts_version(ts);
#endif
    
    return 0;
}


/*******************************************************	
********************************************************/
static s32 goodix_ts_remove(struct i2c_client *client)
{
    struct goodix_ts_data *ts = i2c_get_clientdata(client);

    dev_notice(&client->dev,"The driver is removing...\n");
    
#ifdef CONFIG_HAS_EARLYSUSPEND
    unregister_early_suspend(&ts->early_suspend);
#endif

#ifdef CREATE_WR_NODE
//johncao    
//    uninit_wr_node();
#endif

    if (ts && ts->use_irq) 
    {
        free_irq(client->irq, ts);
//johncao	
//        GPIO_DIRECTION_INPUT(INT_PORT);
//        GPIO_FREE(INT_PORT);
    }
    else if(ts)
        hrtimer_cancel(&ts->timer);

    if (ts && ts->use_reset)
    {
//johncao	    
//        GPIO_DIRECTION_INPUT(RESET_PORT);
//        GPIO_FREE(RESET_PORT);
    }
    
    i2c_set_clientdata(client, NULL);
    input_unregister_device(ts->input_dev);
    input_free_device(ts->input_dev);
    kfree(ts);
    return success;
}

static s32 goodix_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
    s32 ret;
    struct goodix_ts_data *ts = i2c_get_clientdata(client);

    if (ts->irq_is_disable == 2)
    {
        return 0;
    }

    if (ts->use_irq)
    {
        if (!ts->irq_is_disable)
        {
            disable_irq(client->irq);
            ts->irq_is_disable = 1;
        }
    }
    else
    {
        hrtimer_cancel(&ts->timer);
    }
    
    if (ts->power) 
    {
        ret = ts->power(ts, 0);
        if (ret <= 0)
            DEBUG_MSG(KERN_ERR "goodix_ts_resume power off failed\n");
    }
    return 0;
}

static s32 goodix_ts_resume(struct i2c_client *client)
{
    s32 ret;
    struct goodix_ts_data *ts = i2c_get_clientdata(client);

    if (ts->irq_is_disable == 2)
    {
        return 0;
    }

    if (ts->power) 
    {
        ret = ts->power(ts, 1);
        if (ret <= 0)
            DEBUG_MSG(KERN_ERR "goodix_ts_resume power on failed\n");
    }

    if (ts->use_irq)
    {
        ts->irq_is_disable = 0;
        enable_irq(client->irq);
    }
    else
    {
        hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
    }

    return success;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void goodix_ts_early_suspend(struct early_suspend *h)
{
    struct goodix_ts_data *ts;
    ts = container_of(h, struct goodix_ts_data, early_suspend);
    goodix_ts_suspend(ts->client, PMSG_SUSPEND);
}

static void goodix_ts_late_resume(struct early_suspend *h)
{
    struct goodix_ts_data *ts;
    ts = container_of(h, struct goodix_ts_data, early_suspend);
    goodix_ts_resume(ts->client);
}
#endif
//******************************Begin of firmware update surpport*******************************

#ifndef AUTO_UPDATE_GUITAR
static void guitar_reset(s32 ms)
{
//johncao
/*
    GPIO_DIRECTION_OUTPUT(RESET_PORT, 0);
    GPIO_SET_VALUE(RESET_PORT, 0);
    msleep(ms);

    GPIO_DIRECTION_INPUT(RESET_PORT);
    GPIO_PULL_UPDOWN(RESET_PORT, 0);
*/
    msleep(20);
    return;
}
#endif


//
static struct of_device_id gt82x_ts_dt_ids[] = {
		{ .compatible = "eeti,gt82x_ts" },
		{ /* sentinel */ }
};
static const struct i2c_device_id goodix_ts_id[] = {
    { GOODIX_I2C_NAME, 0 },
    { }
};

static struct i2c_driver goodix_ts_driver = {
    .probe      = goodix_ts_probe,
    .remove     = goodix_ts_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
    .suspend    = goodix_ts_suspend,
    .resume     = goodix_ts_resume,
#endif
    .id_table   = goodix_ts_id,
    .driver     = {
        .name   = GOODIX_I2C_NAME,
        .owner  = THIS_MODULE,
	.of_match_table= of_match_ptr(gt82x_ts_dt_ids),
    },
};

/*******************************************************	
********************************************************/
static  goodix_ts_init(void)
{

printk("%s:%s:%d\n",__FILE__,__FUNCTION__,__LINE__);
    goodix_wq = create_workqueue("goodix_wq");       
       	//create a work queue and worker thread
    if (!goodix_wq)
    {
        DEBUG_MSG(KERN_ALERT "creat workqueue faiked\n");
        return -ENOMEM;
    }
printk("%s:%s:%d\n",__FILE__,__FUNCTION__,__LINE__);
    return i2c_add_driver(&goodix_ts_driver);
}

/*******************************************************	
********************************************************/
static void __exit goodix_ts_exit(void)
{
    DEBUG_MSG(KERN_ALERT "Touchscreen driver of guitar exited.\n");
    i2c_del_driver(&goodix_ts_driver);
    if (goodix_wq)
        destroy_workqueue(goodix_wq);        //release our work queue
}

late_initcall(goodix_ts_init);               
module_exit(goodix_ts_exit);

MODULE_DESCRIPTION("Goodix Touchscreen Driver");
MODULE_LICENSE("GPL");

