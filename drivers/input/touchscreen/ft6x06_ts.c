/* drivers/input/touchscreen/ft5x06_ts.c
 *
 * FocalTech ft6x06 TouchScreen driver.
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
 */

#include <linux/i2c.h>
#include <linux/input.h>
#include "ft6x06_ts.h"
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/semaphore.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/syscalls.h>
#include <linux/unistd.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/timer.h>

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/rtc.h>
#include <linux/unistd.h>
#include <linux/interrupt.h>
//#include <mach/gpio_sf02.h>
//#include <mach/gpio.h>
#include <linux/gpio.h>
#include <linux/io.h>
#include <linux/of_gpio.h>

#define SABRESD_FT6X06_INT      IMX_GPIO_NR(1, 1)
//#define FTS_CTL_IIC
//#define SYSFS_DEBUG
#define FTS_APK_DEBUG
#ifdef FTS_CTL_IIC
#include "focaltech_ctl.h"
#endif
#ifdef SYSFS_DEBUG
#include "ft6x06_ex_fun.h"
#endif
struct ts_event {
	u16 au16_x[CFG_MAX_TOUCH_POINTS];	/*x coordinate */
	u16 au16_y[CFG_MAX_TOUCH_POINTS];	/*y coordinate */
	u8 au8_touch_event[CFG_MAX_TOUCH_POINTS];	/*touch event:
					0 -- down; 1-- contact; 2 -- contact */
	u8 au8_finger_id[CFG_MAX_TOUCH_POINTS];	/*touch ID */
	u16 pressure;
	u8 touch_point;
};

struct ft6x06_ts_data {
	unsigned int irq;
	unsigned int x_max;
	unsigned int y_max;
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct ts_event event;
	struct ft6x06_platform_data *pdata;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
};

#define FTS_POINT_UP		0x01
#define FTS_POINT_DOWN		0x00
#define FTS_POINT_CONTACT	0x02


VIRTUAL_KEY_POINT home,back,menu;
//////////////////////////////////////


#define FT6X06_KEY_HOME    172
#define FT6X06_KEY_MENU    139
#define FT6X06_KEY_BACK    158
#define FT6X06_KEY_SEARCH  217

//add ben
struct ft6x06_platform_data ft6x06_pdata =
{
	//.x_max=480,
	//.y_max=1000,
	.x_max=480,
	.y_max=800,
	//.irqflags=IRQF_TRIGGER_LOW,
	//.reset=SABRESD_FT6X06_RST,
};

static ssize_t virtual_keys_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    printk("read virtual_keys_shows\n");
    return sprintf(buf,
         __stringify(EV_KEY) ":" __stringify(FT6X06_KEY_MENU)   ":60:900:60:60"
     ":" __stringify(EV_KEY) ":" __stringify(FT6X06_KEY_HOME)   ":180:900:60:60"
     ":" __stringify(EV_KEY) ":" __stringify(FT6X06_KEY_BACK)   ":300:900:60:60"
     ":" __stringify(EV_KEY) ":" __stringify( FT6X06_KEY_SEARCH) ":420:900:60:60"
     "\n");
}




static struct kobj_attribute virtual_keys_attr = {
    .attr = {
        .name = "virtualkeys.ft6x06_ts",    //FT5X0X_NAME 一定要与触摸屏设备名称一致，不然会找不到指定的sys文件
        .mode = S_IRUGO,
    },
    .show = &virtual_keys_show,
};


static struct attribute *properties_attrs[] = {
        &virtual_keys_attr.attr,
        NULL
};

static struct attribute_group properties_attr_group = {
    .attrs = properties_attrs,
};

 
static void ft6x06_ts_virtual_keys_init(void)
{
    int ret;
    struct kobject *properties_kobj;
    
    properties_kobj = kobject_create_and_add("board_properties", NULL);
    if (properties_kobj)
        ret = sysfs_create_group(properties_kobj,  &properties_attr_group);
    if (!properties_kobj || ret)
        pr_err("failed to create board_properties\n");    
}

//////////////////////////////////////

/*
*ft6x06_i2c_Read-read data and write data by i2c
*@client: handle of i2c
*@writebuf: Data that will be written to the slave
*@writelen: How many bytes to write
*@readbuf: Where to store data read from slave
*@readlen: How many bytes to read
*
*Returns negative errno, else the number of messages executed
*
*
*/
int ft6x06_i2c_Read(struct i2c_client *client, char *writebuf,
		    int writelen, char *readbuf, int readlen)
{
	int ret;

	if (writelen > 0) {
		struct i2c_msg msgs[] = {
			{
			 .addr = client->addr,
			 .flags = 0,
			 .len = writelen,
			 .buf = writebuf,
			 },
			{
			 .addr = client->addr,
			 .flags = I2C_M_RD,
			 .len = readlen,
			 .buf = readbuf,
			 },
		};
		ret = i2c_transfer(client->adapter, msgs, 2);
		if (ret < 0)
			dev_err(&client->dev, "f%s: i2c read error.\n",
				__func__);
	} else {
		struct i2c_msg msgs[] = {
			{
			 .addr = client->addr,
			 .flags = I2C_M_RD,
			 .len = readlen,
			 .buf = readbuf,
			 },
		};
		ret = i2c_transfer(client->adapter, msgs, 1);
		if (ret < 0)
			dev_err(&client->dev, "%s:i2c read error.\n", __func__);
	}
	return ret;
}
/*write data by i2c*/
int ft6x06_i2c_Write(struct i2c_client *client, char *writebuf, int writelen)
{
	int ret;

	struct i2c_msg msg[] = {
		{
		 .addr = client->addr,
		 .flags = 0,
		 .len = writelen,
		 .buf = writebuf,
		 },
	};

	ret = i2c_transfer(client->adapter, msg, 1);
	if (ret < 0)
		dev_err(&client->dev, "%s i2c write error.\n", __func__);

	return ret;
}

/*release the point*/
static void ft6x06_ts_release(struct ft6x06_ts_data *data)
{
	input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, 0);
	input_sync(data->input_dev);
}

/*Read touch point information when the interrupt  is asserted.*/
static int ft6x06_read_Touchdata(struct ft6x06_ts_data *data)
{
	struct ts_event *event = &data->event;
	u8 buf[POINT_READ_BUF] = { 0 };
	int ret = -1;
	int i = 0;
	u8 pointid = FT_MAX_ID;

	ret = ft6x06_i2c_Read(data->client, buf, 1, buf, POINT_READ_BUF);
	if (ret < 0) {
		dev_err(&data->client->dev, "%s read touchdata failed.\n",
			__func__);
		return ret;
	}


	memset(event, 0, sizeof(struct ts_event));

	//event->touch_point = buf[2] & 0x0F;

	event->touch_point = 0;
	for (i = 0; i < CFG_MAX_TOUCH_POINTS; i++) {
	//for (i = 0; i < event->touch_point; i++) {
		pointid = (buf[FT_TOUCH_ID_POS + FT_TOUCH_STEP * i]) >> 4;
		//printk("********touch_point:%d pointid:%d\n",buf[2] & 0x0F,pointid);
		if (pointid >= FT_MAX_ID)
			break;
		else
			event->touch_point++;
		event->au16_x[i] =
		    (s16) (buf[FT_TOUCH_X_H_POS + FT_TOUCH_STEP * i] & 0x0F) <<
		    8 | (s16) buf[FT_TOUCH_X_L_POS + FT_TOUCH_STEP * i];
		event->au16_y[i] =
		    (s16) (buf[FT_TOUCH_Y_H_POS + FT_TOUCH_STEP * i] & 0x0F) <<
		    8 | (s16) buf[FT_TOUCH_Y_L_POS + FT_TOUCH_STEP * i];
		event->au8_touch_event[i] =
		    buf[FT_TOUCH_EVENT_POS + FT_TOUCH_STEP * i] >> 6;
		event->au8_finger_id[i] =
		    (buf[FT_TOUCH_ID_POS + FT_TOUCH_STEP * i]) >> 4;
	}
	
	event->pressure = FT_PRESS;

	return 0;
}

/*
*report the point information
*/
static void ft6x06_report_value(struct ft6x06_ts_data *data)
{
	struct ts_event *event = &data->event;
        int i = 0;
        int up_point = 0;

	//printk("ft6x06_report_value:%d\n", event->touch_point);
	//for (i = 0; i < event->touch_point; i++) {
		if (event->au16_x[i] < data->x_max
			&& event->au16_y[i] < data->y_max) {
			//input_report_key(data->input_dev,BTN_TOUCH,1);
			//input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, 10);
			//input_report_abs(data->input_dev, ABS_PRESSURE, 10);
			input_report_abs(data->input_dev, ABS_X, event->au16_x[i]);
			input_report_abs(data->input_dev, ABS_Y, event->au16_y[i]);

			if (event->au8_touch_event[i] == FTS_POINT_DOWN
				|| event->au8_touch_event[i] == FTS_POINT_CONTACT){
				//printk("down status.. \n");
				input_report_key(data->input_dev,BTN_TOUCH,1);
				input_report_abs(data->input_dev, ABS_PRESSURE, 1);
			}
			else{
				//printk("up status.. \n");
				input_report_key(data->input_dev,BTN_TOUCH,0);
				input_report_abs(data->input_dev, ABS_PRESSURE, 0);
			}

			input_sync(data->input_dev);
		}
	//}
}
#if 0
static void ft6x06_report_value(struct ft6x06_ts_data *data)
{
	struct ts_event *event = &data->event;
	int i = 0;
	int up_point = 0;
	//int touch_point = 0;

	for (i = 0; i < event->touch_point; i++) {
		/* LCD view area */
	//printk("********x:%d********\n",event->au16_x[i]);
	//printk("********y:%d********\n",event->au16_y[i]);
		if (event->au16_x[i] < data->x_max
		    && event->au16_y[i] < data->y_max) {
			input_report_abs(data->input_dev, ABS_MT_POSITION_X,
					 //480-event->au16_x[i]);
					 event->au16_x[i]);
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y,
					 event->au16_y[i]);
//			input_report_abs(data->input_dev, ABS_MT_PRESSURE,
//					 event->pressure);
			input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, 1);
			input_report_abs(data->input_dev, ABS_MT_TRACKING_ID,
					 event->au8_finger_id[i]);
//			input_report_abs(data->input_dev, ABS_MT_TRACKING_ID,i);
			if (event->au8_touch_event[i] == FTS_POINT_DOWN
			    || event->au8_touch_event[i] == FTS_POINT_CONTACT)
			{
				input_report_abs(data->input_dev, ABS_MT_PRESSURE,event->pressure);
                //printk("***************down\n");
				input_report_key( data->input_dev , BTN_TOUCH , 1);    // pressed
			}

			else 
			{
				input_report_abs(data->input_dev,ABS_MT_PRESSURE,0);
//				input_report_abs(data->input_dev,
//						 ABS_MT_TOUCH_MAJOR, 0);
				input_report_key(data->input_dev , BTN_TOUCH , 0);    // realse
               // printk("***************up\n");
				up_point++;
			}
			//touch_point ++;
			input_mt_sync(data->input_dev);
		}

	}
	
	if (event->touch_point > 0)
		input_sync(data->input_dev);
	if (event->touch_point == up_point)
		ft6x06_ts_release(data);

}
#endif

/*The ft6x06 device will signal the host about TRIGGER_FALLING.
*Processed when the interrupt is asserted.
*/
static irqreturn_t ft6x06_ts_interrupt(int irq, void *dev_id)
{
	struct ft6x06_ts_data *ft6x06_ts = dev_id;
	int ret = 0;
//	printk("***********%s:%s:%d\n",__FILE__,__FUNCTION__,__LINE__);
	disable_irq_nosync(ft6x06_ts->irq);

	ret = ft6x06_read_Touchdata(ft6x06_ts);
	if (ret == 0)
		ft6x06_report_value(ft6x06_ts);

	enable_irq(ft6x06_ts->irq);

	return IRQ_HANDLED;
}

//#ifdef CONFIG_PM

#ifdef CONFIG_HAS_EARLYSUSPEND

static void ft6x06_ts_suspend(struct early_suspend *handler)
{
	struct ft6x06_ts_data *ts = container_of(handler, struct ft6x06_ts_data,
						early_suspend);

	dev_dbg(&ts->client->dev, "[FTS]ft6x06 suspend\n");
	disable_irq(ts->pdata->irq);
}

static void ft6x06_ts_resume(struct early_suspend *handler)
{
	struct ft6x06_ts_data *ts = container_of(handler, struct ft6x06_ts_data,
						early_suspend);

	dev_dbg(&ts->client->dev, "[FTS]ft6x06 resume.\n");
	gpio_set_value(ts->pdata->reset, 0);
	msleep(20);
	gpio_set_value(ts->pdata->reset, 1);
	enable_irq(ts->pdata->irq);
}
#else
#define ft6x06_ts_suspend	NULL
#define ft6x06_ts_resume		NULL
#endif

static int ft6x06_ts_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	//struct ft6x06_platform_data *pdata =
	//    (struct ft6x06_platform_data *)client->dev.platform_data;
	struct ft6x06_platform_data *pdata = (struct ft6x06_platform_data *)&ft6x06_pdata;
	struct ft6x06_ts_data *ft6x06_ts;
	struct input_dev *input_dev;
	int err = 0;
	unsigned char uc_reg_value;
	unsigned char uc_reg_addr;
	int gpio_int;
	struct device_node *np = client->dev.of_node;	

    printk("ft6x06 probe .......\n");
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}

	ft6x06_ts = kzalloc(sizeof(struct ft6x06_ts_data), GFP_KERNEL);

	if (!ft6x06_ts) {
		err = -ENOMEM;
		goto exit_alloc_data_failed;
	}
	
	i2c_set_clientdata(client, ft6x06_ts);
	//ft6x06_ts->irq = client->irq;

	ft6x06_ts->client = client;
	ft6x06_ts->pdata = pdata;
	ft6x06_ts->x_max = pdata->x_max - 1;
	ft6x06_ts->y_max = pdata->y_max - 1;
#ifdef CONFIG_PM
/*	
	err = gpio_request(pdata->reset, "ft6x06 reset");
	if (err < 0) {
		dev_err(&client->dev, "%s:failed to set gpio reset.\n",
			__func__);
		goto exit_request_reset;
	}
*/	
#endif

#if 1
       //add ben
        pdata->reset = of_get_named_gpio(np, "gpio_rst", 0);
        if (!gpio_is_valid(pdata->reset))
                printk("error gpio_rst.. \n");

        gpio_int = of_get_named_gpio(np, "gpio_int", 0);
        if (!gpio_is_valid(gpio_int))
                printk("error gpio_int.. \n");
        
        client->irq = gpio_to_irq(gpio_int);
	ft6x06_ts->irq = client->irq;	

	gpio_request(pdata->reset, "ft6x06 reset");
	gpio_direction_output(pdata->reset, 1);

	printk("*********interrupt****************%d\n",client->irq);
	err = request_threaded_irq(client->irq, NULL, ft6x06_ts_interrupt,
//	err = request_irq(client->irq, ft6x06_ts_interrupt,
//				   pdata->irqflags, client->dev.driver->name,
//				   IRQF_TRIGGER_FALLING |IRQF_ONESHOT, client->dev.driver->name,
				   IRQF_TRIGGER_LOW |IRQF_ONESHOT, client->dev.driver->name,
				   ft6x06_ts);
	if (err < 0) {
		dev_err(&client->dev, "ft6x06_probe: request irq failed\n");
		goto exit_irq_request_failed;
	}
	disable_irq(client->irq);
#endif
	
	input_dev = input_allocate_device();
	if (!input_dev) {
		err = -ENOMEM;
		dev_err(&client->dev, "failed to allocate input device\n");
		goto exit_input_dev_alloc_failed;
	}
//////
#if 0
	ft6x06_ts_virtual_keys_init();
	set_bit(KEY_HOME,  input_dev->keybit);
    	set_bit(KEY_MENU,  input_dev->keybit);
    	set_bit(KEY_BACK,  input_dev->keybit);
    	set_bit(KEY_SEARCH,  input_dev->keybit);//这里我把4个virtualkey的位图初始化设置了一下，开始这里忘了，其它地方都正确，4个virtualkey还是没效果


//////
	ft6x06_ts->input_dev = input_dev;

	set_bit(ABS_MT_TOUCH_MAJOR, input_dev->absbit);
	set_bit(ABS_MT_POSITION_X, input_dev->absbit);
	set_bit(ABS_MT_POSITION_Y, input_dev->absbit);
	set_bit(ABS_MT_PRESSURE, input_dev->absbit);

	input_set_abs_params(input_dev,
			     ABS_MT_POSITION_X, 0, ft6x06_ts->x_max, 0, 0);
	input_set_abs_params(input_dev,
//			     ABS_MT_POSITION_Y, 0, ft6x06_ts->y_max, 0, 0);
			     ABS_MT_POSITION_Y, 0, 800, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, PRESS_MAX, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_PRESSURE, 0, PRESS_MAX, 0, 0);
	input_set_abs_params(input_dev,
			     ABS_MT_TRACKING_ID, 0, CFG_MAX_TOUCH_POINTS, 0, 0);

	set_bit(EV_KEY, input_dev->evbit);
	set_bit(EV_ABS, input_dev->evbit);
#endif
	ft6x06_ts->input_dev = input_dev;

      /* config input device */
        __set_bit(EV_SYN, input_dev->evbit);
        __set_bit(EV_KEY, input_dev->evbit);
        __set_bit(EV_ABS, input_dev->evbit);

      	set_bit(BTN_TOUCH, input_dev->keybit);
        set_bit(ABS_X, input_dev->absbit);
    	set_bit(ABS_Y, input_dev->absbit);
        set_bit(ABS_PRESSURE, input_dev->absbit);

        input_set_abs_params(input_dev, ABS_X, 0, ft6x06_ts->x_max, 0, 0);
        input_set_abs_params(input_dev, ABS_Y, 0, ft6x06_ts->y_max, 0, 0);
	
/////////////////////////
	input_dev->name = FT6X06_NAME;
	err = input_register_device(input_dev);
	if (err) {
		dev_err(&client->dev,
			"ft6x06_ts_probe: failed to register input device: %s\n",
			dev_name(&client->dev));
		goto exit_input_register_device_failed;
	}
	/*make sure CTP already finish startup process */
	msleep(150);

	/*get some register information */
	uc_reg_addr = FT6x06_REG_FW_VER;
	ft6x06_i2c_Read(client, &uc_reg_addr, 1, &uc_reg_value, 1);
//	dev_dbg(&client->dev, "[FTS] Firmware version = 0x%x\n", uc_reg_value);
	printk("[FTS] Firmware version = 0x%x\n", uc_reg_value);

	uc_reg_addr = FT6x06_REG_POINT_RATE;
	ft6x06_i2c_Read(client, &uc_reg_addr, 1, &uc_reg_value, 1);
//	dev_dbg(&client->dev, "[FTS] report rate is %dHz.\n",
	printk("[FTS] report rate is %dHz.\n",
		uc_reg_value * 10);

	uc_reg_addr = FT6x06_REG_THGROUP;
	ft6x06_i2c_Read(client, &uc_reg_addr, 1, &uc_reg_value, 1);
	dev_dbg(&client->dev, "[FTS] touch threshold is %d.\n",
		uc_reg_value * 4);
// zeng 2015.05.23 +++++	
#ifdef CONFIG_HAS_EARLYSUSPEND
		printk("==register_early_suspend =\n");
		//ft6x06_ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
		ft6x06_ts->early_suspend.level = 51;
		ft6x06_ts->early_suspend.suspend = ft6x06_ts_suspend;
		ft6x06_ts->early_suspend.resume = ft6x06_ts_resume;
		register_early_suspend(&ft6x06_ts->early_suspend);
#endif
// zeng +++ end +++
	
#ifdef SYSFS_DEBUG
	ft6x06_create_sysfs(client);
#endif

#ifdef FTS_CTL_IIC
	if (ft_rw_iic_drv_init(client) < 0)
		dev_err(&client->dev, "%s:[FTS] create fts control iic driver failed\n",
				__func__);
#endif
	enable_irq(client->irq);
//	gpio_direction_output(SABRESD_FT6X06_INT,0);
    printk("ft6x06 tp probe is ok\n");
	return 0;

exit_input_register_device_failed:
	input_free_device(input_dev);

exit_input_dev_alloc_failed:
	free_irq(client->irq, ft6x06_ts);
#ifdef CONFIG_PM
exit_request_reset:
	gpio_free(ft6x06_ts->pdata->reset);
#endif

exit_irq_request_failed:
	i2c_set_clientdata(client, NULL);
	kfree(ft6x06_ts);

exit_alloc_data_failed:
exit_check_functionality_failed:
	return err;
}



static int ft6x06_ts_remove(struct i2c_client *client)
{
	struct ft6x06_ts_data *ft6x06_ts;
	ft6x06_ts = i2c_get_clientdata(client);
	input_unregister_device(ft6x06_ts->input_dev);
	#ifdef CONFIG_PM
	gpio_free(ft6x06_ts->pdata->reset);
	#endif

	#ifdef SYSFS_DEBUG
	ft6x06_release_sysfs(client);
	#endif
	#ifdef FTS_CTL_IIC
	ft_rw_iic_drv_exit();
	#endif
	free_irq(client->irq, ft6x06_ts);
	kfree(ft6x06_ts);
	i2c_set_clientdata(client, NULL);
	return 0;
}

static const struct of_device_id ft_ids[] = {
    { .compatible = "ft,ft6x06_ts" },
    {},
};
MODULE_DEVICE_TABLE(of, ft_ids);

static const struct i2c_device_id ft6x06_ts_id[] = {
	{FT6X06_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, ft6x06_ts_id);

static struct i2c_driver ft6x06_ts_driver = {
	.probe = ft6x06_ts_probe,
	.remove = ft6x06_ts_remove,
	.id_table = ft6x06_ts_id,
	//.suspend = ft6x06_ts_suspend,
	//.resume = ft6x06_ts_resume,
	.driver = {
		   .name = FT6X06_NAME,
		   .owner = THIS_MODULE,
		   .of_match_table = of_match_ptr(ft_ids),
		   },
};

static int __init ft6x06_ts_init(void)
{
	int ret;
	ret = i2c_add_driver(&ft6x06_ts_driver);
	if (ret) {
		printk(KERN_WARNING "Adding ft6x06 driver failed "
		       "(errno = %d)\n", ret);
	} else {
		pr_info("Successfully added driver %s\n",
			ft6x06_ts_driver.driver.name);
	}

	return ret;
}

static void __exit ft6x06_ts_exit(void)
{
	i2c_del_driver(&ft6x06_ts_driver);
}

module_init(ft6x06_ts_init);
module_exit(ft6x06_ts_exit);

MODULE_AUTHOR("<luowj>");
MODULE_DESCRIPTION("FocalTech ft6x06 TouchScreen driver");
MODULE_LICENSE("GPL");
