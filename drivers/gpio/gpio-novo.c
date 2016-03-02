#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <asm/mach-types.h>
#include <linux/gpio.h>
#include <linux/io.h>
#include <linux/of_gpio.h>

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/rtc.h>
#include <linux/unistd.h>
#include <linux/interrupt.h>

enum Pin
{
    e_p20_1=0, 
	e_p20_10, 
};

static int __novo_gpio_status[4] = { 0 };
static int gpioClass[20];

static int __novo_gpio_probe(struct platform_device *pdev)       
{
	struct device_node *np = pdev->dev.of_node;
	int ret;

	//p20_1--input
 	gpioClass[e_p20_1] = of_get_named_gpio(np, "p20_1", 0);
	if (gpio_is_valid(gpioClass[e_p20_1])) 
	{	
		ret = gpio_request(gpioClass[e_p20_1], "p20_1");
		if (ret)
		{
			printk("gpio_request p20_1 failed ret = %d\n", ret);		
			return ret;
		}

		gpio_direction_input(gpioClass[e_p20_1]);
	}

	//p20_10--out
    gpioClass[e_p20_10] = of_get_named_gpio(np, "p20_10", 0);
    if (gpio_is_valid(gpioClass[e_p20_10]))
    {
        ret = gpio_request(gpioClass[e_p20_10], "p20_10");
        if (ret)
        {
            printk("gpio_request p20_10 failed ret = %d\n", ret);
            return ret;
        }

        gpio_direction_output(gpioClass[e_p20_10], 0);
    }

	return 0;
#if 0
         int ret;
         ret = gpio_request(EXYNOS4_GPX1(6), "GPX1");
         if(ret)
                   printk("x4412-led: request gpio GPX1(6) fail\n");
         s3c_gpio_setpull(EXYNOS4_GPX1(6), S3C_GPIO_PULL_UP);
         gpio_direction_output(EXYNOS4_GPX1(6), 1);
         ret = gpio_request(EXYNOS4_GPX1(7), "GPX1");
         if(ret)
                   printk("x4412-led: request gpio GPX1(7) fail\n");
         s3c_gpio_setpull(EXYNOS4_GPX1(7), S3C_GPIO_PULL_UP);
         gpio_direction_output(EXYNOS4_GPX1(7), 1);
         ret = gpio_request(EXYNOS4_GPX2(6), "GPX2");
         if(ret)
                   printk("x4412-led: request gpio GPX2(6) fail\n");
         s3c_gpio_setpull(EXYNOS4_GPX2(6), S3C_GPIO_PULL_UP);
         gpio_direction_output(EXYNOS4_GPX2(6), 1);
         ret = gpio_request(EXYNOS4_GPX2(7), "GPX2");
         if(ret)
                   printk("x4412-led: request gpio GPX2(7) fail\n");
         s3c_gpio_setpull(EXYNOS4_GPX2(7), S3C_GPIO_PULL_UP);
         gpio_direction_output(EXYNOS4_GPX2(7), 1);
         __x4412_led_status[0] = 0;   //定义四盏LED灯的默认状态
         __x4412_led_status[1] = 0;
         __x4412_led_status[2] = 0;
         __x4412_led_status[3] = 0;
#endif
}

static void __novo_gpio_remove(struct platform_device *pdev)    //释放GPIO口
{
#if 0
         gpio_free(EXYNOS4_GPX1(6));
         gpio_free(EXYNOS4_GPX1(7));
         gpio_free(EXYNOS4_GPX2(6));
         gpio_free(EXYNOS4_GPX2(7));
#endif
}

static ssize_t novo_gpio_read(struct device *dev, struct device_attribute *attr, char *buf)
{
	if(!strcmp(attr->attr.name, "p20_1"))         
    {
		
    	if(gpio_get_value(gpioClass[e_p20_1]) != 0)
        	return strlcpy(buf, "1\n", 3);
        else
           	return strlcpy(buf, "0\n", 3);
   	}
	#if 0
    else if(!strcmp(attr->attr.name, "led2"))
         {
                   if(__x4412_led_status[1] != 0)
                            return strlcpy(buf, "1\n", 3);
                   else
                            return strlcpy(buf, "0\n", 3);
         }
         else if(!strcmp(attr->attr.name, "led3"))
         {
                   if(__x4412_led_status[2] != 0)
                            return strlcpy(buf, "1\n", 3);
                   else
                            return strlcpy(buf, "0\n", 3);
         }
         else if(!strcmp(attr->attr.name, "led4"))
         {
                   if(__x4412_led_status[3] != 0)
                            return strlcpy(buf, "1\n", 3);
                   else
                            return strlcpy(buf, "0\n", 3);
         }
		#endif
         return strlcpy(buf, "\n", 3);
}

static ssize_t novo_gpio_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long on = simple_strtoul(buf, NULL, 10);	

    if(!strcmp(attr->attr.name, "p20_10"))
    {
		if (on)
			gpio_direction_output(gpioClass[e_p20_10], 1);
		else
			gpio_direction_output(gpioClass[e_p20_10], 0);
    }
    else if (!strcmp(attr->attr.name, "unexport"))
    {
    }
    else if (!strcmp(attr->attr.name, "direction"))
    {
    }
    else if (!strcmp(attr->attr.name, "value"))
    {
    }

#if 0
         unsigned long on = simple_strtoul(buf, NULL, 10);//从应用读取需要写入的数据
         if(!strcmp(attr->attr.name, "led1"))
         {
                   if(on)                   //如果收到1，则将对应标志位置1，同时将对应GPIO置0，对应LED灯亮
                   {
                            gpio_direction_output(EXYNOS4_GPX1(6), 0);
                            __x4412_led_status[0] = 1;
                   }
                   else             //如果收到0，则将对应标志位清0，同时将对应GPIO置1，对应LED灯灭
                   {
                            gpio_direction_output(EXYNOS4_GPX1(6), 1);
                            __x4412_led_status[0] = 0;
                   }
         }
         else if(!strcmp(attr->attr.name, "led2"))
         {
                   if(on)
                   {
                            gpio_direction_output(EXYNOS4_GPX1(7), 0);
                            __x4412_led_status[1] = 1;
                   }
                   else
                   {
                            gpio_direction_output(EXYNOS4_GPX1(7), 1);
                            __x4412_led_status[1] = 0;
                   }
         }
         else if(!strcmp(attr->attr.name, "led3"))
         {
                   if(on)
                   {
                            gpio_direction_output(EXYNOS4_GPX2(6), 0);
                            __x4412_led_status[2] = 1;
                   }
                   else
                   {
                            gpio_direction_output(EXYNOS4_GPX2(6), 1);
                            __x4412_led_status[2] = 0;
                   }
         }
         else if(!strcmp(attr->attr.name, "led4"))
         {
                   if(on)
                   {
                            gpio_direction_output(EXYNOS4_GPX2(7), 0);
                            __x4412_led_status[3] = 1;
                   }
                   else
                   {
                            gpio_direction_output(EXYNOS4_GPX2(7), 1);
                            __x4412_led_status[3] = 0;
                   }
         }

         return count;
#endif
}

//kobject目录下的四个文件对应的属性，读写函数
static DEVICE_ATTR(p20_1, 0666, novo_gpio_read, novo_gpio_write);
static DEVICE_ATTR(p20_10, 0666, novo_gpio_read, novo_gpio_write);
static DEVICE_ATTR(direction, 0666, novo_gpio_read, novo_gpio_write);
static DEVICE_ATTR(value, 0666, novo_gpio_read, novo_gpio_write);

//static DEVICE_ATTR(led3, 0666, x4412_led_read, x4412_led_write);
//static DEVICE_ATTR(led4, 0666, x4412_led_read, x4412_led_write);
static struct attribute * novo_gpio_sysfs_entries[] = {         
	&dev_attr_p20_1.attr,
	&dev_attr_p20_10.attr,	
    NULL,
};

static struct attribute_group novo_gpio_attr_group = {
         .name         = NULL,
         .attrs  = novo_gpio_sysfs_entries,   
};

static int novo_gpio_probe(struct platform_device *pdev)
{
#if 0
	if (__novo_gpio_probe(pdev) == 0)
    	return sysfs_create_group(&pdev->dev.kobj, &novo_gpio_attr_group);
	else
		return 0;
#endif

	int rst,ret;
	int bt_pwr_row4,bt_sw_gpio4,bt_wup_gpio9,bt_rst_gpio7;
	int cts_gnd,rts_gnd;
	struct device_node* np = pdev->dev.of_node;

	rst = of_get_named_gpio(np, "wf111_rst", 0);
	if (!gpio_is_valid(rst)){
    	printk("can not find wf111_rst gpio pins\n");
        return -1;
	}
    ret = gpio_request(rst, "wf111_rst");
    if(ret){
    	printk("request gpio wf111_rst failed\n");
        return;
  	}


	gpio_direction_output(rst, 1);
	gpio_set_value(rst, 0);
	mdelay(100);
	gpio_set_value(rst, 1);

	//bt
	bt_pwr_row4 = of_get_named_gpio(np, "bt_pwr_row4", 0);
    if (!gpio_is_valid(bt_pwr_row4)){
        printk("can not find bt_pwr_row4 gpio pins\n");
        return -1;
    }
    ret = gpio_request(bt_pwr_row4, "bt_pwr_row4");
    if(ret){
        printk("request gpio bt_pwr_row4 failed\n");
        return;
    }
	gpio_direction_output(bt_pwr_row4, 1);	


    bt_sw_gpio4 = of_get_named_gpio(np, "bt_sw_gpio4", 0);
    if (!gpio_is_valid(bt_sw_gpio4)){
        printk("can not find bt_sw_gpio4 gpio pins\n");
        return -1;
    }
    ret = gpio_request(bt_sw_gpio4, "bt_sw_gpio4");
    if(ret){
        printk("request gpio bt_sw_gpio4 failed\n");
        return;
    }
    gpio_direction_output(bt_sw_gpio4, 1);

	bt_rst_gpio7 = of_get_named_gpio(np, "bt_rst_gpio7", 0);
    if (!gpio_is_valid(bt_rst_gpio7)){
        printk("can not find bt_rst_gpio7 gpio pins\n");
        return -1;
    }
    ret = gpio_request(bt_rst_gpio7, "bt_rst_gpio7");
    if(ret){
        printk("request gpio bt_rst_gpio7 failed\n");
        return;
    }
    gpio_direction_output(bt_rst_gpio7, 1);
    gpio_set_value(bt_rst_gpio7, 0);
    mdelay(100);
    gpio_set_value(bt_rst_gpio7, 1);

    bt_wup_gpio9 = of_get_named_gpio(np, "bt_wup_gpio9", 0);
    if (!gpio_is_valid(bt_wup_gpio9)){
        printk("can not find bt_wup_gpio9 gpio pins\n");
        return -1;
    }
    ret = gpio_request(bt_wup_gpio9, "bt_wup_gpio9");
    if(ret){
        printk("request gpio bt_wup_gpio9 failed\n");
        return;
    }
    gpio_direction_output(bt_wup_gpio9, 0);
	mdelay(100);
    gpio_set_value(bt_wup_gpio9, 1);


	//rts,cts
    cts_gnd = of_get_named_gpio(np, "cts_gnd", 0);
    if (!gpio_is_valid(cts_gnd)){
        printk("can not find cts_gnd gpio pins\n");
        return -1;
    }
    ret = gpio_request(cts_gnd, "cts_gnd");
    if(ret){
        printk("request gpio cts_gnd failed\n");
        return;
    }
    gpio_direction_output(cts_gnd, 0);


    rts_gnd = of_get_named_gpio(np, "rts_gnd", 0);
    if (!gpio_is_valid(rts_gnd)){
        printk("can not find rts_gnd gpio pins\n");
        return -1;
    }
    ret = gpio_request(rts_gnd, "rts_gnd");
    if(ret){
        printk("request gpio rts_gnd failed\n");
        return;
    }
    gpio_direction_output(rts_gnd, 0);
	
	return 0;
}

static int novo_gpio_remove(struct platform_device *pdev)
{
#if 0
         __novo_gpio_remove(pdev);                                     
         sysfs_remove_group(&pdev->dev.kobj, &novo_gpio_attr_group);
#endif
         return 0;
}

static const struct of_device_id gpio_dt_ids[] = {
    { .compatible = "novo,gpio", },
    { /* sentinel */ }
};

static struct platform_driver novo_gpio_driver = {
	.probe		= novo_gpio_probe,               
   	.remove     = novo_gpio_remove,
	.driver =
    {
        .name = "novo-gpio",
        .owner = THIS_MODULE,
        .of_match_table = gpio_dt_ids,
    }
};

static int __init novo_gpio_init(void)
{
	int ret;

    printk("novo gpio driver \n");

   	ret = platform_driver_register(&novo_gpio_driver);            
    if(ret)
    	printk("failed to register novo gpio driver \n");
         

	return ret;
}

static void novo_gpio_exit(void)
{
	platform_driver_unregister(&novo_gpio_driver);
}

module_init(novo_gpio_init);
module_exit(novo_gpio_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("ben");
MODULE_DESCRIPTION("hio novo gpio driver");

