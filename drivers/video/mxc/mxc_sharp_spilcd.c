/*
 * MTD SPI driver for ST M25Pxx (and similar) serial flash chips
 *
 * Author: Mike Lavender, mike@steroidmicros.com
 *
 * Copyright (c) 2005, Intec Automation Inc.
 *
 * Some parts are based on lart.c by Abraham Van Der Merwe
 *
 * Cleaned up and generalized based on mtd_dataflash.c
 *
 * This code is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/init.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/math64.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/mod_devicetable.h>
#include <linux/delay.h>
#include <linux/mtd/cfi.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/gpio.h>
#include <linux/spi/spi.h>
#include <linux/spi/flash.h>
#include <asm/setup.h>
#include <linux/workqueue.h>

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

#define DEBUG printk
//#define DEBUG(...)

int spi1_clk,spi1_in,spi1_out,spi1_ss;
int lcd_bl_vdd_en,lcd_bl_en,lcd_vdd_en,lcd_reset;
unsigned int delay=100;
unsigned short cmd;
unsigned int sculld_qset = 0;

#define SABRESD_LCD_BL_VDD_EN   lcd_bl_vdd_en
#define SABRESD_LCD_BL_EN       lcd_bl_en
#define SABRESD_LCD_VDD_EN      lcd_vdd_en
#define SABRESD_LCD_RESET       lcd_reset

#define SABRESD_SPI1_CLK        spi1_clk
#define SABRESD_SPI1_IN         spi1_in
#define SABRESD_SPI1_OUT        spi1_out
#define SABRESD_SPI1_SS         spi1_ss

unsigned short  spi_xmit(unsigned short send_data,int len)
{
    int  i = 0;
    unsigned short read_bit = 0;
    unsigned short read_data = 0;

    unsigned short mask[] = {0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80, 0x100, 0x200, 0x400, 0x800, 0x1000, 0x2000, 0x4000, 0x8000};



    gpio_direction_output(SABRESD_SPI1_SS, 0);
    gpio_direction_output(SABRESD_SPI1_CLK, 1);  // ?³öow
    udelay(delay);
    gpio_direction_output(SABRESD_SPI1_CLK, 0);  // ?³öow
    udelay(delay);
    for (i=(len*8-1); i>=0; i--)
    {
        gpio_direction_output(SABRESD_SPI1_OUT, ((send_data & mask[i]) >> i));
        gpio_direction_output(SABRESD_SPI1_CLK, 1);   // CLK-µ?
//        gpio_direction_output(SABRESD_SPI1_OUT, ((send_data & mask[i]) >> i)); 
        udelay(delay);
        gpio_direction_output(SABRESD_SPI1_CLK, 0);   // CLK-¸?
        udelay(delay);
        if ((i==8) && (len==2))
        {
                gpio_direction_output(SABRESD_SPI1_CLK, 1);   // CLK-µ?         
                udelay(delay);
                gpio_direction_output(SABRESD_SPI1_CLK, 0);   // CLK-¸?
                udelay(delay);
        }

    }
    gpio_direction_output(SABRESD_SPI1_SS, 1);  // ?³ö?     
    gpio_direction_output(SABRESD_SPI1_CLK, 0);   // CLK-µ?    gpio_direction_output(SABRESD_SPI1_OUT, 0);   // CLK-µ?
    gpio_direction_output(SABRESD_SPI1_OUT, 0); 
    gpio_direction_output(SABRESD_SPI1_SS, 1);  // ?³ö?     

    return read_data;
}

ssize_t sculld_show_qset(struct device_driver *driver, char *buf)
{
        return snprintf(buf, PAGE_SIZE, "%d\n", sculld_qset);
}
ssize_t sculld_store_qset(struct device_driver *driver, const char *buf,
                size_t count)
{
	int rc = 0;
	sculld_qset = simple_strtol(buf, NULL, 0);

        if (sculld_qset == 1)
        {
                printk("1111 \n");
	
		rc = gpio_request(spi1_clk, "spi1_clk");
        if (rc)
        {
                DEBUG("...lcd_probe... error3\n");
        }

        rc = gpio_request(spi1_in, "spi1_in");
        if (rc)
        {
                DEBUG("...lcd_probe... error4\n");
        }

        rc = gpio_request(spi1_out, "spi1_out");
        if (rc)
        {
                DEBUG("...lcd_probe... error5\n");
        }

        rc = gpio_request(spi1_ss, "spi1_ss");
        if (rc)
        {
                DEBUG("...lcd_probe... error6\n");
        }

        gpio_direction_output(SABRESD_SPI1_CLK, 0);
        gpio_direction_output(SABRESD_SPI1_OUT, 0);
        gpio_direction_output(SABRESD_SPI1_IN, 0);
        gpio_direction_output(SABRESD_SPI1_SS, 1);
		
		rc = gpio_request(SABRESD_LCD_RESET     ,"lcd_reset");
        if (rc)
        {
                DEBUG("...lcd_probe... error7\n");
        }

        rc = gpio_request(lcd_bl_vdd_en  ,"lcd_bl_vdd_en11");
        if (rc)
        {
                DEBUG("...lcd_probe... error8\n");
        }

        rc = gpio_request(SABRESD_LCD_BL_EN  ,"lcd_bl_en11");
        if (rc)
        {
                DEBUG("...lcd_probe... error9\n");
        }

        rc = gpio_request(SABRESD_LCD_VDD_EN  ,"lcd_vdd_en11");
        if (rc)
        {
                DEBUG("...lcd_probe... error10\n");
        }

        gpio_direction_output(SABRESD_LCD_BL_VDD_EN, 1);
        gpio_direction_output(SABRESD_LCD_BL_EN, 1);
        gpio_direction_output(SABRESD_LCD_VDD_EN, 0);

        msleep(10);
        gpio_direction_output(SABRESD_LCD_RESET, 0);
        msleep(10);
        gpio_direction_output(SABRESD_LCD_RESET, 1);

		//spi command
        msleep(10);
        cmd=0x29;
        spi_xmit(cmd, 1);

        msleep(5);
        cmd=0x11;
        spi_xmit(cmd, 1);

        msleep(150);
        cmd=0x0036;
        spi_xmit(cmd, 2);

        msleep(10);
        cmd=0x703a;
        spi_xmit(cmd, 2);

        msleep(10);
        cmd=0x00B0;
        spi_xmit(cmd, 2);

        msleep(10);
        cmd=0x03B0;
        spi_xmit(cmd, 2);		
	
	}

	return count;
}
static DRIVER_ATTR(qset, S_IRUGO | S_IWUSR, sculld_show_qset, sculld_store_qset);

#if 0
static int sharp_spi_probe(struct spi_device *spi)
{
        struct device_node *np = spi->dev.of_node;
        int rc =0;
	int command[10];

        DEBUG(".........sharp_spi_probe..... \n");

        if (np == NULL)
                DEBUG("...lcd_probe... error1\n");
	
	lcd_bl_vdd_en = of_get_named_gpio(np, "lcd_bl_vdd_en", 0);
        lcd_bl_en = of_get_named_gpio(np, "lcd_bl_en", 0);
        lcd_vdd_en = of_get_named_gpio(np, "lcd_vdd_en", 0);
        lcd_reset = of_get_named_gpio(np, "lcd_reset", 0);

       	rc = gpio_request(SABRESD_LCD_RESET     ,"lcd_reset");
        if (rc)
        {
                DEBUG("...lcd_probe... error7\n");
        }

        rc = gpio_request(lcd_bl_vdd_en  ,"lcd_bl_vdd_en11");
        if (rc)
        {
                DEBUG("...lcd_probe... error8\n");
        }

        rc = gpio_request(SABRESD_LCD_BL_EN  ,"lcd_bl_en11");
        if (rc)
        {
                DEBUG("...lcd_probe... error9\n");
        }

        rc = gpio_request(SABRESD_LCD_VDD_EN  ,"lcd_vdd_en11");
        if (rc)
        {
                DEBUG("...lcd_probe... error10\n");
        }

        gpio_direction_output(SABRESD_LCD_BL_VDD_EN, 1);
        gpio_direction_output(SABRESD_LCD_BL_EN, 1);
        gpio_direction_output(SABRESD_LCD_VDD_EN, 0);

        msleep(10);
        gpio_direction_output(SABRESD_LCD_RESET, 0);
        msleep(10);
        gpio_direction_output(SABRESD_LCD_RESET, 1);

        //spi command
	command[0] = 0x29;
	spi_write(spi, (const u8 *)&command, 1);
	msleep(10);

        command[0] = 0x11;
        spi_write(spi, (const u8 *)&command, 1);
        msleep(150);

        command[1] = 0x00;command[0] = 0x36;
        spi_write(spi, (const u8 *)&command, 2);
        msleep(10);

        command[1] = 0x70;command[0] = 0x3a;
        spi_write(spi, (const u8 *)&command, 2);
        msleep(10);

        command[1] = 0x00;command[0] = 0xb0;
        spi_write(spi, (const u8 *)&command, 2);
        msleep(10);
	
        command[1] = 0x03;command[0] = 0xb0;
        spi_write(spi, (const u8 *)&command, 2);
        msleep(10);
	return 0;
}
#endif
#if 1
static int sharp_spi_probe(struct spi_device *spi)
{
	struct device_node *np = spi->dev.of_node;
	int rc =0;

	DEBUG(".........sharp_spi_probe..... \n");

	driver_create_file(spi->dev.driver, &driver_attr_qset);

	if (np == NULL)
                DEBUG("...lcd_probe... error1\n");

        spi1_clk = of_get_named_gpio(np, "spi1_clk", 0);
        spi1_in = of_get_named_gpio(np, "spi1_in", 0);
        spi1_out = of_get_named_gpio(np, "spi1_out", 0);
        spi1_ss = of_get_named_gpio(np, "spi1_ss", 0);

        lcd_bl_vdd_en = of_get_named_gpio(np, "lcd_bl_vdd_en", 0);
        lcd_bl_en = of_get_named_gpio(np, "lcd_bl_en", 0);
        lcd_vdd_en = of_get_named_gpio(np, "lcd_vdd_en", 0);
        lcd_reset = of_get_named_gpio(np, "lcd_reset", 0);

        if ((!gpio_is_valid(spi1_clk)) || (!gpio_is_valid(spi1_in)) || (!gpio_is_valid(spi1_out)) || (!gpio_is_valid(spi1_ss)) ||
                (!gpio_is_valid(lcd_bl_vdd_en)) || (!gpio_is_valid(lcd_bl_en)) || (!gpio_is_valid(lcd_vdd_en)) || (!gpio_is_valid(lcd_reset)))
                DEBUG("...lcd_probe... error2\n");

        //      
	#if 1
        rc = gpio_request(spi1_clk, "spi1_clk");
        if (rc)
        {
                DEBUG("...lcd_probe... error3\n");
        }

        rc = gpio_request(spi1_in, "spi1_in");
        if (rc)
        {
                DEBUG("...lcd_probe... error4\n");
        }

        rc = gpio_request(spi1_out, "spi1_out");
        if (rc)
        {
                DEBUG("...lcd_probe... error5\n");
        }

        rc = gpio_request(spi1_ss, "spi1_ss");
        if (rc)
        {
                DEBUG("...lcd_probe... error6\n");
        }

        gpio_direction_output(SABRESD_SPI1_CLK, 0);
        gpio_direction_output(SABRESD_SPI1_OUT, 0);
        gpio_direction_output(SABRESD_SPI1_IN, 0);
        gpio_direction_output(SABRESD_SPI1_SS, 1);

        rc = gpio_request(SABRESD_LCD_RESET     ,"lcd_reset");
        if (rc)
        {
                DEBUG("...lcd_probe... error7\n");
        }

        rc = gpio_request(lcd_bl_vdd_en  ,"lcd_bl_vdd_en11");
        if (rc)
        {
                DEBUG("...lcd_probe... error8\n");
        }

        rc = gpio_request(SABRESD_LCD_BL_EN  ,"lcd_bl_en11");
        if (rc)
        {
                DEBUG("...lcd_probe... error9\n");
        }

        rc = gpio_request(SABRESD_LCD_VDD_EN  ,"lcd_vdd_en11");
        if (rc)
        {
                DEBUG("...lcd_probe... error10\n");
        }

        gpio_direction_output(SABRESD_LCD_BL_VDD_EN, 1);
        gpio_direction_output(SABRESD_LCD_BL_EN, 1);
        gpio_direction_output(SABRESD_LCD_VDD_EN, 0);

        msleep(10);
        gpio_direction_output(SABRESD_LCD_RESET, 0);
        msleep(10);
        gpio_direction_output(SABRESD_LCD_RESET, 1);

        //spi command

        msleep(10);
        cmd=0x29;
        spi_xmit(cmd, 1);

        msleep(5);
        cmd=0x11;
        spi_xmit(cmd, 1);

        msleep(150);
        cmd=0x0036;
        spi_xmit(cmd, 2);

        msleep(10);
        cmd=0x703a;
        spi_xmit(cmd, 2);

        msleep(10);
        cmd=0x00B0;
        spi_xmit(cmd, 2);

        msleep(10);
        cmd=0x03B0;
        spi_xmit(cmd, 2);
	#endif
	return 0;
}
#endif

static const struct of_device_id spidev_sharp_ids[] = {
    { .compatible = "spidev" },
    {},
};
MODULE_DEVICE_TABLE(of, spidev_sharp_ids);

static struct spi_driver sharp_driver = {
	.driver = {
		.name	= "spidev",
		.bus	= &spi_bus_type,
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(spidev_sharp_ids),
	},
	.probe	= sharp_spi_probe,
//	.remove	= __devexit_p(sharp_spi_remove),
//	.shutdown = sharp_shutdown;
//	.resume = sharp_resume,
//	.suspend = sharp_suspend,

	/* REVISIT: many of these chips have deep power-down modes, which
	 * should clearly be entered on suspend() to minimize power use.
	 * And also when they're otherwise idle...
	 */
};

static int __init sharp_spi_init(void)
{
	return spi_register_driver(&sharp_driver);
}


static void __exit sharp_spi_exit(void)
{
	spi_unregister_driver(&sharp_driver);
}


module_init(sharp_spi_init);
module_exit(sharp_spi_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("johncao");
MODULE_DESCRIPTION("SPI driver for SHARP LCD");
