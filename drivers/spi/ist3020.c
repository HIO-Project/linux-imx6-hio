/*
 i MTD SPI driver for ST M25Pxx (and similar) serial flash chips
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
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
//#include <mach/gpio.h>
#include <linux/delay.h>
#include <asm/uaccess.h>
#include <linux/slab.h>

#include <linux/spi/spi.h>
#include <linux/shenonmxc.h>
#include <asm/io.h>
#ifdef CONFIG_PWR_CD
#include <linux/proc_fs.h>
#endif
#include "ist3020.h"

#define IST3020_COUNT 1
#define IST3020_NAME "ist3020lcd_ctrl"

#ifdef CONFIG_WIFI_IC
#ifdef CONFIG_RTL8188EU
#define WIFI_NAME "RTL8188EU"
#elif defined(CONFIG_RTL8723AU)
#define WIFI_NAME "RTL8723AU"
#endif 
#endif

static int major;
static dev_t dev_id;
static struct cdev* ist3020_cdev;
static struct device* ist3020_device;
static struct class* sysfs_class;
static struct spi_device* this_spi;
static uint8_t *lcd_buf;
static uint8_t contrast;
static unsigned int IST3020_CTL_A0;

#ifdef CONFIG_RII_USBHUB_STAT_PWR
static unsigned int IST3020_nRST;
static unsigned int SATA_PWR;
#endif

#ifdef CONFIG_PWR_CD
static unsigned int PWR_CD;
static int PWR_IRQ;
static DEFINE_MUTEX(irq_lock);
static DEFINE_MUTEX(pwr_lock);
struct proc_dir_entry *pwr_root_dir;
static int PWR_OFF = 0;
static unsigned int pwr_hold;
#endif

#ifdef CONFIG_WIFI_BT
unsigned int BT_RST_N;
unsigned int BT_WAKE;
unsigned int BT_HOST_WAKE;
unsigned int WL_REG_ON;
unsigned int WL_HOST_WAKE;
EXPORT_SYMBOL(BT_RST_N);
EXPORT_SYMBOL(BT_WAKE);
EXPORT_SYMBOL(BT_HOST_WAKE);
EXPORT_SYMBOL(WL_HOST_WAKE);
EXPORT_SYMBOL(WL_REG_ON);
unsigned int EIM_D31;
#endif

static unsigned int IST3020_backlight;
#if 0
static unsigned int key7;
static unsigned int key1;
static unsigned int key2;
static unsigned int key3;
static unsigned int key4;
static unsigned int key5;
#endif
static DEFINE_MUTEX(ist3020_lock);

static int probe_done = 0;
static int impossible = 0;
static void printASCII(unsigned int x, unsigned int y, char* str);
static void clearRam();

//-----------------------------------
// display data (192x64)
//-----------------------------------
static uint8_t Logo[]={
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0x80,0xc0,0xe0,0xe0,
0xf0,0xf8,0xe0,0xec,0x4c,0x5e,0x5e,0x1e,0x3f,0x3f,0x7f,0x7e,0xfe,0xfc,0xfd,0xf9,
0xfb,0xf3,0xf7,0xe7,0xef,0xcf,0xde,0x9e,0xbe,0x3e,0x7c,0x7c,0xf8,0xf8,0xf8,0xf0,
0xe0,0xe0,0xc0,0x80,0x80,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0xc0,0xe0,0xf8,0xfc,0xfe,0xff,0x3f,0x1f,0x0f,0x87,0xc7,
0xe3,0xf1,0xf9,0xfc,0xfe,0xff,0xff,0xfe,0xfe,0xfc,0x7c,0x38,0x18,0x00,0x01,0x01,
0x03,0x03,0x07,0x0f,0x0f,0x3f,0xff,0xff,0xff,0xff,0xff,0xfe,0xfc,0xf9,0xf1,0xe3,
0x07,0x07,0x0f,0x1f,0x3f,0xff,0xfe,0xfc,0xf8,0xe0,0xc0,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x08,0x0c,0x8c,0xec,0x6c,0x8c,0x8c,
0x8c,0xcc,0xcc,0x7c,0xf8,0xe0,0xa0,0x80,0x80,0x80,0x00,0x00,0x00,0x00,0x80,0x80,
0x80,0x80,0x80,0x00,0x00,0x00,0x00,0x80,0xe0,0xe0,0x80,0x80,0x80,0x00,0x00,0x00,
0x00,0x00,0x80,0x80,0x80,0x80,0x80,0x00,0x00,0xc0,0xe0,0xa0,0x80,0x80,0x80,0x00,
0x00,0x00,0x00,0x80,0xb8,0x38,0x38,0x00,0x00,0x80,0x80,0x00,0x00,0x80,0x80,0x00,
0x00,0x00,0x00,0x00,0x80,0x80,0x80,0x80,0x80,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0xc0,0xf0,0xfe,0x7f,0x3f,0x9f,0xcf,0xe7,0xf1,0xf8,0xfc,0xfe,0xff,0xff,0xff,
0xff,0x7f,0x3f,0x1f,0x0f,0x0f,0xf3,0xf1,0xf1,0xf0,0xf0,0xf0,0xf0,0xf0,0xf0,0xf0,
0xf0,0xf0,0xf8,0xf8,0xfc,0xff,0xff,0xff,0xff,0xff,0xff,0x7f,0x7f,0x3f,0x1f,0x0f,
0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x07,0x1f,0x7f,0xff,0xff,0xfe,0xf0,0xc0,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0c,0x0f,0x07,0x01,0x01,0x01,
0x01,0x00,0x00,0x00,0x06,0x01,0x00,0x0c,0x0f,0x0f,0x08,0x08,0x08,0x07,0x0f,0x09,
0x08,0x08,0x0b,0x0b,0x08,0x48,0x7c,0x3f,0x03,0x00,0x08,0x0d,0x0f,0x0b,0x08,0x08,
0x07,0x0f,0x09,0x08,0x0c,0x0f,0x0f,0x08,0x08,0x06,0x01,0x00,0x0c,0x0f,0x0f,0x08,
0x08,0x08,0x0c,0x0f,0x0f,0x08,0x08,0x08,0x0c,0x0f,0x07,0x02,0x0d,0x0f,0x0f,0x08,
0x08,0x08,0x07,0xff,0xf9,0xc8,0x7c,0x3f,0x0b,0x08,0x08,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0xfc,0xff,0xff,0xf0,0xf6,0x07,0x0f,0x0f,0x1f,0x1f,0x3f,0x1f,0x0f,0x07,0x03,0x01,
0x00,0x00,0x00,0x00,0x00,0xf0,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0x3f,0x3f,
0x7f,0x7f,0xff,0xff,0xfd,0xf9,0xf9,0xf9,0xf0,0xf0,0xe0,0xe0,0xc0,0xc0,0x80,0x80,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x07,0xff,0xff,0xff,0xff,0xfc,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xc0,0xe0,0x20,0x00,0x00,0x00,0xe0,0xe0,
0xe0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0xc0,0xe0,0x20,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0x00,0x00,0x00,
0x00,0x00,0x00,0xe0,0xe0,0xe0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x3f,0xff,0xff,0xff,0xff,0xe0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x03,0x07,0x0f,0x1f,0x1f,0x3f,0x3f,0x3f,0x7f,0xf8,0xf0,
0xf0,0xe0,0xf0,0xf8,0xf1,0xe1,0xe3,0xc3,0xc7,0x87,0x0f,0x0f,0x1f,0x1f,0x3f,0x3f,
0x7f,0x7f,0xfe,0xfe,0xfc,0xfc,0xf8,0xf8,0xf0,0xe0,0xc0,0x9f,0x3f,0x7f,0xff,0x3f,
0x00,0x00,0x00,0x00,0x00,0x00,0x30,0x3e,0x3f,0x21,0x20,0x20,0x30,0x3e,0x3e,0x20,
0x20,0x20,0x1c,0x3e,0x26,0x22,0x22,0x26,0x26,0x20,0x20,0x1c,0x3e,0x26,0x22,0x22,
0x2e,0x2c,0x20,0x20,0x30,0x3e,0x1e,0x08,0x34,0x3e,0x3e,0x20,0x20,0x20,0x1c,0x3e,
0x26,0x22,0x22,0x26,0x26,0x20,0x20,0x1c,0x3e,0x26,0x22,0x22,0x2e,0x2c,0x20,0x20,
0x1c,0x3c,0x26,0x22,0x32,0x3f,0x3f,0x21,0x20,0x20,0x00,0x00,0x00,0x00,0x00,0x00,
0x30,0x3e,0x1e,0x08,0x24,0x3e,0x1e,0x08,0x04,0x3e,0x3e,0x20,0x20,0x20,0x30,0x3e,
0x2e,0x20,0x30,0x3e,0x2e,0x20,0x20,0x20,0x30,0x30,0x38,0x27,0x3f,0x3c,0x20,0x20,
0x20,0x30,0x3e,0x3e,0x20,0x20,0x20,0x1c,0x3e,0x26,0x22,0x22,0x26,0x26,0x20,0x20,
0x00,0x03,0x0f,0x7f,0xff,0xff,0xfe,0xf8,0xe0,0x80,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0xc0,0xe0,0xf0,0xf8,0xfc,0xfe,0xff,0xff,
0xff,0xff,0x7f,0xbf,0x9f,0x8f,0x87,0x03,0x01,0x00,0x00,0x00,0x00,0x00,0x80,0x80,
0x80,0xc0,0xe0,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0x00,0x03,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0xc0,0xc0,
0xc0,0xc0,0xc0,0xc0,0xc0,0xc0,0xc0,0xc0,0x80,0x00,0x00,0x80,0x80,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x80,
0x80,0x80,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x03,0x07,0x1f,0x3f,0x7f,0xff,0xfc,0xf8,0xf0,0xe0,0xe0,
0x80,0x20,0x70,0x78,0xfc,0xfe,0xff,0xff,0xff,0xff,0x7f,0x3f,0x1f,0x0f,0x07,0x03,
0x01,0x00,0x00,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0x0f,0x0f,0x8f,0x8f,0xcf,
0xef,0xef,0xef,0xef,0xef,0xef,0x6f,0x27,0x17,0x03,0x03,0x01,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xc0,0xf8,
0x7e,0x16,0x18,0x18,0x18,0x0c,0x0c,0x07,0xc3,0xf8,0xff,0x87,0x80,0x80,0x70,0xf8,
0x98,0x88,0x88,0xb8,0xb0,0x80,0x80,0x70,0xf0,0x98,0x88,0xc8,0xf8,0xf8,0x80,0x80,
0xc0,0xc0,0xe0,0x9c,0xfe,0xf0,0x80,0x80,0x80,0x70,0xf8,0x98,0x88,0x88,0xb8,0xb0,
0x80,0x80,0x00,0x00,0x00,0x00,0x00,0x00,0xc0,0xf8,0xb8,0x80,0xc0,0xf8,0xb8,0x80,
0xc0,0xf8,0x38,0x00,0x70,0xf0,0x98,0x88,0xc8,0xf8,0xf8,0x80,0x80,0xc0,0xf8,0xfb,
0x83,0x83,0x80,0xc8,0xfc,0xfe,0x8f,0x88,0x80,0x80,0x00,0x00,0xe0,0xe0,0xe0,0x00,
0x00,0xe0,0xe0,0xe0,0x00,0x00,0xe0,0xe0,0xe0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x03,0x07,0x07,
0x0f,0x1f,0x1f,0x1e,0x3e,0x3c,0x7d,0x7d,0x7d,0x7c,0xfc,0xf8,0xf8,0xf8,0xf8,0xf8,
0xf8,0xf8,0x80,0x3f,0x7f,0xff,0xff,0xff,0xff,0x7f,0x7f,0x00,0x1f,0x1f,0x1f,0x0f,
0x07,0x07,0x03,0x01,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
};

static int ist3020_write_reg(struct spi_device *spi, uint8_t dat)
{
    struct spi_message msg;
    uint8_t buf_wdat[1];

    int status = 0;

    struct spi_transfer index_xfer = {
        .len            = 1,
    };

    spi_message_init(&msg);
    buf_wdat[0] = dat;
    index_xfer.tx_buf = buf_wdat;

    gpio_set_value(IST3020_CTL_A0, 0);
    spi_message_add_tail(&index_xfer, &msg);
    status = spi_sync(spi, &msg);
//    udelay(50);

    if(status){
        printk("spi_sync failed status = %d\n", status);
    }

    return status;
}

#ifdef CONFIG_PWR_CD
int rii_pwr_off(void)
{
    gpio_direction_output(pwr_hold, 0);
}
EXPORT_SYMBOL(rii_pwr_off);
#endif

static int ist3020_write_data(struct spi_device *spi, uint8_t dat)
{
    struct spi_message msg;
    uint8_t buf_wdat[1];

    int status = 0;

    struct spi_transfer index_xfer = {
        .len            = 1,
    };

    spi_message_init(&msg);
    buf_wdat[0] = dat;
    index_xfer.tx_buf = buf_wdat;

    gpio_set_value(IST3020_CTL_A0, 1);
    spi_message_add_tail(&index_xfer, &msg);
    status = spi_sync(spi, &msg);
    //udelay(50);

    if(status){
        printk("spi_sync failed status = %d\n", status);
    }

    return status;
}

static ssize_t ist3020_write(struct file * filp, const char __user * buf, size_t count, loff_t * ppos)
{
    //printk("IST3020 write called\n");
    int ret;
    char *str;
    uint8_t x, y, w, h;

    ret = copy_from_user(lcd_buf, buf, count);
    if(ret){
        printk("copy to lcd_buf failed\n");
        return 0;
    }

    x = *(lcd_buf + 1);
    y = *(lcd_buf + 3);
    w = *(lcd_buf + 5);
    h = *(lcd_buf + 7);
    str = lcd_buf + 5;
    //printk("IST3020 write passed copy_from_user check\n");
    switch(*lcd_buf){
        case 0://print ASCII
            //printk("IST3020 write called with value '1'\n");
            printASCII(x, y, str);
            break;
        case 1://Chinese characters
            break;
        case 2://reserved
            break;
        case 3://reserved
            break;
        case 4://reserved
//            printk("IST3020 write with value '4'\n");
//            printk("IST3020 write called with x=%d,y=%d,w=%d,h=%d\n", x, y, w, h);
            if(count >= 9 + w * (h / 8))
            {
//                printk("IST3020: Check passed...\n");
                uint8_t i, j;
                unsigned int k = 0;
                uint8_t x2 = x;
                uint8_t y2 = y;
                x2 += 0x20;
                for(i = 0; i < h / 8; i++){
//                    printk("In inner loop...i=%d\n",i);
                    ist3020_write_reg(this_spi, 0xb0 | y2);  //set page addr
                    ist3020_write_reg(this_spi, ((uint8_t)x2 >> 4) | 0x10); //set column addr of hight bits_4
                    ist3020_write_reg(this_spi, (uint8_t)x2 & 0xf); //set column addr of low bits_4
                    for(j = 0; j < w; j++){
//                        printk("In innerest loop...j=%d,k=%d\n",j,k);
                        ist3020_write_data(this_spi, lcd_buf[k+9]);
                        k++;
                    }
                    y2++;
                }
            }
            else
                printk("IST3020 ERROR: Buffer too small!\n");
            break;
        case 5://drawing
            break;
        default:
            return 0;
    }
    return 0;
}
static ssize_t ist3020_read(struct file * filp, char __user * buf, size_t count, loff_t * ppos)
{
    return 0;
}
/*
static void init_keys()
{
    int i;
    for(i = 0; i < 6; i++)
}
*/
static int ist3020_open(struct inode * node, struct file *filp)
{

    return 0;
}

static int ist3020_set_contrast(uint8_t * arg)
{
    if(*arg >= IST3020_MAX_CONTRAST)
        contrast = IST3020_MAX_CONTRAST;
    else
        contrast = *arg;
    printk("WWJ=======%s called contrast = 0x%2x\n", __func__, contrast);
    mutex_lock(&ist3020_lock);
    ist3020_write_reg(this_spi, 0x81);
    msleep(50);
    ist3020_write_reg(this_spi, contrast);
    mutex_unlock(&ist3020_lock);
    return 0;
}

static long ist3020_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    printk("WWJ========%s start\n", __func__);
    //printk("IST3020: BOBTEST\n");
    uint8_t cntrst;
    switch(cmd){
        case IST3020LCD_IOCTL_CLEAR:
            clearRam();
            break;
        case IST3020LCD_IOCTL_SET_CONTRAST:
            ist3020_set_contrast((uint8_t *)arg);
            break;
        case IST3020LCD_IOCTL_GET_CONTRAST:
            *(uint8_t*)arg = contrast; 
            break;
        case IST3020LCD_IOCTL_BACKLIGHT_ON:
	    gpio_direction_output(IST3020_backlight, 0);
            break;
        case IST3020LCD_IOCTL_BACKLIGHT_OFF:
	    gpio_direction_output(IST3020_backlight, 1);
            break;
        default:
            printk("illegal command\n");
            return -1;
    }
    return 0;
}
static const struct file_operations ist3020_user_fops={
        .owner  = THIS_MODULE,
        .write  = ist3020_write,
        .read   = NULL,
        .open   = NULL,
        .unlocked_ioctl = ist3020_ioctl,
};

#ifdef CONFIG_PWR_CD

static int show_powr_off(struct seq_file* seq, void* data)
{
    char status[3] = {0};
    sprintf(status, "%d\n", PWR_OFF);
    seq_puts(seq, status);
}

static int pwr_open(struct inode* inode, struct file* file)
{
    return single_open(file, show_powr_off, inode->i_private);
}

static const struct file_operations pwr_status_fops={
    .owner = THIS_MODULE,
    .open = pwr_open,
    .read = seq_read,
    .write = NULL,
};
#endif

#ifdef CONFIG_WIFI_IC
static int show_name(struct seq_file* seq, void* data)
{
    char wifi_name[20] = {0};
    sprintf(wifi_name, "%s\n", WIFI_NAME);
    seq_puts(seq, wifi_name);
}
static int ic_open(struct inode* inode, struct file* file)
{
    return single_open(file, show_name, inode->i_private);
}
static const struct file_operations wifi_ic_fops={
    .owner = THIS_MODULE,
    .open = ic_open,
    .read = seq_read,
    .write = NULL,
};
#endif

static void clearRam()
{
    printk("WWJ==========%s start\n", __func__);
    int i, j;
    uint8_t x = 0x20;  //the point (0, 0) is corresponding (0x20, 0)
    for(i = 0; i < 8; i++){
        ist3020_write_reg(this_spi, 0xb0 | i); //set page addr
        ist3020_write_reg(this_spi, 0x12);  //set column addr
        ist3020_write_reg(this_spi, 0x00);
        for(j = 0; j < 192; j++){
            ist3020_write_data(this_spi, 0x00);
        }
    }
}

static void fillRam(int x, int y)
{
    int i, j;
    for(i = 0; i < 8; i++){
        ist3020_write_reg(this_spi, 0xb0 | i); //set page addr
        ist3020_write_reg(this_spi, 0x12);  //set column addr
        ist3020_write_reg(this_spi, 0x00);
        for(j = 0; j < 192; j++){
            ist3020_write_data(this_spi, 0xff);
        }
    }
}

static void printASCII(unsigned int x, unsigned int y, char* str)
{
    int addr;
    int j;
    x += 0x20;  //the point (0, 0) is corresponding (0x20, 0)
    ist3020_write_reg(this_spi, 0xb0 | (uint8_t)y);
    ist3020_write_reg(this_spi, ((uint8_t)x >> 4) | 0x10);
    ist3020_write_reg(this_spi, (uint8_t)x & 0x0f);
    while(*str > 0){
        addr = *str++;
        addr = (addr - 0x20) * 8;
        for(j = 0; j < 6; j++){
            ist3020_write_data(this_spi, ASCIITAB[addr + j]);
        }
    }
}

//------汉字字符写入函数-----------------
static void printGB(uint8_t x, uint8_t y, unsigned long *pstr)
// 坐标(x,y)，x为水平方向像素列；y为垂直方向字符行（8点像素/行）
{
    unsigned long addr;
    uint8_t j,n;
    x += 0x20;
    while(*pstr>0){
        addr=*pstr++;                     // 取汉字代码
        addr=(addr-1)*32;                 // 计算汉字字模起始地址
        for (n=0;n<2;n++)
        {
            ist3020_write_reg(this_spi, y | 0xb0);  // 设置页地址
            ist3020_write_reg(this_spi, (x>>4)|0x10); // 设置列地址高4位
            ist3020_write_reg(this_spi, x & 0x0f);   // 设置列地址低4位
            for (j=0;j<16;j++)             // 写16字节字模数据
            {
                ist3020_write_data(this_spi, CCTAB[addr+j+16*n]);  // 写字模数据
            }
            y=y+1;
        }                             // 页地址加1
        y=y-2;                           // 页地址修正原值
        x=x+16;                          // 列地址修正下一个汉字位置
    }
}
static void showBmp(unsigned int x, unsigned int y, unsigned int width, unsigned int height, uint8_t* bmp)
{
    uint8_t i, j;
    unsigned int k = 0;
    x += 0x20;
    for(i = 0; i < height; i++){
        ist3020_write_reg(this_spi, 0xb0 | y);  //set page addr
        ist3020_write_reg(this_spi, ((uint8_t)x >> 4) | 0x10); //set column addr of hight bits_4
        ist3020_write_reg(this_spi, (uint8_t)x & 0xf); //set column addr of low bits_4
        for(j = 0; j < width; j++){
            ist3020_write_data(this_spi, bmp[k]);
            k++;
        }
        y++;
    }
}

static int ist3020lcd_init()
{
    int ret;
    
    gpio_set_value(IST3020_nRST, 0);
    msleep(50);
    gpio_set_value(IST3020_nRST, 1);
    msleep(50);

    contrast = 0x20;
    //lock the lock
    mutex_lock(&ist3020_lock);
    //Initial Display Line 0
    //printk("D16_mux_ctrl = %d\n", __raw_readl(ioremap(0x20e0140, 4)));
    //printk("D18_mux_ctrl = %d\n", __raw_readl(ioremap(0x20e014c, 4)));
    ret = ist3020_write_reg(this_spi, 0x40);
    //ADC =0
    ret = ist3020_write_reg(this_spi, 0xa0);
    //Reverse Display ON/OFF=OFF
    ret = ist3020_write_reg(this_spi, 0xa6);
    //Entire Display ON/OFF=OFF
    ret = ist3020_write_reg(this_spi, 0xa4);
    //LCD Bias select 1/9
    ret = ist3020_write_reg(this_spi, 0xa2);
    //SHL slect 0
    ret = ist3020_write_reg(this_spi, 0xc8);
    //set refresh hz
    ret = ist3020_write_reg(this_spi, 0x82);
    ret = ist3020_write_reg(this_spi, 0x0e);
    //set number of line reverse
    ret = ist3020_write_reg(this_spi, 0x30);
    //RA/RB
    ret = ist3020_write_reg(this_spi, 0x23);
    //open vc
    ret = ist3020_write_reg(this_spi, 0x2c);
    msleep(50);
    //open vc,vr
    ret = ist3020_write_reg(this_spi, 0x2e);
    msleep(50);
    //open vc,vr, vf
    ret = ist3020_write_reg(this_spi, 0x2f);
    msleep(50);
    //set contrast 
    ret = ist3020_write_reg(this_spi, 0x81);
    ret = ist3020_write_reg(this_spi, contrast);
    //Built-in OSC =ON
    ret = ist3020_write_reg(this_spi, 0xab);
    //Display ON/OFF = ON
    ret = ist3020_write_reg(this_spi, 0xaf);
    mutex_unlock(&ist3020_lock);
    printk("ist3020_write ret = %d\n", ret);
    return 0;
}

#ifdef CONFIG_WIFI_BT
static ssize_t bt_rst(struct device* dev, struct device_attribute* attr, const char* buf, size_t count)
{
    unsigned long long value;
    value = simple_strtoul(buf, NULL, 0);
    gpio_direction_output(BT_RST_N, value ? 1:0);
    
    return count;
}
static DEVICE_ATTR(bt_rst_ctl, 0666, NULL, bt_rst);
#endif

#ifdef CONFIG_RII_USBHUB_STAT_PWR
static ssize_t sata_pwr_rst(struct device* dev, struct device_attribute* attr, const char* buf, size_t count)
{
    unsigned long long value;
    value = simple_strtoul(buf, NULL, 0);
    if(1 == value){
        gpio_direction_output(SATA_PWR, 0);
        msleep(20);
        gpio_direction_output(SATA_PWR, 1);
    }
    return count;
}
static DEVICE_ATTR(sata_rst, 0222, NULL, sata_pwr_rst);
#endif

#ifdef CONFIG_PWR_CD
static irqreturn_t pwr_irq_handler(int irq, void *data){
    printk("WWJ======%s start\n", __func__);
    mutex_lock(&irq_lock);
    disable_irq_nosync(irq);
    mutex_unlock(&irq_lock);
    return IRQ_WAKE_THREAD; //trigger the pwr_irq_fun func
}

static irqreturn_t pwr_irq_fun(int irq, void* data){

    int value;
    printk("WWJ========%s start\n", __func__);
    msleep(50);
    mutex_lock(&pwr_lock);
    value = gpio_get_value(PWR_CD);
    if(value)
        PWR_OFF = 1;
    else
        PWR_OFF = 0;
    mutex_unlock(&pwr_lock);
    enable_irq(irq);
}
#endif

static int ist3020lcd_probe(struct spi_device *spi)
{
    printk("WWJ========%s start\n", __func__);
    int ret;
#ifdef CONFIG_RII_USBHUB_STAT_PWR
    int usb_hub, bt_rst;
#endif

#ifdef CONFIG_PWR_CD
    int irq_registered = 0;
    pwr_root_dir = proc_mkdir("power", NULL);
    //struct proc_dir_entry* power_off;
#endif

#ifdef CONFIG_WIFI_IC
    struct proc_dir_entry *wifi_ic = proc_mkdir("wifi_ic", NULL);
#endif

    unsigned long str[] = {0x0c, 0x0d, 0x06, 0x07, 0x08, 0x09, 0xa, 0x0b, 0x00};

    this_spi = spi;
    struct device_node* np = spi->dev.of_node;
    if(!of_device_is_available(np)){
        printk("spi->dev.node is unavailable\n");
        return -ENODEV;
    }
#ifdef CONFIG_RII_USBHUB_STAT_PWR	
    usb_hub = of_get_named_gpio(np, "usb-hub", 0);
    if (!gpio_is_valid(usb_hub)){
         printk("can not find usb_hub gpio pins\n");
         return -ENODEV;
    }
/*
    bt_rst = of_get_named_gpio(np, "bt_rst", 0);
     if (!gpio_is_valid(bt_rst)){
         printk("can not find bt_rst gpio pins\n");
         return -ENODEV;
    }
*/
    ret = gpio_request(usb_hub, "usb-hub");
    if(ret){
        printk("request gpio usb-hub failed\n");
        return ret;
    }
/*
      ret = gpio_request(bt_rst, "bt_rst");
    if(ret){
        printk("request gpio bt_rst failed\n");
        return ret;
    }
*/
    gpio_direction_output(usb_hub, 0);
    msleep(50);
    gpio_direction_output(usb_hub, 1);
 /* 
   gpio_direction_output(bt_rst, 0);
    msleep(50);
    gpio_direction_output(bt_rst, 1);
   */ 
#endif

#ifdef CONFIG_PWR_CD

    pwr_hold = of_get_named_gpio(np, "pwr_hold", 0);
    if(!gpio_is_valid(pwr_hold)){
        printk("can't find pwr_hold gpio pin\n");
        return -ENODEV;
    }
    ret = gpio_request(pwr_hold, "pwr_hold");
    if(ret){
        printk("request gpio pwr_hold failed\n");
        return ret;
    }

    PWR_CD = of_get_named_gpio(np, "pwr_cd", 0);
    if(!gpio_is_valid(PWR_CD)){
        printk("can't find pwr_cd gpio pin\n");
        return -ENODEV;
    }

    ret = gpio_request(PWR_CD, "pwr_cd");
    if(ret){
        printk("request gpio pwr_cd failed\n");
        return ret;
    }
    PWR_IRQ = gpio_to_irq(PWR_CD);
    if(PWR_IRQ < 0){
        printk("gpio pwr_cd to irq failed \n");
        return -EINVAL;
    }
    ret = request_threaded_irq(PWR_IRQ, pwr_irq_handler, pwr_irq_fun, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, "pwr_cd", &spi->dev);
    if(ret){
        printk(KERN_WARNING "WWJ======%s:request_irq failed\n", __func__);
        goto irq_fail;
    }else
        irq_registered = 1;

    proc_create("power_off", 0440, pwr_root_dir, &pwr_status_fops);
#ifdef CONFIG_WIFI_IC
    proc_create("name", 0440, wifi_ic, &wifi_ic_fops);
#endif
    /*
    if(!power_off){
        printk("%s:proc_create power_off failed\n", __func__);
    }else{
        proc_set_size(power_off, sizeof(PWR_OFF));
    }*/
#endif

    IST3020_nRST = of_get_named_gpio(np, "ist3020-nRst", 0);
    if (!gpio_is_valid(IST3020_nRST)){
         printk("can not find ist3020-nRst gpio pins\n");
         return -ENODEV;
    }

#ifdef CONFIG_RII_USBHUB_STAT_PWR
    SATA_PWR = of_get_named_gpio(np, "sata-power", 0);
    if (!gpio_is_valid(SATA_PWR)){
         printk("can not find stat_power gpio pins\n");
         return -ENODEV;
    }
#endif
           
    IST3020_CTL_A0 = of_get_named_gpio(np, "ist3020-A0", 0);
    if(!gpio_is_valid(IST3020_CTL_A0)){
        printk("can not find ist3020-A0 gpio pin\n");
        return -ENODEV;
    }
    IST3020_backlight = of_get_named_gpio(np, "ist3020-backlight", 0);
    if(!gpio_is_valid(IST3020_backlight)){
        printk("can not find ist3020-backlight gpio pin\n");
        return -ENODEV;
    }

    ret = gpio_request(IST3020_nRST, "ist3020_rst");
    if(ret){
        printk("request gpio ist3020_rst failed\n");
        return ret;
    }
    gpio_direction_output(IST3020_nRST, 1);

#ifdef CONFIG_RII_USBHUB_STAT_PWR 
    ret = gpio_request(SATA_PWR, "stat_power");
      if(ret){
        printk("request gpio stat_power failed\n");
        return ret;
    }
    gpio_direction_output(SATA_PWR, 1);

    ret = sysfs_create_file(&spi->dev.kobj, &dev_attr_sata_rst.attr);
    if(ret)
        printk("------------sysfs_create_file failed---------\n");
#endif

#ifdef CONFIG_WIFI_BT
    ret = sysfs_create_file(&spi->dev.kobj, &dev_attr_bt_rst_ctl.attr);
    if(ret)
        printk("------------sysfs_create_file failed---------\n");
#endif
    ret = gpio_request(IST3020_CTL_A0, "ist3020_a0");
    if(ret){
        printk("request gpio ist3020_a0 failed\n");
        return ret;
    }
    gpio_direction_output(IST3020_CTL_A0, 1);

    ret = gpio_request(IST3020_backlight, "ist3020-backlight");
    if(ret){
        printk("request gpio ist3020-backlight failed\n");
        return ret;
    }
    gpio_direction_output(IST3020_backlight, 0);


    lcd_buf = (uint8_t*)kmalloc(IST3020_WIDTH * IST3020_HEIGHT * 10, GFP_KERNEL);
    if(!lcd_buf){
        printk(KERN_ERR "kmalloc lcd_buf failed\n");
        return -1;
    }
    

    ist3020lcd_init();

#ifdef CONFIG_WIFI_BT
    
    BT_RST_N = of_get_named_gpio(np, "bt_rst_n", 0);
    if (!gpio_is_valid(BT_RST_N)){
         printk("can not find BT_RST_N gpio pins\n");
    }
    
    BT_WAKE = of_get_named_gpio(np, "bt_wake", 0);
    if (!gpio_is_valid(BT_WAKE)){
         printk("can not find BT_WAKE gpio pins\n");
    }
    BT_HOST_WAKE = of_get_named_gpio(np, "bt_host_wake", 0);
    if (!gpio_is_valid(BT_HOST_WAKE)){
         printk("can not find BT_HOST_WAKE gpio pins\n");
    }
    WL_HOST_WAKE =of_get_named_gpio(np, "wl_host_wake", 0);
    if(!gpio_is_valid(WL_HOST_WAKE)){
        printk("can not find WL_HOST_WAKE gpio pins\n");
    }
    WL_REG_ON =of_get_named_gpio(np, "wl_reg_on", 0);
    if(!gpio_is_valid(WL_REG_ON)){
        printk("can not find WL_REG_ON gpio pins\n");
    }  

    /*
    EIM_D31 = of_get_named_gpio(np, "eim_d31", 0);
    if(!gpio_is_valid(EIM_D31)){
        printk("can not find EIM_D31 gpio pins\n");
    }
    */
    
    ret = gpio_request(BT_RST_N, "BT_RST_N");
    if(ret){
        printk("request gpio BT_RST_N failed\n");
    }
    
    /*
    ret = gpio_request(EIM_D31, "EIM_D31");
    if(ret){
        printk("request gpio EIM_D31 failed\n");
    }
    */
    
    gpio_direction_output(BT_RST_N, 0);
    //gpio_direction_output(EIM_D31, 0);
    msleep(50);
    gpio_direction_output(BT_RST_N, 1);
    //msleep(5);
    
    //gpio_direction_output(EIM_D31, 1);
    ret = gpio_request(BT_WAKE, "BT_WAKE");
    if(ret){
        printk("request gpio BT_WAKE failed\n");
    }
    
    gpio_direction_output(BT_WAKE, 1);
    ret = gpio_request(BT_HOST_WAKE, "BT_HOST_WAKE");
    if(ret){
        printk("request gpio BT_HOST_WAKE failed\n");
    }
    ret = gpio_request(WL_HOST_WAKE, "WL_HOST_WAKE");
    if(ret){
        printk("request gpio WL_HOST_WAKE failed\n");
    }
    /*
    ret = gpio_request(WL_REG_ON, "WL_REG_ON");
    if(ret){
        printk("request gpio WL_REG_ON failed\n");
    }
    gpio_direction_output(WL_REG_ON, 1);
    */
#endif   
    ret = alloc_chrdev_region(&dev_id, 0, IST3020_COUNT, IST3020_NAME);
    if(ret){
        printk(KERN_ERR IST3020_NAME ": alloc_chrdev_region failed\n");
        return -1;
    }
    major = MAJOR(dev_id);

    ist3020_cdev = cdev_alloc();
    ist3020_cdev->ops = &ist3020_user_fops;
    ist3020_cdev->owner = THIS_MODULE;
    cdev_add(ist3020_cdev, dev_id, IST3020_COUNT);

    sysfs_class = class_create(THIS_MODULE, "ist3020");
    if (IS_ERR(sysfs_class)) {
        printk(KERN_ERR "osst :W: Unable to register sysfs class\n");
        return PTR_ERR(sysfs_class);
    }

    ist3020_device = device_create(sysfs_class, NULL, dev_id,
                                                NULL, IST3020_NAME);

    if (IS_ERR(ist3020_device)) {
        printk(KERN_WARNING "osst :W: Unable to add sysfs class member %s\n", IST3020_NAME);
        return PTR_ERR(ist3020_device);
    }
    clearRam();
    //printASCII(10, 4, "SHENZHEN SHENCLOUD LTD.");
    //printASCII(17, 6, "Version1.0.0");
    //msleep(1000);
    //clearRam();
    //printk("KEY_ROW2 = 0x%x\n", __raw_readl(ioremap(0x20e0260, 4)));

    while(0){
        //ist3020_write_reg(this_spi, 0x40);
    }

    showBmp(0, 0, 192, 8, Logo);
    probe_done = 1;
    //printGB(4, 3, str);
    //while(1);
#if 0    
    x = 0x1F;
    y = 0;
    clearRam();
    msleep(500);
    fillRam(0, 0);
    msleep(1000);
   
    while(1){
        clearRam();
        printk("y = %d\n", y);
        printASCII(x, y, "SHENZHEN TOPWAY LTD.");
        msleep(500);
        y++;
        if(y == 8)
            while(1);
         //write_screen(Logo);
        //printk("WWJ================\n");
    }
#endif
    return 0;
#ifdef CONFIG_PWR_CD
irq_fail:
    if(irq_registered)
        free_irq(PWR_IRQ, &spi->dev);
#endif
}


static int ist3020lcd_remove(struct spi_device *spi)
{

    cdev_del(ist3020_cdev);
    unregister_chrdev_region(dev_id, IST3020_COUNT);
    return 0;
}

static const struct of_device_id spidev_ist3020_ids[] = {
    { .compatible = "ist3020lcd_ctrl" },
    {},
};
MODULE_DEVICE_TABLE(of, spidev_ist3020_ids);
static struct spi_driver ist3020_driver = {
    .driver = {
        .name   = "ist3020lcd_ctrl",
        //.bus    = &spi_bus_type,
        .owner  = THIS_MODULE,
        .of_match_table = of_match_ptr(spidev_ist3020_ids),
    },
    .probe  = ist3020lcd_probe,
    .remove = ist3020lcd_remove,

    /* REVISIT: many of these chips have deep power-down modes, which
     * should clearly be entered on suspend() to minimize power use.
     * And also when they're otherwise idle...
     */
};


static int __init ist3020lcd_ctrl_init(void)
{
    return spi_register_driver(&ist3020_driver);
}


static void __exit ist3020lcd_ctrl_exit(void)
{
    spi_unregister_driver(&ist3020_driver);
}


module_init(ist3020lcd_ctrl_init);
module_exit(ist3020lcd_ctrl_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("WenJing Wang");
MODULE_DESCRIPTION("vk32xx generic serial port driver");
