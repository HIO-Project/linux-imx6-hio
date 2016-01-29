/*
 * AD7091R5 Analog -> Digital converter driver
 *
 * Copyright 2014 Analog Devices Inc.
 * Author: Paul Cercueil <paul.cercueil@analog.com>
 *
 * Licensed under the GPL-2.
*/

#include "ad7091r-base.h"
#include <linux/i2c.h>
#include <linux/iio/iio.h>
#include <linux/module.h>

#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/spi/spi.h>
#include <linux/regulator/consumer.h>
#include <linux/err.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/module.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer.h>
#include <linux/iio/trigger.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/adc/ad_sigma_delta.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/events.h>
#include <linux/iio/buffer.h>
#include <linux/iio/driver.h>
#include <linux/iio/kfifo_buf.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>


#if 0
static const struct iio_event_spec ad7091r5_events[] = {
	{
	.type = IIO_EV_TYPE_THRESH,
	.dir = IIO_EV_DIR_RISING,
 	.mask_separate = BIT(IIO_EV_INFO_VALUE) |
 			BIT(IIO_EV_INFO_ENABLE),
 	},
 	{
 		.type = IIO_EV_TYPE_THRESH,
 		.dir = IIO_EV_DIR_FALLING,
 		.mask_separate = BIT(IIO_EV_INFO_VALUE) |
 			BIT(IIO_EV_INFO_ENABLE),
 	},
 	{
 		.type = IIO_EV_TYPE_THRESH,
 		.dir = IIO_EV_DIR_EITHER,
 		.mask_separate = BIT(IIO_EV_INFO_HYSTERESIS),
 	},
 };
 
#define AD7091R_CHANNEL(idx, bits, ev, num_ev) { \
	.type = IIO_VOLTAGE, \
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW), \
	.indexed = 1, \
	.channel = idx, \
	.event_spec = ev, \
	.num_event_specs = num_ev, \
}

static const struct iio_chan_spec ad7091r5_channels_irq[] = {
	AD7091R_CHANNEL(0, 12, ad7091r5_events, ARRAY_SIZE(ad7091r5_events)),
	AD7091R_CHANNEL(1, 12, ad7091r5_events, ARRAY_SIZE(ad7091r5_events)),
	AD7091R_CHANNEL(2, 12, ad7091r5_events, ARRAY_SIZE(ad7091r5_events)),
	AD7091R_CHANNEL(3, 12, ad7091r5_events, ARRAY_SIZE(ad7091r5_events)),
};

static const struct iio_chan_spec ad7091r5_channels_noirq[] = {
	AD7091R_CHANNEL(0, 12, NULL, 0),
	AD7091R_CHANNEL(1, 12, NULL, 0),
	AD7091R_CHANNEL(2, 12, NULL, 0),
	AD7091R_CHANNEL(3, 12, NULL, 0),
};
#endif

//add ben
#define AD7091R5_EV_M						\
		(IIO_EV_BIT(IIO_EV_TYPE_THRESH, IIO_EV_DIR_RISING)	\
	 | IIO_EV_BIT(IIO_EV_TYPE_THRESH, IIO_EV_DIR_FALLING)	\
	 | IIO_EV_BIT(IIO_EV_TYPE_THRESH, IIO_EV_DIR_EITHER))
	 
#define AD7091R_CHANNEL(idx, bits, ev) { \
	.type = IIO_VOLTAGE, \
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW), \
	.indexed = 1, \
	.channel = idx, \
	.event_mask = ev, \
}

static const struct iio_chan_spec ad7091r5_channels_irq[] = {
	AD7091R_CHANNEL(0, 12, AD7091R5_EV_M),
	AD7091R_CHANNEL(1, 12, AD7091R5_EV_M),
	AD7091R_CHANNEL(2, 12, AD7091R5_EV_M),
	AD7091R_CHANNEL(3, 12, AD7091R5_EV_M),
};

static const struct iio_chan_spec ad7091r5_channels_noirq[] = {
	AD7091R_CHANNEL(0, 12, NULL),
	AD7091R_CHANNEL(1, 12, NULL),
	AD7091R_CHANNEL(2, 12, NULL),
	AD7091R_CHANNEL(3, 12, NULL),
};

#undef AD7091R_CHANNEL

static const struct ad7091r_chip_info ad7091r5_chip_info_irq = {
	.channels = ad7091r5_channels_irq,
	.num_channels = ARRAY_SIZE(ad7091r5_channels_irq),
};

static const struct ad7091r_chip_info ad7091r5_chip_info_noirq = {
	.channels = ad7091r5_channels_noirq,
	.num_channels = ARRAY_SIZE(ad7091r5_channels_noirq),
};

static int ad7091r5_i2c_probe(struct i2c_client *i2c,
		const struct i2c_device_id *id)
{
	const struct ad7091r_chip_info *chip_info;
	struct regmap *map = devm_regmap_init_i2c(i2c, &ad7091r_regmap_config);
	if (IS_ERR(map))
		return PTR_ERR(map);

	if (i2c->irq)
		chip_info = &ad7091r5_chip_info_irq;
	else
		chip_info = &ad7091r5_chip_info_noirq;

	printk("ben:ad7091r5_i2c_probe \n");
	return ad7091r_probe(&i2c->dev, id->name, chip_info, map, i2c->irq);
}

static int ad7091r5_i2c_remove(struct i2c_client *i2c)
{
	return ad7091r_remove(&i2c->dev);
}

static const struct i2c_device_id ad7091r5_i2c_ids[] = {
	{"ad7091r5", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, ad7091r5_i2c_ids);

static struct i2c_driver ad7091r5_driver = {
	.driver = {
		.name = "ad7091r5",
		.owner = THIS_MODULE,
	},
	.probe = ad7091r5_i2c_probe,
	.remove = ad7091r5_i2c_remove,
	.id_table = ad7091r5_i2c_ids,
};
module_i2c_driver(ad7091r5_driver);

MODULE_AUTHOR("Paul Cercueil <paul.cercueil@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD7091R5 multi-channel ADC driver");
MODULE_LICENSE("GPL v2");
