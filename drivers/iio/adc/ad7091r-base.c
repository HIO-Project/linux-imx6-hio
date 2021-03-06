/*
 * AD7091R5 Analog -> Digital converter driver
 *
 * Copyright 2014 Analog Devices Inc.
 * Author: Paul Cercueil <paul.cercueil@analog.com>
 *
 * Licensed under the GPL-2.
 */

#include "ad7091r-base.h"

#include <linux/bitops.h>
#include <linux/iio/events.h>
#include <linux/iio/iio.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/regmap.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>

#define AD7091R_REG_RESULT  0
#define AD7091R_REG_CHANNEL 1
#define AD7091R_REG_CONF    2
#define AD7091R_REG_ALERT   3
#define AD7091R_REG_CH_LOW_LIMIT(ch) ((ch) * 3 + 4)
#define AD7091R_REG_CH_HIGH_LIMIT(ch) ((ch) * 3 + 5)
#define AD7091R_REG_CH_HYSTERESIS(ch) ((ch) * 3 + 6)

/* AD7091R_REG_RESULT */
#define REG_RESULT_CH_ID(x)	    (((x) >> 13) & 0x3)
#define REG_RESULT_CONV_RESULT(x)   ((x) & 0xfff)

/* AD7091R_REG_CONF */
#define REG_CONF_AUTO   BIT(8)
#define REG_CONF_CMD    BIT(10)

#define REG_CONF_MODE_MASK  (REG_CONF_AUTO | REG_CONF_CMD)

enum ad7091r_mode {
	AD7091R_MODE_SAMPLE,
	AD7091R_MODE_COMMAND,
	AD7091R_MODE_AUTOCYCLE,
};

struct ad7091r_state {
	struct device *dev;
	struct regmap *map;
	const struct ad7091r_chip_info *chip_info;
	enum ad7091r_mode mode;
};

static int ad7091r_set_mode(struct ad7091r_state *st, enum ad7091r_mode mode)
{
	int ret;

	switch (mode) {
	case AD7091R_MODE_SAMPLE:
		ret = regmap_update_bits(st->map, AD7091R_REG_CONF,
				REG_CONF_MODE_MASK, 0);
		break;
	case AD7091R_MODE_COMMAND:
		ret = regmap_update_bits(st->map, AD7091R_REG_CONF,
				REG_CONF_MODE_MASK, REG_CONF_CMD);
		break;
	case AD7091R_MODE_AUTOCYCLE:
		ret = regmap_update_bits(st->map, AD7091R_REG_CONF,
				REG_CONF_MODE_MASK, REG_CONF_AUTO);
		break;
 	default:
 		ret = -EINVAL;
 		break;
 	}
 
 	if (!ret)
 		st->mode = mode;
 	return ret;
}
 
static int ad7091r_set_channel(struct ad7091r_state *st, unsigned channel)
{
	unsigned int foo;
	int ret;

	/* AD7091R_REG_CHANNEL is a 8-bit register */
	ret = regmap_write(st->map, AD7091R_REG_CHANNEL,
			BIT(channel) | (BIT(channel) << 8));
	if (ret)
		return ret;

	/* There is a latency of one conversion before the channel conversion
	 * sequence is updated */
	return regmap_read(st->map, AD7091R_REG_RESULT, &foo);
}

static int ad7091r_read_one(struct iio_dev *iio_dev,
		unsigned channel, unsigned int *read_val)
{
	struct ad7091r_state *st = iio_priv(iio_dev);
	unsigned val;
	int ret;

	/* TODO: locking */
	ret = ad7091r_set_channel(st, channel);
	if (ret)
		return ret;

	ret = regmap_read(st->map, AD7091R_REG_RESULT, &val);
	if (ret)
		return ret;

	if (REG_RESULT_CH_ID(val) != channel)
		return -EIO;

	*read_val = REG_RESULT_CONV_RESULT(val);
	return 0;
}

static int ad7091r_read_raw(struct iio_dev *iio_dev,
			   struct iio_chan_spec const *chan,
			   int *val, int *val2, long m)
{
	struct ad7091r_state *st = iio_priv(iio_dev);
	unsigned int read_val;
	int ret;

	mutex_lock(&iio_dev->mlock);

	switch (m) {
	case IIO_CHAN_INFO_RAW:
		if (st->mode != AD7091R_MODE_COMMAND) {
			ret = -EBUSY;
			goto unlock;
		}

		ret = ad7091r_read_one(iio_dev, chan->channel, &read_val);
		if (ret)
			goto unlock;

		*val = read_val;
		ret = IIO_VAL_INT;
		break;

	default:
		ret = -EINVAL;
		break;
	}

unlock:
	mutex_unlock(&iio_dev->mlock);
	return ret;
}

static const struct iio_info ad7091r_info = {
	.read_raw = ad7091r_read_raw,
	.driver_module = THIS_MODULE,
};

static irqreturn_t ad7091r_event_handler(int irq, void *private)
{
 	struct ad7091r_state *st = (struct ad7091r_state *) private;
 	struct iio_dev *iio_dev = dev_get_drvdata(st->dev);
 	unsigned i, read_val;
 	int ret;
 	s64 timestamp = iio_get_time_ns();
 
 	ret = regmap_read(st->map, AD7091R_REG_ALERT, &read_val);
 	if (ret)
 		return IRQ_HANDLED; /* TODO */
 
 	for (i = 0; i < st->chip_info->num_channels; i++) {
		if (read_val & BIT(i * 2))
			iio_push_event(iio_dev,
 					IIO_UNMOD_EVENT_CODE(IIO_VOLTAGE, i,
 						IIO_EV_TYPE_THRESH,
 						IIO_EV_DIR_RISING), timestamp);
 		if (read_val & BIT(i * 2 + 1))
 			iio_push_event(iio_dev,
 					IIO_UNMOD_EVENT_CODE(IIO_VOLTAGE, i,
 						IIO_EV_TYPE_THRESH,
 						IIO_EV_DIR_FALLING), timestamp);
 	}
 
 	return IRQ_HANDLED;
}

int ad7091r_probe(struct device *dev, const char *name,
 		const struct ad7091r_chip_info *chip_info,
 		struct regmap *map, int irq)
{
 	struct iio_dev *iio_dev;
 	struct ad7091r_state *st;
 	int ret;
 
 	//iio_dev = devm_iio_device_alloc(dev, sizeof(*st));
	iio_dev = iio_device_alloc(sizeof(*st));
 	if (!iio_dev)
 		return -ENOMEM;
 
 	st = iio_priv(iio_dev);
 	st->dev = dev;
 	st->chip_info = chip_info;
 	st->map = map;
 	dev_set_drvdata(dev, iio_dev);
 
 	iio_dev->dev.parent = dev;
 	iio_dev->name = name;
 	iio_dev->info = &ad7091r_info;
 	iio_dev->modes = INDIO_DIRECT_MODE;
 
 	iio_dev->num_channels = chip_info->num_channels;
 	iio_dev->channels = chip_info->channels;
 
 	if (irq) {
 		ret = devm_request_threaded_irq(dev, irq, NULL,
 				ad7091r_event_handler,
 				IRQF_TRIGGER_FALLING | IRQF_ONESHOT, name, st);
 		if (ret)
 			return ret;
	}

 	/* Use command mode by default */
 	ret = ad7091r_set_mode(st, AD7091R_MODE_COMMAND);
 	if (ret < 0)
 		return ret;
 
 	ret = iio_device_register(iio_dev);
 	if (ret)
 		return ret;
 
 	dev_dbg(dev, "Probed\n");
 	return 0;
}
EXPORT_SYMBOL_GPL(ad7091r_probe);
 
int ad7091r_remove(struct device *dev)
{
 	struct iio_dev *iio_dev = dev_get_drvdata(dev);
 
 	iio_device_unregister(iio_dev);
 	return 0;
}
EXPORT_SYMBOL_GPL(ad7091r_remove);

static bool ad7091r_writeable_reg(struct device *dev, unsigned reg)
{
 	switch (reg) {
 	case AD7091R_REG_RESULT:
 	case AD7091R_REG_ALERT:
 		return false;
 	default:
 		return true;
 	}
}
 
static bool ad7091r_volatile_reg(struct device *dev, unsigned reg)
{
 	switch (reg) {
 	case AD7091R_REG_RESULT:
 	case AD7091R_REG_ALERT:
 		return true;
 	default:
 		return false;
 	}
}
 
const struct regmap_config ad7091r_regmap_config = {
	.reg_bits = 8,
	.val_bits = 16,
	.writeable_reg = ad7091r_writeable_reg,
	.volatile_reg = ad7091r_volatile_reg,
};
EXPORT_SYMBOL_GPL(ad7091r_regmap_config);
 
//MODULE_AUTHOR("Paul Cercueil <paul.cercueil@analog.com>");
//MODULE_DESCRIPTION("Analog Devices AD7091Rx multi-channel converters");
//MODULE_LICENSE("GPL v2");
