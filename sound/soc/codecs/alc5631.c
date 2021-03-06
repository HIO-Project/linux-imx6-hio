/*
 * alc5631.c  --  ALC5631 ALSA Soc Audio driver
 *
 * Copyright 2011 Realtek Microelectronics
 *
 * Author: flove <flove@realtek.com>
 *
 * Based on WM8753.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/io.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/tlv.h>

#include "alc5631.h"

#define ALC5631_DEBUG
#define ALC5631_DEMO 0 /* only for demo; please remove it */
#if 1// ALC5631_DEMO

static struct snd_soc_codec *pcodec;
struct alc5631_init_reg {
	u8 reg;
	u16 val;
};

static struct alc5631_init_reg init_list[] = {
	{ALC5631_SOFT_VOL_CTRL, 0x0},
	{ALC5631_SPK_OUT_VOL		, 0xc8c8},
	{ALC5631_HP_OUT_VOL		, 0xc0c0 /*0xc0c0*/}, //FAE
	//JOIN 31	{ALC5631_MONO_AXO_1_2_VOL	, 0xa080},
	{ALC5631_ADC_REC_MIXER		, 0xb0b0}, //0xb0b0
	{ALC5631_MIC_CTRL_2		, 0x8800}, //0x3300
	{ALC5631_OUTMIXER_L_CTRL		, 0xdfc0/*0xdfC0*/}, //0x1a
	{ALC5631_OUTMIXER_R_CTRL		, 0xdfc0/*0xdfC0*/}, //0x1c
	{ALC5631_SPK_MIXER_CTRL		, 0xd8d8}, //0x28
	{ALC5631_SPK_MONO_OUT_CTRL	, 0x6c00}, //0x2a
	//	{ALC5631_GEN_PUR_CTRL_REG	, 0x4e00},
	{ALC5631_SPK_MONO_HP_OUT_CTRL	, 0x0000},
	{ALC5631_INT_ST_IRQ_CTRL_2	, 0x0f18}, //0x0f18
	{ALC5631_MIC_CTRL_1	        	, 0x0000/*0x8080*/}, //0x0e
	//	{ALC5631_PLL_CTRL, 0x4809}, //0x44
	{ALC5631_PWR_MANAG_ADD2, 0xffff /*0x8000*/}, //0x3a
	{ALC5631_GEN_PUR_CTRL_REG, 0x3e00},
	//	{ALC5631_PWR_MANAG_ADD3, 0x8000 /*0x8000*/}, //0x3c
	//	{ALC5631_GLOBAL_CLK_CTRL, 0x4000}, //0x42
	//	{ALC5631_SDP_CTRL, 0x0000},//0x34
};
#define ALC5631_INIT_REG_LEN ARRAY_SIZE(init_list)

static int alc5631_reg_init(struct snd_soc_codec *codec)
{
	int i;
	int reg;

	for (i = 0; i < ALC5631_INIT_REG_LEN; i++){
#ifdef ALC5631_DEBUG
		printk("alc5631 write %x -> %x ", init_list[i].reg, init_list[i].val);
#endif
		reg = snd_soc_write(codec, init_list[i].reg, init_list[i].val);

#ifdef ALC5631_DEBUG
		printk("(return %d) \n", reg);
#endif
	}

	return 0;
}
#endif

struct alc5631_priv {
	struct snd_soc_codec *codec;
	int codec_version;
	int master;
	int sysclk;
	int rx_rate;
	int bclk_rate;
	int dmic_used_flag;
};

static const u16 alc5631_reg[ALC5631_VENDOR_ID2 + 1] = {
#if 1
	[ALC5631_SPK_OUT_VOL] = 0x8888,
	[ALC5631_HP_OUT_VOL] = 0x8080,
	[ALC5631_MONO_AXO_1_2_VOL] = 0xa080,
	[ALC5631_AUX_IN_VOL] = 0x0808,
	[ALC5631_ADC_REC_MIXER] = 0xf0f0,
	[ALC5631_VDAC_DIG_VOL] = 0x0010,
	[ALC5631_OUTMIXER_L_CTRL] = 0xffc0,
	[ALC5631_OUTMIXER_R_CTRL] = 0xffc0,
	[ALC5631_AXO1MIXER_CTRL] = 0x88c0,
	[ALC5631_AXO2MIXER_CTRL] = 0x88c0,
	[ALC5631_DIG_MIC_CTRL] = 0x3000,
	[ALC5631_MONO_INPUT_VOL] = 0x8808,
	[ALC5631_SPK_MIXER_CTRL] = 0xf8f8,
	[ALC5631_SPK_MONO_OUT_CTRL] = 0xfc00,
	[ALC5631_SPK_MONO_HP_OUT_CTRL] = 0x4440, //0xcccc,//0x4440,
	[ALC5631_SDP_CTRL] = 0x8000, //0x8002, //0x8000
	[ALC5631_MONO_SDP_CTRL] = 0x8000,
	[ALC5631_STEREO_AD_DA_CLK_CTRL] = 0x2010, //0x2010,
	[ALC5631_GEN_PUR_CTRL_REG] = 0x0e00,
	[ALC5631_INT_ST_IRQ_CTRL_2] = 0x071a, //0x071a,
	[ALC5631_MISC_CTRL] = 0x2040,
	[ALC5631_DEPOP_FUN_CTRL_2] = 0x4000, //0x8000,
	[ALC5631_SOFT_VOL_CTRL] = 0x0, //0x07e0,
	[ALC5631_ALC_CTRL_1] = 0x0206,
	[ALC5631_ALC_CTRL_3] = 0x2000,
	[ALC5631_PSEUDO_SPATL_CTRL] = 0x0553,
//johncao
//
	[ALC5631_MIC_CTRL_2] = 0x0040,
	[ALC5631_PWR_MANAG_ADD2] = 0xffff,//0x0060,
//	[ALC5631_PSEUDO_SPATL_CTRL] = 0x0553,
//	[ALC5631_PSEUDO_SPATL_CTRL] = 0x0553,
#else
	[0x00] = 0x0000,
	[0x02] = 0xffe8,
	[0x04] = 0x5f5f,
	[0x06] = 0xa080,
	[0x0a] = 0x0808,
	[0x0c] = 0x0000,
	[0x0e] = 0x8080,
	[0x10] = 0x0000,
	[0x12] = 0x0000,
	[0x14] = 0xb0b0,
	[0x16] = 0x0000,
	[0x18] = 0x0010,
	[0x1a] = 0xdfc0,
	[0x1c] = 0xdfc0,
	[0x1e] = 0x88c0,
	[0x20] = 0x88c0,
	[0x22] = 0x3300,
	[0x24] = 0x3000,
	[0x26] = 0x8808,
	[0x28] = 0xd8d8,
	[0x2a] = 0x6c00,
	[0x2c] = 0xcccc,
	[0x34] = 0x0000,
	[0x36] = 0x8000,
	[0x38] = 0x1000,
	[0x3a] = 0x9380,
	[0x3b] = 0x0002,
	[0x3c] = 0xe01e,
	[0x3e] = 0x0000,
	[0x40] = 0x0e00,
	[0x42] = 0x4000,
	[0x44] = 0x363b,
	[0x48] = 0x0000,
	[0x4a] = 0x0f1c,
	[0x4c] = 0x0000,
	[0x52] = 0x2040,
	[0x54] = 0xc003,
	[0x56] = 0x8000,
	[0x5a] = 0x0000,
	[0x5c] = 0x07e0,
	[0x64] = 0x0206,
	[0x65] = 0x0000,
	[0x66] = 0x2000,
	[0x68] = 0x0553,
	[0x6a] = 0x0056,
	[0x6c] = 0x302f,
	[0x6e] = 0x0000,
	[0x7a] = 0x0000,
	[0x7c] = 0x0000,
	[0x7e] = 0x0000,
#endif
};

/**
 * alc5631_index_write - Write private register.
 * @codec: SoC audio codec device.
 * @reg: Private register index.
 * @value: Private register Data.
 *
 * Modify private register for advanced setting. It can be written through
 * private index (0x6a) and data (0x6c) register.
 *
 * Returns 0 for success or negative error code.
 */
static int alc5631_index_write(struct snd_soc_codec *codec,
		unsigned int reg, unsigned int value)
{
	int ret;
#ifdef ALC5631_DEBUG
	printk("%s ,  reg = %d, value = %d \n", __func__, reg, value);
#endif
	ret = snd_soc_write(codec, ALC5631_INDEX_ADD, reg);
	if (ret < 0) {
		dev_err(codec->dev, "Failed to set private addr: %d\n", ret);
		goto err;
	}
	ret = snd_soc_write(codec, ALC5631_INDEX_DATA, value);
	if (ret < 0) {
		dev_err(codec->dev, "Failed to set private value: %d\n", ret);
		goto err;
	}
	return 0;

err:
	return ret;
}

/**
 * alc5631_index_read - Read private register.
 * @codec: SoC audio codec device.
 * @reg: Private register index.
 *
 * Read advanced setting from private register. It can be read through
 * private index (0x6a) and data (0x6c) register.
 *
 * Returns private register value or negative error code.
 */
static unsigned int alc5631_index_read(
		struct snd_soc_codec *codec, unsigned int reg)
{
	int ret;

#ifdef ALC5631_DEBUG
	printk("%s ,  reg = %d \n", __func__, reg);
#endif
	ret = snd_soc_write(codec, ALC5631_INDEX_ADD, reg);
	if (ret < 0) {
		dev_err(codec->dev, "Failed to set private addr: %d\n", ret);
		return ret;
	}
	return snd_soc_read(codec, ALC5631_INDEX_DATA);
}

/**
 * alc5631_index_update_bits - update private register bits
 * @codec: audio codec
 * @reg: Private register index.
 * @mask: register mask
 * @value: new value
 *
 * Writes new register value.
 *
 * Returns 1 for change, 0 for no change, or negative error code.
 */
static int alc5631_index_update_bits(struct snd_soc_codec *codec,
		unsigned int reg, unsigned int mask, unsigned int value)
{
	unsigned int old, new;
	int change, ret;

	ret = alc5631_index_read(codec, reg);
	if (ret < 0) {
		dev_err(codec->dev, "Failed to read private reg: %d\n", ret);
		goto err;
	}

	old = ret;
	new = (old & ~mask) | (value & mask);
	change = old != new;
	if (change) {
		ret = alc5631_index_write(codec, reg, new);
		if (ret < 0) {
			dev_err(codec->dev,
					"Failed to write private reg: %d\n", ret);
			goto err;
		}
	}
	return change;

err:
	return ret;
}

static int alc5631_reset(struct snd_soc_codec *codec)
{
	return snd_soc_write(codec, ALC5631_RESET, 0);
}

static int alc5631_volatile_register(struct snd_soc_codec *codec,
		unsigned int reg)
{
	switch (reg) {
		case ALC5631_RESET:
		case ALC5631_INT_ST_IRQ_CTRL_2:
		case ALC5631_INDEX_ADD:
		case ALC5631_INDEX_DATA:
		case ALC5631_EQ_CTRL:
			return 1;
		default:
			return 0;
	}
}

static int alc5631_readable_register(struct snd_soc_codec *codec,
		unsigned int reg)
{
	switch (reg) {
		case ALC5631_RESET:
		case ALC5631_SPK_OUT_VOL:
		case ALC5631_HP_OUT_VOL:
		case ALC5631_MONO_AXO_1_2_VOL:
		case ALC5631_AUX_IN_VOL:
		case ALC5631_STEREO_DAC_VOL_1:
		case ALC5631_MIC_CTRL_1:
		case ALC5631_STEREO_DAC_VOL_2:
		case ALC5631_ADC_CTRL_1:
		case ALC5631_ADC_REC_MIXER:
		case ALC5631_ADC_CTRL_2:
		case ALC5631_VDAC_DIG_VOL:
		case ALC5631_OUTMIXER_L_CTRL:
		case ALC5631_OUTMIXER_R_CTRL:
		case ALC5631_AXO1MIXER_CTRL:
		case ALC5631_AXO2MIXER_CTRL:
		case ALC5631_MIC_CTRL_2:
		case ALC5631_DIG_MIC_CTRL:
		case ALC5631_MONO_INPUT_VOL:
		case ALC5631_SPK_MIXER_CTRL:
		case ALC5631_SPK_MONO_OUT_CTRL:
		case ALC5631_SPK_MONO_HP_OUT_CTRL:
		case ALC5631_SDP_CTRL:
		case ALC5631_MONO_SDP_CTRL:
		case ALC5631_STEREO_AD_DA_CLK_CTRL:
		case ALC5631_PWR_MANAG_ADD1:
		case ALC5631_PWR_MANAG_ADD2:
		case ALC5631_PWR_MANAG_ADD3:
		case ALC5631_PWR_MANAG_ADD4:
		case ALC5631_GEN_PUR_CTRL_REG:
		case ALC5631_GLOBAL_CLK_CTRL:
		case ALC5631_PLL_CTRL:
		case ALC5631_INT_ST_IRQ_CTRL_1:
		case ALC5631_INT_ST_IRQ_CTRL_2:
		case ALC5631_GPIO_CTRL:
		case ALC5631_MISC_CTRL:
		case ALC5631_DEPOP_FUN_CTRL_1:
		case ALC5631_DEPOP_FUN_CTRL_2:
		case ALC5631_JACK_DET_CTRL:
		case ALC5631_SOFT_VOL_CTRL:
		case ALC5631_ALC_CTRL_1:
		case ALC5631_ALC_CTRL_2:
		case ALC5631_ALC_CTRL_3:
		case ALC5631_PSEUDO_SPATL_CTRL:
		case ALC5631_INDEX_ADD:
		case ALC5631_INDEX_DATA:
		case ALC5631_EQ_CTRL:
		case ALC5631_VENDOR_ID:
		case ALC5631_VENDOR_ID1:
		case ALC5631_VENDOR_ID2:
			return 1;
		default:
			return 0;
	}
}

/**
 * alc5631_headset_detect - Detect headset.
 * @codec: SoC audio codec device.
 * @jack_insert: Jack insert or not.
 *
 * Detect whether is headset or not when jack inserted.
 *
 * Returns detect status.
 */
int alc5631_headset_detect(struct snd_soc_codec *codec, int jack_insert)
{
	int jack_type;

	if(jack_insert) {
		snd_soc_update_bits(codec, ALC5631_PWR_MANAG_ADD2,
				ALC5631_PWR_MICBIAS1_VOL, ALC5631_PWR_MICBIAS1_VOL);
		snd_soc_update_bits(codec, ALC5631_MIC_CTRL_2,
				ALC5631_MICBIAS1_S_C_DET_MASK |
				ALC5631_MICBIAS1_SHORT_CURR_DET_MASK,
				ALC5631_MICBIAS1_S_C_DET_ENA |
				ALC5631_MICBIAS1_SHORT_CURR_DET_600UA);
		msleep(50);
		if (alc5631_index_read(codec, 0x4a) & 0x2)
			jack_type = ALC5631_HEADPHO_DET;
		else
			jack_type = ALC5631_HEADSET_DET;
	} else {
		snd_soc_update_bits(codec, ALC5631_MIC_CTRL_2,
				ALC5631_MICBIAS1_S_C_DET_MASK,
				ALC5631_MICBIAS1_S_C_DET_DIS);
		jack_type = ALC5631_NO_JACK;
	}

	return jack_type;
}
EXPORT_SYMBOL(alc5631_headset_detect);

static const DECLARE_TLV_DB_SCALE(out_vol_tlv, -4650, 150, 0);
static const DECLARE_TLV_DB_SCALE(dac_vol_tlv, -95625, 375, 0);
static const DECLARE_TLV_DB_SCALE(in_vol_tlv, -3450, 150, 0);
/* {0, +20, +24, +30, +35, +40, +44, +50, +52} dB */
static unsigned int mic_bst_tlv[] = {
	TLV_DB_RANGE_HEAD(7),
	0, 0, TLV_DB_SCALE_ITEM(0, 0, 0),
	1, 1, TLV_DB_SCALE_ITEM(2000, 0, 0),
	2, 2, TLV_DB_SCALE_ITEM(2400, 0, 0),
	3, 5, TLV_DB_SCALE_ITEM(3000, 500, 0),
	6, 6, TLV_DB_SCALE_ITEM(4400, 0, 0),
	7, 7, TLV_DB_SCALE_ITEM(5000, 0, 0),
	8, 8, TLV_DB_SCALE_ITEM(5200, 0, 0),
};

static int alc5631_dmic_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct alc5631_priv *alc5631 = snd_soc_codec_get_drvdata(codec);

	ucontrol->value.integer.value[0] = alc5631->dmic_used_flag;

	return 0;
}

static int alc5631_dmic_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct alc5631_priv *alc5631 = snd_soc_codec_get_drvdata(codec);

	alc5631->dmic_used_flag = ucontrol->value.integer.value[0];
	return 0;
}

/* MIC Input Type */
static const char *alc5631_input_mode[] = {
	"Single ended", "Differential"};

static const SOC_ENUM_SINGLE_DECL(
		alc5631_mic1_mode_enum, ALC5631_MIC_CTRL_1,
		ALC5631_MIC1_DIFF_INPUT_SHIFT, alc5631_input_mode);

static const SOC_ENUM_SINGLE_DECL(
		alc5631_mic2_mode_enum, ALC5631_MIC_CTRL_1,
		ALC5631_MIC2_DIFF_INPUT_SHIFT, alc5631_input_mode);

/* MONO Input Type */
static const SOC_ENUM_SINGLE_DECL(
		alc5631_monoin_mode_enum, ALC5631_MONO_INPUT_VOL,
		ALC5631_MONO_DIFF_INPUT_SHIFT, alc5631_input_mode);

/* SPK Ratio Gain Control */
static const char *alc5631_spk_ratio[] = {"1.00x", "1.09x", "1.27x", "1.44x",
	"1.56x", "1.68x", "1.99x", "2.34x"};

static const SOC_ENUM_SINGLE_DECL(
		alc5631_spk_ratio_enum, ALC5631_GEN_PUR_CTRL_REG,
		ALC5631_SPK_AMP_RATIO_CTRL_SHIFT, alc5631_spk_ratio);

static const struct snd_kcontrol_new alc5631_snd_controls[] = {
	/* MIC */
	SOC_ENUM("MIC1 Mode Control",  alc5631_mic1_mode_enum),
	SOC_SINGLE_TLV("MIC1 Boost", ALC5631_MIC_CTRL_2,
			ALC5631_MIC1_BOOST_SHIFT, /*5*/4/*8*/, 0, mic_bst_tlv),
	SOC_ENUM("MIC2 Mode Control", alc5631_mic2_mode_enum),
	SOC_SINGLE_TLV("MIC2 Boost", ALC5631_MIC_CTRL_2,
			ALC5631_MIC2_BOOST_SHIFT, 4/*8*//*6*/, 0, mic_bst_tlv),
	/* MONO IN */
	SOC_ENUM("MONOIN Mode Control", alc5631_monoin_mode_enum),
	SOC_DOUBLE_TLV("MONOIN_RX Capture Volume", ALC5631_MONO_INPUT_VOL,
			ALC5631_L_VOL_SHIFT, ALC5631_R_VOL_SHIFT,
			ALC5631_VOL_MASK, 1, in_vol_tlv),
	/* AXI */
	SOC_DOUBLE_TLV("AXI Capture Volume", ALC5631_AUX_IN_VOL,
			ALC5631_L_VOL_SHIFT, ALC5631_R_VOL_SHIFT,
			ALC5631_VOL_MASK, 1, in_vol_tlv),
	/* ADC */
	SOC_DOUBLE_TLV("PCM Record Volume", ALC5631_ADC_CTRL_2,
			ALC5631_L_VOL_SHIFT, ALC5631_R_VOL_SHIFT,
			ALC5631_DAC_VOL_MASK, /*1*/1, dac_vol_tlv),
	SOC_DOUBLE("PCM Record Switch", ALC5631_ADC_CTRL_1,
			ALC5631_L_MUTE_SHIFT, ALC5631_R_MUTE_SHIFT, 1, 1),
	/* DAC */
	SOC_DOUBLE_TLV("PCM Playback Volume", ALC5631_STEREO_DAC_VOL_2,
			ALC5631_L_VOL_SHIFT, ALC5631_R_VOL_SHIFT,
			ALC5631_DAC_VOL_MASK, /*1*/1, dac_vol_tlv),
	SOC_DOUBLE("PCM Playback Switch", ALC5631_STEREO_DAC_VOL_1,
			ALC5631_L_MUTE_SHIFT, ALC5631_R_MUTE_SHIFT, 1, 1),
	/* AXO */
	SOC_SINGLE("AXO1 Playback Switch", ALC5631_MONO_AXO_1_2_VOL,
			ALC5631_L_MUTE_SHIFT, 1, 1),
	SOC_SINGLE("AXO2 Playback Switch", ALC5631_MONO_AXO_1_2_VOL,
			ALC5631_R_VOL_SHIFT, 1, 1),
	/* OUTVOL */
	SOC_DOUBLE("OUTVOL Channel Switch", ALC5631_SPK_OUT_VOL,
			ALC5631_L_EN_SHIFT, ALC5631_R_EN_SHIFT, 1, 0),

	/* SPK */
	SOC_DOUBLE("Speaker Playback Switch", ALC5631_SPK_OUT_VOL,
			ALC5631_L_MUTE_SHIFT, ALC5631_R_MUTE_SHIFT, 1, 1),
	SOC_DOUBLE_TLV("Speaker Playback Volume", ALC5631_SPK_OUT_VOL,
			ALC5631_L_VOL_SHIFT, ALC5631_R_VOL_SHIFT, ALC5631_VOL_MASK/*39*/, 1, out_vol_tlv),
	/* MONO OUT */
	SOC_SINGLE("MONO Playback Switch", ALC5631_MONO_AXO_1_2_VOL,
			ALC5631_MUTE_MONO_SHIFT, 1, 1),
	/* HP */
/*//FAE*/	SOC_DOUBLE("HP Playback Switch", ALC5631_HP_OUT_VOL,
/*//FAE*/			ALC5631_L_MUTE_SHIFT, ALC5631_R_MUTE_SHIFT, 1, 1),
	SOC_DOUBLE_TLV("HP Playback Volume", ALC5631_HP_OUT_VOL,
			ALC5631_L_VOL_SHIFT, ALC5631_R_VOL_SHIFT,
			ALC5631_VOL_MASK, 1, out_vol_tlv),
	/* DMIC */
	SOC_SINGLE_EXT("DMIC Switch", 0, 0, 1, 0,
			alc5631_dmic_get, alc5631_dmic_put),
	SOC_DOUBLE("DMIC Capture Switch", ALC5631_DIG_MIC_CTRL,
			ALC5631_DMIC_L_CH_MUTE_SHIFT,
			ALC5631_DMIC_R_CH_MUTE_SHIFT, 1, 1),

	/* SPK Ratio Gain Control */
	SOC_ENUM("SPK Ratio Control", alc5631_spk_ratio_enum),
};

static int check_sysclk1_source(struct snd_soc_dapm_widget *source,
		struct snd_soc_dapm_widget *sink)
{
	unsigned int reg;

	reg = snd_soc_read(source->codec, ALC5631_GLOBAL_CLK_CTRL);
#ifdef ALC5631_DEBUG
	printk(" %s reg = 0x%x \n", __func__, reg);
#endif
	return reg & ALC5631_SYSCLK_SOUR_SEL_PLL;
}

static int check_dmic_used(struct snd_soc_dapm_widget *source,
		struct snd_soc_dapm_widget *sink)
{
	struct alc5631_priv *alc5631 = snd_soc_codec_get_drvdata(source->codec);
	return alc5631->dmic_used_flag;
}

static int check_dacl_to_outmixl(struct snd_soc_dapm_widget *source,
		struct snd_soc_dapm_widget *sink)
{
	unsigned int reg;

	reg = snd_soc_read(source->codec, ALC5631_OUTMIXER_L_CTRL);
	return !(reg & ALC5631_M_DAC_L_TO_OUTMIXER_L);
}

 /*JOIN debug pop */
#ifdef ALC5631_DEBUG
static int check_i2s_power(struct snd_soc_dapm_widget *source, 
		struct snd_soc_dapm_widget *sink)
{
	
//	snd_soc_update_bits(source->codec, ALC5631_PWR_MANAG_ADD4,
//			ALC5631_PWR_HP_R_OUT_VOL|ALC5631_PWR_HP_L_OUT_VOL,
//			0);
#ifdef ALC5631_DEBUG
	printk(" %s \n", __func__);
	printk("%s<<<REG-02:%x\n", __func__, snd_soc_read(source->codec, ALC5631_SPK_OUT_VOL));
	//snd_soc_write(pcodec, ALC5631_SPK_OUT_VOL, 0x4848);	
#endif
	return 1;
}
static int check_voice_dac_power(struct snd_soc_dapm_widget *source, 
		struct snd_soc_dapm_widget *sink)
{
#ifdef ALC5631_DEBUG
	printk(" %s \n", __func__);
#endif
	return 1;
}
static int check_dac_ref_power(struct snd_soc_dapm_widget *source, 
		struct snd_soc_dapm_widget *sink)
{
//	snd_soc_update_bits(source->codec, ALC5631_PWR_MANAG_ADD4,
//			ALC5631_PWR_HP_R_OUT_VOL|ALC5631_PWR_HP_L_OUT_VOL,
//			0);
#ifdef ALC5631_DEBUG
	printk(" %s \n", __func__);
	printk("%s<<<REG-02:%x\n", __func__, snd_soc_read(source->codec, ALC5631_SPK_OUT_VOL));
#endif
	return 1;
}
static int check_outmixr_mixer_power(struct snd_soc_dapm_widget *source, 
		struct snd_soc_dapm_widget *sink)
{
#ifdef ALC5631_DEBUG
	printk(" %s \n", __func__);
#endif
	return 1;
}
static int check_outmixl_mixer_power(struct snd_soc_dapm_widget *source, 
		struct snd_soc_dapm_widget *sink)
{
#ifdef ALC5631_DEBUG
	printk(" %s \n", __func__);
#endif
	return 1;
}
static int check_spkmixl_mixer_power(struct snd_soc_dapm_widget *source, 
		struct snd_soc_dapm_widget *sink)
{
#ifdef ALC5631_DEBUG
	printk(" %s \n", __func__);
#endif
	return 1;
}
static int check_spkmixr_mixer_power(struct snd_soc_dapm_widget *source, 
		struct snd_soc_dapm_widget *sink)
{
#ifdef ALC5631_DEBUG
	printk(" %s \n", __func__);
#endif
	return 1;
}
static int check_vmic_power(struct snd_soc_dapm_widget *source, 
		struct snd_soc_dapm_widget *sink)
{
#ifdef ALC5631_DEBUG
	printk(" %s \n", __func__);
#endif
	return 1;
}
static int check_left_spkvol_mux_power(struct snd_soc_dapm_widget *source, 
		struct snd_soc_dapm_widget *sink)
{
#ifdef ALC5631_DEBUG
	printk(" %s \n", __func__);
#endif
	return 1;
}
static int check_right_spkvol_mux_power(struct snd_soc_dapm_widget *source, 
		struct snd_soc_dapm_widget *sink)
{
#ifdef ALC5631_DEBUG
	printk(" %s \n", __func__);
#endif
	return 1;
}
static int check_right_outvol_mux_power(struct snd_soc_dapm_widget *source, 
		struct snd_soc_dapm_widget *sink)
{
#ifdef ALC5631_DEBUG
	printk(" %s \n", __func__);
#endif
	return 1;
}
static int check_left_outvol_mux_power(struct snd_soc_dapm_widget *source, 
		struct snd_soc_dapm_widget *sink)
{
#ifdef ALC5631_DEBUG
	printk(" %s \n", __func__);
#endif
	return 1;
}
static int check_spolmix_mixer_power(struct snd_soc_dapm_widget *source, 
		struct snd_soc_dapm_widget *sink)
{
#ifdef ALC5631_DEBUG
	printk(" %s \n", __func__);
#endif
	return 1;
}
static int check_spormix_mixer_power(struct snd_soc_dapm_widget *source, 
		struct snd_soc_dapm_widget *sink)
{
#ifdef ALC5631_DEBUG
	printk(" %s \n", __func__);
#endif
	return 1;
}
static int check_recmixr_mixer_power(struct snd_soc_dapm_widget *source, 
		struct snd_soc_dapm_widget *sink)
{
#ifdef ALC5631_DEBUG
	printk(" %s \n", __func__);
#endif
	return 1;
}
static int check_recmixl_mixer_power(struct snd_soc_dapm_widget *source, 
		struct snd_soc_dapm_widget *sink)
{
#ifdef ALC5631_DEBUG
	printk(" %s \n", __func__);
#endif
	return 1;
}
static int check_voice_dac_boost_power(struct snd_soc_dapm_widget *source, 
		struct snd_soc_dapm_widget *sink)
{
#ifdef ALC5631_DEBUG
	printk(" %s \n", __func__);
#endif
	return 1;
}
static int check_left_dac_power(struct snd_soc_dapm_widget *source, 
		struct snd_soc_dapm_widget *sink)
{
#ifdef ALC5631_DEBUG
	printk(" %s \n", __func__);
#endif
	return 1;
}
static int check_right_dac_power(struct snd_soc_dapm_widget *source, 
		struct snd_soc_dapm_widget *sink)
{
#ifdef ALC5631_DEBUG
	printk(" %s \n", __func__);
#endif
	return 1;
}
static int check_spor_mux_power(struct snd_soc_dapm_widget *source, 
		struct snd_soc_dapm_widget *sink)
{
#ifdef ALC5631_DEBUG
	printk(" %s \n", __func__);
#endif
	return 1;
}
static int check_spol_mux_power(struct snd_soc_dapm_widget *source, 
		struct snd_soc_dapm_widget *sink)
{
#ifdef ALC5631_DEBUG
	printk(" %s \n", __func__);
#endif
	return 1;
}

static int check_hp_depop_power(struct snd_soc_dapm_widget *source,
		struct snd_soc_dapm_widget *sink)
{
#ifdef ALC5631_DEBUG
	printk(" %s \n", __func__);
#endif
}

static int check_class_d_power(struct snd_soc_dapm_widget *source,
		struct snd_soc_dapm_widget *sink)
{
#ifdef ALC5631_DEBUG
	printk(" %s \n", __func__);
	printk("%s<<<REG-02:%x\n", __func__, snd_soc_read(source->codec, ALC5631_SPK_OUT_VOL));
	//snd_soc_write(pcodec, ALC5631_SPK_OUT_VOL, 0x4848);	
#endif
//	unsigned int reg;
//	reg  = snd_soc_read(source->codec, ALC5631_PWR_MANAG_ADD1);

//	snd_soc_write(source->codec, ALC5631_SOFT_VOL_CTRL, 0x0);
//	printk("[04] = 0x%x \n" , snd_soc_read(source->codec, 0x04));
//	snd_soc_write(source->codec, 0x04, 0x8080);
	//snd_soc_write(source->codec, 0x0c, 0x8080);
	//snd_soc_write(source->codec, 0x1a, 0xffc0);
	//snd_soc_write(source->codec, 0x1c, 0xffc0);
	//snd_soc_write(source->codec, 0x28, 0xf8f8);
	//snd_soc_write(source->codec, 0x2a, 0xfc00);
	//snd_soc_write(source->codec, 0x14, 0xffff);
	//snd_soc_write(source->codec, 0x1e, 0xffff);
	//snd_soc_write(source->codec, 0x20, 0xffff);


//	snd_soc_write(source->codec, ALC5631_PWR_MANAG_ADD1, 0x93e0);
//	snd_soc_write(source->codec, ALC5631_PWR_MANAG_ADD2, 0xf002); //0xf002);
#if 0
	printk("[02] = 0x%x \n" , snd_soc_read(source->codec, 0x02));
	printk("[0c] = 0x%x \n" , snd_soc_read(source->codec, 0x0c));
	printk("[1a] = 0x%x \n" , snd_soc_read(source->codec, 0x1a));
	printk("[14] = 0x%x \n" , snd_soc_read(source->codec, 0x14));
	printk("[28] = 0x%x \n" , snd_soc_read(source->codec, 0x28));
	printk("[2a] = 0x%x \n" , snd_soc_read(source->codec, 0x2a));
	printk("[1e] = 0x%x \n" , snd_soc_read(source->codec, 0x1e));
	printk("[20] = 0x%x \n" , snd_soc_read(source->codec, 0x20));
	
	printk("[3a] = 0x%x \n" , snd_soc_read(source->codec, 0x3a));
	printk("[3b] = 0x%x \n" , snd_soc_read(source->codec, 0x3b));
	printk("[3c] = 0x%x \n" , snd_soc_read(source->codec, 0x3c));
	printk("[3e] = 0x%x \n" , snd_soc_read(source->codec, 0x3e));
	//write 0c = 8080;
	snd_soc_write(source->codec, 0x0c, 0x8080);
	snd_soc_write(source->codec, 0x1a, 0xffc0);
	snd_soc_write(source->codec, 0x1c, 0xffc0);
	snd_soc_write(source->codec, 0x28, 0xf8f8);
	snd_soc_write(source->codec, 0x2a, 0xfc00);
	snd_soc_write(source->codec, 0x14, 0xffff);
	snd_soc_write(source->codec, 0x1e, 0xffff);
	snd_soc_write(source->codec, 0x20, 0xffff);

		snd_soc_write(source->codec, ALC5631_PWR_MANAG_ADD1, 0x1000);
	int i = 0;
	for (i = 0 ; i < 10 ; i ++){
	
		snd_soc_write(source->codec, ALC5631_PWR_MANAG_ADD1, 0x93e0);
		snd_soc_write(source->codec, ALC5631_PWR_MANAG_ADD2, 0xf002); //0xf002);
		snd_soc_write(source->codec, ALC5631_PWR_MANAG_ADD3, 0xe01e);
		snd_soc_write(source->codec, ALC5631_PWR_MANAG_ADD4, 0xcc00);

		msleep(100);

		snd_soc_write(source->codec, ALC5631_PWR_MANAG_ADD1, 0x1000);
		snd_soc_write(source->codec, ALC5631_PWR_MANAG_ADD2, 0x0000);
		//snd_soc_write(source->codec, ALC5631_PWR_MANAG_ADD3, 0x0000);
		snd_soc_write(source->codec, ALC5631_PWR_MANAG_ADD4, 0x0000);
	}
#endif
	//	printk(" %s,  === 0x%x \n", __func__, snd_soc_read(source->codec, ALC5631_PWR_MANAG_ADD2));
	//	printk(" %s,  === 0x%x \n", __func__, snd_soc_read(source->codec, ALC5631_PWR_MANAG_ADD3));
	//	printk(" %s,  === 0x%x \n", __func__, snd_soc_read(source->codec, ALC5631_PWR_MANAG_ADD4));

/*	if( reg & ALC5631_PWR_CLASS_D){
		printk(" %s , is close \n", __func__);

		snd_soc_update_bits(source->codec, ALC5631_PWR_MANAG_ADD1,
				         ALC5631_EN_CAP_FREE_DEPOP,0);

		snd_soc_write(source->codec, ALC5631_PWR_MANAG_ADD1, 0x0000);
		snd_soc_write(source->codec, ALC5631_PWR_MANAG_ADD2, 0x0000);
		snd_soc_write(source->codec, ALC5631_PWR_MANAG_ADD3, 0x0000);
		snd_soc_write(source->codec, ALC5631_PWR_MANAG_ADD4, 0x0000);
	}else{
		printk( " %s , is open \n", __func__);
		snd_soc_write(source->codec, ALC5631_PWR_MANAG_ADD1, 0x93e0);
		snd_soc_write(source->codec, ALC5631_PWR_MANAG_ADD2, 0xf002);
		snd_soc_write(source->codec, ALC5631_PWR_MANAG_ADD3, 0xe01e);
		snd_soc_write(source->codec, ALC5631_PWR_MANAG_ADD4, 0xcc00);
	}
	*/
	return 1;
}
#endif
static int check_dacr_to_outmixr(struct snd_soc_dapm_widget *source,
		struct snd_soc_dapm_widget *sink)
{
	unsigned int reg;

	reg = snd_soc_read(source->codec, ALC5631_OUTMIXER_R_CTRL);
	return !(reg & ALC5631_M_DAC_R_TO_OUTMIXER_R);
}

static int check_dacl_to_spkmixl(struct snd_soc_dapm_widget *source,
		struct snd_soc_dapm_widget *sink)
{
	unsigned int reg;

	reg = snd_soc_read(source->codec, ALC5631_SPK_MIXER_CTRL);
	return !(reg & ALC5631_M_DAC_L_TO_SPKMIXER_L);
}

static int check_dacr_to_spkmixr(struct snd_soc_dapm_widget *source,
		struct snd_soc_dapm_widget *sink)
{
	unsigned int reg;

	reg = snd_soc_read(source->codec, ALC5631_SPK_MIXER_CTRL);
	return !(reg & ALC5631_M_DAC_R_TO_SPKMIXER_R);
}

static int check_vdac_to_outmix(struct snd_soc_dapm_widget *source,
		struct snd_soc_dapm_widget *sink)
{
	unsigned int reg, ret = 1;

	reg = snd_soc_read(source->codec, ALC5631_OUTMIXER_L_CTRL);
	if (reg & ALC5631_M_VDAC_TO_OUTMIXER_L) {
		reg = snd_soc_read(source->codec, ALC5631_OUTMIXER_R_CTRL);
		if (reg & ALC5631_M_VDAC_TO_OUTMIXER_R)
			ret = 0;
	}
	return ret;
}

static int check_adcl_select(struct snd_soc_dapm_widget *source,
		struct snd_soc_dapm_widget *sink)
{
	unsigned int reg, ret = 0;

	reg = snd_soc_read(source->codec, ALC5631_ADC_REC_MIXER);

#ifdef ALC5631_DEBUG
	printk(" %s ========================= reg = 0x%x \n", __func__ , reg);
#endif
	if (reg & ALC5631_M_MIC2_TO_RECMIXER_R)
		if (!(reg & ALC5631_M_MIC1_TO_RECMIXER_L))
			ret = 1;
	return ret;
}

static int check_adcr_select(struct snd_soc_dapm_widget *source,
		struct snd_soc_dapm_widget *sink)
{
	unsigned int reg, ret = 0;

	reg = snd_soc_read(source->codec, ALC5631_ADC_REC_MIXER);
	if (reg & ALC5631_M_MIC1_TO_RECMIXER_L)
		if (!(reg & ALC5631_M_MIC2_TO_RECMIXER_R))
			ret = 1;
	return ret;
}

/**
 * onebit_depop_power_stage - auto depop in power stage.
 * @enable: power on/off
 *
 * When power on/off headphone, the depop sequence is done by hardware.
 */
static void onebit_depop_power_stage(struct snd_soc_codec *codec, int enable)
{
	unsigned int soft_vol, hp_zc;

	/* enable one-bit depop function */
	snd_soc_update_bits(codec, ALC5631_DEPOP_FUN_CTRL_2,
			ALC5631_EN_CAP_FREE_DEPOP, 0);
	//JOIN 		ALC5631_EN_ONE_BIT_DEPOP, 0);

	/* keep soft volume and zero crossing setting */
	soft_vol = snd_soc_read(codec, ALC5631_SOFT_VOL_CTRL);
	snd_soc_write(codec, ALC5631_SOFT_VOL_CTRL, 0);
	hp_zc = snd_soc_read(codec, ALC5631_INT_ST_IRQ_CTRL_2);
	snd_soc_write(codec, ALC5631_INT_ST_IRQ_CTRL_2, hp_zc & 0xf7ff);
	if (enable) {
		/* config one-bit depop parameter */
		alc5631_index_write(codec, ALC5631_TEST_MODE_CTRL, 0x84c0);
		alc5631_index_write(codec, ALC5631_SPK_INTL_CTRL, 0x309f);
		alc5631_index_write(codec, ALC5631_CP_INTL_REG2, 0x6530);
		/* power on capless block */
		snd_soc_write(codec, ALC5631_DEPOP_FUN_CTRL_2,
				ALC5631_EN_CAP_FREE_DEPOP);
	} else {
		/* power off capless block */
		snd_soc_write(codec, ALC5631_DEPOP_FUN_CTRL_2, 0);
		msleep(100);
	}

	/* recover soft volume and zero crossing setting */
	snd_soc_write(codec, ALC5631_SOFT_VOL_CTRL, soft_vol);
	snd_soc_write(codec, ALC5631_INT_ST_IRQ_CTRL_2, hp_zc);
}

/**
 * onebit_depop_mute_stage - auto depop in mute stage.
 * @enable: mute/unmute
 *
 * When mute/unmute headphone, the depop sequence is done by hardware.
 */
static void onebit_depop_mute_stage(struct snd_soc_codec *codec, int enable)
{
	unsigned int soft_vol, hp_zc;

	/* enable one-bit depop function */
	snd_soc_update_bits(codec, ALC5631_DEPOP_FUN_CTRL_2,
			ALC5631_EN_CAP_FREE_DEPOP,0);
	//JOIN		ALC5631_EN_ONE_BIT_DEPOP, 0);

	/* keep soft volume and zero crossing setting */
	soft_vol = snd_soc_read(codec, ALC5631_SOFT_VOL_CTRL);
	snd_soc_write(codec, ALC5631_SOFT_VOL_CTRL, 0);
	hp_zc = snd_soc_read(codec, ALC5631_INT_ST_IRQ_CTRL_2);
	snd_soc_write(codec, ALC5631_INT_ST_IRQ_CTRL_2, hp_zc & 0xf7ff);
	if (enable) {
		schedule_timeout_uninterruptible(msecs_to_jiffies(10));
		/* config one-bit depop parameter */
		alc5631_index_write(codec, ALC5631_SPK_INTL_CTRL, 0x307f);
		snd_soc_update_bits(codec, ALC5631_HP_OUT_VOL,
				ALC5631_L_MUTE | ALC5631_R_MUTE, 0);
		msleep(300);
	} else {
		snd_soc_update_bits(codec, ALC5631_HP_OUT_VOL,
				ALC5631_L_MUTE | ALC5631_R_MUTE,
				ALC5631_L_MUTE | ALC5631_R_MUTE);
		msleep(100);
	}

	/* recover soft volume and zero crossing setting */
	snd_soc_write(codec, ALC5631_SOFT_VOL_CTRL, soft_vol);
	snd_soc_write(codec, ALC5631_INT_ST_IRQ_CTRL_2, hp_zc);
}

/**
 * onebit_depop_power_stage - step by step depop sequence in power stage.
 * @enable: power on/off
 *
 * When power on/off headphone, the depop sequence is done in step by step.
 */
static void depop_seq_power_stage(struct snd_soc_codec *codec, int enable)
{
	unsigned int soft_vol, hp_zc;

	/* depop control by register */
	snd_soc_update_bits(codec, ALC5631_DEPOP_FUN_CTRL_2,
			ALC5631_EN_CAP_FREE_DEPOP, ALC5631_EN_CAP_FREE_DEPOP);
	//JOIN ALC5631_EN_ONE_BIT_DEPOP, ALC5631_EN_ONE_BIT_DEPOP);

	/* keep soft volume and zero crossing setting */
	soft_vol = snd_soc_read(codec, ALC5631_SOFT_VOL_CTRL);
	snd_soc_write(codec, ALC5631_SOFT_VOL_CTRL, 0);
	hp_zc = snd_soc_read(codec, ALC5631_INT_ST_IRQ_CTRL_2);
#ifdef ALC5631_DEBUG
	printk(" %s soft_vol = %d, hp_zc = %d \n", __func__, soft_vol, hp_zc);
	printk("%s REG-02 = 0x%x \n",__func__, snd_soc_read(codec, 0x02));
#endif
	snd_soc_write(codec, ALC5631_INT_ST_IRQ_CTRL_2, hp_zc & 0xf7ff);
	if (enable) {

		/*JOIN mute hp noise */
		//SPORMIX Mixer SPOLMIX Mixer
		//		sun_soc_update_bits(codec, ALC5631_SPK_MONO_OUT_CTRL,

		//		snd_soc_update_bits(codec, ALC5631_HP_OUT_VOL,
		//			ALC5631_L_MUTE | ALC5631_R_MUTE,
		//			ALC5631_L_MUTE | ALC5631_R_MUTE);
		//		msleep(100);
		/*****************/

		/* config depop sequence parameter */
		alc5631_index_write(codec, ALC5631_SPK_INTL_CTRL, 0x303e);

		/* power on headphone and charge pump */
#ifdef ALC5631_DEBUG
	printk("%s [3c] = 0x%x \n" ,__func__, snd_soc_read(codec, 0x3c));
#endif
		snd_soc_update_bits(codec, ALC5631_PWR_MANAG_ADD3,
				ALC5631_PWR_CHARGE_PUMP | ALC5631_PWR_HP_L_AMP |
				ALC5631_PWR_HP_R_AMP,
				ALC5631_PWR_CHARGE_PUMP | ALC5631_PWR_HP_L_AMP |
				ALC5631_PWR_HP_R_AMP);

		/* power on soft generator and depop mode2 */
		snd_soc_write(codec, ALC5631_DEPOP_FUN_CTRL_1,
				ALC5631_POW_ON_SOFT_GEN | ALC5631_EN_DEPOP2_FOR_HP);
		msleep(100);

		/* stop depop mode */
		snd_soc_update_bits(codec, ALC5631_PWR_MANAG_ADD3,
				ALC5631_PWR_HP_DEPOP_DIS, ALC5631_PWR_HP_DEPOP_DIS);
	} else {
		/* config depop sequence parameter */
		alc5631_index_write(codec, ALC5631_SPK_INTL_CTRL, 0x303F);
		snd_soc_write(codec, ALC5631_DEPOP_FUN_CTRL_1,
				ALC5631_POW_ON_SOFT_GEN | ALC5631_EN_MUTE_UNMUTE_DEPOP |
				ALC5631_PD_HPAMP_L_ST_UP | ALC5631_PD_HPAMP_R_ST_UP);
		msleep(75);
		snd_soc_write(codec, ALC5631_DEPOP_FUN_CTRL_1,
				ALC5631_POW_ON_SOFT_GEN | ALC5631_PD_HPAMP_L_ST_UP |
				ALC5631_PD_HPAMP_R_ST_UP);

		/* start depop mode */
		snd_soc_update_bits(codec, ALC5631_PWR_MANAG_ADD3,
				ALC5631_PWR_HP_DEPOP_DIS, 0);

		/* config depop sequence parameter */
		snd_soc_write(codec, ALC5631_DEPOP_FUN_CTRL_1,
				ALC5631_POW_ON_SOFT_GEN | ALC5631_EN_DEPOP2_FOR_HP |
				ALC5631_PD_HPAMP_L_ST_UP | ALC5631_PD_HPAMP_R_ST_UP);
		msleep(80);
		snd_soc_write(codec, ALC5631_DEPOP_FUN_CTRL_1,
				ALC5631_POW_ON_SOFT_GEN);

		/* power down headphone and charge pump */
		snd_soc_update_bits(codec, ALC5631_PWR_MANAG_ADD3,
				ALC5631_PWR_CHARGE_PUMP | ALC5631_PWR_HP_L_AMP |
				ALC5631_PWR_HP_R_AMP, 0);
	}

	/* recover soft volume and zero crossing setting */
#ifdef ALC5631_DEBUG
	printk(" %s soft_vol = %d, hp_zc = %d \n", __func__, soft_vol, hp_zc);
#endif
	snd_soc_write(codec, ALC5631_SOFT_VOL_CTRL, soft_vol);
	snd_soc_write(codec, ALC5631_INT_ST_IRQ_CTRL_2, hp_zc);
}

/**
 * depop_seq_mute_stage - step by step depop sequence in mute stage.
 * @enable: mute/unmute
 *
 * When mute/unmute headphone, the depop sequence is done in step by step.
 */
static void depop_seq_mute_stage(struct snd_soc_codec *codec, int enable)
{
	unsigned int soft_vol, hp_zc;

	/* depop control by register */
	snd_soc_update_bits(codec, ALC5631_DEPOP_FUN_CTRL_2,
			ALC5631_EN_CAP_FREE_DEPOP, ALC5631_EN_CAP_FREE_DEPOP);
	//ALC5631_EN_ONE_BIT_DEPOP, ALC5631_EN_ONE_BIT_DEPOP);

	/* keep soft volume and zero crossing setting */
	soft_vol = snd_soc_read(codec, ALC5631_SOFT_VOL_CTRL);
	snd_soc_write(codec, ALC5631_SOFT_VOL_CTRL, 0);
	hp_zc = snd_soc_read(codec, ALC5631_INT_ST_IRQ_CTRL_2);

#ifdef ALC5631_DEBUG
	printk(" %s soft_vol = %d, hp_zc = %d \n", __func__, soft_vol, hp_zc);
#endif
	snd_soc_write(codec, ALC5631_INT_ST_IRQ_CTRL_2, hp_zc & 0xf7ff);
	if (enable) {
		schedule_timeout_uninterruptible(msecs_to_jiffies(10));

		/* config depop sequence parameter */
		alc5631_index_write(codec, ALC5631_SPK_INTL_CTRL, 0x302f);
		snd_soc_write(codec, ALC5631_DEPOP_FUN_CTRL_1,
				ALC5631_POW_ON_SOFT_GEN | ALC5631_EN_MUTE_UNMUTE_DEPOP |
				ALC5631_EN_HP_R_M_UN_MUTE_DEPOP |
				ALC5631_EN_HP_L_M_UN_MUTE_DEPOP);

//		snd_soc_update_bits(codec, ALC5631_HP_OUT_VOL,
//				(0x01 << 14) | (0x01 << 6),
//				(0x01 << 14) | (0x01 << 6));
//		msleep(160);
#ifdef ALC5631_DEBUG
printk("%s ready open hp 0x04 = 0x%x \n",__func__, snd_soc_read(codec, 0x04));
printk("%s REG-02 = 0x%x \n",__func__, snd_soc_read(codec, 0x02));
#endif
		snd_soc_update_bits(codec, ALC5631_HP_OUT_VOL,
				ALC5631_L_MUTE | ALC5631_R_MUTE,0);


	//	/*spk mute */
	//	snd_soc_update_bits(codec, ALC5631_SPK_OUT_VOL,
	//		ALC5631_L_MUTE | ALC5631_R_MUTE, 0);
printk("%s REG-02 = 0x%x \n",__func__, snd_soc_read(codec, 0x02));
	//+++MQ
		//alc5631_reg_set(1);
		//snd_soc_write(pcodec, 0x02, 0x4848);	
	//+++MQ
		msleep(160);
	} else {
	//+++MQ
	//alc5631_reg_set(0);
	//+++MQ
		/* config depop sequence parameter */
		alc5631_index_write(codec, ALC5631_SPK_INTL_CTRL, 0x302f);
		snd_soc_write(codec, ALC5631_DEPOP_FUN_CTRL_1,
				ALC5631_POW_ON_SOFT_GEN | ALC5631_EN_MUTE_UNMUTE_DEPOP |
				ALC5631_EN_HP_R_M_UN_MUTE_DEPOP |
				ALC5631_EN_HP_L_M_UN_MUTE_DEPOP);



		snd_soc_update_bits(codec, ALC5631_HP_OUT_VOL,
				ALC5631_L_MUTE | ALC5631_R_MUTE,
				ALC5631_L_MUTE | ALC5631_R_MUTE);


		/*spk mute */
//		snd_soc_update_bits(codec, ALC5631_SPK_OUT_VOL,
//			ALC5631_L_MUTE | ALC5631_R_MUTE, 
//			ALC5631_L_MUTE | ALC5631_R_MUTE);
//		msleep(150);

		//snd_soc_update_bits(codec, ALC5631_HP_OUT_VOL,
		//		(0x01 << 14) | (0x01 << 6),
		//		0);
		msleep(150);
	}

	/* recover soft volume and zero crossing setting */
#ifdef ALC5631_DEBUG
	printk(" %s soft_vol = %d, hp_zc = %d \n", __func__, soft_vol, hp_zc);
#endif

	/*JOIN 09/03	msleep(1500);
	snd_soc_write(codec, 0x0c, 0x0);
	snd_soc_write(codec, 0x1a, 0xdfc0);
	snd_soc_write(codec, 0x1c, 0xdfc0);
	snd_soc_write(codec, 0x28, 0xd8d8);
	snd_soc_write(codec, 0x2a, 0x6c00);
	snd_soc_write(codec, 0x14, 0xb0b0);
	snd_soc_write(codec, 0x1e, 0x88c0);
	snd_soc_write(codec, 0x20, 0x88c0);
*/

	snd_soc_write(codec, ALC5631_SOFT_VOL_CTRL, soft_vol | 0x07e0);
	snd_soc_write(codec, ALC5631_INT_ST_IRQ_CTRL_2, hp_zc);
}

static int hp_event(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;
	struct alc5631_priv *alc5631 = snd_soc_codec_get_drvdata(codec);
	unsigned int reg_02;
	reg_02 = snd_soc_read(codec, 0x02);
	snd_soc_write(codec, 0x02, 0xc8c8);
	/*
	   static int once = 0;

	   if(once){
	   printk( "hp_evetn return 0\n");
	   return 0;
	   }
	   once = 1;
	   */
	switch (event) {
		case SND_SOC_DAPM_PRE_PMD:
#ifdef ALC5631_DEBUG
			printk(" %s , power down \n", __func__ );
#endif
			if (alc5631->codec_version) {
				onebit_depop_mute_stage(codec, 0);
				onebit_depop_power_stage(codec, 0);
			} else {
				depop_seq_mute_stage(codec, 0);
				depop_seq_power_stage(codec, 0);
			}
			break;

		case SND_SOC_DAPM_POST_PMU:
#ifdef ALC5631_DEBUG
			printk(" %s , power up \n", __func__ );
#endif
			if (alc5631->codec_version) {
				onebit_depop_power_stage(codec, 1);
				onebit_depop_mute_stage(codec, 1);
			} else {
				depop_seq_power_stage(codec, 1);
				depop_seq_mute_stage(codec, 1);
			}
			break;

		default:
			break;
	}

	snd_soc_write(codec, 0x02, reg_02);
	return 0;
}

static int set_dmic_params(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;
	struct alc5631_priv *alc5631 = snd_soc_codec_get_drvdata(codec);

	switch (alc5631->rx_rate) {
		case 44100:
		case 48000:
#ifdef ALC5631_DEBUG
			printk(" %s 44100 48000 \n", __func__);
#endif
			snd_soc_update_bits(codec, ALC5631_DIG_MIC_CTRL,
					ALC5631_DMIC_CLK_CTRL_MASK,
					ALC5631_DMIC_CLK_CTRL_TO_32FS);
			break;

		case 32000:
		case 22050:
#ifdef ALC5631_DEBUG
			printk(" %s 32000 22050 \n", __func__);
#endif
			snd_soc_update_bits(codec, ALC5631_DIG_MIC_CTRL,
					ALC5631_DMIC_CLK_CTRL_MASK,
					ALC5631_DMIC_CLK_CTRL_TO_64FS);
			break;

		case 16000:
		case 11025:
		case 8000:
#ifdef ALC5631_DEBUG
			printk(" %s 16000 11025 8000 \n", __func__);
#endif
			snd_soc_update_bits(codec, ALC5631_DIG_MIC_CTRL,
					ALC5631_DMIC_CLK_CTRL_MASK,
					ALC5631_DMIC_CLK_CTRL_TO_128FS);
			break;

		default:
			return -EINVAL;
	}

	return 0;
}

static const struct snd_kcontrol_new alc5631_recmixl_mixer_controls[] = {
	SOC_DAPM_SINGLE("OUTMIXL Capture Switch", ALC5631_ADC_REC_MIXER,
			ALC5631_M_OUTMIXL_RECMIXL_BIT, 1, 1),
	SOC_DAPM_SINGLE("MIC1_BST1 Capture Switch", ALC5631_ADC_REC_MIXER,
			ALC5631_M_MIC1_RECMIXL_BIT, 1, 1),
	SOC_DAPM_SINGLE("AXILVOL Capture Switch", ALC5631_ADC_REC_MIXER,
			ALC5631_M_AXIL_RECMIXL_BIT, 1, 1),
	SOC_DAPM_SINGLE("MONOIN_RX Capture Switch", ALC5631_ADC_REC_MIXER,
			ALC5631_M_MONO_IN_RECMIXL_BIT, 1, 1),
};

static const struct snd_kcontrol_new alc5631_recmixr_mixer_controls[] = {
	SOC_DAPM_SINGLE("MONOIN_RX Capture Switch", ALC5631_ADC_REC_MIXER,
			ALC5631_M_MONO_IN_RECMIXR_BIT, 1, 1),
	SOC_DAPM_SINGLE("AXIRVOL Capture Switch", ALC5631_ADC_REC_MIXER,
			ALC5631_M_AXIR_RECMIXR_BIT, 1, 1),
	SOC_DAPM_SINGLE("MIC2_BST2 Capture Switch", ALC5631_ADC_REC_MIXER,
			ALC5631_M_MIC2_RECMIXR_BIT, 1, 1),
	SOC_DAPM_SINGLE("OUTMIXR Capture Switch", ALC5631_ADC_REC_MIXER,
			ALC5631_M_OUTMIXR_RECMIXR_BIT, 1, 1),
};

static const struct snd_kcontrol_new alc5631_spkmixl_mixer_controls[] = {
	SOC_DAPM_SINGLE("RECMIXL Playback Switch", ALC5631_SPK_MIXER_CTRL,
			ALC5631_M_RECMIXL_SPKMIXL_BIT, 1, 1),
	SOC_DAPM_SINGLE("MIC1_P Playback Switch", ALC5631_SPK_MIXER_CTRL,
			ALC5631_M_MIC1P_SPKMIXL_BIT, 1, 1),
	SOC_DAPM_SINGLE("DACL Playback Switch", ALC5631_SPK_MIXER_CTRL,
			ALC5631_M_DACL_SPKMIXL_BIT, 1, 1),
	SOC_DAPM_SINGLE("OUTMIXL Playback Switch", ALC5631_SPK_MIXER_CTRL,
			ALC5631_M_OUTMIXL_SPKMIXL_BIT, 1, 1),
};

static const struct snd_kcontrol_new alc5631_spkmixr_mixer_controls[] = {
	SOC_DAPM_SINGLE("OUTMIXR Playback Switch", ALC5631_SPK_MIXER_CTRL,
			ALC5631_M_OUTMIXR_SPKMIXR_BIT, 1, 1),
	SOC_DAPM_SINGLE("DACR Playback Switch", ALC5631_SPK_MIXER_CTRL,
			ALC5631_M_DACR_SPKMIXR_BIT, 1, 1),
	SOC_DAPM_SINGLE("MIC2_P Playback Switch", ALC5631_SPK_MIXER_CTRL,
			ALC5631_M_MIC2P_SPKMIXR_BIT, 1, 1),
	SOC_DAPM_SINGLE("RECMIXR Playback Switch", ALC5631_SPK_MIXER_CTRL,
			ALC5631_M_RECMIXR_SPKMIXR_BIT, 1, 1),
};

static const struct snd_kcontrol_new alc5631_outmixl_mixer_controls[] = {
	SOC_DAPM_SINGLE("RECMIXL Playback Switch", ALC5631_OUTMIXER_L_CTRL,
			ALC5631_M_RECMIXL_OUTMIXL_BIT, 1, 1),
	SOC_DAPM_SINGLE("RECMIXR Playback Switch", ALC5631_OUTMIXER_L_CTRL,
			ALC5631_M_RECMIXR_OUTMIXL_BIT, 1, 1),
	SOC_DAPM_SINGLE("DACL Playback Switch", ALC5631_OUTMIXER_L_CTRL,
			ALC5631_M_DACL_OUTMIXL_BIT, 1, 1),
	SOC_DAPM_SINGLE("MIC1_BST1 Playback Switch", ALC5631_OUTMIXER_L_CTRL,
			ALC5631_M_MIC1_OUTMIXL_BIT, 1, 1),
	SOC_DAPM_SINGLE("MIC2_BST2 Playback Switch", ALC5631_OUTMIXER_L_CTRL,
			ALC5631_M_MIC2_OUTMIXL_BIT, 1, 1),
	SOC_DAPM_SINGLE("MONOIN_RXP Playback Switch", ALC5631_OUTMIXER_L_CTRL,
			ALC5631_M_MONO_INP_OUTMIXL_BIT, 1, 1),
	SOC_DAPM_SINGLE("AXILVOL Playback Switch", ALC5631_OUTMIXER_L_CTRL,
			ALC5631_M_AXIL_OUTMIXL_BIT, 1, 1),
	SOC_DAPM_SINGLE("AXIRVOL Playback Switch", ALC5631_OUTMIXER_L_CTRL,
			ALC5631_M_AXIR_OUTMIXL_BIT, 1, 1),
	SOC_DAPM_SINGLE("VDAC Playback Switch", ALC5631_OUTMIXER_L_CTRL,
			ALC5631_M_VDAC_OUTMIXL_BIT, 1, 1),
};

static const struct snd_kcontrol_new alc5631_outmixr_mixer_controls[] = {
	SOC_DAPM_SINGLE("VDAC Playback Switch", ALC5631_OUTMIXER_R_CTRL,
			ALC5631_M_VDAC_OUTMIXR_BIT, 1, 1),
	SOC_DAPM_SINGLE("AXIRVOL Playback Switch", ALC5631_OUTMIXER_R_CTRL,
			ALC5631_M_AXIR_OUTMIXR_BIT, 1, 1),
	SOC_DAPM_SINGLE("AXILVOL Playback Switch", ALC5631_OUTMIXER_R_CTRL,
			ALC5631_M_AXIL_OUTMIXR_BIT, 1, 1),
	SOC_DAPM_SINGLE("MONOIN_RXN Playback Switch", ALC5631_OUTMIXER_R_CTRL,
			ALC5631_M_MONO_INN_OUTMIXR_BIT, 1, 1),
	SOC_DAPM_SINGLE("MIC2_BST2 Playback Switch", ALC5631_OUTMIXER_R_CTRL,
			ALC5631_M_MIC2_OUTMIXR_BIT, 1, 1),
	SOC_DAPM_SINGLE("MIC1_BST1 Playback Switch", ALC5631_OUTMIXER_R_CTRL,
			ALC5631_M_MIC1_OUTMIXR_BIT, 1, 1),
	SOC_DAPM_SINGLE("DACR Playback Switch", ALC5631_OUTMIXER_R_CTRL,
			ALC5631_M_DACR_OUTMIXR_BIT, 1, 1),
	SOC_DAPM_SINGLE("RECMIXR Playback Switch", ALC5631_OUTMIXER_R_CTRL,
			ALC5631_M_RECMIXR_OUTMIXR_BIT, 1, 1),
	SOC_DAPM_SINGLE("RECMIXL Playback Switch", ALC5631_OUTMIXER_R_CTRL,
			ALC5631_M_RECMIXL_OUTMIXR_BIT, 1, 1),
};

static const struct snd_kcontrol_new alc5631_AXO1MIX_mixer_controls[] = {
	SOC_DAPM_SINGLE("MIC1_BST1 Playback Switch", ALC5631_AXO1MIXER_CTRL,
			ALC5631_M_MIC1_AXO1MIX_BIT , 1, 1),
	SOC_DAPM_SINGLE("MIC2_BST2 Playback Switch", ALC5631_AXO1MIXER_CTRL,
			ALC5631_M_MIC2_AXO1MIX_BIT, 1, 1),
	SOC_DAPM_SINGLE("OUTVOLL Playback Switch", ALC5631_AXO1MIXER_CTRL,
			ALC5631_M_OUTMIXL_AXO1MIX_BIT , 1 , 1),
	SOC_DAPM_SINGLE("OUTVOLR Playback Switch", ALC5631_AXO1MIXER_CTRL,
			ALC5631_M_OUTMIXR_AXO1MIX_BIT, 1, 1),
};

static const struct snd_kcontrol_new alc5631_AXO2MIX_mixer_controls[] = {
	SOC_DAPM_SINGLE("MIC1_BST1 Playback Switch", ALC5631_AXO2MIXER_CTRL,
			ALC5631_M_MIC1_AXO2MIX_BIT, 1, 1),
	SOC_DAPM_SINGLE("MIC2_BST2 Playback Switch", ALC5631_AXO2MIXER_CTRL,
			ALC5631_M_MIC2_AXO2MIX_BIT, 1, 1),
	SOC_DAPM_SINGLE("OUTVOLL Playback Switch", ALC5631_AXO2MIXER_CTRL,
			ALC5631_M_OUTMIXL_AXO2MIX_BIT, 1, 1),
	SOC_DAPM_SINGLE("OUTVOLR Playback Switch", ALC5631_AXO2MIXER_CTRL,
			ALC5631_M_OUTMIXR_AXO2MIX_BIT, 1 , 1),
};

static const struct snd_kcontrol_new alc5631_spolmix_mixer_controls[] = {
	SOC_DAPM_SINGLE("SPKVOLL Playback Switch", ALC5631_SPK_MONO_OUT_CTRL,
			ALC5631_M_SPKVOLL_SPOLMIX_BIT, 1, 1),
	SOC_DAPM_SINGLE("SPKVOLR Playback Switch", ALC5631_SPK_MONO_OUT_CTRL,
			ALC5631_M_SPKVOLR_SPOLMIX_BIT, 1, 1),
};

static const struct snd_kcontrol_new alc5631_spormix_mixer_controls[] = {
	SOC_DAPM_SINGLE("SPKVOLL Playback Switch", ALC5631_SPK_MONO_OUT_CTRL,
			ALC5631_M_SPKVOLL_SPORMIX_BIT, 1, 1),
	SOC_DAPM_SINGLE("SPKVOLR Playback Switch", ALC5631_SPK_MONO_OUT_CTRL,
			ALC5631_M_SPKVOLR_SPORMIX_BIT, 1, 1),
};

static const struct snd_kcontrol_new alc5631_monomix_mixer_controls[] = {
	SOC_DAPM_SINGLE("OUTVOLL Playback Switch", ALC5631_SPK_MONO_OUT_CTRL,
			ALC5631_M_OUTVOLL_MONOMIX_BIT, 1, 1),
	SOC_DAPM_SINGLE("OUTVOLR Playback Switch", ALC5631_SPK_MONO_OUT_CTRL,
			ALC5631_M_OUTVOLR_MONOMIX_BIT, 1, 1),
};

/* Left SPK Volume Input */
static const char *alc5631_spkvoll_sel[] = {"Vmid", "SPKMIXL"};

static const SOC_ENUM_SINGLE_DECL(
		alc5631_spkvoll_enum, ALC5631_SPK_OUT_VOL,
		ALC5631_L_EN_SHIFT, alc5631_spkvoll_sel);

static const struct snd_kcontrol_new alc5631_spkvoll_mux_control =
SOC_DAPM_ENUM("Left SPKVOL SRC", alc5631_spkvoll_enum);

/* Left HP Volume Input */
static const char *alc5631_hpvoll_sel[] = {"Vmid", "OUTMIXL"};

static const SOC_ENUM_SINGLE_DECL(
		alc5631_hpvoll_enum, ALC5631_HP_OUT_VOL,
		ALC5631_L_EN_SHIFT, alc5631_hpvoll_sel);

static const struct snd_kcontrol_new alc5631_hpvoll_mux_control =
SOC_DAPM_ENUM("Left HPVOL SRC", alc5631_hpvoll_enum);

/* Left Out Volume Input */
static const char *alc5631_outvoll_sel[] = {"Vmid", "OUTMIXL"};

static const SOC_ENUM_SINGLE_DECL(
		alc5631_outvoll_enum, ALC5631_MONO_AXO_1_2_VOL,
		ALC5631_L_EN_SHIFT, alc5631_outvoll_sel);

static const struct snd_kcontrol_new alc5631_outvoll_mux_control =
SOC_DAPM_ENUM("Left OUTVOL SRC", alc5631_outvoll_enum);

/* Right Out Volume Input */
static const char *alc5631_outvolr_sel[] = {"Vmid", "OUTMIXR"};

static const SOC_ENUM_SINGLE_DECL(
		alc5631_outvolr_enum, ALC5631_MONO_AXO_1_2_VOL,
		ALC5631_R_EN_SHIFT, alc5631_outvolr_sel);

static const struct snd_kcontrol_new alc5631_outvolr_mux_control =
SOC_DAPM_ENUM("Right OUTVOL SRC", alc5631_outvolr_enum);

/* Right HP Volume Input */
static const char *alc5631_hpvolr_sel[] = {"Vmid", "OUTMIXR"};

static const SOC_ENUM_SINGLE_DECL(
		alc5631_hpvolr_enum, ALC5631_HP_OUT_VOL,
		ALC5631_R_EN_SHIFT, alc5631_hpvolr_sel);

static const struct snd_kcontrol_new alc5631_hpvolr_mux_control =
SOC_DAPM_ENUM("Right HPVOL SRC", alc5631_hpvolr_enum);

/* Right SPK Volume Input */
static const char *alc5631_spkvolr_sel[] = {"Vmid", "SPKMIXR"};

static const SOC_ENUM_SINGLE_DECL(
		alc5631_spkvolr_enum, ALC5631_SPK_OUT_VOL,
		ALC5631_R_EN_SHIFT, alc5631_spkvolr_sel);

static const struct snd_kcontrol_new alc5631_spkvolr_mux_control =
SOC_DAPM_ENUM("Right SPKVOL SRC", alc5631_spkvolr_enum);

/* SPO Left Channel Input */
static const char *alc5631_spol_src_sel[] = {
	"SPOLMIX", "MONOIN_RX", "VDAC", "DACL"};

static const SOC_ENUM_SINGLE_DECL(
		alc5631_spol_src_enum, ALC5631_SPK_MONO_HP_OUT_CTRL,
		ALC5631_SPK_L_MUX_SEL_SHIFT, alc5631_spol_src_sel);

static const struct snd_kcontrol_new alc5631_spol_mux_control =
SOC_DAPM_ENUM("SPOL SRC", alc5631_spol_src_enum);

/* SPO Right Channel Input */
static const char *alc5631_spor_src_sel[] = {
	"SPORMIX", "MONOIN_RX", "VDAC", "DACR"};

static const SOC_ENUM_SINGLE_DECL(
		alc5631_spor_src_enum, ALC5631_SPK_MONO_HP_OUT_CTRL,
		ALC5631_SPK_R_MUX_SEL_SHIFT, alc5631_spor_src_sel);

static const struct snd_kcontrol_new alc5631_spor_mux_control =
SOC_DAPM_ENUM("SPOR SRC", alc5631_spor_src_enum);

/* MONO Input */
static const char *alc5631_mono_src_sel[] = {"MONOMIX", "MONOIN_RX", "VDAC"};

static const SOC_ENUM_SINGLE_DECL(
		alc5631_mono_src_enum, ALC5631_SPK_MONO_HP_OUT_CTRL,
		ALC5631_MONO_MUX_SEL_SHIFT, alc5631_mono_src_sel);

static const struct snd_kcontrol_new alc5631_mono_mux_control =
SOC_DAPM_ENUM("MONO SRC", alc5631_mono_src_enum);

/* Left HPO Input */
static const char *alc5631_hpl_src_sel[] = {"Left HPVOL", "Left DAC"};

static const SOC_ENUM_SINGLE_DECL(
		alc5631_hpl_src_enum, ALC5631_SPK_MONO_HP_OUT_CTRL,
		ALC5631_HP_L_MUX_SEL_SHIFT, alc5631_hpl_src_sel);

static const struct snd_kcontrol_new alc5631_hpl_mux_control =
SOC_DAPM_ENUM("HPL SRC", alc5631_hpl_src_enum);

/* Right HPO Input */
static const char *alc5631_hpr_src_sel[] = {"Right HPVOL", "Right DAC"};

static const SOC_ENUM_SINGLE_DECL(
		alc5631_hpr_src_enum, ALC5631_SPK_MONO_HP_OUT_CTRL,
		ALC5631_HP_R_MUX_SEL_SHIFT, alc5631_hpr_src_sel);

static const struct snd_kcontrol_new alc5631_hpr_mux_control =
SOC_DAPM_ENUM("HPR SRC", alc5631_hpr_src_enum);

static const struct snd_soc_dapm_widget alc5631_dapm_widgets[] = {
	/* Vmid */
	SND_SOC_DAPM_VMID("Vmid"),
	/* PLL1 */
	SND_SOC_DAPM_SUPPLY("PLL1", ALC5631_PWR_MANAG_ADD2,
			ALC5631_PWR_PLL1_BIT, 0, NULL, 0),

	/* Input Side */
	/* Input Lines */
	SND_SOC_DAPM_INPUT("MIC1"),
	SND_SOC_DAPM_INPUT("MIC2"),
	SND_SOC_DAPM_INPUT("AXIL"),
	SND_SOC_DAPM_INPUT("AXIR"),
	SND_SOC_DAPM_INPUT("MONOIN_RXN"),
	SND_SOC_DAPM_INPUT("MONOIN_RXP"),
	SND_SOC_DAPM_INPUT("DMICDAT"), //DMIC

	/* MICBIAS */
	SND_SOC_DAPM_MICBIAS("MIC Bias1", ALC5631_PWR_MANAG_ADD2,
			ALC5631_PWR_MICBIAS1_VOL_BIT, 1),
	SND_SOC_DAPM_MICBIAS("MIC Bias2", ALC5631_PWR_MANAG_ADD2,
			ALC5631_PWR_MICBIAS2_VOL_BIT, 1),

	/* Boost */
	SND_SOC_DAPM_PGA("MIC1 Boost", ALC5631_PWR_MANAG_ADD2,
			ALC5631_PWR_MIC1_BOOT_GAIN_BIT, 0, NULL, 0),
	SND_SOC_DAPM_PGA("MIC2 Boost", ALC5631_PWR_MANAG_ADD2,
			ALC5631_PWR_MIC2_BOOT_GAIN_BIT, 0, NULL, 0),
	SND_SOC_DAPM_PGA("MONOIN_RXP Boost", ALC5631_PWR_MANAG_ADD4,
			ALC5631_PWR_MONO_IN_P_VOL_BIT, 0, NULL, 0),
	SND_SOC_DAPM_PGA("MONOIN_RXN Boost", ALC5631_PWR_MANAG_ADD4,
			ALC5631_PWR_MONO_IN_N_VOL_BIT, 0, NULL, 0),
	SND_SOC_DAPM_PGA("AXIL Boost", ALC5631_PWR_MANAG_ADD4,
			ALC5631_PWR_AXIL_IN_VOL_BIT, 0, NULL, 0),
	SND_SOC_DAPM_PGA("AXIR Boost", ALC5631_PWR_MANAG_ADD4,
			ALC5631_PWR_AXIR_IN_VOL_BIT, 0, NULL, 0),

	/* MONO In */
	SND_SOC_DAPM_MIXER("MONO_IN", SND_SOC_NOPM, 0, 0, NULL, 0),

	/* REC Mixer */
	SND_SOC_DAPM_MIXER("RECMIXL Mixer", ALC5631_PWR_MANAG_ADD2,
			ALC5631_PWR_RECMIXER_L_BIT, 0,
			&alc5631_recmixl_mixer_controls[0],
			ARRAY_SIZE(alc5631_recmixl_mixer_controls)),
	SND_SOC_DAPM_MIXER("RECMIXR Mixer", ALC5631_PWR_MANAG_ADD2,
			ALC5631_PWR_RECMIXER_R_BIT, 0,
			&alc5631_recmixr_mixer_controls[0],
			ARRAY_SIZE(alc5631_recmixr_mixer_controls)),
	/* Because of record duplication for L/R channel,
	 * L/R ADCs need power up at the same time */
	SND_SOC_DAPM_MIXER("ADC Mixer", SND_SOC_NOPM, 0, 0, NULL, 0),

	/* DMIC */
	SND_SOC_DAPM_SUPPLY("DMIC Supply", ALC5631_DIG_MIC_CTRL,
			ALC5631_DMIC_ENA_SHIFT, 0,
			set_dmic_params, SND_SOC_DAPM_PRE_PMU),
	/* ADC Data Srouce */
	SND_SOC_DAPM_SUPPLY("Left ADC Select", ALC5631_INT_ST_IRQ_CTRL_2,
			ALC5631_ADC_DATA_SEL_MIC1_SHIFT, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("Right ADC Select", ALC5631_INT_ST_IRQ_CTRL_2,
			ALC5631_ADC_DATA_SEL_MIC2_SHIFT, 0, NULL, 0),

	/* ADCs */
	SND_SOC_DAPM_ADC("Left ADC", "HIFI Capture",
			ALC5631_PWR_MANAG_ADD1, ALC5631_PWR_ADC_L_CLK_BIT, 0),
	SND_SOC_DAPM_ADC("Right ADC", "HIFI Capture",
			ALC5631_PWR_MANAG_ADD1, ALC5631_PWR_ADC_R_CLK_BIT, 0),

	/* DAC and ADC supply power */
	SND_SOC_DAPM_SUPPLY("I2S", ALC5631_PWR_MANAG_ADD1,
			ALC5631_PWR_MAIN_I2S_BIT, 0/*defult 0*/, NULL, 0),
	SND_SOC_DAPM_SUPPLY("DAC REF", ALC5631_PWR_MANAG_ADD1,
			ALC5631_PWR_DAC_REF_BIT, 0, NULL, 0),

	/* Output Side */
	/* DACs */
	SND_SOC_DAPM_DAC("Left DAC", "HIFI Playback",
			ALC5631_PWR_MANAG_ADD1, ALC5631_PWR_DAC_L_CLK_BIT, 0),
	SND_SOC_DAPM_DAC("Right DAC", "HIFI Playback",
			ALC5631_PWR_MANAG_ADD1, ALC5631_PWR_DAC_R_CLK_BIT, 0),
	SND_SOC_DAPM_DAC("Voice DAC", "Voice DAC Mono Playback",
			SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_PGA("Voice DAC Boost", SND_SOC_NOPM, 0, 0, NULL, 0),
	/* DAC supply power */
	SND_SOC_DAPM_SUPPLY("Left DAC To Mixer", ALC5631_PWR_MANAG_ADD1,
			ALC5631_PWR_DAC_L_TO_MIXER_BIT, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("Right DAC To Mixer", ALC5631_PWR_MANAG_ADD1,
			ALC5631_PWR_DAC_R_TO_MIXER_BIT, 0, NULL, 0),

	/* Left SPK Mixer */
	SND_SOC_DAPM_MIXER("SPKMIXL Mixer", ALC5631_PWR_MANAG_ADD2,
			ALC5631_PWR_SPKMIXER_L_BIT, 0,
			&alc5631_spkmixl_mixer_controls[0],
			ARRAY_SIZE(alc5631_spkmixl_mixer_controls)),
	/* Left Out Mixer */
	SND_SOC_DAPM_MIXER("OUTMIXL Mixer", ALC5631_PWR_MANAG_ADD2,
			ALC5631_PWR_OUTMIXER_L_BIT, 0,
			&alc5631_outmixl_mixer_controls[0],
			ARRAY_SIZE(alc5631_outmixl_mixer_controls)),
	/* Right Out Mixer */
	SND_SOC_DAPM_MIXER("OUTMIXR Mixer", ALC5631_PWR_MANAG_ADD2,
			ALC5631_PWR_OUTMIXER_R_BIT, 0,
			&alc5631_outmixr_mixer_controls[0],
			ARRAY_SIZE(alc5631_outmixr_mixer_controls)),
	/* Right SPK Mixer */
	SND_SOC_DAPM_MIXER("SPKMIXR Mixer", ALC5631_PWR_MANAG_ADD2,
			ALC5631_PWR_SPKMIXER_R_BIT, 0,
			&alc5631_spkmixr_mixer_controls[0],
			ARRAY_SIZE(alc5631_spkmixr_mixer_controls)),

	/* Volume Mux */
	SND_SOC_DAPM_MUX("Left SPKVOL Mux", ALC5631_PWR_MANAG_ADD4,
			ALC5631_PWR_SPK_L_VOL_BIT, 0,
			&alc5631_spkvoll_mux_control),
	SND_SOC_DAPM_MUX("Left HPVOL Mux", ALC5631_PWR_MANAG_ADD4,
			ALC5631_PWR_HP_L_OUT_VOL_BIT, 0,
			&alc5631_hpvoll_mux_control),
	SND_SOC_DAPM_MUX("Left OUTVOL Mux", ALC5631_PWR_MANAG_ADD4,
			ALC5631_PWR_LOUT_VOL_BIT, 0,
			&alc5631_outvoll_mux_control),
	SND_SOC_DAPM_MUX("Right OUTVOL Mux", ALC5631_PWR_MANAG_ADD4,
			ALC5631_PWR_ROUT_VOL_BIT, 0,
			&alc5631_outvolr_mux_control),
	SND_SOC_DAPM_MUX("Right HPVOL Mux", ALC5631_PWR_MANAG_ADD4,
			ALC5631_PWR_HP_R_OUT_VOL_BIT, 0,
			&alc5631_hpvolr_mux_control),
	SND_SOC_DAPM_MUX("Right SPKVOL Mux", ALC5631_PWR_MANAG_ADD4,
			ALC5631_PWR_SPK_R_VOL_BIT, 0,
			&alc5631_spkvolr_mux_control),

	/* DAC To HP */
	SND_SOC_DAPM_PGA_S("Left DAC_HP", 0, SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_PGA_S("Right DAC_HP", 0, SND_SOC_NOPM, 0, 0, NULL, 0),

	/* HP Depop */
	SND_SOC_DAPM_PGA_S("HP Depop", 1, SND_SOC_NOPM, 0, 0,
			hp_event, SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMU),

	/* AXO1 Mixer */
	SND_SOC_DAPM_MIXER("AXO1MIX Mixer", ALC5631_PWR_MANAG_ADD3,
			ALC5631_PWR_AXO1MIXER_BIT, 0,
			&alc5631_AXO1MIX_mixer_controls[0],
			ARRAY_SIZE(alc5631_AXO1MIX_mixer_controls)),
	/* SPOL Mixer */
	SND_SOC_DAPM_MIXER("SPOLMIX Mixer", SND_SOC_NOPM, 0, 0,
			&alc5631_spolmix_mixer_controls[0],
			ARRAY_SIZE(alc5631_spolmix_mixer_controls)),
	/* MONO Mixer */
	SND_SOC_DAPM_MIXER("MONOMIX Mixer", ALC5631_PWR_MANAG_ADD3,
			ALC5631_PWR_MONOMIXER_BIT, 0,
			&alc5631_monomix_mixer_controls[0],
			ARRAY_SIZE(alc5631_monomix_mixer_controls)),
	/* SPOR Mixer */
	SND_SOC_DAPM_MIXER("SPORMIX Mixer", SND_SOC_NOPM, 0, 0,
			&alc5631_spormix_mixer_controls[0],
			ARRAY_SIZE(alc5631_spormix_mixer_controls)),
	/* AXO2 Mixer */
	SND_SOC_DAPM_MIXER("AXO2MIX Mixer", ALC5631_PWR_MANAG_ADD3,
			ALC5631_PWR_AXO2MIXER_BIT, 0,
			&alc5631_AXO2MIX_mixer_controls[0],
			ARRAY_SIZE(alc5631_AXO2MIX_mixer_controls)),

	/* Mux */
	SND_SOC_DAPM_MUX("SPOL Mux", SND_SOC_NOPM, 0, 0,
			&alc5631_spol_mux_control),
	SND_SOC_DAPM_MUX("SPOR Mux", SND_SOC_NOPM, 0, 0,
			&alc5631_spor_mux_control),
	SND_SOC_DAPM_MUX("MONO Mux", SND_SOC_NOPM, 0, 0,
			&alc5631_mono_mux_control),
	SND_SOC_DAPM_MUX("HPL Mux", SND_SOC_NOPM, 0, 0,
			&alc5631_hpl_mux_control),
	SND_SOC_DAPM_MUX("HPR Mux", SND_SOC_NOPM, 0, 0,
			&alc5631_hpr_mux_control),

	/* AMP supply */
	SND_SOC_DAPM_SUPPLY("MONO Depop", ALC5631_PWR_MANAG_ADD3,
			ALC5631_PWR_MONO_DEPOP_DIS_BIT, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("Class D", ALC5631_PWR_MANAG_ADD1,
			ALC5631_PWR_CLASS_D_BIT, 0, NULL, 0),

	/* Output Lines */
	SND_SOC_DAPM_OUTPUT("AUXO1"),
	SND_SOC_DAPM_OUTPUT("AUXO2"),
	SND_SOC_DAPM_OUTPUT("SPOL"),
	SND_SOC_DAPM_OUTPUT("SPOR"),
	SND_SOC_DAPM_OUTPUT("HPOL"),
	SND_SOC_DAPM_OUTPUT("HPOR"),
	SND_SOC_DAPM_OUTPUT("MONO"),
};

static const struct snd_soc_dapm_route alc5631_dapm_routes[] = {
	{"MIC1 Boost", NULL, "MIC1"},
	{"MIC2 Boost", NULL, "MIC2"},
	{"MONOIN_RXP Boost", NULL, "MONOIN_RXP"},
	{"MONOIN_RXN Boost", NULL, "MONOIN_RXN"},
	{"AXIL Boost", NULL, "AXIL"},
	{"AXIR Boost", NULL, "AXIR"},

	{"MONO_IN", NULL, "MONOIN_RXP Boost"},
	{"MONO_IN", NULL, "MONOIN_RXN Boost"},

	{"RECMIXL Mixer", "OUTMIXL Capture Switch", "OUTMIXL Mixer"},
	{"RECMIXL Mixer", "MIC1_BST1 Capture Switch", "MIC1 Boost"},
	{"RECMIXL Mixer", "AXILVOL Capture Switch", "AXIL Boost"},
	{"RECMIXL Mixer", "MONOIN_RX Capture Switch", "MONO_IN"},

	{"RECMIXR Mixer", "OUTMIXR Capture Switch", "OUTMIXR Mixer"},
	{"RECMIXR Mixer", "MIC2_BST2 Capture Switch", "MIC2 Boost"},
	{"RECMIXR Mixer", "AXIRVOL Capture Switch", "AXIR Boost"},
	{"RECMIXR Mixer", "MONOIN_RX Capture Switch", "MONO_IN"},

	{"ADC Mixer", NULL, "RECMIXL Mixer"},
	{"ADC Mixer", NULL, "RECMIXR Mixer"},

	{"Left ADC", NULL, "ADC Mixer"},
	{"Left ADC", NULL, "Left ADC Select", check_adcl_select},
	{"Left ADC", NULL, "PLL1", check_sysclk1_source},
	{"Left ADC", NULL, "I2S"},
	{"Left ADC", NULL, "DAC REF"},

	{"Right ADC", NULL, "ADC Mixer"},
	{"Right ADC", NULL, "Right ADC Select", check_adcr_select},
	{"Right ADC", NULL, "PLL1", check_sysclk1_source},
	{"Right ADC", NULL, "I2S"},
	{"Right ADC", NULL, "DAC REF"},

	{"DMICDAT", NULL, "DMIC Supply", check_dmic_used},
	{"Left ADC", NULL, "DMICDAT"},
	{"Right ADC", NULL, "DMICDAT"},

	{"Left DAC", NULL, "PLL1", check_sysclk1_source},
#ifdef ALC5631_DEBUG
	{"Left DAC", NULL, "I2S",check_i2s_power},
	{"Left DAC", NULL, "DAC REF", check_dac_ref_power},
#else
	{"Left DAC", NULL, "I2S"},
	{"Left DAC", NULL, "DAC REF"},
#endif

	{"Right DAC", NULL, "PLL1", check_sysclk1_source},
#ifdef ALC5631_DEBUG
	{"Right DAC", NULL, "I2S",check_i2s_power},
	{"Right DAC", NULL, "DAC REF",check_dac_ref_power},
#else
	{"Right DAC", NULL, "I2S"},
	{"Right DAC", NULL, "DAC REF"},
#endif

#ifdef ALC5631_DEBUG
	{"Voice DAC Boost", NULL, "Voice DAC",check_voice_dac_power},
#else
	{"Voice DAC Boost", NULL, "Voice DAC"},
#endif

	{"SPKMIXL Mixer", NULL, "Left DAC To Mixer", check_dacl_to_spkmixl},
	{"SPKMIXL Mixer", "RECMIXL Playback Switch", "RECMIXL Mixer"},
	{"SPKMIXL Mixer", "MIC1_P Playback Switch", "MIC1"},
	{"SPKMIXL Mixer", "DACL Playback Switch", "Left DAC"},
	{"SPKMIXL Mixer", "OUTMIXL Playback Switch", "OUTMIXL Mixer"},
	
	{"SPKMIXR Mixer", NULL, "Right DAC To Mixer", check_dacr_to_spkmixr},
	{"SPKMIXR Mixer", "OUTMIXR Playback Switch", "OUTMIXR Mixer"},
	{"SPKMIXR Mixer", "DACR Playback Switch", "Right DAC"},
	{"SPKMIXR Mixer", "MIC2_P Playback Switch", "MIC2"},
	{"SPKMIXR Mixer", "RECMIXR Playback Switch", "RECMIXR Mixer"},

	{"OUTMIXL Mixer", NULL, "Left DAC To Mixer", check_dacl_to_outmixl},
	{"OUTMIXL Mixer", "RECMIXL Playback Switch", "RECMIXL Mixer"},
	{"OUTMIXL Mixer", "RECMIXR Playback Switch", "RECMIXR Mixer"},
	{"OUTMIXL Mixer", "DACL Playback Switch", "Left DAC"},
	{"OUTMIXL Mixer", "MIC1_BST1 Playback Switch", "MIC1 Boost"},
	{"OUTMIXL Mixer", "MIC2_BST2 Playback Switch", "MIC2 Boost"},
	{"OUTMIXL Mixer", "MONOIN_RXP Playback Switch", "MONOIN_RXP Boost"},
	{"OUTMIXL Mixer", "AXILVOL Playback Switch", "AXIL Boost"},
	{"OUTMIXL Mixer", "AXIRVOL Playback Switch", "AXIR Boost"},
	{"OUTMIXL Mixer", "VDAC Playback Switch", "Voice DAC Boost"},

	{"OUTMIXR Mixer", NULL, "Right DAC To Mixer", check_dacr_to_outmixr},
	{"OUTMIXR Mixer", "RECMIXL Playback Switch", "RECMIXL Mixer"},
	{"OUTMIXR Mixer", "RECMIXR Playback Switch", "RECMIXR Mixer"},
	{"OUTMIXR Mixer", "DACR Playback Switch", "Right DAC"},
	{"OUTMIXR Mixer", "MIC1_BST1 Playback Switch", "MIC1 Boost"},
	{"OUTMIXR Mixer", "MIC2_BST2 Playback Switch", "MIC2 Boost"},
	{"OUTMIXR Mixer", "MONOIN_RXN Playback Switch", "MONOIN_RXN Boost"},
	{"OUTMIXR Mixer", "AXILVOL Playback Switch", "AXIL Boost"},
	{"OUTMIXR Mixer", "AXIRVOL Playback Switch", "AXIR Boost"},
	{"OUTMIXR Mixer", "VDAC Playback Switch", "Voice DAC Boost"},

	{"Left SPKVOL Mux",  "SPKMIXL", "SPKMIXL Mixer"},
	{"Left SPKVOL Mux",  "Vmid", "Vmid"},
	{"Left HPVOL Mux",  "OUTMIXL", "OUTMIXL Mixer"},
	{"Left HPVOL Mux",  "Vmid", "Vmid"},
	{"Left OUTVOL Mux",  "OUTMIXL", "OUTMIXL Mixer"},
	{"Left OUTVOL Mux",  "Vmid", "Vmid"},
	{"Right OUTVOL Mux",  "OUTMIXR", "OUTMIXR Mixer"},
	{"Right OUTVOL Mux",  "Vmid", "Vmid"},
	{"Right HPVOL Mux",  "OUTMIXR", "OUTMIXR Mixer"},
	{"Right HPVOL Mux",  "Vmid", "Vmid"},
	{"Right SPKVOL Mux",  "SPKMIXR", "SPKMIXR Mixer"},
	{"Right SPKVOL Mux",  "Vmid", "Vmid"},

	{"AXO1MIX Mixer", "MIC1_BST1 Playback Switch", "MIC1 Boost"},
	{"AXO1MIX Mixer", "OUTVOLL Playback Switch", "Left OUTVOL Mux"},
	{"AXO1MIX Mixer", "OUTVOLR Playback Switch", "Right OUTVOL Mux"},
	{"AXO1MIX Mixer", "MIC2_BST2 Playback Switch", "MIC2 Boost"},

	{"AXO2MIX Mixer", "MIC1_BST1 Playback Switch", "MIC1 Boost"},
	{"AXO2MIX Mixer", "OUTVOLL Playback Switch", "Left OUTVOL Mux"},
	{"AXO2MIX Mixer", "OUTVOLR Playback Switch", "Right OUTVOL Mux"},
	{"AXO2MIX Mixer", "MIC2_BST2 Playback Switch", "MIC2 Boost"},

	{"SPOLMIX Mixer", "SPKVOLL Playback Switch", "Left SPKVOL Mux"},
	{"SPOLMIX Mixer", "SPKVOLR Playback Switch", "Right SPKVOL Mux"},

	{"SPORMIX Mixer", "SPKVOLL Playback Switch", "Left SPKVOL Mux"},
	{"SPORMIX Mixer", "SPKVOLR Playback Switch", "Right SPKVOL Mux"},

	{"MONOMIX Mixer", "OUTVOLL Playback Switch", "Left OUTVOL Mux"},
	{"MONOMIX Mixer", "OUTVOLR Playback Switch", "Right OUTVOL Mux"},

	{"SPOL Mux", "SPOLMIX", "SPOLMIX Mixer"},
	{"SPOL Mux", "MONOIN_RX", "MONO_IN"},
	{"SPOL Mux", "VDAC", "Voice DAC Boost"},
	{"SPOL Mux", "DACL", "Left DAC"},

	{"SPOR Mux", "SPORMIX", "SPORMIX Mixer"},
	{"SPOR Mux", "MONOIN_RX", "MONO_IN"},
	{"SPOR Mux", "VDAC", "Voice DAC Boost"},
	{"SPOR Mux", "DACR", "Right DAC"},

	{"MONO Mux", "MONOMIX", "MONOMIX Mixer"},
	{"MONO Mux", "MONOIN_RX", "MONO_IN"},
	{"MONO Mux", "VDAC", "Voice DAC Boost"},

	{"Right DAC_HP", NULL, "Right DAC"},
	{"Left DAC_HP", NULL, "Left DAC"},

	{"HPL Mux", "Left HPVOL", "Left HPVOL Mux"},
	{"HPL Mux", "Left DAC", "Left DAC_HP"},
	{"HPR Mux", "Right HPVOL", "Right HPVOL Mux"},
	{"HPR Mux", "Right DAC", "Right DAC_HP"},

	{"HP Depop", NULL, "HPL Mux"},
	{"HP Depop", NULL, "HPR Mux"},

	{"AUXO1", NULL, "AXO1MIX Mixer"},
	{"AUXO2", NULL, "AXO2MIX Mixer"},

#ifdef ALC5631_DEBUG
	{"SPOL", NULL, "Class D", check_class_d_power},
	{"SPOL", NULL, "SPOL Mux", check_spol_mux_power},
	{"SPOR", NULL, "Class D", check_class_d_power},
	{"SPOR", NULL, "SPOR Mux", check_spor_mux_power},
#else
	{"SPOL", NULL, "Class D"},
	{"SPOL", NULL, "SPOL Mux"},
	{"SPOR", NULL, "Class D"},
	{"SPOR", NULL, "SPOR Mux"},
#endif

#ifdef ALC5631_DEBUG
	{"HPOL", NULL, "HP Depop", check_hp_depop_power},
	{"HPOR", NULL, "HP Depop", check_hp_depop_power},
#else
	{"HPOL", NULL, "HP Depop"},
	{"HPOR", NULL, "HP Depop"},
#endif

	{"MONO", NULL, "MONO Depop"},
	{"MONO", NULL, "MONO Mux"},
};

struct coeff_clk_div {
	u32 mclk;
	u32 bclk;
	u32 rate;
	u16 reg_val;
};

/* PLL divisors */
struct pll_div {
	u32 pll_in;
	u32 pll_out;
	u16 reg_val;
};

static const struct pll_div codec_master_pll_div[] = {
	{22579200,	11289600,  0x0622},  //qiang_debug added
	{16500000, 11289600, 0x4809 },
	{13200000,  5644800,  0x101c}, //lrck 0x101f = 18.2k
	//     0x101c = 22.1k
	{24000000, 11289600, 0x263f },   
	//lrck 0x0139 = 5.11k
	//lrck 0x2139 = 59.65k
	//lrck 0x1a39 = 47.72k
	//lrck 0x1739 = 42,6k
	//lrck 0x1839 = 44.3k
	//lrck 0x183a = 40.6k
	//lrck 0x193a = 42.2k
	//lrck 0x1a3a = 43.7k
	//lrck 0x1b3b = 41.8k
	//lrck 0x1c3b = 43.2k
	//lrck 0x1f3c = 44.2k
	//lrck 0x263f = 44.12k
	
	{13200000, 11289600, 0x2d39 },   
	//lrck 0x363b = 44.4k
	//lrck 0x353b = 43.6k
	//lrck 0x353a = 47.2k
	//lrck 0x333a = 45.5k
	//lrck 0x323a = 44.6k
	//lrck 0x313a = 43.8k
	//lrck 0x2c39 = 43.1k
	//lrck 0x2d39 = 44.06k

	//43 K 0x067f
	//48   0x067e
	//48.5 0x066f
	//42.4 0x056f
	//41.5 0x056f
	//40.4 0x034f
	//43   0x034e
	//45.1 0x056e
	//44.1 0x045e
	{21999999, 11289600, 0x2a26 },   
	//54 k 0x0f30
	//51.5 k 0x0e30
	//48.5 k 0x0d30
	//45.1 k 0x0c30
	//34.9 k 0x0b21
	//37.5 k 0xc21
	//40.2 k 0x0d21
	//43   k 0xe21
	//64.4 k 0x1e22
	//36.8 k 0x1e25
	//43   k 0x1e24
	//39.1 k 0x2025
	//41.4 k 0x2225
	//42.5 k 0x2325
	//44.8 k 0x2525
	//46.3 k 0x2c26
	//45.3 k 0x2b26
	//44.3 k 0x2a26
	{2048000,  8192000,  0x0ea0},
	{3686400,  8192000,  0x4e27},
	{12000000,  8192000,  0x456b},
	{13000000,  8192000,  0x495f},
	{13100000,  8192000,  0x0320},
	{2048000,  11289600,  0xf637},
	{3686400,  11289600,  0x2f22},
	{12000000,  11289600,  0x3e2f},
	{13000000,  11289600,  0x4d5b},
	{13100000,  11289600,  0x363b},
	{2048000,  16384000,  0x1ea0},
	{3686400,  16384000,  0x9e27},
	{12000000,  16384000,  0x452b},
	{13000000,  16384000,  0x542f},
	{13100000,  16384000,  0x03a0},
	{2048000,  16934400,  0xe625},
	{3686400,  16934400,  0x9126},
	{12000000,  16934400,  0x4d2c},
	{13000000,  16934400,  0x742f},
	{13100000,  16934400,  0x3c27},
	{2048000,  22579200,  0x2aa0},
	{3686400,  22579200,  0x2f20},
	{12000000,  22579200,  0x7e2f},
	{13000000,  22579200,  0x742f},
	{13100000,  22579200,  0x3c27},
	{2048000,  24576000,  0x2ea0},
	{3686400,  24576000,  0xee27},
	{12000000,  24576000,  0x2915},
	{13000000,  24576000,  0x772e},
	{13100000,  24576000,  0x0d20},
	{26000000,  24576000,  0x2027},
	{26000000,  22579200,  0x392f},
	{24576000,  22579200,  0x0921},
	{24576000,  24576000,  0x02a0},
};

static const struct pll_div codec_slave_pll_div[] = {
	{16500000, 11289600, 0x4809 },
	{256000,  2048000,  0x46f0},
	{256000,  4096000,  0x3ea0},
	{352800,  5644800,  0x3ea0},
	{512000,  8192000,  0x3ea0},
	{1024000,  8192000,  0x46f0},
	{705600,  11289600,  0x3ea0},
	{1024000,  16384000,  0x3ea0},
	{1411200,  22579200,  0x3ea0},
	{1536000,  24576000,  0x3ea0},
	{2048000,  16384000,  0x1ea0},
	{2822400,  22579200,  0x1ea0},
	{2822400,  45158400,  0x5ec0},
	{5644800,  45158400,  0x46f0},
	{3072000,  24576000,  0x1ea0},
	{3072000,  49152000,  0x5ec0},
	{6144000,  49152000,  0x46f0},
	{705600,  11289600,  0x3ea0},
	{705600,  8467200,  0x3ab0},
	{24576000,  24576000,  0x02a0},
	{1411200,  11289600,  0x1690},
	{2822400,  11289600,  0x0a90},
	{1536000,  12288000,  0x1690},
	{3072000,  12288000,  0x0a90},
};

struct coeff_clk_div coeff_div[] = {
	/* sysclk is 256fs */
	{2048000,  8000 * 32,  8000, 0x1000},
	{2048000,  8000 * 64,  8000, 0x0000},
	{2822400,  11025 * 32,  11025,  0x1000},
	{2822400,  11025 * 64,  11025,  0x0000},
	{4096000,  16000 * 32,  16000,  0x1000},
	{4096000,  16000 * 64,  16000,  0x0000},
	{5644800,  22050 * 32,  22050,  0x1000},
	{5644800,  22050 * 64,  22050,  0x0000},
	{8192000,  32000 * 32,  32000,  0x1000},
	{8192000,  32000 * 64,  32000,  0x0000},
	{11289600,  44100 * 32,  44100,  0x1000},
	{11289600,  44100 * 64,  44100,  0x0000},
	{12288000,  48000 * 32,  48000,  0x1000},
	{12288000,  48000 * 64,  48000,  0x0000},
	{22579200,  88200 * 32,  88200,  0x1000},
	{22579200,  88200 * 64,  88200,  0x0000},
	{24576000,  96000 * 32,  96000,  0x1000},
	{24576000,  96000 * 64,  96000,  0x0000},
	/* sysclk is 512fs */
	{4096000,  8000 * 32,  8000, 0x3000},
	{4096000,  8000 * 64,  8000, 0x2000},
	{5644800,  11025 * 32,  11025, 0x3000},
	{5644800,  11025 * 64,  11025, 0x2000},
	{8192000,  16000 * 32,  16000, 0x3000},
	{8192000,  16000 * 64,  16000, 0x2000},
	{11289600,  22050 * 32,  22050, 0x3000},
	{11289600,  22050 * 64,  22050, 0x2000},
	{16384000,  32000 * 32,  32000, 0x3000},
	{16384000,  32000 * 64,  32000, 0x2000},
	{22579200,  44100 * 32,  44100, 0x3000},
	{22579200,  44100 * 64,  44100, 0x2000},
	{24576000,  48000 * 32,  48000, 0x3000},
	{24576000,  48000 * 64,  48000, 0x2000},
	{45158400,  88200 * 32,  88200, 0x3000},
	{45158400,  88200 * 64,  88200, 0x2000},
	{49152000,  96000 * 32,  96000, 0x3000},
	{49152000,  96000 * 64,  96000, 0x2000},
	/* sysclk is 24.576Mhz or 22.5792Mhz */
	{24576000,  8000 * 32,  8000,  0x7080},
	{24576000,  8000 * 64,  8000,  0x6080},
	{24576000,  16000 * 32,  16000,  0x5080},
	{24576000,  16000 * 64,  16000,  0x4080},
	{24576000,  24000 * 32,  24000,  0x5000},
	{24576000,  24000 * 64,  24000,  0x4000},
	{24576000,  32000 * 32,  32000,  0x3080},
	{24576000,  32000 * 64,  32000,  0x2080},
	{22579200,  11025 * 32,  11025,  0x7000},
	{22579200,  11025 * 64,  11025,  0x6000},
	{22579200,  22050 * 32,  22050,  0x5000},
	{22579200,  22050 * 64,  22050,  0x4000},
};

static int get_coeff(int mclk, int rate, int timesofbclk)
{
	int i;
	printk("%s----%d, %d, %d\n", __func__, mclk, rate, timesofbclk);

	for (i = 0; i < ARRAY_SIZE(coeff_div); i++) {
		if (coeff_div[i].mclk == mclk && coeff_div[i].rate == rate &&
				(coeff_div[i].bclk / coeff_div[i].rate) == timesofbclk)
			return i;
	}
	return -EINVAL;
}

static int alc5631_hifi_pcm_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec;
	struct alc5631_priv *alc5631 = snd_soc_codec_get_drvdata(codec);
	int timesofbclk = 32, coeff;
	unsigned int iface = 0;

	dev_dbg(codec->dev, "enter %s\n", __func__);
	alc5631->bclk_rate = snd_soc_params_to_bclk(params);

#ifdef ALC5631_DEBUG
	printk("================%s===========, bclk_rate = %d \n", __func__, alc5631->bclk_rate);
#endif
	if (alc5631->bclk_rate < 0) {
		dev_err(codec->dev, "Fail to get BCLK rate\n");
		return alc5631->bclk_rate;
	}
	alc5631->rx_rate = params_rate(params);

	if (alc5631->master)
		coeff = get_coeff(alc5631->sysclk, alc5631->rx_rate,
				alc5631->bclk_rate / alc5631->rx_rate);
	else
		coeff = get_coeff(alc5631->sysclk, alc5631->rx_rate,
				timesofbclk);
	printk("bclk_rate = %d, rx_rate = %d, coeff = %d master:%d \n", alc5631->bclk_rate, alc5631->rx_rate, coeff, alc5631->master);
	if (coeff < 0) {
		dev_err(codec->dev, "Fail to get coeff\n");
		return -EINVAL;
	}
#ifdef ALC5631_DEBUG
	printk("bclk_rate = %d, rx_rate = %d, coeff = %d \n", alc5631->bclk_rate, alc5631->rx_rate, coeff);
#endif
	switch (params_format(params)) {
		case SNDRV_PCM_FORMAT_S16_LE:
			break;
		case SNDRV_PCM_FORMAT_S20_3LE:
			iface |= ALC5631_SDP_I2S_DL_20;
			break;
		case SNDRV_PCM_FORMAT_S24_LE:
			iface |= ALC5631_SDP_I2S_DL_24;
			break;
		case SNDRV_PCM_FORMAT_S8:
		case SNDRV_PCM_FORMAT_S32_LE:
			iface |= ALC5631_SDP_I2S_DL_8;
			break;
		default:
			return -EINVAL;
	}
#ifdef ALC5631_DEBUG
	printk(" iface = %d \n", iface);
#endif
	//	iface |= 0x2; //JOIN ++ debug
	snd_soc_update_bits(codec, ALC5631_SDP_CTRL,
			ALC5631_SDP_I2S_DL_MASK, iface);
	snd_soc_write(codec, ALC5631_STEREO_AD_DA_CLK_CTRL,
			coeff_div[coeff].reg_val);

	return 0;
}

static int alc5631_hifi_codec_set_dai_fmt(struct snd_soc_dai *codec_dai,
		unsigned int fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct alc5631_priv *alc5631 = snd_soc_codec_get_drvdata(codec);
	unsigned int iface = 0;

	dev_dbg(codec->dev, "enter %s\n", __func__);
	snd_soc_write(pcodec, 0x02, 0xc8c8);	
#ifdef ALC5631_DEBUG
	printk(" %s \n", __func__);
#endif

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
		case SND_SOC_DAIFMT_CBM_CFM:
			alc5631->master = 1;
			break;
		case SND_SOC_DAIFMT_CBS_CFS:
			iface |= ALC5631_SDP_MODE_SEL_SLAVE;
			alc5631->master = 0;
			break;
		default:
			return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
		case SND_SOC_DAIFMT_I2S:
			break;
		case SND_SOC_DAIFMT_LEFT_J:
			iface |= ALC5631_SDP_I2S_DF_LEFT;
			break;
		case SND_SOC_DAIFMT_DSP_A:
			iface |= ALC5631_SDP_I2S_DF_PCM_A;
			break;
		case SND_SOC_DAIFMT_DSP_B:
			iface  |= ALC5631_SDP_I2S_DF_PCM_B;
			break;
		default:
			return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
		case SND_SOC_DAIFMT_NB_NF:
			break;
		case SND_SOC_DAIFMT_IB_NF:
			iface |= ALC5631_SDP_I2S_BCLK_POL_CTRL;
			break;
		default:
			return -EINVAL;
	}

#ifdef ALC5631_DEBUG
	printk(" %s iface = %d REG-02:%x\n", __func__, iface, snd_soc_read(codec, ALC5631_SPK_OUT_VOL));
#endif
	snd_soc_write(codec, ALC5631_SDP_CTRL, iface);

	return 0;
}

static int alc5631_hifi_codec_set_dai_sysclk(struct snd_soc_dai *codec_dai,
		int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct alc5631_priv *alc5631 = snd_soc_codec_get_drvdata(codec);

	dev_dbg(codec->dev, "enter %s, syclk=%d\n", __func__, freq);
#ifdef ALC5631_DEBUG
	printk("enter %s, syclk=%d REG-02:%x\n", __func__, freq, snd_soc_read(codec, ALC5631_SPK_OUT_VOL));
#endif

	if ((freq >= (256 * 8000)) && (freq <= (512 * 96000))) {
		alc5631->sysclk = freq;
#ifdef ALC5631_DEBUG
		printk("2 enter %s, syclk=%d\n", __func__, freq);
#endif
		return 0;
	}

	return -EINVAL;
}

static int alc5631_codec_set_dai_pll(struct snd_soc_dai *codec_dai, int pll_id,
		int source, unsigned int freq_in, unsigned int freq_out)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct alc5631_priv *alc5631 = snd_soc_codec_get_drvdata(codec);
	int i, ret = -EINVAL;

	dev_dbg(codec->dev, "enter %s\n", __func__);

#ifdef ALC5631_DEBUG
	printk("==%s freq_in = %d , freq_out = %d REG-02:%x\n", __func__, freq_in, freq_out, snd_soc_read(codec, ALC5631_SPK_OUT_VOL));
#endif

	if (!freq_in || !freq_out) {
		dev_dbg(codec->dev, "PLL disabled\n");

		snd_soc_update_bits(codec, ALC5631_GLOBAL_CLK_CTRL,
				ALC5631_SYSCLK_SOUR_SEL_MASK,
				ALC5631_SYSCLK_SOUR_SEL_MCLK);

		return 0;
	}

#ifdef ALC5631_DEBUG
	printk("%s freq_in = %d , freq_out = %d \n", __func__, freq_in, freq_out);
#endif
	if (alc5631->master) {
		for (i = 0; i < ARRAY_SIZE(codec_master_pll_div); i++)
			if (freq_in == codec_master_pll_div[i].pll_in &&
					freq_out == codec_master_pll_div[i].pll_out) {
#ifdef ALC5631_DEBUG
				printk("chnge Pll in master mode \n");
#endif
				dev_info(codec->dev,
						"change PLL in master mode\n");
				snd_soc_write(codec, ALC5631_PLL_CTRL,
						codec_master_pll_div[i].reg_val);
				schedule_timeout_uninterruptible(
						msecs_to_jiffies(20));
				snd_soc_update_bits(codec,
						ALC5631_GLOBAL_CLK_CTRL,
						ALC5631_SYSCLK_SOUR_SEL_MASK |
						ALC5631_PLLCLK_SOUR_SEL_MASK /*|
									       ALC5631_PLLCLK_PRE_DIV2*/, // 1/2 clk
						ALC5631_SYSCLK_SOUR_SEL_PLL |
						ALC5631_PLLCLK_SOUR_SEL_MCLK/*|
									      ALC5631_PLLCLK_PRE_DIV2*/); // 1/2 clk
					ret = 0;
				break;
			}
	} else {
		for (i = 0; i < ARRAY_SIZE(codec_slave_pll_div); i++)
			if (freq_in == codec_slave_pll_div[i].pll_in &&
					freq_out == codec_slave_pll_div[i].pll_out) {
#ifdef ALC5631_DEBUG
				printk("chnge Pll in slave mode \n");
#endif
				dev_info(codec->dev,
						"change PLL in slave mode\n");
				snd_soc_write(codec, ALC5631_PLL_CTRL,
						codec_slave_pll_div[i].reg_val);
				schedule_timeout_uninterruptible(
						msecs_to_jiffies(20));
				snd_soc_update_bits(codec,
						ALC5631_GLOBAL_CLK_CTRL,
						ALC5631_SYSCLK_SOUR_SEL_MASK |
						ALC5631_PLLCLK_SOUR_SEL_MASK,
						ALC5631_SYSCLK_SOUR_SEL_PLL |
						ALC5631_PLLCLK_SOUR_SEL_BCLK);
				ret = 0;
				break;
			}
	}

	return ret;
}

static int alc5631_set_bias_level(struct snd_soc_codec *codec,
		enum snd_soc_bias_level level)
{
	switch (level) {
		case SND_SOC_BIAS_ON:
#ifdef ALC5631_DEBUG
			printk(" %s snd_soc_bias_on \n", __func__);
#endif
		snd_soc_write(codec, ALC5631_SPK_MONO_HP_OUT_CTRL, 0xcc4c);	
		snd_soc_write(codec, ALC5631_GEN_PUR_CTRL_REG, 0x4e00);	
#if ALC5631_DEMO

			snd_soc_update_bits(codec, ALC5631_SPK_OUT_VOL,
					ALC5631_L_MUTE | ALC5631_R_MUTE,0);
			snd_soc_update_bits(codec, ALC5631_HP_OUT_VOL,
					ALC5631_L_MUTE | ALC5631_R_MUTE,0);
			//mute dac
			snd_soc_update_bits(codec, ALC5631_STEREO_DAC_VOL_1,
					ALC5631_L_MUTE | ALC5631_R_MUTE, 0);
#endif
			break;

		case SND_SOC_BIAS_PREPARE:
#ifdef ALC5631_DEBUG
			printk(" %s snd_soc_bias_prepare \n", __func__);
#endif
#if ALC5631_DEMO
			snd_soc_update_bits(codec, ALC5631_PWR_MANAG_ADD2,
					ALC5631_PWR_MICBIAS1_VOL | ALC5631_PWR_MICBIAS2_VOL,
					ALC5631_PWR_MICBIAS1_VOL | ALC5631_PWR_MICBIAS2_VOL);
#endif

			if(codec->dapm.bias_level == SND_SOC_BIAS_ON){
#ifdef ALC5631_DEBUG
				printk("%s snd soc prepare (SND_SOC_BIAS_ON) \n", __func__);
#endif


			}else if(codec->dapm.bias_level == SND_SOC_BIAS_STANDBY){
#ifdef ALC5631_DEBUG
				printk("%s snd soc prepare (SND_SOC_BIAS_STANDBY) \n", __func__);
#endif
				//	snd_soc_write(codec, 0x1a, 0xdfc0);	
				//	snd_soc_write(codec, 0x1c, 0xdfc0);	
				//	snd_soc_write(codec, 0x28, 0xd8d8);	
				//	snd_soc_write(codec, 0x2a, 0x6c00);	
			}

			break;

		case SND_SOC_BIAS_STANDBY:
#ifdef ALC5631_DEBUG
			printk("%s snd soc bias standby \n", __func__);
#endif

			/*	//mute dac
				snd_soc_update_bits(codec, ALC5631_STEREO_DAC_VOL_1,
				ALC5631_L_MUTE | ALC5631_R_MUTE,ALC5631_L_MUTE | ALC5631_R_MUTE);
			//mute apk
			snd_soc_update_bits(codec, ALC5631_SPK_OUT_VOL,
			ALC5631_L_MUTE | ALC5631_R_MUTE,ALC5631_L_MUTE | ALC5631_R_MUTE);
			snd_soc_update_bits(codec, ALC5631_HP_OUT_VOL,
			ALC5631_L_MUTE | ALC5631_R_MUTE,ALC5631_L_MUTE | ALC5631_R_MUTE);
			*/
			if (codec->dapm.bias_level == SND_SOC_BIAS_OFF) {
#ifdef ALC5631_DEBUG
				printk("%s snd soc bias standby (SND_SOC_BIAS_OFF)\n", __func__);
	printk("%s [3c] = 0x%x \n" ,__func__, snd_soc_read(codec, 0x3c));
#endif
				snd_soc_update_bits(codec, ALC5631_PWR_MANAG_ADD3,
						ALC5631_PWR_VREF | ALC5631_PWR_MAIN_BIAS,
						ALC5631_PWR_VREF | ALC5631_PWR_MAIN_BIAS);
				msleep(80);
				snd_soc_update_bits(codec, ALC5631_PWR_MANAG_ADD3,
						ALC5631_PWR_FAST_VREF_CTRL,
						ALC5631_PWR_FAST_VREF_CTRL);
				codec->cache_only = false;
				snd_soc_cache_sync(codec);

			}

			break;

		case SND_SOC_BIAS_OFF:
/*			
#ifdef ALC5631_DEBUG
			printk(" %s snd_soc_bias_off \n", __func__);
#endif
			snd_soc_write(codec, ALC5631_PWR_MANAG_ADD1, 0x0000);
			snd_soc_write(codec, ALC5631_PWR_MANAG_ADD2, 0x0000);
			snd_soc_write(codec, ALC5631_PWR_MANAG_ADD3, 0x0000);
			snd_soc_write(codec, ALC5631_PWR_MANAG_ADD4, 0x0000);
*/
#ifdef ALC5631_DEBUG
			printk(" %s snd_soc_bias_off \n", __func__);
#endif
			break;

		default:
			break;
	}
	codec->dapm.bias_level = level;

	return 0;
}

static int alc5631_probe(struct snd_soc_codec *codec)
{
	struct alc5631_priv *alc5631 = snd_soc_codec_get_drvdata(codec);
	unsigned int val;
	int ret;

#ifdef ALC5631_DEBUG
	printk("codec %s ....\n", __func__);
#endif
	
	pcodec = codec;
	
	codec->cache_sync = 1;
//johncao	
	codec->dapm.idle_bias_off = 1;
//	codec->dapm.idle_bias_off = 0;

	ret = snd_soc_codec_set_cache_io(codec,8 /*8*/, 16, SND_SOC_I2C);
	if (ret != 0) {
		dev_err(codec->dev, "Failed to set cache I/O: %d\n", ret);
		return ret;
	}
#ifdef ALC5631_DEBUG
	printk("codec %s ....set cache ok\n", __func__);
#endif

	val = alc5631_index_read(codec, ALC5631_ADDA_MIXER_INTL_REG3);
	if (val & 0x0002){
#ifdef ALC5631_DEBUG
		printk("codec %s .... version = 1\n", __func__);
#endif
		alc5631->codec_version = 1;
	}else{
#ifdef ALC5631_DEBUG
		printk("codec %s .... version = 0\n", __func__);
#endif
		alc5631->codec_version = 0;
	}

	alc5631_reset(codec);
	/* enable HP zero cross */
	snd_soc_write(codec, ALC5631_INT_ST_IRQ_CTRL_2, 0x8f18);
	/* power off ClassD auto Recovery */
	if (alc5631->codec_version)
		snd_soc_update_bits(codec, ALC5631_INT_ST_IRQ_CTRL_2,
				0x2000, 0x2000);
	else
		snd_soc_update_bits(codec, ALC5631_INT_ST_IRQ_CTRL_2,
				0x2000, 0);
	/* DMIC */
	//alc5631->dmic_used_flag = 1;
	if (alc5631->dmic_used_flag) {

#ifdef ALC5631_DEBUG
		printk(" %s dmic used flag = true \n",__func__ );
#endif
		//	snd_soc_update_bits(codec, ALC5631_INT_ST_IRQ_CTRL_2, 0x8000, 0x8000);
		snd_soc_update_bits(codec, ALC5631_GPIO_CTRL,
				ALC5631_GPIO_PIN_FUN_SEL_MASK |
				ALC5631_GPIO_DMIC_FUN_SEL_MASK,
				ALC5631_GPIO_PIN_FUN_SEL_GPIO_DIMC |
				ALC5631_GPIO_DMIC_FUN_SEL_DIMC);
		snd_soc_update_bits(codec, ALC5631_DIG_MIC_CTRL,
				ALC5631_DMIC_L_CH_LATCH_MASK |
				ALC5631_DMIC_R_CH_LATCH_MASK,
				ALC5631_DMIC_L_CH_LATCH_FALLING |
				ALC5631_DMIC_R_CH_LATCH_RISING);
	}

#if 1// ALC5631_DEMO
	alc5631_reg_init(codec);
#endif

	snd_soc_update_bits(codec, ALC5631_PWR_MANAG_ADD3,
			ALC5631_PWR_VREF | ALC5631_PWR_MAIN_BIAS,
			ALC5631_PWR_VREF | ALC5631_PWR_MAIN_BIAS);
			

	msleep(80);
	snd_soc_update_bits(codec, ALC5631_PWR_MANAG_ADD3,
			ALC5631_PWR_FAST_VREF_CTRL, ALC5631_PWR_FAST_VREF_CTRL);


//set 0.7*AVDD not 0.9*AVDD
        snd_soc_update_bits(codec, ALC5631_MIC_CTRL_2,
                                ALC5631_MICBIAS1_VOLT_CTRL_MASK,
                                ALC5631_MICBIAS1_VOLT_CTRL_75P);

//MICBIAS1 MICBIAS2 power on
        snd_soc_update_bits(codec, ALC5631_PWR_MANAG_ADD2,
                                ALC5631_PWR_MICBIAS1_VOL | ALC5631_PWR_MICBIAS2_VOL,
                                ALC5631_PWR_MICBIAS1_VOL | ALC5631_PWR_MICBIAS2_VOL);								
							
/*
//pll enable

        snd_soc_write(codec, ALC5631_PLL_CTRL,0x0622);											
		
//mclk enbale
        snd_soc_update_bits(codec, ALC5631_GLOBAL_CLK_CTRL,
                                ALC5631_SYSCLK_SOUR_SEL_MASK|ALC5631_PLLCLK_SOUR_SEL_MASK,
                                ALC5631_SYSCLK_SOUR_SEL_PLL|ALC5631_PLLCLK_SOUR_SEL_BCLK);		
*/


//johncao
	codec->dapm.bias_level = SND_SOC_BIAS_STANDBY;
//	codec->dapm.bias_level = SND_SOC_BIAS_OFF;
	alc5631->codec = codec;

	//snd_soc_add_controls(codec, alc5631_snd_controls,
	//		ARRAY_SIZE(alc5631_snd_controls));
	snd_soc_add_codec_controls(codec, alc5631_snd_controls,
			ARRAY_SIZE(alc5631_snd_controls));
	snd_soc_dapm_new_controls(&codec->dapm, alc5631_dapm_widgets,
			ARRAY_SIZE(alc5631_dapm_widgets));
	snd_soc_dapm_add_routes(&codec->dapm, alc5631_dapm_routes,
			ARRAY_SIZE(alc5631_dapm_routes));
	printk("MQ=%s==CCOSR reg2:%x\n",__FUNCTION__,  __raw_readl(ioremap(0x020c4060, 4)));

	return 0;
}

static int alc5631_remove(struct snd_soc_codec *codec)
{
#ifdef ALC5631_DEBUG
	printk("codec %s ....\n", __func__);
#endif
	alc5631_set_bias_level(codec, SND_SOC_BIAS_OFF);
	return 0;
}

void alc5631_reg_set(int if_play) 
{
#ifdef ALC5631_DEBUG
	printk("MQ===%s===if_play:%d REG-02:%x\n", __FUNCTION__, if_play, snd_soc_read(pcodec, ALC5631_SPK_OUT_VOL));
#endif
	if (if_play) {
		alc5631_index_write(pcodec, ALC5631_EQ_BW_HIP, 0x1bbc);
		alc5631_index_write(pcodec, ALC5631_EQ_PRE_VOL_CTRL, 0x8007);
		alc5631_index_write(pcodec, ALC5631_EQ_POST_VOL_CTRL, 0x000f);
		snd_soc_write(pcodec, ALC5631_EQ_CTRL, 0x4090);	
		snd_soc_write(pcodec, ALC5631_ALC_CTRL_1, 0x0a0f);	
		snd_soc_write(pcodec, ALC5631_STEREO_DAC_VOL_1, 0x0020);	
		snd_soc_write(pcodec, ALC5631_ALC_CTRL_3, 0x6000);	
		snd_soc_write(pcodec, ALC5631_SPK_OUT_VOL, 0x4848);	
	
	} else {
		snd_soc_write(pcodec, ALC5631_EQ_CTRL, 0x4000);	
		snd_soc_write(pcodec, ALC5631_ALC_CTRL_3, 0x2000);	
		snd_soc_write(pcodec, ALC5631_SPK_OUT_VOL, 0xc8c8);	
	
	}
}
EXPORT_SYMBOL(alc5631_reg_set);

//power down
void alc5631_power_down(void)
{
	alc5631_set_bias_level(pcodec, SND_SOC_BIAS_OFF);
}
EXPORT_SYMBOL(alc5631_power_down);

#ifdef CONFIG_PM
static int alc5631_suspend(struct snd_soc_codec *codec, pm_message_t state)
{
	alc5631_set_bias_level(codec, SND_SOC_BIAS_OFF);
	return 0;
}

static int alc5631_resume(struct snd_soc_codec *codec)
{
//	alc5631_set_bias_level(codec, SND_SOC_BIAS_STANDBY);
	return 0;
}
#else
#define alc5631_suspend NULL
#define alc5631_resume NULL
#endif

#define ALC5631_STEREO_RATES SNDRV_PCM_RATE_8000_96000
//#define ALC5631_STEREO_RATES (SNDRV_PCM_RATE_8000_48000 |\
//		SNDRV_PCM_RATE_88200 | SNDRV_PCM_RATE_96000)

#define ALC5631_FORMAT	(SNDRV_PCM_FMTBIT_S16_LE | \
	SNDRV_PCM_FMTBIT_S20_3LE | \
	SNDRV_PCM_FMTBIT_S24_LE | \
	SNDRV_PCM_FMTBIT_S8)
//#define ALC5631_FORMAT (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE |\
//		SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE)


	struct snd_soc_dai_ops alc5631_ops = {
		.hw_params = alc5631_hifi_pcm_params,
		.set_fmt = alc5631_hifi_codec_set_dai_fmt,
		.set_sysclk = alc5631_hifi_codec_set_dai_sysclk,
		.set_pll = alc5631_codec_set_dai_pll,
	};

struct snd_soc_dai_driver alc5631_dai = {
	//	{
	.name = "alc5631-hifi",
	//	.id = 1,
	.playback = {
		.stream_name = "HIFI Playback",
		.channels_min = 2,
		.channels_max = 2,
		.rate_min	= 8000,
		.rate_max	= 48000,
		.rates = ALC5631_STEREO_RATES,/*SNDRV_PCM_RATE_8000 | 
			SNDRV_PCM_RATE_11025 |
			SNDRV_PCM_RATE_16000 | 
			SNDRV_PCM_RATE_22050| 
			SNDRV_PCM_RATE_32000 |
			SNDRV_PCM_RATE_44100 |
			SNDRV_PCM_RATE_48000,
*/
		.formats = ALC5631_FORMAT,
	},
	.capture = {
		.stream_name = "HIFI Capture",
		.channels_min = 1,
		.channels_max = 2,
		.rate_min 	= 44100,
		.rate_max	= 48000,
		.rates = ALC5631_STEREO_RATES,
		.formats = ALC5631_FORMAT,
	},
	.ops = &alc5631_ops,
	//		.symmetric_rates = 1,
	//	},
};

static struct snd_soc_codec_driver soc_codec_dev_alc5631 = {
	.probe = alc5631_probe,
	.remove = alc5631_remove,
	.suspend = alc5631_suspend,
	.resume = alc5631_resume,
	.set_bias_level = alc5631_set_bias_level,
	.reg_cache_size = ALC5631_VENDOR_ID2 + 1,
	.reg_word_size = sizeof(u16),
	.reg_cache_default = alc5631_reg,
	.volatile_register = alc5631_volatile_register,
	.readable_register = alc5631_readable_register,
	.reg_cache_step = 1,
	/*.controls = alc5631_snd_controls,
	  .num_controls = ARRAY_SIZE(alc5631_snd_controls),
	  .dapm_widgets = alc5631_dapm_widgets,
	  .num_dapm_widgets = ARRAY_SIZE(alc5631_dapm_widgets),
	  .dapm_routes = alc5631_dapm_routes,
	  .num_dapm_routes = ARRAY_SIZE(alc5631_dapm_routes),*/
};

static const struct i2c_device_id alc5631_i2c_id[] = {
	{ "alc5631", 1 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, alc5631_i2c_id);

//static __devinit int alc5631_i2c_probe(struct i2c_client *i2c,
//		const struct i2c_device_id *id)
static	int alc5631_i2c_probe(struct i2c_client *i2c,
		const struct i2c_device_id *id)
{
	struct alc5631_priv *alc5631;
	int ret;

#ifdef ALC5631_DEBUG
	printk("codec %s ....\n", __func__);
#endif
	alc5631 = kzalloc(sizeof(struct alc5631_priv), GFP_KERNEL);
	if (NULL == alc5631)
		return -ENOMEM;

#ifdef ALC5631_DEBUG
	printk("codec %s .... kzalloc addr = 0x%x \n", __func__, i2c->addr);
#endif

	/*
	   while(1){
	   mdelay(20);
	   i2c_smbus_write_byte(i2c, 1);
	   }
	   */
	i2c_set_clientdata(i2c, alc5631);

	ret = snd_soc_register_codec(&i2c->dev, &soc_codec_dev_alc5631,
			&alc5631_dai, 1 /*ARRAY_SIZE(alc5631_dai)*/);
	if (ret < 0){
#ifdef ALC5631_DEBUG
		printk("codec %s soc register err~!!\n", __func__);
#endif
		kfree(alc5631);
	}

#ifdef ALC5631_DEBUG
	printk("codec %s .... ok \n", __func__);
#endif
	return ret;
}

//static __devexit int alc5631_i2c_remove(struct i2c_client *client)
static int alc5631_i2c_remove(struct i2c_client *client)
{
#ifdef ALC5631_DEBUG
	printk("codec %s i2c err~!!\n", __func__);
#endif
	snd_soc_unregister_codec(&client->dev);
	kfree(i2c_get_clientdata(client));
	return 0;
}

static const struct of_device_id alc5631_dt_ids[] = {
	{ .compatible = "realtek,alc5631", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, alc5631_dt_ids);

struct i2c_driver alc5631_i2c_driver = {
	.driver = {
		.name = "alc5631",
		.owner = THIS_MODULE,
		.of_match_table = alc5631_dt_ids,
	},
	.probe = alc5631_i2c_probe,
	//.remove   = __devexit_p(alc5631_i2c_remove),
	.remove   = alc5631_i2c_remove,
	.id_table = alc5631_i2c_id,
};

static int __init alc5631_modinit(void)
{
#ifdef ALC5631_DEBUG
	printk("codec %s ....\n", __func__);
#endif
	return i2c_add_driver(&alc5631_i2c_driver);
}
module_init(alc5631_modinit);

static void __exit alc5631_modexit(void)
{
	i2c_del_driver(&alc5631_i2c_driver);
}
module_exit(alc5631_modexit);

MODULE_DESCRIPTION("ASoC alc5631 driver");
MODULE_AUTHOR("flove <flove@realtek.com>");
MODULE_LICENSE("GPL");
//MODULE_ALIAS("platform:alc5631-codec");
