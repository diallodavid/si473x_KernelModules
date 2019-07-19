/*
 * sound/soc/codecs/si473x.c -- Codec driver for SI473X chips
 *
 * Copyright (C) 2012 Innovative Converged Devices(ICD)
 * Copyright (C) 2013 Andrey Smirnov
 * Copyright (C) 2016 rpi Receiver
 *
 * Author: Andrey Smirnov <andrew.smirnov@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <linux/regmap.h>
#include <sound/soc.h>
#include <sound/initval.h>

#include <linux/i2c.h>

#include "linux/mfd/si473x-core.h"

#define SI473X_SYSCLK_MCLK 0

#define SI473X_DIGITAL_IO_OUTPUT_WIDTH_MASK	0x03
#define SI473X_DIGITAL_IO_OUTPUT_FORMAT_MASK	(0x78)

enum si473x_daudio_formats {
	SI473X_DAUDIO_MODE_I2S		= (0x0 << 3),
	SI473X_DAUDIO_MODE_DSP_A	= (0x8 << 3),
	SI473X_DAUDIO_MODE_DSP_B	= (0xC << 3),
	SI473X_DAUDIO_MODE_LEFT_J	= (0x6 << 3),

	SI473X_DAUDIO_MODE_IB		= BIT(7),
};

enum si473x_pcm_format {
	SI473X_PCM_FORMAT_S16_LE	= 0,
	SI473X_PCM_FORMAT_S20_3LE	= 1,
	SI473X_PCM_FORMAT_S24_LE	= 2,
	SI473X_PCM_FORMAT_S8		= 3,
};

static const struct snd_kcontrol_new si473x_snd_controls[] = {
	SOC_SINGLE("Tuner Volume", SI473X_PROP_RX_VOLUME, 0, 0x3f, 0),
	SOC_SINGLE("Tuner Left Mute Switch", SI473X_PROP_RX_HARD_MUTE, 1, 1, 0),
	SOC_SINGLE("Tuner Right Mute Switch",
		SI473X_PROP_RX_HARD_MUTE, 0, 1, 0),
	SOC_SINGLE("Tuner Force mono Switch",
		SI473X_PROP_DIGITAL_OUTPUT_FORMAT, 2, 1, 0),
};

static const struct snd_soc_dapm_widget si473x_dapm_widgets[] = {
	SND_SOC_DAPM_OUTPUT("LOUT"),
	SND_SOC_DAPM_OUTPUT("ROUT"),
};

static const struct snd_soc_dapm_route si473x_dapm_routes[] = {
	{ "Capture", NULL, "LOUT" },
	{ "Capture", NULL, "ROUT" },
};

static int si473x_codec_set_dai_fmt(struct snd_soc_dai *codec_dai,
				    unsigned int fmt)
{
	struct si473x_core *core = i2c_mfd_cell_to_core(codec_dai->dev);
	int err;
	u16 format = 0;

	if ((fmt & SND_SOC_DAIFMT_MASTER_MASK) != SND_SOC_DAIFMT_CBS_CFS)
		return -EINVAL;

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_DSP_A:
		format |= SI473X_DAUDIO_MODE_DSP_A;
		break;
	case SND_SOC_DAIFMT_DSP_B:
		format |= SI473X_DAUDIO_MODE_DSP_B;
		break;
	case SND_SOC_DAIFMT_I2S:
		format |= SI473X_DAUDIO_MODE_I2S;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		format |= SI473X_DAUDIO_MODE_LEFT_J;
		break;
	default:
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		break;
	default:
		return -EINVAL;
	}

	si473x_core_lock(core);

	err = snd_soc_component_update_bits(codec_dai->component,
		SI473X_PROP_DIGITAL_OUTPUT_FORMAT,
		SI473X_DIGITAL_IO_OUTPUT_FORMAT_MASK,
		format);

	si473x_core_unlock(core);
	if (err < 0) {
		dev_err(codec_dai->component->dev, "Failed to set output format\n");
		return err;
	}

	return 0;
}

static int si473x_codec_hw_params(struct snd_pcm_substream *substream,
				  struct snd_pcm_hw_params *params,
				  struct snd_soc_dai *dai)
{
	struct si473x_core *core = i2c_mfd_cell_to_core(dai->dev);
	int rate, width, err;

	rate = params_rate(params);
	if (rate < 32000 || rate > 48000) {
		dev_err(dai->component->dev, "Rate: %d is not supported\n", rate);
		return -EINVAL;
	}

	switch (params_width(params)) {
	case 8:
		width = SI473X_PCM_FORMAT_S8;
		break;
	case 16:
		width = SI473X_PCM_FORMAT_S16_LE;
		break;
	case 20:
		width = SI473X_PCM_FORMAT_S20_3LE;
		break;
	case 24:
		width = SI473X_PCM_FORMAT_S24_LE;
		break;
	default:
		return -EINVAL;
	}

	si473x_core_lock(core);

	err = snd_soc_component_update_bits(dai->component, SI473X_PROP_DIGITAL_OUTPUT_FORMAT,
				  SI473X_DIGITAL_IO_OUTPUT_WIDTH_MASK, width);
	if (err < 0) {
		dev_err(dai->component->dev, "Failed to set output width\n");
		goto out;
	}

	err = snd_soc_component_write(dai->component, SI473X_PROP_DIGITAL_OUTPUT_SAMPLE_RATE,
			    rate);
	if (err < 0) {
		dev_err(dai->component->dev, "Failed to set sample rate\n");
		goto out;
	}

out:
	si473x_core_unlock(core);

	return err;
}

static int si473x_startup(struct snd_pcm_substream *substream,
				  struct snd_soc_dai *dai)
{
	struct si473x_core *core = i2c_mfd_cell_to_core(dai->dev);
	int err;

	si473x_core_lock(core);

	err = snd_soc_component_write(dai->component, SI473X_PROP_DIGITAL_OUTPUT_SAMPLE_RATE,
			    0);
	if (err < 0)
		dev_err(dai->component->dev, "Failed to disable digital audio output\n");
	core->power_up_parameters.mode = SI473X_MODE_DIGITAL_AUDIO_OUTPUTS;

	si473x_core_unlock(core);

	return err;
}

static void si473x_shutdown(struct snd_pcm_substream *substream,
				  struct snd_soc_dai *dai)
{
	struct si473x_core *core = i2c_mfd_cell_to_core(dai->dev);
	int err;

	si473x_core_lock(core);

	err = snd_soc_component_write(dai->component, SI473X_PROP_DIGITAL_OUTPUT_SAMPLE_RATE,
			    0);
	if (err < 0)
		dev_err(dai->component->dev, "Failed to disable digital audio output\n");
	/* turn audio outputs off */
	core->power_up_parameters.mode = 0;

	si473x_core_unlock(core);
}
static int si473x_set_sysclk(struct snd_soc_dai *codec_dai, int clk_id,
	unsigned int freq, int dir)
{
	struct si473x_core *core = i2c_mfd_cell_to_core(codec_dai->dev);
	struct snd_soc_component *component = codec_dai->component;
	int err;
	unsigned int refclk_freq, prescaler;

	if (dir == SND_SOC_CLOCK_OUT)
		return -EINVAL;

	if (clk_id != SI473X_SYSCLK_MCLK)
		return -EINVAL;

	if (freq == 0)
		return 0;

	si473x_core_lock(core);

	snd_soc_component_read(component, SI473X_PROP_REFCLK_FREQ, &refclk_freq);
	if (refclk_freq == -1)
		dev_err(component->dev, "Failed to read REFCLK_FREQ\n");

	prescaler = freq / SI473X_PROP_REFCLK_FREQ_MAX + 1;
	while (freq % prescaler) {
		prescaler++;
		if (prescaler > freq / SI473X_PROP_REFCLK_FREQ_MIN) {
			dev_err(component->dev, "Failed to calculate REFCLK_PRESCALE and REFCLK_FREQ\n");
			return -EINVAL;
		}
	}
	refclk_freq = freq / prescaler;
	si473x_core_unlock(core);

	if (((freq / refclk_freq) < SI473X_PROP_REFCLK_PRESCALE_MIN) ||
		((freq / refclk_freq) > SI473X_PROP_REFCLK_PRESCALE_MAX)) {
		return -EINVAL;
	}

	si473x_core_lock(core);

	err = snd_soc_component_write(component, SI473X_PROP_REFCLK_FREQ,
			    refclk_freq);
	if (err < 0)
		dev_err(component->dev, "Failed to set REFCLK_FREQ\n");
	err = snd_soc_component_write(component, SI473X_PROP_REFCLK_PRESCALE,
			    freq / refclk_freq);
	if (err < 0)
		dev_err(component->dev, "Failed to set REFCLK_PRESCALE\n");

	si473x_core_unlock(core);

	return err;
}

static struct snd_soc_dai_ops si473x_dai_ops = {
	.hw_params	= si473x_codec_hw_params,
	.set_sysclk	= si473x_set_sysclk,
	.set_fmt	= si473x_codec_set_dai_fmt,
	.startup	= si473x_startup,
	.shutdown	= si473x_shutdown,
};

static struct snd_soc_dai_driver si473x_dai = {
	.name		= "si473x-component",
	.capture	= {
		.stream_name	= "Capture",
		.channels_min	= 2,
		.channels_max	= 2,

		.rates = SNDRV_PCM_RATE_32000 |
		SNDRV_PCM_RATE_44100 |
		SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S8 |
		SNDRV_PCM_FMTBIT_S16_LE |
		SNDRV_PCM_FMTBIT_S20_3LE |
		SNDRV_PCM_FMTBIT_S24_LE
	},
	.ops		= &si473x_dai_ops,
};

static int si473x_probe(struct snd_soc_component *component)
{
	snd_soc_component_init_regmap(component,
				dev_get_regmap(component->dev->parent, NULL));

	return 0;
}

static struct snd_soc_component_driver soc_component_dev_si473x = {
	.probe			= si473x_probe,
	.dapm_widgets = si473x_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(si473x_dapm_widgets),
	.dapm_routes = si473x_dapm_routes,
	.num_dapm_routes = ARRAY_SIZE(si473x_dapm_routes),
	.idle_bias_on		= 1,
	.use_pmdown_time	= 1,
	.endianness		= 1,
	.non_legacy_dai_naming	= 1,
};

static int si473x_platform_probe(struct platform_device *pdev)
{
	return devm_snd_soc_register_component(&pdev->dev, &soc_component_dev_si473x,
				      &si473x_dai, 1);
}

MODULE_ALIAS("platform:si473x-codec");

static struct platform_driver si473x_platform_driver = {
	.driver		= {
		.name	= "si473x-codec",
	},
	.probe		= si473x_platform_probe,
};
module_platform_driver(si473x_platform_driver);

MODULE_AUTHOR("rpi Receiver <rpi-receiver@htl-steyr.ac.at>");
MODULE_DESCRIPTION("ASoC Si473x codec driver");
MODULE_LICENSE("GPL");
