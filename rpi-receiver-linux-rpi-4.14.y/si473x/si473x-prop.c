/*
 * drivers/mfd/si473x-prop.c -- Subroutines to access
 * properties of si473x chips
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
 */
#include <linux/module.h>

#include "si473x-core.h"

static const struct regmap_range si4731_d62_fm_reg_ranges[] = {
		regmap_reg_range(SI473X_PROP_GPO_IEN, SI473X_PROP_GPO_IEN),
		regmap_reg_range(SI473X_PROP_DIGITAL_OUTPUT_FORMAT,
			SI473X_PROP_DIGITAL_OUTPUT_FORMAT),
		regmap_reg_range(SI473X_PROP_DIGITAL_OUTPUT_SAMPLE_RATE,
			SI473X_PROP_DIGITAL_OUTPUT_SAMPLE_RATE),
		regmap_reg_range(SI473X_PROP_REFCLK_FREQ,
			SI473X_PROP_REFCLK_PRESCALE),
		regmap_reg_range(SI473X_PROP_FM_DEEMPHASIS,
			SI473X_PROP_FM_DEEMPHASIS),
		regmap_reg_range(SI473X_PROP_FM_CHANNEL_FILTER,
			SI473X_PROP_FM_CHANNEL_FILTER),
		regmap_reg_range(SI473X_PROP_FM_MAX_TUNE_ERROR,
			SI473X_PROP_FM_MAX_TUNE_ERROR),
		regmap_reg_range(SI473X_PROP_FM_RSQ_INT_SOURCE,
			SI473X_PROP_FM_RSQ_RSSI_LO_THRESHOLD),
		regmap_reg_range(SI473X_PROP_FM_RSQ_MULTIPATH_HI_THRESHOLD,
			SI473X_PROP_FM_RSQ_BLEND_THRESHOLD),
		regmap_reg_range(SI473X_PROP_FM_SOFT_MUTE_SLOPE,
			SI473X_PROP_FM_SOFT_MUTE_ATTACK_RATE),
		regmap_reg_range(SI473X_PROP_FM_SEEK_BAND_BOTTOM,
			SI473X_PROP_FM_SEEK_TUNE_RSSI_TRESHOLD),
		regmap_reg_range(SI473X_PROP_FM_RDS_INT_SOURCE,
			SI473X_PROP_FM_RDS_CONFIDENCE),
		regmap_reg_range(SI473X_PROP_FM_BLEND_RSSI_STEREO_THRESHOLD,
			SI473X_PROP_FM_BLEND_MULTIPATH_RELEASE_RATE),
		regmap_reg_range(SI473X_PROP_FM_HICUT_SNR_HIGH_THRESHOLD,
			SI473X_PROP_FM_HICUT_CUTOFF_FREQUENCY),
		regmap_reg_range(SI473X_PROP_RX_VOLUME,
			SI473X_PROP_RX_HARD_MUTE),
};

static const struct regmap_access_table si4731_d62_fm_write_ranges_table = {
		.yes_ranges     = si4731_d62_fm_reg_ranges,
		.n_yes_ranges   = ARRAY_SIZE(si4731_d62_fm_reg_ranges),
};

static const struct reg_default si4731_d62_fm_reg_defaults[] = {
		{ SI473X_PROP_GPO_IEN, 0x0000 },
		{ SI473X_PROP_DIGITAL_OUTPUT_FORMAT, 0x0000 },
		{ SI473X_PROP_DIGITAL_OUTPUT_SAMPLE_RATE, 0x0000 },
		/* digital audio output disabled */
		{ SI473X_PROP_REFCLK_FREQ, 0x8000 }, /* 32768Hz */
		{ SI473X_PROP_REFCLK_PRESCALE, 0x0001 },
		{ SI473X_PROP_FM_DEEMPHASIS, 0x0002 },
		{ SI473X_PROP_FM_CHANNEL_FILTER, 0x0001 },
		{ SI473X_PROP_FM_MAX_TUNE_ERROR, 0x0014 },
		{ SI473X_PROP_FM_RSQ_INT_SOURCE, 0x0000 },
		{ SI473X_PROP_FM_RSQ_SNR_HI_THRESHOLD, 0x007F },
		{ SI473X_PROP_FM_RSQ_SNR_LO_THRESHOLD, 0x0000 },
		{ SI473X_PROP_FM_RSQ_RSSI_HI_THRESHOLD, 0x007F },
		{ SI473X_PROP_FM_RSQ_RSSI_LO_THRESHOLD, 0x0000 },
		{ SI473X_PROP_FM_RSQ_MULTIPATH_HI_THRESHOLD, 0x007F },
		{ SI473X_PROP_FM_RSQ_MULTIPATH_LO_THRESHOLD, 0x0000 },
		{ SI473X_PROP_FM_RSQ_BLEND_THRESHOLD, 0x0081 },
		{ SI473X_PROP_FM_SOFT_MUTE_SLOPE, 0x0002 },
		{ SI473X_PROP_FM_SOFT_MUTE_MAX_ATTENUATION, 0x0010 },
		{ SI473X_PROP_FM_SOFT_MUTE_SNR_THRESHOLD, 0x0004 },
		{ SI473X_PROP_FM_SOFT_MUTE_RELEASE_RATE, 0x2000 },
		{ SI473X_PROP_FM_SOFT_MUTE_ATTACK_RATE, 0x2000 },
		{ SI473X_PROP_FM_SEEK_BAND_BOTTOM, 0x222E },
		{ SI473X_PROP_FM_SEEK_BAND_TOP, 0x2A26 },
		{ SI473X_PROP_FM_SEEK_FREQ_SPACING, 0x000A },
		{ SI473X_PROP_FM_SEEK_TUNE_SNR_THRESHOLD, 0x0003 },
		{ SI473X_PROP_FM_SEEK_TUNE_RSSI_TRESHOLD, 0x0014 },
		{ SI473X_PROP_FM_RDS_INT_SOURCE, 0x0000 },
		{ SI473X_PROP_FM_RDS_INT_FIFO_COUNT, 0x0000 },
		{ SI473X_PROP_FM_RDS_CONFIG, 0x0000 },
		{ SI473X_PROP_FM_RDS_CONFIDENCE, 0x1111 },
		{ SI473X_PROP_FM_BLEND_RSSI_STEREO_THRESHOLD, 0x0031 },
		{ SI473X_PROP_FM_BLEND_RSSI_MONO_THRESHOLD, 0x001E },
		{ SI473X_PROP_FM_BLEND_RSSI_ATTACK_RATE, 0x0FA0 },
		{ SI473X_PROP_FM_BLEND_RSSI_RELEASE_RATE, 0x0190 },
		{ SI473X_PROP_FM_BLEND_SNR_STEREO_THRESHOLD, 0x001B },
		{ SI473X_PROP_FM_BLEND_SNR_MONO_THRESHOLD, 0x000E },
		{ SI473X_PROP_FM_BLEND_SNR_ATTACK_RATE, 0x0FA0 },
		{ SI473X_PROP_FM_BLEND_SNR_RELEASE_RATE, 0x0190 },
		{ SI473X_PROP_FM_BLEND_MULTIPATH_STEREO_THRESHOLD, 0x0014 },
		{ SI473X_PROP_FM_BLEND_MULTIPATH_MONO_THRESHOLD, 0x003C },
		{ SI473X_PROP_FM_BLEND_MULTIPATH_ATTACK_RATE, 0x0FA0 },
		{ SI473X_PROP_FM_BLEND_MULTIPATH_RELEASE_RATE, 0x0028 },
		{ SI473X_PROP_FM_HICUT_SNR_HIGH_THRESHOLD, 0x0018 },
		{ SI473X_PROP_FM_HICUT_SNR_LOW_THRESHOLD, 0x000F },
		{ SI473X_PROP_FM_HICUT_ATTACK_RATE, 0x4E20 },
		{ SI473X_PROP_FM_HICUT_RELEASE_RATE, 0x0014 },
		{ SI473X_PROP_FM_HICUT_MULTIPATH_TRIGGER_THRESHOLD, 0x0014 },
		{ SI473X_PROP_FM_HICUT_MULTIPATH_END_THRESHOLD, 0x003C },
		{ SI473X_PROP_FM_HICUT_CUTOFF_FREQUENCY, 0x0000 },
		{ SI473X_PROP_RX_VOLUME, 0x003F },
		{ SI473X_PROP_RX_HARD_MUTE, 0x0000 },
};

static const struct regmap_range si4731_d62_am_reg_ranges[] = {
		regmap_reg_range(SI473X_PROP_GPO_IEN, SI473X_PROP_GPO_IEN),
		regmap_reg_range(SI473X_PROP_DIGITAL_OUTPUT_FORMAT,
			SI473X_PROP_DIGITAL_OUTPUT_FORMAT),
		regmap_reg_range(SI473X_PROP_DIGITAL_OUTPUT_SAMPLE_RATE,
			SI473X_PROP_DIGITAL_OUTPUT_SAMPLE_RATE),
		regmap_reg_range(SI473X_PROP_REFCLK_FREQ,
			SI473X_PROP_REFCLK_PRESCALE),
		regmap_reg_range(SI473X_PROP_AM_DEEMPHASIS,
			SI473X_PROP_AM_DEEMPHASIS),
		regmap_reg_range(SI473X_PROP_AM_CHANNEL_FILTER,
			SI473X_PROP_AM_AUTOMATIC_VOLUME_CONTROL_MAX_GAIN),
		regmap_reg_range(SI473X_PROP_AM_RSQ_INTERRUPTS,
			SI473X_PROP_AM_RSQ_RSSI_LOW_THRESHOLD),
		regmap_reg_range(SI473X_PROP_AM_SOFT_MUTE_RATE,
			SI473X_PROP_AM_SOFT_MUTE_SNR_THRESHOLD),
		regmap_reg_range(SI473X_PROP_AM_SEEK_BAND_BOTTOM,
			SI473X_PROP_AM_SEEK_RSSI_THRESHOLD),
		regmap_reg_range(SI473X_PROP_RX_VOLUME,
			SI473X_PROP_RX_HARD_MUTE),
};

static const struct regmap_access_table si4731_d62_am_write_ranges_table = {
		.yes_ranges     = si4731_d62_am_reg_ranges,
		.n_yes_ranges   = ARRAY_SIZE(si4731_d62_am_reg_ranges),
};

static const struct reg_default si4731_d62_am_reg_defaults[] = {
		{ SI473X_PROP_GPO_IEN, 0x0000 },
		{ SI473X_PROP_DIGITAL_OUTPUT_FORMAT, 0x0000 },
		{ SI473X_PROP_DIGITAL_OUTPUT_SAMPLE_RATE, 0x0000 },
		/* digital audio output disabled */
		{ SI473X_PROP_REFCLK_FREQ, 0x8000 }, /* 32768Hz */
		{ SI473X_PROP_REFCLK_PRESCALE, 0x0001 },
		{ SI473X_PROP_AM_DEEMPHASIS, 0x0000 },
		{ SI473X_PROP_AM_CHANNEL_FILTER, 0x0003 },
		{ SI473X_PROP_AM_AUTOMATIC_VOLUME_CONTROL_MAX_GAIN, 0x1543 },
		{ SI473X_PROP_AM_RSQ_INTERRUPTS, 0x0000 },
		{ SI473X_PROP_AM_RSQ_SNR_HIGH_THRESHOLD, 0x007F },
		{ SI473X_PROP_AM_RSQ_SNR_LOW_THRESHOLD, 0x0000 },
		{ SI473X_PROP_AM_RSQ_RSSI_HIGH_THRESHOLD, 0x007F },
		{ SI473X_PROP_AM_RSQ_RSSI_LOW_THRESHOLD, 0x0000 },
		{ SI473X_PROP_AM_SOFT_MUTE_RATE, 0x0040 },
		{ SI473X_PROP_AM_SOFT_MUTE_SLOPE, 0x0001 },
		{ SI473X_PROP_AM_SOFT_MUTE_MAX_ATTENUATION, 0x0008 },
		{ SI473X_PROP_AM_SOFT_MUTE_SNR_THRESHOLD, 0x0008 },
		{ SI473X_PROP_AM_SEEK_BAND_BOTTOM, 0x0208 },
		{ SI473X_PROP_AM_SEEK_BAND_TOP, 0x06AE },
		{ SI473X_PROP_AM_SEEK_FREQ_SPACING, 0x000A },
		{ SI473X_PROP_AM_SEEK_SNR_THRESHOLD, 0x0005 },
		{ SI473X_PROP_AM_SEEK_RSSI_THRESHOLD, 0x0019 },
		{ SI473X_PROP_RX_VOLUME, 0x003F },
		{ SI473X_PROP_RX_HARD_MUTE, 0x0000 },
};

static int si473x_core_regmap_write(void *context, unsigned int reg,
			unsigned int val)
{
	return si473x_core_cmd_set_property(context, reg, val);
}

static int si473x_core_regmap_read(void *context, unsigned int reg,
			unsigned int *val)
{
	struct si473x_core *core = context;
	int err;

	err = si473x_core_cmd_get_property(core, reg);
	if (err < 0)
		return err;

	*val = err;

	return 0;
}

static const struct regmap_config si4731_d62_fm_regmap_config = {
		.name = "si4731_d62_fm",
		.reg_bits = 16,
		.val_bits = 16,
		.max_register = SI473X_PROP_RX_HARD_MUTE + 1,
		.wr_table = &si4731_d62_fm_write_ranges_table,
		.reg_read = si473x_core_regmap_read,
		.reg_write = si473x_core_regmap_write,
		.reg_defaults = si4731_d62_fm_reg_defaults,
		.num_reg_defaults = ARRAY_SIZE(si4731_d62_fm_reg_defaults),
		.cache_type = REGCACHE_RBTREE,
};

static const struct regmap_config si4731_d62_am_regmap_config = {
		.name = "si4731_d62_am",
		.reg_bits = 16,
		.val_bits = 16,
		.max_register = SI473X_PROP_RX_HARD_MUTE + 1,
		.wr_table = &si4731_d62_am_write_ranges_table,
		.reg_read = si473x_core_regmap_read,
		.reg_write = si473x_core_regmap_write,
		.reg_defaults = si4731_d62_am_reg_defaults,
		.num_reg_defaults = ARRAY_SIZE(si4731_d62_am_reg_defaults),
		.cache_type = REGCACHE_RBTREE,
};

struct regmap *devm_regmap_init_si473x(struct si473x_core *core)
{
	struct regmap *regmap;
	u16 fm_seek_tune_rssi_treshold, fm_seek_tune_snr_threshold,
		fm_max_tune_error;
	u16 am_seek_rssi_threshold, am_seek_snr_threshold;

	switch (core->power_up_parameters.func) {
	default:
	case SI473X_FUNC_FM_RECEIVER:
		regmap = devm_regmap_init(&core->client->dev, NULL,
					core, &si4731_d62_fm_regmap_config);
		if (IS_ERR(regmap))
			return regmap;
		regcache_mark_dirty(regmap);
		regcache_cache_only(regmap, true);
		if (!of_property_read_u16((&core->client->dev)->of_node,
				"fm_seek_tune_rssi_treshold",
				&fm_seek_tune_rssi_treshold))
			if (regmap_write(regmap,
					SI473X_PROP_FM_SEEK_TUNE_RSSI_TRESHOLD,
					fm_seek_tune_rssi_treshold) < 0)
				dev_err(&core->client->dev, "Failed to set fm_seek_tune_rssi_treshold");

		if (!of_property_read_u16((&core->client->dev)->of_node,
				"fm_seek_tune_snr_threshold",
				&fm_seek_tune_snr_threshold))
			if (regmap_write(regmap,
					SI473X_PROP_FM_SEEK_TUNE_SNR_THRESHOLD,
					fm_seek_tune_snr_threshold) < 0)
				dev_err(&core->client->dev, "Failed to set fm_seek_tune_snr_threshold");

		if (!of_property_read_u16((&core->client->dev)->of_node,
				"fm_max_tune_error",
				&fm_max_tune_error))
			if (regmap_write(regmap, SI473X_PROP_FM_MAX_TUNE_ERROR,
					fm_max_tune_error) < 0)
				dev_err(&core->client->dev, "Failed to set fm_max_tune_error");
		return regmap;

	case SI473X_FUNC_AM_RECEIVER:
		regmap = devm_regmap_init(&core->client->dev, NULL,
					core, &si4731_d62_am_regmap_config);
		if (IS_ERR(regmap))
			return regmap;
		regcache_mark_dirty(regmap);
		regcache_cache_only(regmap, true);
		if (!of_property_read_u16((&core->client->dev)->of_node,
				"am_seek_rssi_threshold",
				&am_seek_rssi_threshold))
			if (regmap_write(regmap,
					SI473X_PROP_AM_SEEK_RSSI_THRESHOLD,
					am_seek_rssi_threshold) < 0)
				dev_err(&core->client->dev, "Failed to set am_seek_rssi_threshold");

		if (!of_property_read_u16((&core->client->dev)->of_node,
				"am_seek_snr_threshold",
				&am_seek_snr_threshold))
			if (regmap_write(regmap,
					SI473X_PROP_AM_SEEK_SNR_THRESHOLD,
					am_seek_snr_threshold) < 0)
				dev_err(&core->client->dev, "Failed to set am_seek_snr_threshold");
		return regmap;
	}
}
EXPORT_SYMBOL_GPL(devm_regmap_init_si473x);
MODULE_AUTHOR("rpi Receiver <rpi-receiver@htl-steyr.ac.at>");
MODULE_DESCRIPTION("Si473x AM/FM device driver");
MODULE_LICENSE("GPL");
