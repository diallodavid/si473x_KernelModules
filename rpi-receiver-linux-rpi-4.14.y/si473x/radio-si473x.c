/*
 * drivers/media/radio/radio-si473x.c -- V4L2 driver for SI473X chips
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
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/atomic.h>
#include <linux/mutex.h>
#include <media/v4l2-common.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-event.h>
#include <media/v4l2-device.h>

#include "si473x-core.h"

#define FREQ_MUL (10000000 / 625)

#define DRIVER_NAME "si473x-radio"
#define DRIVER_CARD "SI473x AM/FM Receiver"

enum si473x_freq_bands {
	SI473X_BAND_FM,
	SI473X_BAND_AM,
};

static const struct v4l2_frequency_band si473x_bands[] = {
	[SI473X_BAND_FM] = {
		.type		= V4L2_TUNER_RADIO,
		.index		= SI473X_BAND_FM,
		.capability	= V4L2_TUNER_CAP_LOW
		| V4L2_TUNER_CAP_STEREO
		| V4L2_TUNER_CAP_RDS
		| V4L2_TUNER_CAP_RDS_BLOCK_IO
		| V4L2_TUNER_CAP_FREQ_BANDS,
		.rangelow	=  64 * FREQ_MUL,
		.rangehigh	= 108 * FREQ_MUL,
		.modulation	= V4L2_BAND_MODULATION_FM,
	},
	[SI473X_BAND_AM] = {
		.type		= V4L2_TUNER_RADIO,
		.index		= SI473X_BAND_AM,
		.capability	= V4L2_TUNER_CAP_LOW
		| V4L2_TUNER_CAP_FREQ_BANDS,
		.rangelow	= 0.52 * FREQ_MUL,
		.rangehigh	= 30 * FREQ_MUL,
		.modulation	= V4L2_BAND_MODULATION_AM,
	},
};

static inline bool si473x_radio_freq_is_inside_of_the_band(u32 freq, int band)
{
	return freq >= si473x_bands[band].rangelow &&
		freq <= si473x_bands[band].rangehigh;
}

static inline bool si473x_radio_range_is_inside_of_the_band(u32 low, u32 high,
							    int band)
{
	return low  >= si473x_bands[band].rangelow &&
		high <= si473x_bands[band].rangehigh;
}

static int si473x_radio_s_ctrl(struct v4l2_ctrl *ctrl);
static int si473x_radio_g_volatile_ctrl(struct v4l2_ctrl *ctrl);

static const struct v4l2_ctrl_ops si473x_ctrl_ops = {
	.g_volatile_ctrl	= si473x_radio_g_volatile_ctrl,
	.s_ctrl			= si473x_radio_s_ctrl,
};

struct si473x_radio;

/**
 * struct si473x_radio_ops - vtable of tuner functions
 *
 * This table holds pointers to functions implementing particular
 * operations depending on the mode in which the tuner chip was
 * configured to start in. If the function is not supported
 * corresponding element is set to #NULL.
 *
 * @tune_freq: Tune chip to a specific frequency
 * @seek_start: Start station seeking
 * @rsq_status: Get Received Signal Quality(RSQ) status
 * @rds_blckcnt: Get received RDS blocks count
 * @acf_status: Get the status of Automatically Controlled
 * Features(ACF)
 * @agc_status: Get Automatic Gain Control(AGC) status
 */
struct si473x_radio_ops {
	int (*tune_freq)(struct si473x_core *core,
		struct si473x_tune_freq_args *args);
	int (*seek_start)(struct si473x_core *core, bool seekup, bool wrap);
	int (*rsq_status)(struct si473x_core *core,
		struct si473x_rsq_status_args *args,
		struct si473x_rsq_status_report *report);
	int (*agc_status)(struct si473x_core *core,
			  struct si473x_agc_status_report *report);
	int seek_band_bottom;
	int seek_band_top;
	int seek_freq_spacing;
};

/**
 * struct si473x_radio - radio device
 *
 * @core: Pointer to underlying core device
 * @videodev: Pointer to video device created by V4L2 subsystem
 * @ops: Vtable of functions. See struct si473x_radio_ops for details
 * @kref: Reference counter
 * @core_lock: An r/w semaphore to brebvent the deletion of underlying
 * core structure is the radio device is being used
 */
struct si473x_radio {
	struct v4l2_device v4l2dev;
	struct video_device videodev;
	struct v4l2_ctrl_handler ctrl_handler;

	struct si473x_core  *core;
	/* This field should not be accesses unless core lock is held */
	const struct si473x_radio_ops *ops;

	u32 audmode;
};

static inline struct si473x_radio *
v4l2_dev_to_radio(struct v4l2_device *d)
{
	return container_of(d, struct si473x_radio, v4l2dev);
}

static inline struct si473x_radio *
v4l2_ctrl_handler_to_radio(struct v4l2_ctrl_handler *d)
{
	return container_of(d, struct si473x_radio, ctrl_handler);
}

/* si473x_vidioc_querycap - query device capabilities */
static int si473x_radio_querycap(struct file *file, void *priv,
				 struct v4l2_capability *capability)
{
	struct si473x_radio *radio = video_drvdata(file);

	strlcpy(capability->driver, radio->v4l2dev.name,
		sizeof(capability->driver));
	strlcpy(capability->card,   DRIVER_CARD, sizeof(capability->card));
	snprintf(capability->bus_info, sizeof(capability->bus_info),
		 "platform:%s", radio->v4l2dev.name);

	capability->device_caps = V4L2_CAP_TUNER
		| V4L2_CAP_RADIO
		| V4L2_CAP_HW_FREQ_SEEK
		| V4L2_CAP_RDS_CAPTURE
		| V4L2_CAP_READWRITE;

	capability->capabilities = capability->device_caps
		| V4L2_CAP_DEVICE_CAPS;
	return 0;
}

static int si473x_radio_enum_freq_bands(struct file *file, void *priv,
					struct v4l2_frequency_band *band)
{
	int err;
	struct si473x_radio *radio = video_drvdata(file);

	if (band->tuner != 0)
		return -EINVAL;

	switch (radio->core->chip_id) {
		/* AM/FM tuners -- all bands are supported */
	case SI473X_CHIP_SI4731:
		if (band->index < ARRAY_SIZE(si473x_bands)) {
			*band = si473x_bands[band->index];
			err = 0;
		} else {
			err = -EINVAL;
		}
		break;
		/* FM companion tuner chips -- only FM bands are supported */
	default:
		err = -EINVAL;
	}

	return err;
}

/* vidioc_g_tuner - get tuner attributes */
static int si473x_radio_g_tuner(struct file *file, void *priv,
				struct v4l2_tuner *tuner)
{
	int err;
	struct si473x_rsq_status_report report;
	struct si473x_rds_status_report rds_report;
	struct si473x_radio *radio = video_drvdata(file);

	struct si473x_rsq_status_args args = {
		.stcack		= false,
	};

	if (tuner->index != 0)
		return -EINVAL;

	tuner->type       = V4L2_TUNER_RADIO;
	tuner->capability = V4L2_TUNER_CAP_LOW
		/* Measure frequencies in multiples of 62.5 Hz */
		| V4L2_TUNER_CAP_STEREO
		| V4L2_TUNER_CAP_HWSEEK_BOUNDED
		| V4L2_TUNER_CAP_HWSEEK_WRAP
		| V4L2_TUNER_CAP_HWSEEK_PROG_LIM;

	si473x_core_lock(radio->core);

	if (si473x_core_has_am(radio->core)) {
		strlcpy(tuner->name, "AM/FM", sizeof(tuner->name));
		tuner->rxsubchans = V4L2_TUNER_SUB_MONO;
		tuner->capability |= V4L2_TUNER_CAP_RDS
			| V4L2_TUNER_CAP_RDS_BLOCK_IO
			| V4L2_TUNER_CAP_FREQ_BANDS;

		tuner->rangelow = si473x_bands[SI473X_BAND_AM].rangelow;
	} else {
		strlcpy(tuner->name, "FM", sizeof(tuner->name));
		tuner->rxsubchans = V4L2_TUNER_SUB_MONO;
		tuner->capability |= V4L2_TUNER_CAP_RDS
			| V4L2_TUNER_CAP_RDS_BLOCK_IO
			| V4L2_TUNER_CAP_FREQ_BANDS;
		tuner->rangelow = si473x_bands[SI473X_BAND_FM].rangelow;
	}

	tuner->audmode = radio->audmode;

	tuner->afc = 1;
	tuner->rangehigh = si473x_bands[SI473X_BAND_FM].rangehigh;

	err = radio->ops->rsq_status(radio->core,
				     &args, &report);
	if (err < 0) {
		tuner->signal = 0;
	} else {
		/*
		 * tuner->signal value range: 0x0000 .. 0xFFFF,
		 * report.rssi: 0 .. 127dBuV
		 */
		tuner->signal = (report.rssi) << 9;
		/*
		 * Indicates stereo pilot presence
		 */
		if (report.pilot)
			tuner->rxsubchans |= V4L2_TUNER_SUB_STEREO;
		err = si473x_core_cmd_fm_rds_status(radio->core,
				true, false, false, &rds_report);
		if (err < 0)
			return err;
		if (rds_report.rdssync)
			tuner->rxsubchans |= V4L2_TUNER_SUB_RDS;
	}
	si473x_core_unlock(radio->core);

	return err;
}

/* vidioc_s_tuner - set tuner attributes */
static int si473x_radio_s_tuner(struct file *file, void *priv,
				const struct v4l2_tuner *tuner)
{
	struct si473x_radio *radio = video_drvdata(file);
	int err;

	if (tuner->index != 0)
		return -EINVAL;

	if (tuner->audmode == V4L2_TUNER_MODE_MONO ||
	    tuner->audmode == V4L2_TUNER_MODE_STEREO)
		radio->audmode = tuner->audmode;
	else
		radio->audmode = V4L2_TUNER_MODE_STEREO;

	if (radio->audmode == V4L2_TUNER_MODE_MONO) {
		si473x_core_lock(radio->core);
		err = regmap_write(radio->core->regmap,
				   SI473X_PROP_FM_BLEND_MONO_THRESHOLD, 0x7F);
		if (err)
			dev_warn(&radio->videodev.dev,
				"Error forcing MONO\n");
		si473x_core_unlock(radio->core);
		}
	if (radio->audmode == V4L2_TUNER_MODE_STEREO) {
		si473x_core_lock(radio->core);
		err = regmap_write(radio->core->regmap,
				   SI473X_PROP_FM_BLEND_MONO_THRESHOLD, 0x1E);
		if (err)
			dev_warn(&radio->videodev.dev,
				"Error setting mono blend to default\n");
		si473x_core_unlock(radio->core);
	}

	return 0;
}

static int si473x_radio_init_vtable(struct si473x_radio *radio,
				    enum si473x_func func)
{
	static const struct si473x_radio_ops fm_ops = {
		.tune_freq			= si473x_core_cmd_fm_tune_freq,
		.seek_start			= si473x_core_cmd_fm_seek_start,
		.rsq_status			= si473x_core_cmd_fm_rsq_status,
		.agc_status			= si473x_core_cmd_agc_status,
		.seek_band_bottom	= SI473X_PROP_FM_SEEK_BAND_BOTTOM,
		.seek_band_top		= SI473X_PROP_FM_SEEK_BAND_TOP,
		.seek_freq_spacing	= SI473X_PROP_FM_SEEK_FREQ_SPACING,
	};

	static const struct si473x_radio_ops am_ops = {
		.tune_freq		= si473x_core_cmd_am_tune_freq,
		.seek_start		= si473x_core_cmd_am_seek_start,
		.rsq_status		= si473x_core_cmd_am_rsq_status,
		.agc_status		= NULL,
		.seek_band_bottom	= SI473X_PROP_AM_SEEK_BAND_BOTTOM,
		.seek_band_top		= SI473X_PROP_AM_SEEK_BAND_TOP,
		.seek_freq_spacing	= SI473X_PROP_AM_SEEK_FREQ_SPACING,
	};

	switch (func) {
	case SI473X_FUNC_FM_RECEIVER:
		radio->ops = &fm_ops;
		return 0;

	case SI473X_FUNC_AM_RECEIVER:
		radio->ops = &am_ops;
		return 0;
	default:
		WARN(1, "Unexpected tuner function value\n");
		return -EINVAL;
	}
}

static int si473x_radio_do_post_powerup_init(struct si473x_radio *radio,
		enum si473x_func func)
{
	int err;

	err = regcache_sync(radio->core->regmap);
		if (err < 0)
			return err;

	return si473x_radio_init_vtable(radio, func);

}

static int si473x_radio_change_func(struct si473x_radio *radio,
		enum si473x_func func)
{
	int err;
	bool soft;
	int si473x_prop_rx_volume, si473x_prop_rx_hard_mute,
		si473x_prop_digital_output_format;
	int si473x_prop_digital_output_sample_rate,
		si473x_prop_refclk_freq, si473x_prop_refclk_prescale;
	/*
	 * Since power/up down is a very time consuming operation,
	 * try to avoid doing it if the requested mode matches the one
	 * the tuner is in
	 */
	if (func == radio->core->power_up_parameters.func)
		return 0;

	soft = true;
	err = si473x_core_stop(radio->core, soft);
	if (err < 0) {
		/*
		 * OK, if the chip does not want to play nice let's
		 * try to reset it in more brutal way
		 */
		soft = false;
		err = si473x_core_stop(radio->core, soft);
		if (err < 0)
			return err;
	}
	/*
	 * Set the desired radio tuner function
	 */
	radio->core->power_up_parameters.func = func;
	/*
	 * preserve ALSA State while changing regmap_read
	 */
	err = regmap_read(radio->core->regmap,
			SI473X_PROP_RX_VOLUME, &si473x_prop_rx_volume);
	if (err < 0)
		return err;
	err = regmap_read(radio->core->regmap,
			SI473X_PROP_RX_HARD_MUTE, &si473x_prop_rx_hard_mute);
	if (err < 0)
		return err;
	err = regmap_read(radio->core->regmap,
			SI473X_PROP_DIGITAL_OUTPUT_FORMAT,
			&si473x_prop_digital_output_format);
	if (err < 0)
		return err;
	err = regmap_read(radio->core->regmap,
			SI473X_PROP_DIGITAL_OUTPUT_SAMPLE_RATE,
			&si473x_prop_digital_output_sample_rate);
	if (err < 0)
		return err;
	err = regmap_read(radio->core->regmap,
			SI473X_PROP_REFCLK_FREQ, &si473x_prop_refclk_freq);
	if (err < 0)
		return err;
	err = regmap_read(radio->core->regmap,
			SI473X_PROP_REFCLK_PRESCALE,
			&si473x_prop_refclk_prescale);
	if (err < 0)
		return err;

	regmap_exit(radio->core->regmap);
	radio->core->regmap = devm_regmap_init_si473x(radio->core);
	if (IS_ERR(radio->core->regmap)) {
		err = PTR_ERR(radio->core->regmap);
		dev_err(radio->v4l2dev.dev,
			"Failed to allocate register map: %d\n",
			err);
		return err;
	}
	err = regmap_write(radio->core->regmap,
			SI473X_PROP_RX_VOLUME, si473x_prop_rx_volume);
	if (err < 0)
		return err;
	err = regmap_write(radio->core->regmap,
			SI473X_PROP_RX_HARD_MUTE, si473x_prop_rx_hard_mute);
	if (err < 0)
		return err;
	err = regmap_write(radio->core->regmap,
			SI473X_PROP_DIGITAL_OUTPUT_FORMAT,
			si473x_prop_digital_output_format);
	if (err < 0)
		return err;
	err = regmap_write(radio->core->regmap,
			SI473X_PROP_DIGITAL_OUTPUT_SAMPLE_RATE,
			si473x_prop_digital_output_sample_rate);
	if (err < 0)
		return err;
	err = regmap_write(radio->core->regmap,
			SI473X_PROP_REFCLK_FREQ, si473x_prop_refclk_freq);
	if (err < 0)
		return err;
	err = regmap_write(radio->core->regmap,
			SI473X_PROP_REFCLK_PRESCALE,
			si473x_prop_refclk_prescale);
	if (err < 0)
		return err;

	err = si473x_core_start(radio->core, soft);
	if (err < 0)
		return err;

	/*
	 * No need to do the rest of manipulations for the bootlader
	 * mode
	 */
	if (func != SI473X_FUNC_FM_RECEIVER &&
	    func != SI473X_FUNC_AM_RECEIVER)
		return err;

	return si473x_radio_do_post_powerup_init(radio, func);
}

static int si473x_radio_g_frequency(struct file *file, void *priv,
			      struct v4l2_frequency *f)
{
	int err;
	struct si473x_radio *radio = video_drvdata(file);

	if (f->tuner != 0 ||
	    f->type  != V4L2_TUNER_RADIO)
		return -EINVAL;

	si473x_core_lock(radio->core);

	if (radio->ops->rsq_status) {
		struct si473x_rsq_status_report report;
		struct si473x_rsq_status_args   args = {
			.stcack		= false,
		};

		err = radio->ops->rsq_status(radio->core, &args, &report);
		if (!err)
			f->frequency = si473x_to_v4l2(radio->core,
						      report.readfreq);
	} else {
		err = -EINVAL;
	}

	si473x_core_unlock(radio->core);

	return err;
}

static int si473x_radio_s_frequency(struct file *file, void *priv,
				    const struct v4l2_frequency *f)
{
	int err;
	u32 freq = f->frequency;
	struct si473x_tune_freq_args args;
	struct si473x_radio *radio = video_drvdata(file);

	const u32 midrange = (si473x_bands[SI473X_BAND_AM].rangehigh +
			      si473x_bands[SI473X_BAND_FM].rangelow) / 2;
	const int band = (freq > midrange) ?
		SI473X_BAND_FM : SI473X_BAND_AM;
	const enum si473x_func func = (band == SI473X_BAND_AM) ?
		SI473X_FUNC_AM_RECEIVER : SI473X_FUNC_FM_RECEIVER;

	if (f->tuner != 0 ||
	    f->type  != V4L2_TUNER_RADIO)
		return -EINVAL;

	si473x_core_lock(radio->core);

	freq = clamp(freq,
		     si473x_bands[band].rangelow,
		     si473x_bands[band].rangehigh);

	if (si473x_radio_freq_is_inside_of_the_band(freq,
						    SI473X_BAND_AM) &&
	    (!si473x_core_has_am(radio->core))) {
		err = -EINVAL;
		goto unlock;
	}

	err = si473x_radio_change_func(radio, func);
	if (err < 0)
		goto unlock;

	args.freq		= v4l2_to_si473x(radio->core, freq);
	args.tunemode	= SI473X_TM_VALIDATED_NORMAL_TUNE;
	args.antcap		= 0;

	err = radio->ops->tune_freq(radio->core, &args);

unlock:
	si473x_core_unlock(radio->core);
	return err;
}

static int si473x_radio_s_hw_freq_seek(struct file *file, void *priv,
				       const struct v4l2_hw_freq_seek *seek)
{
	int err;
	enum si473x_func func;
	u32 rangelow, rangehigh;
	struct si473x_radio *radio = video_drvdata(file);

	if (file->f_flags & O_NONBLOCK)
		return -EAGAIN;

	if (seek->tuner != 0 ||
	    seek->type  != V4L2_TUNER_RADIO)
		return -EINVAL;

	si473x_core_lock(radio->core);

	if (!seek->rangelow) {
		err = regmap_read(radio->core->regmap,
				  radio->ops->seek_band_bottom,
				  &rangelow);
		if (!err)
			rangelow = si473x_to_v4l2(radio->core, rangelow);
		else
			goto unlock;
	}
	if (!seek->rangehigh) {
		err = regmap_read(radio->core->regmap,
				  radio->ops->seek_band_top,
				  &rangehigh);
		if (!err)
			rangehigh = si473x_to_v4l2(radio->core, rangehigh);
		else
			goto unlock;
	}

	if (rangelow > rangehigh) {
		err = -EINVAL;
		goto unlock;
	}

	if (si473x_radio_range_is_inside_of_the_band(rangelow, rangehigh,
						     SI473X_BAND_FM)) {
		func = SI473X_FUNC_FM_RECEIVER;

	} else if (si473x_core_has_am(radio->core) &&
		   si473x_radio_range_is_inside_of_the_band(rangelow, rangehigh,
							    SI473X_BAND_AM)) {
		func = SI473X_FUNC_AM_RECEIVER;
	} else {
		err = -EINVAL;
		goto unlock;
	}

	err = si473x_radio_change_func(radio, func);
	if (err < 0)
		goto unlock;

	if (seek->rangehigh) {
		err = regmap_write(radio->core->regmap,
				   radio->ops->seek_band_top,
				   v4l2_to_si473x(radio->core,
						  seek->rangehigh));
		if (err)
			goto unlock;
	}
	if (seek->rangelow) {
		err = regmap_write(radio->core->regmap,
				   radio->ops->seek_band_bottom,
				   v4l2_to_si473x(radio->core,
						  seek->rangelow));
		if (err)
			goto unlock;
	}
	if (seek->spacing) {
		err = regmap_write(radio->core->regmap,
				     radio->ops->seek_freq_spacing,
				     v4l2_to_si473x(radio->core,
						    seek->spacing));
		if (err)
			goto unlock;
	}

	err = radio->ops->seek_start(radio->core,
				     seek->seek_upward,
				     seek->wrap_around);
unlock:
	si473x_core_unlock(radio->core);
	return err;
}

static int si473x_radio_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	int retval;
	struct si473x_radio *radio = v4l2_ctrl_handler_to_radio(ctrl->handler);

	si473x_core_lock(radio->core);

	switch (ctrl->id) {
	default:
		retval = -EINVAL;
		break;
	}
	si473x_core_unlock(radio->core);
	return retval;
}

static int si473x_radio_s_ctrl(struct v4l2_ctrl *ctrl)
{
	int retval;
	struct si473x_radio *radio = v4l2_ctrl_handler_to_radio(ctrl->handler);

	si473x_core_lock(radio->core);

	switch (ctrl->id) {
	case V4L2_CID_RDS_RECEPTION:
		/*
		 * It looks like RDS related properties are
		 * inaccesable when tuner is in AM mode
		 */
		retval = 0;
		if (!si473x_core_is_in_am_receiver_mode(radio->core)) {
			if (ctrl->val) {
				retval = regmap_write(radio->core->regmap,
					SI473X_PROP_FM_RDS_INT_FIFO_COUNT,
					radio->core->rds_fifo_depth);
				if (retval < 0)
					break;

				if (radio->core->client->irq) {
					retval = regmap_write(
						radio->core->regmap,
						SI473X_PROP_FM_RDS_INT_SOURCE,
						SI473X_RDSRECV);
					if (retval < 0)
						break;
				}

			/* Drain RDS FIFO before enabling RDS processing */
				retval = si473x_core_cmd_fm_rds_status(
						radio->core,
						false, true, true, NULL);
				if (retval < 0)
					break;

				retval = regmap_update_bits(radio->core->regmap,
					SI473X_PROP_FM_RDS_CONFIG,
					SI473X_PROP_RDSEN_MASK,
					SI473X_PROP_RDSEN);
			} else {
				retval = regmap_update_bits(radio->core->regmap,
					SI473X_PROP_FM_RDS_CONFIG,
					SI473X_PROP_RDSEN_MASK,
					!SI473X_PROP_RDSEN);
			}
		}
		break;
	case V4L2_CID_TUNE_DEEMPHASIS:
		retval = regmap_write(radio->core->regmap,
			SI473X_PROP_FM_DEEMPHASIS,
			ctrl->val);
		break;

	case V4L2_CID_AUDIO_VOLUME:
		retval = regmap_write(radio->core->regmap,
			SI473X_PROP_RX_VOLUME,
			ctrl->val);
		break;

	default:
		retval = -EINVAL;
		break;
	}

	si473x_core_unlock(radio->core);

	return retval;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int si473x_radio_g_register(struct file *file, void *fh,
				   struct v4l2_dbg_register *reg)
{
	int err;
	unsigned int value;
	struct si473x_radio *radio = video_drvdata(file);

	si473x_core_lock(radio->core);
	reg->size = 2;
	err = regmap_read(radio->core->regmap,
			  (unsigned int)reg->reg, &value);
	reg->val = value;
	si473x_core_unlock(radio->core);

	return err;
}
static int si473x_radio_s_register(struct file *file, void *fh,
				   const struct v4l2_dbg_register *reg)
{

	int err;
	struct si473x_radio *radio = video_drvdata(file);

	si473x_core_lock(radio->core);
	err = regmap_write(radio->core->regmap,
			   (unsigned int)reg->reg,
			   (unsigned int)reg->val);
	si473x_core_unlock(radio->core);

	return err;
}
#endif

static int si473x_radio_fops_open(struct file *file)
{
	struct si473x_radio *radio = video_drvdata(file);
	int err;

	err = v4l2_fh_open(file);
	if (err)
		return err;

	if (v4l2_fh_is_singular_file(file)) {
		si473x_core_lock(radio->core);
		err = si473x_core_set_power_state(radio->core,
			SI473X_POWER_UP_FULL);
		if (err < 0)
			goto done;

		err = si473x_radio_do_post_powerup_init(radio,
			radio->core->power_up_parameters.func);
		if (err < 0)
			goto power_down;

		si473x_core_unlock(radio->core);
		/*Must be done after si473x_core_unlock to prevent a deadlock*/
		v4l2_ctrl_handler_setup(&radio->ctrl_handler);
	}

	return err;

power_down:
	si473x_core_set_power_state(radio->core,
		SI473X_POWER_DOWN);
done:
	si473x_core_unlock(radio->core);
	v4l2_fh_release(file);

	return err;
}

static int si473x_radio_fops_release(struct file *file)
{
	int err;
	struct si473x_radio *radio = video_drvdata(file);

	if (v4l2_fh_is_singular_file(file) &&
		atomic_read(&radio->core->is_alive))
		si473x_core_set_power_state(radio->core,
			SI473X_POWER_DOWN);

	err = v4l2_fh_release(file);

	return err;
}

static ssize_t si473x_radio_fops_read(struct file *file, char __user *buf,
		size_t count, loff_t *ppos)
{
	ssize_t      rval;
	size_t       fifo_len;
	unsigned int copied;

	struct si473x_radio *radio = video_drvdata(file);

	/* block if no new data available */
	if (kfifo_is_empty(&radio->core->rds_fifo)) {
		if (file->f_flags & O_NONBLOCK)
			return -EWOULDBLOCK;

		rval = wait_event_interruptible(radio->core->rds_read_queue,
			(!kfifo_is_empty(&radio->core->rds_fifo) ||
			 !atomic_read(&radio->core->is_alive)));
		if (rval < 0)
			return -EINTR;

		if (!atomic_read(&radio->core->is_alive))
			return -ENODEV;
	}

	fifo_len = kfifo_len(&radio->core->rds_fifo);

	if (kfifo_to_user(&radio->core->rds_fifo, buf,
			min(fifo_len, count), &copied) != 0) {
		dev_warn(&radio->videodev.dev,
			 "Error during FIFO to userspace copy\n");
		rval = -EIO;
	} else {
		rval = (ssize_t)copied;
	}

	return rval;
}

static unsigned int si473x_radio_fops_poll(struct file *file,
		struct poll_table_struct *pts)
{
	struct si473x_radio *radio = video_drvdata(file);
	unsigned long req_events = poll_requested_events(pts);
	unsigned int err = v4l2_ctrl_poll(file, pts);

	if (req_events & (POLLIN | POLLRDNORM)) {
		if (atomic_read(&radio->core->is_alive))
			poll_wait(file, &radio->core->rds_read_queue, pts);

		if (!atomic_read(&radio->core->is_alive))
			err = POLLHUP;

		if (!kfifo_is_empty(&radio->core->rds_fifo))
			err = POLLIN | POLLRDNORM;
	}

	return err;
}

static const struct v4l2_file_operations si473x_fops = {
	.owner			= THIS_MODULE,
	.read			= si473x_radio_fops_read,
	.poll			= si473x_radio_fops_poll,
	.unlocked_ioctl		= video_ioctl2,
	.open			= si473x_radio_fops_open,
	.release		= si473x_radio_fops_release,
};


static const struct v4l2_ioctl_ops si4731_ioctl_ops = {
	.vidioc_querycap		= si473x_radio_querycap,
	.vidioc_g_tuner			= si473x_radio_g_tuner,
	.vidioc_s_tuner			= si473x_radio_s_tuner,

	.vidioc_g_frequency		= si473x_radio_g_frequency,
	.vidioc_s_frequency		= si473x_radio_s_frequency,
	.vidioc_s_hw_freq_seek		= si473x_radio_s_hw_freq_seek,
	.vidioc_enum_freq_bands		= si473x_radio_enum_freq_bands,

	.vidioc_subscribe_event		= v4l2_ctrl_subscribe_event,
	.vidioc_unsubscribe_event	= v4l2_event_unsubscribe,

#ifdef CONFIG_VIDEO_ADV_DEBUG
	.vidioc_g_register		= si473x_radio_g_register,
	.vidioc_s_register		= si473x_radio_s_register,
#endif
};


static const struct video_device si473x_viddev_template = {
	.fops			= &si473x_fops,
	.name			= DRIVER_NAME,
	.release		= video_device_release_empty,
};

static int si473x_radio_probe(struct platform_device *pdev)
{
	int rval;
	struct si473x_radio *radio;
	struct v4l2_ctrl *ctrl;

	static atomic_t instance = ATOMIC_INIT(0);

	radio = devm_kzalloc(&pdev->dev, sizeof(*radio), GFP_KERNEL);
	if (!radio)
		return -ENOMEM;

	radio->core = i2c_mfd_cell_to_core(&pdev->dev);

	v4l2_device_set_name(&radio->v4l2dev, DRIVER_NAME, &instance);

	rval = v4l2_device_register(&pdev->dev, &radio->v4l2dev);
	if (rval) {
		dev_err(&pdev->dev, "Cannot register v4l2_device.\n");
		return rval;
	}

	memcpy(&radio->videodev, &si473x_viddev_template,
	       sizeof(struct video_device));

	radio->videodev.v4l2_dev  = &radio->v4l2dev;
	radio->videodev.ioctl_ops = &si4731_ioctl_ops;

	video_set_drvdata(&radio->videodev, radio);
	platform_set_drvdata(pdev, radio);


	radio->v4l2dev.ctrl_handler = &radio->ctrl_handler;
	v4l2_ctrl_handler_init(&radio->ctrl_handler, 1);

	ctrl = v4l2_ctrl_new_std_menu(&radio->ctrl_handler,
				      &si473x_ctrl_ops,
				      V4L2_CID_TUNE_DEEMPHASIS,
				      V4L2_DEEMPHASIS_75_uS, 0, 0);
	rval = radio->ctrl_handler.error;
	if (ctrl == NULL && rval) {
		dev_err(&pdev->dev, "Could not initialize V4L2_CID_TUNE_DEEMPHASIS control %d\n",
			rval);
		goto exit;
	}

	ctrl = v4l2_ctrl_new_std(&radio->ctrl_handler, &si473x_ctrl_ops,
				 V4L2_CID_RDS_RECEPTION,
				 0, 1, 1, 1);
	rval = radio->ctrl_handler.error;
	if (ctrl == NULL && rval) {
		dev_err(&pdev->dev, "Could not initialize V4L2_CID_RDS_RECEPTION control %d\n",
			rval);
		goto exit;
	}

	ctrl = v4l2_ctrl_new_std(&radio->ctrl_handler, &si473x_ctrl_ops,
				 V4L2_CID_AUDIO_VOLUME,
				 0, 0x3f, 1, 0x3f);
	rval = radio->ctrl_handler.error;
	if (ctrl == NULL && rval) {
		dev_err(&pdev->dev, "Could not initialize V4L2_CID_AUDIO_VOLUME control %d\n",
			rval);
		goto exit;
	}

	/* register video device */
	rval = video_register_device(&radio->videodev, VFL_TYPE_RADIO, -1);
	if (rval < 0) {
		dev_err(&pdev->dev, "Could not register video device\n");
		goto exit;
	}
	return 0;
exit:
	v4l2_ctrl_handler_free(radio->videodev.ctrl_handler);
	return rval;
}

static int si473x_radio_remove(struct platform_device *pdev)
{
	struct si473x_radio *radio = platform_get_drvdata(pdev);

	v4l2_ctrl_handler_free(radio->videodev.ctrl_handler);
	video_unregister_device(&radio->videodev);
	v4l2_device_unregister(&radio->v4l2dev);

	return 0;
}

MODULE_ALIAS("platform:si473x-radio");

static struct platform_driver si473x_radio_driver = {
	.driver		= {
		.name			= DRIVER_NAME,
	},
	.probe		= si473x_radio_probe,
	.remove		= si473x_radio_remove,
};
module_platform_driver(si473x_radio_driver);

MODULE_AUTHOR("rpi Receiver <rpi-receiver@htl-steyr.ac.at>");
MODULE_DESCRIPTION("Driver for Si473x AM/FM Radio");
MODULE_LICENSE("GPL");
