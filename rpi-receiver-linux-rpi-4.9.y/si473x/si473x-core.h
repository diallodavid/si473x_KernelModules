/*
 * include/media/si473x-core.h -- Common definitions for si473x core
 * device
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

#ifndef SI473X_CORE_H
#define SI473X_CORE_H

#include <linux/kfifo.h>
#include <linux/atomic.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/mutex.h>
#include <linux/mfd/core.h>
#include <linux/videodev2.h>

#include "si473x-platform.h"
#include "si473x-reports.h"
/* Command Timeouts */
#define SI473X_DEFAULT_TIMEOUT	100000
#define SI473X_TIMEOUT_TUNE		700000
#define SI473X_TIMEOUT_POWER_UP	330000
#define SI473X_STATUS_POLL_US	0

/* -------------------- si473x-i2c.c ----------------------- */

enum si473x_freq_supported_chips {
	SI473X_CHIP_SI4731 = 0,
};

enum si473x_part_revisions {
	SI4731_REVISION_D62 = 0,
};

enum si473x_mfd_cells {
	SI473X_RADIO_CELL = 0,
	SI473X_CODEC_CELL,
	SI473X_MFD_CELLS,
};

/**
 * enum si473x_power_state - possible power state of the si473x
 * device.
 *
 * @SI473X_POWER_DOWN: In this state the reset line is pulled low.
 *  The device is completely inactive.
 * @SI473X_POWER_UP_FULL: In this state all the power regualtors are
 * turned on, reset line pulled high, IRQ line is enabled(polling is
 * active for polling use scenario) and device is turned on with
 * POWER_UP command. The device is ready to be used.
 */
enum si473x_power_state {
	SI473X_POWER_DOWN		= 0,
	SI473X_POWER_UP_FULL	= 1,
};

/**
 * struct si473x_core - internal data structure representing the
 * underlying "core" device which all the MFD cell-devices use.
 *
 * @client: Actual I2C client used to transfer commands to the chip.
 * @chip_id: Last digit of the chip model(E.g. "7" for SI4737)
 * @cells: MFD cell devices created by this driver.
 * @cmd_lock: Mutex used to serialize all the requests to the core
 * device. This filed should not be used directly. Instead
 * si473x_core_lock()/si473x_core_unlock() should be used to get
 * exclusive access to the "core" device.
 * @users: Active users counter(Used by the radio cell)
 * @rds_read_queue: Wait queue used to wait for RDS data.
 * @rds_fifo: FIFO in which all the RDS data received from the chip is
 * placed.
 * @rds_fifo_drainer: Worker that drains on-chip RDS FIFO.
 * @rds_drainer_is_working: Flag used for launching only one instance
 * of the @rds_fifo_drainer.
 * @rds_drainer_status_lock: Lock used to guard access to the
 * @rds_drainer_is_working variable.
 * @command: Wait queue for wainting on the command comapletion.
 * @cts: Clear To Send flag set upon receiving first status with CTS
 * set.
 * @tuning: Wait queue used for wainting for tune/seek comand
 * completion.
 * @stc: Similar to @cts, but for the STC bit of the status value.
 * @power_up_parameters: Parameters used as argument for POWER_UP
 * command when the device is started.
 * @power_state: Current power state of the device.
 * @gpio_reset: GPIO pin connectet to the RSTB pin of the chip.
 * @status_monitor: Polling worker used in polling use case scenarion
 * (when IRQ is not avalible).
 * @revision: Chip's running firmware revision number(Used for correct
 * command set support).
 */

struct si473x_core {
	struct i2c_client *client;
	struct regmap *regmap;
	int chip_id;
	struct mfd_cell cells[SI473X_MFD_CELLS];

	struct mutex cmd_lock; /* for serializing fm radio operations */
	atomic_t users;

	wait_queue_head_t  rds_read_queue;
	struct kfifo       rds_fifo;
	struct work_struct rds_fifo_drainer;
	bool               rds_drainer_is_working;
	struct mutex       rds_drainer_status_lock;

	wait_queue_head_t command;
	atomic_t          cts;

	wait_queue_head_t tuning;
	atomic_t          stc;
	int               update_interrupt_status;

	struct si473x_power_up_args power_up_parameters;

	enum si473x_power_state power_state;

	int gpio_reset;

	atomic_t is_alive;

	struct delayed_work status_monitor;
#define SI473X_WORK_TO_CORE(w) container_of(to_delayed_work(w),	\
					    struct si473x_core,	\
					    status_monitor)

	int revision;

	int rds_fifo_depth;
};

static inline struct si473x_core *i2c_mfd_cell_to_core(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev->parent);

	return i2c_get_clientdata(client);
}


/**
 * si473x_core_lock() - lock the core device to get an exclusive access
 * to it.
 */
static inline void si473x_core_lock(struct si473x_core *core)
{
	mutex_lock(&core->cmd_lock);
}

/**
 * si473x_core_unlock() - unlock the core device to relinquish an
 * exclusive access to it.
 */
static inline void si473x_core_unlock(struct si473x_core *core)
{
	mutex_unlock(&core->cmd_lock);
}

/* *_TUNE_FREQ family of commands accept frequency in multiples of 10kHz */
static inline u16 hz_to_si473x(struct si473x_core *core, int freq)
{
	u16 result;

	switch (core->power_up_parameters.func) {
	default:
	case SI473X_FUNC_FM_RECEIVER:
		result = freq / 10000;
		break;
	case SI473X_FUNC_AM_RECEIVER:
		result = freq / 1000;
		break;
	}

	return result;
}

static inline int si473x_to_hz(struct si473x_core *core, u16 freq)
{
	int result;

	switch (core->power_up_parameters.func) {
	default:
	case SI473X_FUNC_FM_RECEIVER:
		result = freq * 10000;
		break;
	case SI473X_FUNC_AM_RECEIVER:
		result = freq * 1000;
		break;
	}

	return result;
}

/* Since the V4L2_TUNER_CAP_LOW flag is supplied, V4L2 subsystem
 * mesures frequency in 62.5 Hz units
 */

static inline int hz_to_v4l2(int freq)
{
	return (freq * 10) / 625;
}

static inline int v4l2_to_hz(int freq)
{
	return (freq * 625) / 10;
}

static inline u16 v4l2_to_si473x(struct si473x_core *core, int freq)
{
	return hz_to_si473x(core, v4l2_to_hz(freq));
}

static inline int si473x_to_v4l2(struct si473x_core *core, u16 freq)
{
	return hz_to_v4l2(si473x_to_hz(core, freq));
}



/**
 * struct si473x_get_rev - structure containing result of the
 * GET_REV command.
 *
 * @firmware.partnumber: Final 2 digits of Part Number (HEX).
 * @firmware.major: Firmware major number (ASCII).
 * @firmware.minor: Firmware minor number (ASCII).
 * @patch_id: Firmware Patch ID (HEX).
 * @component_revision.major: Component Major Revision (ASCII).
 * @component_revision.minor: Component Minor Revision (ASCII).
 * @chip_revision: Chip Revision (ASCII).
 */
struct si473x_get_rev {
	u8 partnumber;
	struct {
		u8 major, minor;
	} firmware;
	u16 patch_id;
	struct {
		u8 major, minor;
	} component_revision;
	u8 chip_revision;
};

/**
 * enum si473x_tunemode - enum representing possible tune modes for
 * the chip.
 * @SI473X_TM_INVALIDATED_FAST_TUNE: Unconditionally stay in the new
 * channel after tune, tune status invalid.
 * @SI473X_TM_FREEZE_METRICS: If set will cause the blend, hicut,
 * and softmute to transition as a function of the associated
 * attack/release parameters rather than instantaneously when tuning
 * to alternate station.
 */
enum si473x_tunemode {
	SI473X_TM_VALIDATED_NORMAL_TUNE = 0,
	SI473X_TM_INVALIDATED_FAST_TUNE = BIT(0),
	SI473X_TM_FREEZE_METRICS        = BIT(1),
};

/**
 * struct si473x_rds_status_report - the structure representing the
 * response to 'FM_RDS_STATUS' command
 * @rdsnewblocka: Valid Block A data has been received
 * @rdsnewblockb: Valid Block B data has been received
 * @rdssyncfound: Found RDS synchronization.
 * @rdssynclost: Lost RDS synchronization.
 * @rdsrecv: FIFO filled to minimum number of groups set by RDSFIFOCNT
 * @rdssync: RDS is currently synchronized.
 * @grplost: One or more RDS groups discarded due to FIFO overrun.
 * @rdsfifoused: Number of blocks remaining in the RDS FIFO (0 if
 * empty).
 */
struct si473x_rds_status_report {
	bool rdsnewblocka, rdsnewblockb, rdssyncfound, rdssynclost, rdsrecv;
	bool rdssync, grplost;

	u8 rdsfifoused;
	u8 ble[4];

	struct v4l2_rds_data rds[4];
};

struct si473x_rsq_status_args {
	bool stcack;
};


struct si473x_tune_freq_args {
	int freq;
	enum si473x_tunemode tunemode;
	int antcap;
};

int  si473x_core_stop(struct si473x_core *core, bool soft);
int  si473x_core_start(struct si473x_core *core, bool soft);
int  si473x_core_set_power_state(struct si473x_core *core,
				enum si473x_power_state next_state);
bool si473x_core_has_am(struct si473x_core *core);
bool si473x_core_is_in_am_receiver_mode(struct si473x_core *core);
bool si473x_core_is_powered_up(struct si473x_core *core);

enum si473x_i2c_type {
	SI473X_I2C_SEND,
	SI473X_I2C_RECV
};

int si473x_core_i2c_xfer(struct si473x_core *core,
			 enum si473x_i2c_type,
			 char *buf, int count);


/* -------------------- si473x-cmd.c ----------------------- */

int si473x_core_cmd_get_rev(struct si473x_core *core,
				struct si473x_get_rev *info);
int si473x_core_cmd_set_property(struct si473x_core *core, u16 property,
				u16 value);
int si473x_core_cmd_get_property(struct si473x_core *core, u16 property);
int si473x_core_cmd_fm_seek_start(struct si473x_core *core,
				bool seekup, bool wrap);
int si473x_core_cmd_am_seek_start(struct si473x_core *core,
				bool seekup, bool wrap);
int si473x_core_cmd_fm_rds_status(struct si473x_core *core,
				bool status_only, bool mtfifo, bool intack,
				struct si473x_rds_status_report *report);
int si473x_core_cmd_fm_tune_freq(struct si473x_core *core,
				 struct si473x_tune_freq_args *tuneargs);
int si473x_core_cmd_am_tune_freq(struct si473x_core *core,
				struct si473x_tune_freq_args *tuneargs);
int si473x_core_cmd_am_rsq_status(struct si473x_core *core,
				struct si473x_rsq_status_args *rsqargs,
				struct si473x_rsq_status_report *report);
int si473x_core_cmd_fm_rsq_status(struct si473x_core *core,
				struct si473x_rsq_status_args *rsqargs,
				struct si473x_rsq_status_report *report);
int si473x_core_cmd_power_up(struct si473x_core *core,
				struct si473x_power_up_args *puargs);
int si473x_core_cmd_power_down(struct si473x_core *core);

int si473x_core_cmd_agc_status(struct si473x_core *core,
				struct si473x_agc_status_report *report);

/* Properties  */

enum si473x_interrupt_flags {
	SI473X_STCIEN = BIT(0),
	SI473X_ASQIEN = BIT(1),
	SI473X_RDSIEN = BIT(2),
	SI473X_RSQIEN = BIT(3),

	SI473X_ERRIEN = BIT(6),
	SI473X_CTSIEN = BIT(7),

	SI473X_STCREP = BIT(8),
	SI473X_ASQREP = BIT(9),
	SI473X_RDSREP = BIT(10),
	SI473X_RSQREP = BIT(11),
};

enum si473x_rdsint_sources {
	SI473X_RDSNEWBLOCKB = BIT(5),
	SI473X_RDSNEWBLOCKA = BIT(4),
	SI473X_RDSSYNCFOUND = BIT(2),
	SI473X_RDSSYNCLOST  = BIT(1),
	SI473X_RDSRECV      = BIT(0),
};

enum si473x_status_response_bits {
	SI473X_CTS	  = BIT(7),
	SI473X_ERR	  = BIT(6),
	/* Status response for WB receiver */
	SI473X_RSQ_INT    = BIT(3),
	/* Status response for FM receiver */
	SI473X_FM_RDS_INT = BIT(2),
	SI473X_ASQ_INT    = BIT(1),
	SI473X_STC_INT    = BIT(0),
};

/* -------------------- si473x-prop.c ----------------------- */
enum si473x_common_receiver_properties {
/* Enables interrupt sources */
	SI473X_PROP_GPO_IEN = 0x0001,
/* Configure digital audio outputs */
	SI473X_PROP_DIGITAL_OUTPUT_FORMAT = 0x0102,
/* Configure digital audio output sample rate */
	SI473X_PROP_DIGITAL_OUTPUT_SAMPLE_RATE = 0x0104,
/* Sets frequency of reference clock in Hz
 * The range is 31130 to 34406 Hz, or 0 to disable the AFC. Default is 32768 Hz
 */
	SI473X_PROP_REFCLK_FREQ = 0x0201,
	SI473X_PROP_REFCLK_FREQ_MIN = 31130,
	SI473X_PROP_REFCLK_FREQ_MAX = 34406,
/* Sets the prescaler value for RCLK input*/
	SI473X_PROP_REFCLK_PRESCALE = 0x0202,
	SI473X_PROP_REFCLK_PRESCALE_MIN = 1,
	SI473X_PROP_REFCLK_PRESCALE_MAX = 4095,
/* Sets the output volume */
	SI473X_PROP_RX_VOLUME = 0x4000,
/* Mutes the audio output. L and R audio outputs may be muted independently */
	SI473X_PROP_RX_HARD_MUTE = 0x4001,
};

enum si473x_fm_receiver_properties {
/* Sets deemphasis time constant. Default is 75 μs */
	SI473X_PROP_FM_DEEMPHASIS = 0x1100,
/* Selects bandwidth of channel filter applied at the demodulation stage */
	SI473X_PROP_FM_CHANNEL_FILTER = 0x1102,
/* Selects bandwidth of channel filter applied at the demodulation stage */
	SI473X_PROP_FM_BLEND_STEREO_THRESHOLD = 0x1105,
/* Sets RSSI threshold for mono blend (Full mono below threshold, blend
 * above threshold). To force stereo set this to 0.
 * To force mono set this to 127. Default value is 30 dBμV
 */
	SI473X_PROP_FM_BLEND_MONO_THRESHOLD = 0x1106,
/* Sets the maximum freq error allowed before setting the AFC rail (AFCRL)
 * indicator. Default value is 20 kHz
 */
	SI473X_PROP_FM_MAX_TUNE_ERROR = 0x1108,
/* Configures interrupt related to Received Signal Quality metrics */
	SI473X_PROP_FM_RSQ_INT_SOURCE = 0x1200,
/* Sets high threshold for SNR interrupt */
	SI473X_PROP_FM_RSQ_SNR_HI_THRESHOLD = 0x1201,
/* Sets low threshold for SNR interrupt */
	SI473X_PROP_FM_RSQ_SNR_LO_THRESHOLD = 0x1202,
/* Sets high threshold for RSSI interrupt */
	SI473X_PROP_FM_RSQ_RSSI_HI_THRESHOLD = 0x1203,
/* Sets low threshold for RSSI interrupt */
	SI473X_PROP_FM_RSQ_RSSI_LO_THRESHOLD = 0x1204,
/* Sets high threshold for multipath interrupt */
	SI473X_PROP_FM_RSQ_MULTIPATH_HI_THRESHOLD = 0x1205,
/* Sets low threshold for multipath interrupt */
	SI473X_PROP_FM_RSQ_MULTIPATH_LO_THRESHOLD = 0x1206,
/* Sets the blend threshold for blend interrupt when boundary is crossed */
	SI473X_PROP_FM_RSQ_BLEND_THRESHOLD = 0x1207,
/* Sets the attack and decay rates when entering and leaving soft mute */
	SI473X_PROP_FM_SOFT_MUTE_RATE = 0x1300,
/* Configures attenuation slope during soft mute in dB attenuation per dB SNR
 * below the soft mute SNR threshold. Default value is 2
 */
	SI473X_PROP_FM_SOFT_MUTE_SLOPE = 0x1301,
/* Sets maximum attenuation during soft mute (dB). Set to 0 to
 * disable soft mute. Default is 16 dB
 */
	SI473X_PROP_FM_SOFT_MUTE_MAX_ATTENUATION = 0x1302,
/* Sets SNR threshold to engage soft mute. Default is 4 dB */
	SI473X_PROP_FM_SOFT_MUTE_SNR_THRESHOLD = 0x1303,
/* Sets soft mute release rate. Smaller values provide slower release, and
 * larger values provide faster release.
 * The default is 8192 (approximately 8000 dB/s)
 */
	SI473X_PROP_FM_SOFT_MUTE_RELEASE_RATE = 0x1304,
/* Sets soft mute attack rate. Smaller values provide slower attack, and larger
 * values provide faster attack. The default is 8192 (approximately 8000 dB/s)
 */
	SI473X_PROP_FM_SOFT_MUTE_ATTACK_RATE = 0x1305,
/* Sets the bottom of the FM band for seek. Default is 8750 (87.5 MHz) */
	SI473X_PROP_FM_SEEK_BAND_BOTTOM = 0x1400,
/* Sets the top of the FM band for seek. Default is 10790 (107.9 MHz) */
	SI473X_PROP_FM_SEEK_BAND_TOP = 0x1401,
/* Selects frequency spacing for FM seek. Default value is 10 (100 kHz) */
	SI473X_PROP_FM_SEEK_FREQ_SPACING = 0x1402,
/* Sets the SNR threshold for a valid FM Seek/Tune. Default value is 3 dB */
	SI473X_PROP_FM_SEEK_TUNE_SNR_THRESHOLD = 0x1403,
/* Sets the RSSI threshold for a valid FM Seek/Tune.
 * Default value is 20 dBμV
 */
	SI473X_PROP_FM_SEEK_TUNE_RSSI_TRESHOLD = 0x1404,
/* Configures RDS interrupt behavior */
	SI473X_PROP_FM_RDS_INT_SOURCE = 0x1500,
/* Sets the minimum number of RDS groups stored in the receive FIFO required
 * before RDSRECV is set
 */
	SI473X_PROP_FM_RDS_INT_FIFO_COUNT = 0x1501,
/* Configures RDS setting */
	SI473X_PROP_FM_RDS_CONFIG = 0x1502,
/* Sets the confidence level threshold for each RDS block */
	SI473X_PROP_FM_RDS_CONFIDENCE = 0x1503,
/* Sets the AGC attack rate. Larger values provide slower attack and
 * smaller values provide faster attack.
 * The default is 4 (approximately 1500 dB/s)
 */
	SI473X_PROP_FM_AGC_ATTACK_RATE = 0x1700,
/* Sets the AGC release rate. Larger values provide slower release and
 * smaller values provide faster release.
 * The default is 140 (approximately 43 dB/s)
 */
	SI473X_PROP_FM_AGC_RELEASE_RATE = 0x1701,
/* Sets RSSI threshold for stereo blend. (Full stereo above threshold, blend
 * below threshold.) To force stereo, set this to 0. To force mono,
 * set this to 127. Default value is 49 dBμV
 */
	SI473X_PROP_FM_BLEND_RSSI_STEREO_THRESHOLD = 0x1800,
/* Sets RSSI threshold for mono blend (Full mono below threshold, blend above
 * threshold). To force stereo, set this to 0. To force mono, set this to 127.
 * Default value is 30 dBμV
 */
	SI473X_PROP_FM_BLEND_RSSI_MONO_THRESHOLD = 0x1801,
/* Sets the stereo to mono attack rate for RSSI based blend. Smaller values
 * provide slower attack and larger values provide faster attack.
 * The default is 4000 (approximately 16 ms)
 */
	SI473X_PROP_FM_BLEND_RSSI_ATTACK_RATE = 0x1802,
/* Sets the mono to stereo release rate for RSSI based blend. Smaller values
 * provide slower release and larger values provide faster release.
 * The default is 400 (approximately 164 ms)
 */
	SI473X_PROP_FM_BLEND_RSSI_RELEASE_RATE = 0x1803,
/* Sets SNR threshold for stereo blend (Full stereo above threshold, blend
 * below threshold). To force stereo, set this to 0. To force mono,
 * set this to 127. Default value is 27 dB
 */
	SI473X_PROP_FM_BLEND_SNR_STEREO_THRESHOLD = 0x1804,
/* Sets SNR threshold for mono blend (Full mono below threshold, blend above
 * threshold). To force stereo, set this to 0. To force mono, set this to 127.
 * Default value is 14 dB
 */
	SI473X_PROP_FM_BLEND_SNR_MONO_THRESHOLD = 0x1805,
/* Sets the stereo to mono attack rate for SNR based blend. Smaller values
 * provide slower attack and larger values provide faster attack.
 * The default is 4000 (approximately 16 ms)
 */
	SI473X_PROP_FM_BLEND_SNR_ATTACK_RATE = 0x1806,
/* Sets the mono to stereo release rate for SNR based blend. Smaller values
 * provide slower release and larger values provide faster release.
 * The default is 400 (approximately 164 ms)
 */
	SI473X_PROP_FM_BLEND_SNR_RELEASE_RATE = 0x1807,
/* Sets multipath threshold for stereo blend (Full stereo below threshold, blend
 * above threshold). To force stereo, set this to 100. To force mono,
 * set this to 0. Default value is 20
 */
	SI473X_PROP_FM_BLEND_MULTIPATH_STEREO_THRESHOLD = 0x1808,
/* Sets Multipath threshold for mono blend (Full mono above threshold, blend
 * below threshold). To force stereo, set to 100. To force mono, set to 0.
 * The default is 60
 */
	SI473X_PROP_FM_BLEND_MULTIPATH_MONO_THRESHOLD = 0x1809,
/* Sets the stereo to mono attack rate for Multipath based blend.
 * Smaller values provide slower attack and larger values provide faster
 * attack. The default is 4000 (approximately 16 ms)
 */
	SI473X_PROP_FM_BLEND_MULTIPATH_ATTACK_RATE = 0x180A,
/* Sets the mono to stereo release rate for Multipath based blend.
 * Smaller values provide slower release and larger values provide faster
 * release. The default is 40 (approximately 1.64 s)
 */
	SI473X_PROP_FM_BLEND_MULTIPATH_RELEASE_RATE = 0x180B,
/* Sets the maximum amount of stereo separation */
	SI473X_PROP_FM_BLEND_MAX_STEREO_SEPARATION = 0x180C,
/* Sets the SNR level at which hi-cut begins to band limit.
 * Default value is 24
 */
	SI473X_PROP_FM_HICUT_SNR_HIGH_THRESHOLD = 0x1A00,
/* Sets the SNR level at which hi-cut reaches maximum band limiting.
 * Default value is 15
 */
	SI473X_PROP_FM_HICUT_SNR_LOW_THRESHOLD = 0x1A01,
/* Sets the rate at which hi-cut lowers the cutoff frequency.
 * Default value is 20000 (approximately 3 ms)
 */
	SI473X_PROP_FM_HICUT_ATTACK_RATE = 0x1A02,
/* Sets the rate at which hi-cut increases the cut-off frequency.
 * Default value is 20. (approximately 3.3 s)
 */
	SI473X_PROP_FM_HICUT_RELEASE_RATE = 0x1A03,
/* Sets the MULTIPATH level at which hi-cut begins to band limit.
 * Default value is 20
 */
	SI473X_PROP_FM_HICUT_MULTIPATH_TRIGGER_THRESHOLD = 0x1A04,
/* Sets the MULTIPATH level at which hi-cut reaches maximum band limiting.
 * Default value is 60.
 */
	SI473X_PROP_FM_HICUT_MULTIPATH_END_THRESHOLD = 0x1A05,
/* Sets the maximum band limit frequency for hi-cut and also sets the maximum
 * audio frequency. Default value is 0 (disabled).
 */
	SI473X_PROP_FM_HICUT_CUTOFF_FREQUENCY = 0x1A06,
};

enum si473x_prop_fm_rds_config_bits {
	SI473X_PROP_RDSEN_MASK	= BIT(0),
	SI473X_PROP_RDSEN		= BIT(0),
};

enum si473x_am_receiver_properties {
/* Sets deemphasis time constant. Can be set to 50 μs.
 * Deemphasis is disabled by default
 */
	SI473X_PROP_AM_DEEMPHASIS = 0x3100,
/* Selects the bandwidth of the channel filter for AM reception. The choices
 * are 6, 4, 3, 2, 2.5, 1.8, or 1 (kHz). The default bandwidth is 2 kHz
 */
	SI473X_PROP_AM_CHANNEL_FILTER = 0x3102,
/* Sets the maximum gain for automatic volume control */
	SI473X_PROP_AM_AUTOMATIC_VOLUME_CONTROL_MAX_GAIN = 0x3103,
/* Sets the SW AFC pull-in range */
	SI473X_PROP_AM_MODE_AFC_SW_PULL_IN_RANGE = 0x3104,
/* Sets the SW AFC lock-in */
	SI473X_PROP_AM_MODE_AFC_SW_LOCK_IN_RANGE = 0x3105,
/* Configures interrupt related to Received Signal Quality metrics.
 * All interrupts are disabled by default
 */
	SI473X_PROP_AM_RSQ_INTERRUPTS = 0x3200,
/* Sets high threshold for SNR interrupt */
	SI473X_PROP_AM_RSQ_SNR_HIGH_THRESHOLD = 0x3201,
/* Sets low threshold for SNR interrupt */
	SI473X_PROP_AM_RSQ_SNR_LOW_THRESHOLD = 0x3202,
/* Sets high threshold for RSSI interrupt */
	SI473X_PROP_AM_RSQ_RSSI_HIGH_THRESHOLD = 0x3203,
/* Sets low threshold for RSSI interrupt */
	SI473X_PROP_AM_RSQ_RSSI_LOW_THRESHOLD = 0x3204,
/* Sets the attack and decay rates when entering or leaving soft mute.
 * The default is 278 dB/s
 */
	SI473X_PROP_AM_SOFT_MUTE_RATE = 0x3300,
/* Sets the AM soft mute slope. Default value is a slope of 1 */
	SI473X_PROP_AM_SOFT_MUTE_SLOPE = 0x3301,
/* Sets maximum attenuation during soft mute (dB).
 * Set to 0 to disable soft mute. Default is 8 dB
 */
	SI473X_PROP_AM_SOFT_MUTE_MAX_ATTENUATION = 0x3302,
/* Sets SNR threshold to engage soft mute. Default is 8 dB */
	SI473X_PROP_AM_SOFT_MUTE_SNR_THRESHOLD = 0x3303,
/* Sets softmute release rate. Smaller values provide slower release, and larger
 * values provide faster release. The default is 8192 (approximately 8000 dB/s)
 */
	SI473X_PROP_AM_SOFT_MUTE_RELEASE_RATE = 0x3304,
/* Sets software attack rate. Smaller values provide slower attack, and larger
 * values provide faster attack. The default is 8192 (approximately 8000 dB/s)
 */
	SI473X_PROP_AM_SOFT_MUTE_ATTACK_RATE = 0x3305,
/* Sets the bottom of the AM band for seek. Default is 520 */
	SI473X_PROP_AM_SEEK_BAND_BOTTOM = 0x3400,
/* Sets the top of the AM band for seek. Default is 1710 */
	SI473X_PROP_AM_SEEK_BAND_TOP = 0x3401,
/* Selects frequency spacing for AM seek. Default is 10 kHz spacing */
	SI473X_PROP_AM_SEEK_FREQ_SPACING = 0x3402,
/* Sets the SNR threshold for a valid AM Seek/Tune. If the value is zero then
 * SNR threshold is not considered when doing a seek. Default value is 5 dB
 */
	SI473X_PROP_AM_SEEK_SNR_THRESHOLD = 0x3403,
/* Sets the RSSI threshold for a valid AM Seek/Tune. If the value is zero then
 * RSSI threshold is not considered when doing a seek. Default value is 25 dBμV
 */
	SI473X_PROP_AM_SEEK_RSSI_THRESHOLD = 0x3404,
/* Sets the number of milliseconds the high peak detector must be exceeded
 * before decreasing gain. Default value is 4 (approximately 1400 dB/s)
 */
	SI473X_PROP_AM_AGC_ATTACK_RATE = 0x3702,
/* Sets the number of milliseconds the low peak detector must not be exceeded
 * before increasing the gain. Default value is 140 (approximately 40 dB/s).
 */
	SI473X_PROP_AM_AGC_RELEASE_RATE = 0x3703,
/* Adjusts AM AGC for frontend (external) attenuator and LNA */
	SI473X_PROP_AM_FRONTEND_AGC_CONTROL = 0x3705,
/* Sets the threshold for detecting impulses in dB
 * above the noise floor. Default value is 12
 */
	SI473X_PROP_AM_NB_DETECT_THRESHOLD = 0x3900,
/* Interval in micro-seconds that original samples are replaced by
 * interpolated clean samples. Default value is 55 μs
 */
	SI473X_PROP_AM_NB_INTERVAL = 0x3901,
/* Noise blanking rate in 100 Hz units. Default value is 64 */
	SI473X_PROP_AM_NB_RATE = 0x3902,
/* Sets the bandwidth of the noise floor estimator. Default value is 300 */
	SI473X_PROP_AM_NB_IIR_FILTER = 0x3903,
/* Delay in micro-seconds before applying impulse blanking to the
 *  original samples. Default value is 172.
 */
	SI473X_PROP_AM_NB_DELAY = 0x3904,
};

struct regmap *devm_regmap_init_si473x(struct si473x_core *core);

#endif	/* SI473X_CORE_H */
