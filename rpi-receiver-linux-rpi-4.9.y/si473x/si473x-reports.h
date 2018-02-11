/*
 * include/media/si473x-platform.h
 *
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

#ifndef __SI473X_REPORTS_H__
#define __SI473X_REPORTS_H__

/**
 * struct si473x_rsq_status_report - structure containing received signal
 * quality
 * @bltf:       Band Limit.
 *              Set if seek command hits the band limit or wrapped to
 *              the original frequency.
 * @afcrl:      Set if FREQOFF >= MAX_TUNE_ERROR
 * @valid:      Set if the channel is valid
 *               rssi < FM_VALID_RSSI_THRESHOLD
 *               snr  < FM_VALID_SNR_THRESHOLD
 *               tune_error < FM_VALID_MAX_TUNE_ERROR
 * @readfreq:   Current tuned frequency.
 * @rssi:       Received Signal Strength Indicator(dBuV).
 * @snr:        RF SNR Indicator(dB).
 * @mult:       Multipath indicator
 * @readantcap: Antenna tuning capacity value.
 * @blendint:   Blend Detect Interrupt.
 *              true  -  Blend goes above or below the Blend threshold settings
 *                      SI473X_PROP_FM_RSQ_BLEND_THRESHOLD
 * @multhint:   Multipath Detect High.
 *              true  -  Detected multipath value has exceeded above
 *                       the Multipath high threshold
 *                      SI473X_PROP_FM_RSQ_MULTIPATH_HI_THRESHOLD
 * @multlint:   Multipath Detect Low.
 *              true  - Detected multipath value has fallen below
 *                      the Multipath low threshold
 *                      SI473X_PROP_FM_RSQ_MULTIPATH_LO_THRESHOLD
 * @snrhint:    SNR Detect High.
 *              true  - Received SNR has exceeded above SNR high threshold
 *                      SI473X_PROP_FM_RSQ_SNR_HI_THRESHOLD
 * @snrlint:    SNR Detect Low.
 *              true  - Received SNR has fallen below SNR low threshold
 *                      SI473X_PROP_FM_RSQ_SNR_LO_THRESHOLD
 * @rssihint:   RSSI Detect High.
 *              true  - RSSI has exceeded above RSSI high threshold
 *                      SI473X_PROP_FM_RSQ_RSSI_HI_THRESHOLD
 * @rssilint:   RSSI Detect Low.
 *              true  - RSSI has fallen below RSSI low threshold
 *                      SI473X_PROP_FM_RSQ_RSSI_LO_THRESHOLD
 * @smute:      Indicates soft mute is engaged.
 * @pilot:      Indicates stereo pilot presence.
 * @stblend:    Indicates amount of stereo blend in%
 *              (100 = full stereo, 0 = full mono).
 * @freqoff:    Signed frequency offset.
 */
struct si473x_rsq_status_report {
	__u8  bltf;
	__u8  afcrl;
	__u8  valid;
	__u16 readfreq;
	__s8  rssi;
	__s8  snr;
	__s8  mult;
	__u16 readantcap;
	__u8  blendint;
	__u8  multhint, multlint;
	__u8  snrhint,  snrlint;
	__u8  rssihint, rssilint;
	__u8  smute;
	__u8  pilot;
	__u8  stblend;
	__s8  freqoff;
} __packed;

struct si473x_agc_status_report {
	__u8 read_rfagcdis;
	__u8 read_lna_gain_index;
} __packed;

#endif  /* __SI473X_REPORTS_H__ */
