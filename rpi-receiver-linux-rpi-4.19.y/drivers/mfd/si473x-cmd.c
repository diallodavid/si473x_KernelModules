/*
 * drivers/mfd/si473x-cmd.c -- Subroutines implementing command
 * protocol of si473x series of chips
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
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/atomic.h>
#include <linux/i2c.h>
#include <linux/device.h>
#include <linux/gpio.h>

#include "linux/mfd/si473x-core.h"

#include <asm/unaligned.h>

#define msb(x)                  ((u8)((u16) x >> 8))
#define lsb(x)                  ((u8)((u16) x &  0x00FF))

/* POWER_UP Power up device and mode selection. All Devices
 * Query Library ID currently not supported
 */
#define CMD_POWER_UP				0x01
#define CMD_POWER_UP_NARGS			2
#define CMD_POWER_UP_NRESP			1
/* None (FUNC = 0), Seven (FUNC = 15) */

/* GET_REV Returns revision information on the device. All Devices */
#define CMD_GET_REV					0x10
#define CMD_GET_REV_NARGS			0
#define CMD_GET_REV_NRESP			(8 + 1)
/* Fifteen (Si4705/06 only), Eight (Si4704/2x/3x/4x) */

/* POWER_DOWN Power down device. All Devices */
#define CMD_POWER_DOWN				0x11
#define CMD_POWER_DOWN_NRESP		1

/* SET_PROPERTY Sets the value of a property. All Devices */
#define CMD_SET_PROPERTY			0x12
#define CMD_SET_PROPERTY_NARGS		5
#define CMD_SET_PROPERTY_NRESP		1

/* GET_PROPERTY Retrieves a property’s value. All Devices */
#define CMD_GET_PROPERTY			0x13
#define CMD_GET_PROPERTY_NARGS		3
#define CMD_GET_PROPERTY_NRESP		(3 + 1)

/* GET_INT_STATUS Reads interrupt status bits. All Devices */
#define CMD_GET_INT_STATUS			0x14
#define CMD_GET_INT_STATUS_NARGS	0
#define CMD_GET_INT_STATUS_NRESP	1

/* PATCH_ARGS* Reserved command used for patch file downloads. All Devices
 * not supported
 */

/* PATCH_DATA* Reserved command used for patch file downloads. All Devices
 * not supported
 */

/* FM_TUNE_FREQ Selects the FM tuning frequency. All Devices */
#define CMD_FM_TUNE_FREQ			0x20
#define CMD_FM_TUNE_FREQ_NARGS		4
#define CMD_FM_TUNE_FREQ_NRESP		1
#define CMD_FM_TUNE_FREQ_MASK		(0x03)

/* FM_SEEK_START Begins searching for a valid frequency. All Devices */
#define CMD_FM_SEEK_START			0x21
#define CMD_FM_SEEK_START_NARGS		1
#define CMD_FM_SEEK_START_NRESP		1

/* FM_TUNE_STATUS Queries the status of previous FM_TUNE_FREQ
 * or FM_SEEK_START command. All Devices
 */
#define CMD_FM_TUNE_STATUS			0x22
#define CMD_FM_TUNE_STATUS_NARGS	1
#define CMD_FM_TUNE_STATUS_NRESP	(7 + 1)

/* FM_RSQ_STATUS Queries the status of the Received Signal Quality (RSQ)
 * of the current channel. All Devices
 */
#define CMD_FM_RSQ_STATUS			0x23
#define CMD_FM_RSQ_STATUS_NARGS		1
#define CMD_FM_RSQ_STATUS_NRESP		(7 + 1)

/* FM_RDS_STATUS Returns RDS information for current channel
 * and reads an entry from RDS FIFO.
 * Si4705/06, Si4721, Si474x, Si4731/35/37/39, Si4785
 */
#define CMD_FM_RDS_STATUS			0x24
#define CMD_FM_RDS_STATUS_NARGS		1
#define CMD_FM_RDS_STATUS_NRESP		(12 + 1)

/* FM_AGC_STATUS Queries the current AGC settings All Devices */
#define CMD_FM_AGC_STATUS			0x27
#define CMD_FM_AGC_STATUS_NARGS		0
#define CMD_FM_AGC_STATUS_NRESP		(2 + 1)

/* FM_AGC_OVERRIDE Override AGC setting by disabling and
 * forcing it to a fixed value All Devices
 */
#define CMD_FM_AGC_OVERRIDE			0x28
#define CMD_FM_AGC_OVERRIDE_NARGS	2
#define CMD_FM_AGC_OVERRIDE_NRESP	1

/* AM_TUNE_FREQ Tunes to a given AM frequency. All Devices */
#define CMD_AM_TUNE_FREQ			0x40
#define CMD_AM_TUNE_FREQ_NARGS		5
#define CMD_AM_TUNE_FREQ_NRESP		1
#define CMD_AM_TUNE_FREQ_MASK		(0x01)

/* AM_SEEK_START Begins searching for a valid frequency. All Devices */
#define CMD_AM_SEEK_START			0x41
#define CMD_AM_SEEK_START_NARGS		5
#define CMD_AM_SEEK_START_NRESP		1

/* AM_TUNE_STATUS Queries the status of the already issued AM_TUNE_FREQ
 * or AM_SEEK_START command. All Devices
 */
#define CMD_AM_TUNE_STATUS			0x42
#define CMD_AM_TUNE_STATUS_NARGS	1
#define CMD_AM_TUNE_STATUS_NRESP	(7 + 1)

/* AM_RSQ_STATUS Queries the status of the Received Signal Quality (RSQ)
 * for the current channel. All Devices
 */
#define CMD_AM_RSQ_STATUS			0x43
#define CMD_AM_RSQ_STATUS_NARGS		1
#define CMD_AM_RSQ_STATUS_NRESP		(5 + 1)

/* AM_AGC_STATUS Queries the current AGC settings. All Devices */
#define CMD_AM_AGC_STATUS			0x47
#define CMD_AM_AGC_STATUS_NARGS		0
#define CMD_AM_AGC_STATUS_NRESP		(2 + 1)

/* AM_AGC_OVERRIDE Overrides AGC settings by disabling
 * and forcing it to a fixed value. All Devices
 */
#define CMD_AM_AGC_OVERRIDE			0x48
#define CMD_AM_AGC_OVERRIDE_NARGS		2
#define CMD_AM_AGC_OVERRIDE_NRESP		1

/* GPIO_CTL Configures GPO1, 2, and 3 as output or Hi-Z.
 * All Devices except Si4730-A10
 */
#define CMD_GPIO_CTL				0x80
#define CMD_GPIO_CTL_NARGS			1
#define CMD_GPIO_CTL_NRESP			1

/* GPIO_SET Sets GPO1, 2, and 3 output level (low or high).
 * All Devices except Si4730-A10
 */
#define CMD_GPIO_SET				0x80
#define CMD_GPIO_SET_NARGS			1
#define CMD_GPIO_SET_NRESP			1

#define CMD_MAX_ARGS_COUNT			(10)

enum si473x_agc_status_report_bits {
	SI473X_AGC_READ_RFAGCDIS       = BIT(0),
	SI473X_AGC_READ_LNA_GAIN_INDEX = (0x1f),
};

enum si473x_errors {
	SI473X_ERR_BAD_COMMAND			= 0x10,
	SI473X_ERR_BAD_ARG1				= 0x11,
	SI473X_ERR_BAD_ARG2				= 0x12,
	SI473X_ERR_BAD_ARG3				= 0x13,
	SI473X_ERR_BAD_ARG4				= 0x14,
	SI473X_ERR_BUSY					= 0x18,
	SI473X_ERR_BAD_INTERNAL_MEMORY  = 0x20,
	SI473X_ERR_BAD_PATCH			= 0x30,
	SI473X_ERR_BAD_BOOT_MODE		= 0x31,
	SI473X_ERR_BAD_PROPERTY			= 0x40,
};

static int si473x_core_parse_and_nag_about_error(struct si473x_core *core)
{
	int err;
	char *cause;
	u8 buffer[2];

	if (core->revision != SI4731_REVISION_D62) {
		err = si473x_core_i2c_xfer(core, SI473X_I2C_RECV,
					   buffer, sizeof(buffer));
		if (err == sizeof(buffer)) {
			switch (buffer[1]) {
			case SI473X_ERR_BAD_COMMAND:
				cause = "Bad command";
				err = -EINVAL;
				break;
			case SI473X_ERR_BAD_ARG1:
				cause = "Bad argument #1";
				err = -EINVAL;
				break;
			case SI473X_ERR_BAD_ARG2:
				cause = "Bad argument #2";
				err = -EINVAL;
				break;
			case SI473X_ERR_BAD_ARG3:
				cause = "Bad argument #3";
				err = -EINVAL;
				break;
			case SI473X_ERR_BAD_ARG4:
				cause = "Bad argument #4";
				err = -EINVAL;
				break;
			case SI473X_ERR_BUSY:
				cause = "Chip is busy";
				err = -EBUSY;
				break;
			case SI473X_ERR_BAD_INTERNAL_MEMORY:
				cause = "Bad internal memory";
				err = -EIO;
				break;
			case SI473X_ERR_BAD_PATCH:
				cause = "Bad patch";
				err = -EINVAL;
				break;
			case SI473X_ERR_BAD_BOOT_MODE:
				cause = "Bad boot mode";
				err = -EINVAL;
				break;
			case SI473X_ERR_BAD_PROPERTY:
				cause = "Bad property";
				err = -EINVAL;
				break;
			default:
				cause = "Unknown";
				err = -EIO;
			}

			dev_err(&core->client->dev,
				"[Chip error status]: %s\n", cause);
		} else {
			dev_err(&core->client->dev,
				"Failed to fetch error code\n");
			err = (err >= 0) ? -EIO : err;
		}
	} else {
		err = -EIO;
	}

	return err;
}

/**
 * si473x_core_send_command() - sends a command to si473x and waits its
 * response
 * @core:    si473x_device structure for the device we are
 *            communicating with
 * @command:  command id
 * @args:     command arguments we are sending
 * @argn:     actual size of @args
 * @response: buffer to place the expected response from the device
 * @respn:    actual size of @response
 * @usecs:    amount of time to wait before reading the response (in
 *            usecs)
 *
 * Function returns 0 on succsess and negative error code on
 * failure
 */
static int si473x_core_send_command(struct si473x_core *core,
				    const u8 command,
				    const u8 args[],
				    const int argn,
				    u8 resp[],
				    const int respn,
				    const int usecs)
{
	struct i2c_client *client = core->client;
	int err;
	u8  data[CMD_MAX_ARGS_COUNT + 1];

	if (argn > CMD_MAX_ARGS_COUNT) {
		err = -ENOMEM;
		goto exit;
	}

	if (!client->adapter) {
		err = -ENODEV;
		goto exit;
	}

	/* First send the command and its arguments */
	data[0] = command;
	memcpy(&data[1], args, argn);
	dev_dbg(&client->dev, "Command:\n %*ph\n", argn + 1, data);

	err = si473x_core_i2c_xfer(core, SI473X_I2C_SEND,
				   (char *) data, argn + 1);
	if (err != argn + 1) {
		dev_err(&core->client->dev,
			"Error while sending command 0x%02x\n",
			command);
		err = (err >= 0) ? -EIO : err;
		goto exit;
	}
	/* Set CTS to zero only after the command is send to avoid
	 * possible racing conditions when working in polling mode
	 */
	atomic_set(&core->cts, 0);

	/* if (unlikely(command == CMD_POWER_DOWN) */
	if (!wait_event_timeout(core->command,
				atomic_read(&core->cts),
				usecs_to_jiffies(usecs) + 1))
		dev_warn(&core->client->dev,
			 "(%s) [CMD 0x%02x] Answer timeout.\n",
			 __func__, command);

	/*
	 * When working in polling mode, for some reason the tuner will
	 * report CTS bit as being set in the first status byte read,
	 * but all the consequtive ones will return zeros until the
	 * tuner is actually completed the POWER_UP command. To
	 * workaround that we wait for second CTS to be reported
	 */
	if (unlikely(!core->client->irq && command == CMD_POWER_UP)) {
		if (!wait_event_timeout(core->command,
					atomic_read(&core->cts),
					usecs_to_jiffies(usecs) + 1))
			dev_warn(&core->client->dev,
				 "(%s) Power up took too much time.\n",
				 __func__);
	}

	/* Then get the response */
	err = si473x_core_i2c_xfer(core, SI473X_I2C_RECV, resp, respn);
	if (err != respn) {
		dev_err(&core->client->dev,
			"Error while reading response for command 0x%02x\n",
			command);
		err = (err >= 0) ? -EIO : err;
		goto exit;
	}
	dev_dbg(&client->dev, "Response:\n %*ph\n", respn, resp);

	err = 0;

	if (resp[0] & SI473X_ERR) {
		dev_err(&core->client->dev,
			"[CMD 0x%02x] Chip set error flag\n", command);
		err = si473x_core_parse_and_nag_about_error(core);
		goto exit;
	}

	if (!(resp[0] & SI473X_CTS))
		err = -EBUSY;
exit:
	return err;
}

static int si473x_cmd_clear_stc(struct si473x_core *core)
{
	int err;
	struct si473x_rsq_status_args args = {
		.stcack		= true,
	};

	switch (core->power_up_parameters.func) {
	case SI473X_FUNC_FM_RECEIVER:
		err = si473x_core_cmd_fm_rsq_status(core, &args, NULL);
		break;
	case SI473X_FUNC_AM_RECEIVER:
		err = si473x_core_cmd_am_rsq_status(core, &args, NULL);
		break;
	default:
		err = -EINVAL;
	}

	return err;
}

static int si473x_cmd_tune_seek_freq(struct si473x_core *core,
				     uint8_t cmd,
				     const uint8_t args[], size_t argn,
				     uint8_t *resp, size_t respn)
{
	int err;


	atomic_set(&core->stc, 0);
	err = si473x_core_send_command(core, cmd, args, argn, resp, respn,
				       SI473X_TIMEOUT_TUNE);
	if (!err) {
		core->update_interrupt_status = 1;
		wait_event_killable(core->tuning,
				    atomic_read(&core->stc));
		si473x_cmd_clear_stc(core);
	}

	return err;
}

/**
 * si473x_core_cmd_get_rev() - send 'GET_REV' command to the device
 * @core: device to send the command to
 * @info:  struct si473x_get_rev to fill all the information
 *         returned by the command
 *
 * The command requests the firmware and patch version for currently
 * loaded firmware (dependent on the function of the device FM/AM/WB)
 *
 * Function returns 0 on succsess and negative error code on
 * failure
 */
int si473x_core_cmd_get_rev(struct si473x_core *core,
			      struct si473x_get_rev *info)
{
	int err;
	u8  resp[CMD_GET_REV_NRESP];

	err = si473x_core_send_command(core, CMD_GET_REV,
				       NULL, 0,
				       resp, ARRAY_SIZE(resp),
				       SI473X_DEFAULT_TIMEOUT);

	info->partnumber = resp[1];
	info->firmware.major = resp[2];
	info->firmware.minor = resp[3];
	info->patch_id = ((u16) resp[4] << 8) | resp[5];
	info->component_revision.major = resp[6];
	info->component_revision.minor = resp[7];
	info->chip_revision = resp[8];

	return err;
}
EXPORT_SYMBOL_GPL(si473x_core_cmd_get_rev);

/**
 * si473x_core_cmd_set_property() - send 'SET_PROPERTY' command to the device
 * @core:    device to send the command to
 * @property: property address
 * @value:    property value
 *
 * Function returns 0 on succsess and negative error code on
 * failure
 */
int si473x_core_cmd_set_property(struct si473x_core *core,
				 u16 property, u16 value)
{
	u8       resp[CMD_SET_PROPERTY_NRESP];
	const u8 args[CMD_SET_PROPERTY_NARGS] = {
		0x00,
		msb(property),
		lsb(property),
		msb(value),
		lsb(value),
	};

	return si473x_core_send_command(core, CMD_SET_PROPERTY,
					args, ARRAY_SIZE(args),
					resp, ARRAY_SIZE(resp),
					SI473X_DEFAULT_TIMEOUT);
}
EXPORT_SYMBOL_GPL(si473x_core_cmd_set_property);

/**
 * si473x_core_cmd_get_property() - send 'GET_PROPERTY' command to the device
 * @core:    device to send the command to
 * @property: property address
 *
 * Function return the value of property as u16 on success or a
 * negative error on failure
 */
int si473x_core_cmd_get_property(struct si473x_core *core, u16 property)
{
	int err;
	u8       resp[CMD_GET_PROPERTY_NRESP];
	const u8 args[CMD_GET_PROPERTY_NARGS] = {
		0x00,
		msb(property),
		lsb(property),
	};

	err = si473x_core_send_command(core, CMD_GET_PROPERTY,
				       args, ARRAY_SIZE(args),
				       resp, ARRAY_SIZE(resp),
				       SI473X_DEFAULT_TIMEOUT);
	if (err < 0)
		return err;
	else
		return get_unaligned_be16(resp + 2);
}
EXPORT_SYMBOL_GPL(si473x_core_cmd_get_property);

/**
 * si473x_core_cmd_am_rsq_status - send 'AM_RSQ_STATUS' command to the
 * device
 * @core  - device to send the command to
 * @rsqack - if set command clears RSQINT, SNRINT, SNRLINT, RSSIHINT,
 *           RSSSILINT, BLENDINT, MULTHINT and MULTLINT
 * @attune - when set the values in the status report are the values
 *           that were calculated at tune
 * @cancel - abort ongoing seek/tune opertation
 * @stcack - clear the STCINT bin in status register
 * @report - all signal quality information retured by the command
 *           (if NULL then the output of the command is ignored)
 *
 * Function returns 0 on success and negative error code on failure
 */
int si473x_core_cmd_am_rsq_status(struct si473x_core *core,
				  struct si473x_rsq_status_args *rsqargs,
				  struct si473x_rsq_status_report *report)
{
	int err;
	u8       resp[CMD_AM_RSQ_STATUS_NRESP];
	const u8 args[CMD_AM_RSQ_STATUS_NARGS] = {
		rsqargs->stcack,
	};
	u8       resp_tune[CMD_AM_TUNE_STATUS_NRESP];
	const u8 args_tune[CMD_AM_TUNE_STATUS_NARGS] = {
		rsqargs->stcack,
	};

	err = si473x_core_send_command(core, CMD_AM_RSQ_STATUS,
				       args, ARRAY_SIZE(args),
				       resp, ARRAY_SIZE(resp),
				       SI473X_DEFAULT_TIMEOUT);
	/*
	 * Besides getting received signal quality information this
	 * command can be used to just acknowledge different interrupt
	 * flags in those cases it is useless to copy and parse
	 * received data so user can pass NULL, and thus avoid
	 * unnecessary copying.
	 */
	if (!report)
		return err;

	report->snrhint		= 0x08 & resp[1];
	report->snrlint		= 0x04 & resp[1];
	report->rssihint	= 0x02 & resp[1];
	report->rssilint	= 0x01 & resp[1];

	report->smute		= 0x08 & resp[2];
	report->afcrl		= 0x02 & resp[2];
	report->valid		= 0x01 & resp[2];

	report->rssi		= resp[4];
	report->snr			= resp[5];


	err = si473x_core_send_command(core, CMD_AM_TUNE_STATUS,
				       args_tune, ARRAY_SIZE(args_tune),
				       resp_tune, ARRAY_SIZE(resp_tune),
				       SI473X_DEFAULT_TIMEOUT);
	if (err < 0)
		return err;

	report->bltf		= 0x80 & resp_tune[1];

	report->readfreq	= resp_tune[2] << 8 | resp_tune[3];
	report->readantcap	= resp_tune[6] << 8 | resp_tune[7];

	return err;
}
EXPORT_SYMBOL_GPL(si473x_core_cmd_am_rsq_status);

/**
 * si473x_core_cmd_fm_seek_start - send 'FM_SEEK_START' command to the
 * device
 * @core  - device to send the command to
 * @seekup - if set the direction of the search is 'up'
 * @wrap   - if set seek wraps when hitting band limit
 *
 * This function begins search for a valid station. The station is
 * considered valid when 'FM_VALID_SNR_THRESHOLD' and
 * 'FM_VALID_RSSI_THRESHOLD' and 'FM_VALID_MAX_TUNE_ERROR' criteria
 * are met.
} *
 * Function returns 0 on success and negative error code on failure
 */
int si473x_core_cmd_fm_seek_start(struct si473x_core *core,
				  bool seekup, bool wrap)
{
	u8       resp[CMD_FM_SEEK_START_NRESP];
	const u8 args[CMD_FM_SEEK_START_NARGS] = {
		seekup << 3 | wrap << 2,
	};

	return si473x_cmd_tune_seek_freq(core, CMD_FM_SEEK_START,
					 args, sizeof(args),
					 resp, sizeof(resp));
}
EXPORT_SYMBOL_GPL(si473x_core_cmd_fm_seek_start);

/**
 * si473x_core_cmd_fm_rds_status - send 'FM_RDS_STATUS' command to the
 * device
 * @core - device to send the command to
 * @status_only - if set the data is not removed from RDSFIFO,
 *                RDSFIFOUSED is not decremented and data in all the
 *                rest RDS data contains the last valid info received
 * @mtfifo if set the command clears RDS receive FIFO
 * @intack if set the command clards the RDSINT bit.
 *
 * Function returns 0 on success and negative error code on failure
 */
int si473x_core_cmd_fm_rds_status(struct si473x_core *core,
				  bool status_only,
				  bool mtfifo,
				  bool intack,
				  struct si473x_rds_status_report *report)
{
	int err;
	u8       resp[CMD_FM_RDS_STATUS_NRESP];
	const u8 args[CMD_FM_RDS_STATUS_NARGS] = {
		status_only << 2 | mtfifo << 1 | intack,
	};

	err = si473x_core_send_command(core, CMD_FM_RDS_STATUS,
				       args, ARRAY_SIZE(args),
				       resp, ARRAY_SIZE(resp),
				       SI473X_DEFAULT_TIMEOUT);
	/*
	 * Besides getting RDS status information this command can be
	 * used to just acknowledge different interrupt flags in those
	 * cases it is useless to copy and parse received data so user
	 * can pass NULL, and thus avoid unnecessary copying.
	 */
	if (err < 0 || report == NULL)
		return err;

	report->rdsnewblockb	= 0x20 & resp[1];
	report->rdsnewblocka	= 0x10 & resp[1];
	report->rdssyncfound	= 0x04 & resp[1];
	report->rdssynclost		= 0x02 & resp[1];
	report->rdsrecv			= 0x01 & resp[1];

	report->grplost		= 0x04 & resp[2];
	report->rdssync		= 0x01 & resp[2];

	report->rdsfifoused	= resp[3];

	report->rds[V4L2_RDS_BLOCK_A].block = V4L2_RDS_BLOCK_A;
	report->rds[V4L2_RDS_BLOCK_A].msb = resp[4];
	report->rds[V4L2_RDS_BLOCK_A].lsb = resp[5];

	report->rds[V4L2_RDS_BLOCK_B].block = V4L2_RDS_BLOCK_B;
	report->rds[V4L2_RDS_BLOCK_B].msb = resp[6];
	report->rds[V4L2_RDS_BLOCK_B].lsb = resp[7];

	report->rds[V4L2_RDS_BLOCK_C].block = V4L2_RDS_BLOCK_C;
	report->rds[V4L2_RDS_BLOCK_C].msb = resp[8];
	report->rds[V4L2_RDS_BLOCK_C].lsb = resp[9];

	report->rds[V4L2_RDS_BLOCK_D].block = V4L2_RDS_BLOCK_D;
	report->rds[V4L2_RDS_BLOCK_D].msb = resp[10];
	report->rds[V4L2_RDS_BLOCK_D].lsb = resp[11];

	report->ble[V4L2_RDS_BLOCK_A]	= 0xc0 & resp[12];
	report->ble[V4L2_RDS_BLOCK_B]	= 0x30 & resp[12];
	report->ble[V4L2_RDS_BLOCK_C]	= 0x0c & resp[12];
	report->ble[V4L2_RDS_BLOCK_D]	= 0x03 & resp[12];

	return err;
}
EXPORT_SYMBOL_GPL(si473x_core_cmd_fm_rds_status);

/**
 * si473x_core_cmd_am_seek_start - send 'AM_SEEK_START' command to the
 * device
 * @core  - device to send the command to
 * @seekup - if set the direction of the search is 'up'
 * @wrap   - if set seek wraps when hitting band limit
 *
 * This function begins search for a valid station. The station is
 * considered valid when 'AM_VALID_SNR_THRESHOLD' and
 * 'AM_VALID_RSSI_THRESHOLD' and 'AM_VALID_MAX_TUNE_ERROR' criteria
 * are met.
 *
 * Function returns 0 on success and negative error code on failure
 */
int si473x_core_cmd_am_seek_start(struct si473x_core *core,
				  bool seekup, bool wrap)
{
	u8       resp[CMD_AM_SEEK_START_NRESP];
	const u8 args[CMD_AM_SEEK_START_NARGS] = {
		seekup << 3 | wrap << 2,
	};

	return si473x_cmd_tune_seek_freq(core,  CMD_AM_SEEK_START,
					 args, sizeof(args),
					 resp, sizeof(resp));
}
EXPORT_SYMBOL_GPL(si473x_core_cmd_am_seek_start);

static int si473x_core_cmd_power_up_d62(struct si473x_core *core,
				 struct si473x_power_up_args *puargs)
{
	u8    resp[CMD_POWER_UP_NRESP];
	const bool ctsen  = (core->client->irq != 0);
	const u8 args[CMD_POWER_UP_NARGS] = {
		ctsen << 7 | ctsen << 6 | puargs->func, // Enable interrupts
								puargs->mode,
	};

	return si473x_core_send_command(core, CMD_POWER_UP,
					args, ARRAY_SIZE(args),
					resp, ARRAY_SIZE(resp),
					SI473X_TIMEOUT_POWER_UP);
}

static int si473x_core_cmd_power_down_d62(struct si473x_core *core)
{
	u8 resp[CMD_POWER_DOWN_NRESP];

	return si473x_core_send_command(core, CMD_POWER_DOWN,
					NULL, 0,
					resp, ARRAY_SIZE(resp),
					SI473X_DEFAULT_TIMEOUT);
}

int si473x_core_cmd_am_tune_freq(struct si473x_core *core,
					struct si473x_tune_freq_args *tuneargs)
{
	const int am_freq = tuneargs->freq;
	u8       resp[CMD_AM_TUNE_FREQ_NRESP];
	const u8 args[CMD_AM_TUNE_FREQ_NARGS] = {
		(tuneargs->tunemode & CMD_AM_TUNE_FREQ_MASK),
		msb(am_freq),
		lsb(am_freq),
	};

	return si473x_cmd_tune_seek_freq(core, CMD_AM_TUNE_FREQ,
					 args, sizeof(args),
					 resp, sizeof(resp));
}
EXPORT_SYMBOL_GPL(si473x_core_cmd_am_tune_freq);

int si473x_core_cmd_fm_rsq_status(struct si473x_core *core,
					struct si473x_rsq_status_args *rsqargs,
					struct si473x_rsq_status_report *report)
{
	int err;
	u8       resp[CMD_FM_RSQ_STATUS_NRESP];
	const u8 args[CMD_FM_RSQ_STATUS_NARGS] = {
		rsqargs->stcack,
	};
	u8       resp_tune[CMD_FM_TUNE_STATUS_NRESP];
	const u8 args_tune[CMD_FM_TUNE_STATUS_NARGS] = {
		rsqargs->stcack,
	};

	err = si473x_core_send_command(core, CMD_FM_RSQ_STATUS,
				       args, ARRAY_SIZE(args),
				       resp, ARRAY_SIZE(resp),
				       SI473X_DEFAULT_TIMEOUT);
	/*
	 * Besides getting received signal quality information this
	 * command can be used to just acknowledge interrupt
	 * flags in those cases it is useless to copy and parse
	 * received data so user can pass NULL, and thus avoid
	 * unnecessary copying.
	 */
	if (err < 0 || report == NULL)
		return err;

	report->blendint	= 0x80 & resp[1];
	report->multhint	= 0x20 & resp[1];
	report->multlint	= 0x10 & resp[1];
	report->snrhint		= 0x08 & resp[1];
	report->snrlint		= 0x04 & resp[1];
	report->rssihint	= 0x02 & resp[1];
	report->rssilint	= 0x01 & resp[1];

	report->smute		= 0x08 & resp[2];
	report->afcrl		= 0x02 & resp[2];
	report->valid		= 0x01 & resp[2];

	report->pilot		= 0x80 & resp[3];
	report->stblend		= 0x7f & resp[3];

	report->rssi		= resp[4];
	report->snr			= resp[5];
	report->mult		= resp[6];
	report->freqoff		= resp[7];

	err = si473x_core_send_command(core, CMD_FM_TUNE_STATUS,
				       args_tune, ARRAY_SIZE(args_tune),
				       resp_tune, ARRAY_SIZE(resp_tune),
				       SI473X_DEFAULT_TIMEOUT);
	if (err < 0)
		return err;

	report->bltf		= 0x80 & resp_tune[1];

	report->readfreq	= resp_tune[2] << 8 | resp_tune[3];
	report->readantcap	= resp_tune[7];

	return err;
}
EXPORT_SYMBOL_GPL(si473x_core_cmd_fm_rsq_status);

int si473x_core_cmd_fm_tune_freq(struct si473x_core *core,
					struct si473x_tune_freq_args *tuneargs)
{
	u8       resp[CMD_FM_TUNE_FREQ_NRESP];
	const u8 args[CMD_FM_TUNE_FREQ_NARGS] = {
		(tuneargs->tunemode & CMD_FM_TUNE_FREQ_MASK),
		msb(tuneargs->freq),
		lsb(tuneargs->freq),
	};

	return si473x_cmd_tune_seek_freq(core, CMD_FM_TUNE_FREQ,
					 args, sizeof(args),
					 resp, sizeof(resp));
}
EXPORT_SYMBOL_GPL(si473x_core_cmd_fm_tune_freq);

int si473x_core_cmd_agc_status(struct si473x_core *core,
				  struct si473x_agc_status_report *report)
{
	int err;
	u8 resp[CMD_FM_AGC_STATUS_NRESP];

	if (!report)
		return -EINVAL;

	err = si473x_core_send_command(core, CMD_FM_AGC_STATUS,
				       NULL, 0,
				       resp, ARRAY_SIZE(resp),
				       SI473X_DEFAULT_TIMEOUT);
	if (err < 0)
		return err;

	report->read_rfagcdis       = resp[1] & SI473X_AGC_READ_RFAGCDIS;
	report->read_lna_gain_index = resp[2] & SI473X_AGC_READ_LNA_GAIN_INDEX;

	return err;
}
EXPORT_SYMBOL_GPL(si473x_core_cmd_agc_status);

typedef int (*tune_freq_func_t) (struct si473x_core *core,
				 struct si473x_tune_freq_args *tuneargs);

static struct {
	int (*power_up)(struct si473x_core *core,
			struct si473x_power_up_args *args);
	int (*power_down)(struct si473x_core *core);

} si473x_cmds_vtable[] = {
	[SI4731_REVISION_D62] = {
		.power_up	= si473x_core_cmd_power_up_d62,
		.power_down	= si473x_core_cmd_power_down_d62,
	},
};

int si473x_core_cmd_power_up(struct si473x_core *core,
			     struct si473x_power_up_args *args)
{
	return si473x_cmds_vtable[core->revision].power_up(core, args);
}
EXPORT_SYMBOL_GPL(si473x_core_cmd_power_up);

int si473x_core_cmd_power_down(struct si473x_core *core)
{
	return si473x_cmds_vtable[core->revision].power_down(core);
}
EXPORT_SYMBOL_GPL(si473x_core_cmd_power_down);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("rpi Receiver <rpi-receiver@htl-steyr.ac.at>");
MODULE_DESCRIPTION("API for command exchange for si473x");
