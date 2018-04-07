/*
 * drivers/mfd/si473x-i2c.c -- Core device driver for si473x MFD
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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the GNU
 * General Public License for more details.
 *
 */
#include <linux/module.h>

#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/of_gpio.h>

#include "si473x-core.h"

#define SI473X_MAX_IO_ERRORS			10
#define SI473X_DRIVER_RDS_FIFO_DEPTH	128
/* GET_INT_STATUS Reads interrupt status bits. All Devices */
#define CMD_GET_INT_STATUS				0x14

static inline void si473x_core_schedule_polling_work(struct si473x_core *core)
{
	schedule_delayed_work(&core->status_monitor,
				  usecs_to_jiffies(SI473X_STATUS_POLL_US));
}

/**
 * si473x_core_start() - early chip startup function
 * @core: Core device structure
 * @soft: When set, this flag forces "soft" startup, where "soft"
 * power down is the one done by sending appropriate command instead
 * of using reset pin of the tuner
 *
 * Perform required startup sequence to correctly power
 * up the chip and perform initial configuration. It does the
 * following sequence of actions:
 *		 1. Claims and enables the power supplies VD and VIO1 required
 *			for I2C interface of the chip operation.
 *		 2. Waits for 100us, pulls the reset line up, enables irq,
 *			waits for another 100us as it is specified by the
 *			datasheet.
 *		 3. Sends 'POWER_UP' command to the device with all provided
 *			information about power-up parameters.
 *		 4. Configures, pin multiplexor, disables digital audio and
 *			configures interrupt sources.
 *
 * The function returns zero in case of succes or negative error code
 * otherwise.
 */
int si473x_core_start(struct si473x_core *core, bool soft)
{
	struct i2c_client *client = core->client;
	int err;

	if (!soft) {
		if (gpio_is_valid(core->gpio_reset))
			gpio_set_value_cansleep(core->gpio_reset, 1);

		if (client->irq)
			enable_irq(client->irq);

		udelay(100);

		if (!client->irq) {
			atomic_set(&core->is_alive, 1);
			si473x_core_schedule_polling_work(core);
		}
	} else {
		if (client->irq)
			enable_irq(client->irq);
		else {
			atomic_set(&core->is_alive, 1);
			si473x_core_schedule_polling_work(core);
		}
	}

	err = si473x_core_cmd_power_up(core,
					   &core->power_up_parameters);

	if (err < 0) {
		dev_err(&core->client->dev,
			"Power up failure(err = %d)\n",
			err);
		goto disable_irq;
	}

	if (client->irq)
		atomic_set(&core->is_alive, 1);
	regcache_cache_only(core->regmap, false);

	if (client->irq) {
		err = regmap_write(core->regmap,
				   SI473X_PROP_GPO_IEN,
				   // SI473X_RSQIEN | // Received Signal Quality
				   // SI473X_RSQREP | // RSQ Interrupt Repeat
				   SI473X_RDSIEN | // RDS Interrupt Enable
				   // SI473X_RDSREP | // RDS Interrupt Repeat
				   // SI473X_ASQIEN | // ASQ Interrupt Enable
				   // SI473X_ASQREP | // ASQ Interrupt Repeat
				   SI473X_STCIEN | // Seek/Tune Complete
				   // SI473X_STCREP | // STC Interrupt Repeat
				   // SI473X_ERRIEN | // Error
				   SI473X_CTSIEN); // Clear to Send next command
		if (err < 0) {
			dev_err(&core->client->dev,
				"Failed to configure interrupt sources (err = %d)\n",
				err);
			goto disable_irq;
		}
	}

	return 0;

disable_irq:
	if (err == -ENODEV)
		atomic_set(&core->is_alive, 0);

	if (client->irq)
		disable_irq(client->irq);
	else
		cancel_delayed_work_sync(&core->status_monitor);

	if (gpio_is_valid(core->gpio_reset))
		gpio_set_value_cansleep(core->gpio_reset, 0);

	return err;
}
EXPORT_SYMBOL_GPL(si473x_core_start);

/**
 * si473x_core_stop() - chip power-down function
 * @core: Core device structure
 * @soft: When set, function sends a POWER_DOWN command instead of
 * bringing reset line low
 *
 * Power down the chip by performing following actions:
 * 1. Disable IRQ or stop the polling worker
 * 2. Send the POWER_DOWN command if the power down is soft or bring
 *	  reset line low if not.
 *
 * The function returns zero in case of succes or negative error code
 * otherwise.
 */
int si473x_core_stop(struct si473x_core *core, bool soft)
{
	int err = 0;

	atomic_set(&core->is_alive, 0);

	if (soft)
		err = si473x_core_cmd_power_down(core);
	/* We couldn't disable those before
	 * 'si473x_core_cmd_power_down' since we expect to get CTS
	 * interrupt
	 */
	if (core->client->irq)
		disable_irq(core->client->irq);
	else
		cancel_delayed_work_sync(&core->status_monitor);

	if (!soft) {
		if (gpio_is_valid(core->gpio_reset))
			gpio_set_value_cansleep(core->gpio_reset, 0);
	}
	regcache_mark_dirty(core->regmap);
	regcache_cache_only(core->regmap, true);

	return err;
}
EXPORT_SYMBOL_GPL(si473x_core_stop);

/**
 * si473x_core_set_power_state() - set the level at which the power is
 * supplied for the chip.
 * @core: Core device structure
 * @next_state: enum si473x_power_state describing power state to
 *				switch to.
 *
 * Switch on all the required power supplies
 *
 * This function returns 0 in case of suvccess and negative error code
 * otherwise.
 */
int si473x_core_set_power_state(struct si473x_core *core,
				enum si473x_power_state next_state)
{
	/*
	 * It is not clear form the datasheet if it is possible to
	 * work with device if not all power domains are operational.
	 * So for now the power-up policy is "power-up all the things!"
	 */
	int err = 0;

	if (next_state != core->power_state) {
		switch (next_state) {
		case SI473X_POWER_UP_FULL:
			/*
			 * Startup timing diagram recommends to have a
			 * 100 us delay between enabling of the power
			 * supplies and turning the tuner on.
			 */
			if (core->gpio_reset >= 0) {
				gpio_set_value(core->gpio_reset, 0);
				usleep_range(1000, 1500);
				gpio_set_value(core->gpio_reset, 1);
				usleep_range(1000, 1500);
			}

			err = si473x_core_start(core, false);
			if (err < 0)
				goto reset_si473x;

			core->power_state = next_state;
			break;

		case SI473X_POWER_DOWN:
			core->power_state = next_state;
			err = si473x_core_stop(core, false);
reset_si473x:
			if (core->gpio_reset >= 0) {
				gpio_set_value(core->gpio_reset, 0);
				usleep_range(1000, 1500);
			}
			if (err < 0)
				core->power_state = SI473X_POWER_DOWN;
			break;
		default:
			BUG();
		}
	}

	return err;
}
EXPORT_SYMBOL_GPL(si473x_core_set_power_state);

/**
 * si473x_core_report_drainer_stop() - mark the completion of the RDS
 * buffer drain porcess by the worker.
 *
 * @core: Core device structure
 */
static inline void si473x_core_report_drainer_stop(struct si473x_core *core)
{
	mutex_lock(&core->rds_drainer_status_lock);
	core->rds_drainer_is_working = false;
	mutex_unlock(&core->rds_drainer_status_lock);
}

/**
 * si473x_core_start_rds_drainer_once() - start RDS drainer worker if
 * there is none working, do nothing otherwise
 *
 * @core: Datastructure corresponding to the chip.
 */
static inline void si473x_core_start_rds_drainer_once(struct si473x_core *core)
{
	mutex_lock(&core->rds_drainer_status_lock);
	if (!core->rds_drainer_is_working) {
		core->rds_drainer_is_working = true;
		schedule_work(&core->rds_fifo_drainer);
	}
	mutex_unlock(&core->rds_drainer_status_lock);
}
/**
 * si473x_drain_rds_fifo() - RDS buffer drainer.
 * @work: struct work_struct being ppassed to the function by the
 * kernel.
 *
 * Drain the contents of the RDS FIFO of
 */
static void si473x_core_drain_rds_fifo(struct work_struct *work)
{
	int err;

	struct si473x_core *core = container_of(work, struct si473x_core,
						rds_fifo_drainer);

	struct si473x_rds_status_report report;

	if (si473x_core_is_in_am_receiver_mode(core))
		return;
	si473x_core_lock(core);
	err = si473x_core_cmd_fm_rds_status(core, true, false, false, &report);
	if (!err) {
		int i = report.rdsfifoused;

		dev_dbg(&core->client->dev,
			"%d elements in RDS FIFO. Draining.\n", i);
		for (; i > 0; --i) {
			err = si473x_core_cmd_fm_rds_status(core, false, false,
				(i == 1), &report);
			if (err < 0)
				goto unlock;

			kfifo_in(&core->rds_fifo, report.rds,
				 sizeof(report.rds));
			dev_dbg(&core->client->dev, "RDS data:\n %*ph\n",
				(int)sizeof(report.rds), report.rds);
		}
		dev_dbg(&core->client->dev, "Drrrrained!\n");
		wake_up_interruptible(&core->rds_read_queue);
	}

unlock:
	si473x_core_unlock(core);
	si473x_core_report_drainer_stop(core);
}

/**
 * si473x_core_pronounce_dead()
 *
 * @core: Core device structure
 *
 * Mark the device as being dead and wake up all potentially waiting
 * threads of execution.
 *
 */
static void si473x_core_pronounce_dead(struct si473x_core *core)
{
	dev_info(&core->client->dev, "Core device is dead.\n");

	atomic_set(&core->is_alive, 0);

	/* Wake up al possible waiting processes */
	wake_up_interruptible(&core->rds_read_queue);

	atomic_set(&core->cts, 1);
	wake_up(&core->command);

	atomic_set(&core->stc, 1);
	wake_up(&core->tuning);
}

/**
 * si473x_core_i2c_xfer()
 *
 * @core: Core device structure
 * @type: Transfer type
 * @buf: Transfer buffer for/with data
 * @count: Transfer buffer size
 *
 * Perfrom and I2C transfer(either read or write) and keep a counter
 * of I/O errors. If the error counter rises above the threshold
 * pronounce device dead.
 *
 * The function returns zero on succes or negative error code on
 * failure.
 */
int si473x_core_i2c_xfer(struct si473x_core *core,
			enum si473x_i2c_type type,
			char *buf, int count)
{
	static int io_errors_count;
	int err;

	if (type == SI473X_I2C_SEND)
		err = i2c_master_send(core->client, buf, count);
	else
		err = i2c_master_recv(core->client, buf, count);

	if (err < 0) {
		if (io_errors_count++ > SI473X_MAX_IO_ERRORS)
			si473x_core_pronounce_dead(core);
	} else {
		io_errors_count = 0;
	}

	return err;
}
EXPORT_SYMBOL_GPL(si473x_core_i2c_xfer);

/**
 * si473x_get_status()
 * @core: Core device structure
 *
 * Get the status byte of the core device by berforming one byte I2C
 * read.
 *
 * The function returns a status value or a negative error code on
 * error.
 */
static int si473x_core_get_status(struct si473x_core *core)
{
	u8 data;
	u8 response;
	int err;

	data = CMD_GET_INT_STATUS;
	/* This command should be called after any command that sets the
	 * STCINT, RDSINT, or RSQINT bits
	 * this command should be called after the interrupt
	 * is set to update the STATUS byte
	 */
	if (core->update_interrupt_status) {
		err = si473x_core_i2c_xfer(core, SI473X_I2C_SEND,
			&data, sizeof(data));
		core->update_interrupt_status = 0;
	}
	err = si473x_core_i2c_xfer(core, SI473X_I2C_RECV,
				  &response, sizeof(response));

	return (err < 0) ? err : response;
}

/**
 * si473x_get_and_signal_status() - IRQ dispatcher
 * @core: Core device structure
 *
 * Dispatch the arrived interrupt request based on the value of the
 * status byte reported by the tuner.
 *
 */
static void si473x_core_get_and_signal_status(struct si473x_core *core)
{
	int status = si473x_core_get_status(core);

	if (status < 0) {
		dev_err(&core->client->dev, "Failed to get status\n");
		return;
	}

	if (status & SI473X_CTS) {
		/* Unfortunately completions could not be used for
		 * signalling CTS since this flag cannot be cleared
		 * in status byte, and therefore once it becomes true
		 * multiple calls to 'complete' would cause the
		 * commands following the current one to be completed
		 * before they actually are
		 */
		dev_dbg(&core->client->dev, "[interrupt] CTSINT\n");
		atomic_set(&core->cts, 1);
		wake_up(&core->command);
	}

	if (status & SI473X_FM_RDS_INT) {
		dev_dbg(&core->client->dev, "[interrupt] RDSINT\n");
		si473x_core_start_rds_drainer_once(core);
	}

	if (status & SI473X_STC_INT) {
		dev_dbg(&core->client->dev, "[interrupt] STCINT\n");
		atomic_set(&core->stc, 1);
		wake_up(&core->tuning);
	}
}

static void si473x_core_poll_loop(struct work_struct *work)
{
	struct si473x_core *core = SI473X_WORK_TO_CORE(work);

	si473x_core_get_and_signal_status(core);

	if (atomic_read(&core->is_alive))
		si473x_core_schedule_polling_work(core);
}

static irqreturn_t si473x_core_interrupt(int irq, void *dev)
{
	struct si473x_core *core = dev;

	si473x_core_get_and_signal_status(core);

	return IRQ_HANDLED;
}

/**
 * si473x_firmware_version_to_revision()
 * @core: Core device structure
 * @major: Firmware major number
 * @minor: Firmware minor number
 *
 * Convert a chip's firmware version number into an offset that later
 * will be used to as offset in "vtable" of tuner functions
 *
 * This function returns a positive offset in case of success and a -1
 * in case of failure.
 */
static int si473x_core_fwver_to_revision(struct si473x_core *core,
					 int partnumber, int major, int minor)
{
	switch (partnumber) {
	case 31: // SI4731
		switch (major) {
		case '6':
			switch (minor) {
			case '2': // Firmware Revision "40"
				return SI4731_REVISION_D62;
			default:
				goto unknown_revision;
			}
		default:
			goto unknown_revision;
		}
	default:		/* FALLTHROUG */
		BUG();
		return -1;
	}

unknown_revision:
	dev_err(&core->client->dev,
		"Unsupported version of the firmware: %d.%d\n",
		major, minor);

	return SI4731_REVISION_D62;
}

/**
 * si473x_get_revision_info()
 * @core: Core device structure
 *
 * Get the firmware version number of the device. It is done in
 * following three steps:
 *	  1. Power-up the device
 *	  2. Send the 'GET_REV' command
 *	  3. Powering the device down.
 *
 * The function return zero on success and a negative error code on
 * failure.
 */
static int si473x_core_get_revision_info(struct si473x_core *core)
{
	int rval;
	struct si473x_get_rev info;

	si473x_core_lock(core);
	rval = si473x_core_set_power_state(core, SI473X_POWER_UP_FULL);
	if (rval < 0)
		goto exit;

	rval = si473x_core_cmd_get_rev(core, &info);
	if (rval < 0)
		goto power_down;
	core->revision = si473x_core_fwver_to_revision(core,
							   info.partnumber,
							   info.firmware.major,
							   info.firmware.minor);
power_down:
	si473x_core_set_power_state(core, SI473X_POWER_DOWN);
exit:
	si473x_core_unlock(core);

	return rval;
}

bool si473x_core_has_am(struct si473x_core *core)
{
	return core->chip_id == SI473X_CHIP_SI4731;
}
EXPORT_SYMBOL_GPL(si473x_core_has_am);

bool si473x_core_is_in_am_receiver_mode(struct si473x_core *core)
{
	return si473x_core_has_am(core) &&
		(core->power_up_parameters.func == SI473X_FUNC_AM_RECEIVER);
}
EXPORT_SYMBOL_GPL(si473x_core_is_in_am_receiver_mode);

bool si473x_core_is_powered_up(struct si473x_core *core)
{
	return core->power_state == SI473X_POWER_UP_FULL;
}
EXPORT_SYMBOL_GPL(si473x_core_is_powered_up);

static int si473x_i2c_probe(struct i2c_client *client,
				 const struct i2c_device_id *id)
{
	int rval;
	struct si473x_core			*core;
	struct mfd_cell *cell;
	int				 cell_num;

	core = devm_kzalloc(&client->dev, sizeof(*core), GFP_KERNEL);
	if (!core) {
		dev_err(&client->dev,
			"failed to allocate 'struct si473x_core'\n");
		return -ENOMEM;
	}
	core->client = client;

	core->regmap = devm_regmap_init_si473x(core);
	if (IS_ERR(core->regmap)) {
		rval = PTR_ERR(core->regmap);
		dev_err(&client->dev,
			"Failed to allocate register map: %d\n",
			rval);
		return rval;
	}

	i2c_set_clientdata(client, core);

	atomic_set(&core->is_alive, 0);
	atomic_set(&core->stc, 1);
	core->power_state = SI473X_POWER_DOWN;

	if (client->dev.of_node) {
		core->gpio_reset = of_get_gpio((&client->dev)->of_node, 0);
		if (core->gpio_reset == -EPROBE_DEFER) {
			dev_err(&client->dev, "cannot get SI473X reset\n");
			return -EPROBE_DEFER;
		}
		if (gpio_is_valid(core->gpio_reset)) {
			rval = devm_gpio_request_one(&client->dev,
			core->gpio_reset, GPIOF_OUT_INIT_LOW, "SI473X reset");
			if (rval) {
				dev_err(&client->dev, "cannot request SI473X reset\n");
				return rval;
			}
			gpio_direction_output(core->gpio_reset, 0);
		}
	} else {
		dev_err(&client->dev, "No devicetree data provided\n");
		return -EINVAL;
	}

	mutex_init(&core->cmd_lock);
	init_waitqueue_head(&core->command);
	init_waitqueue_head(&core->tuning);

	rval = kfifo_alloc(&core->rds_fifo,
			   SI473X_DRIVER_RDS_FIFO_DEPTH *
			   sizeof(struct v4l2_rds_data),
			   GFP_KERNEL);
	if (rval) {
		dev_err(&client->dev, "Could not allocate the FIFO\n");
		goto free_gpio;
	}
	mutex_init(&core->rds_drainer_status_lock);
	init_waitqueue_head(&core->rds_read_queue);
	INIT_WORK(&core->rds_fifo_drainer, si473x_core_drain_rds_fifo);

	rval = request_threaded_irq(client->irq, NULL,
					 si473x_core_interrupt,
					 IRQF_TRIGGER_RISING | IRQF_ONESHOT,
					 client->name, core);
	if (client->irq) {
		if (rval < 0) {
			dev_err(&client->dev, "Could not request IRQ %d\n",
				client->irq);
			goto free_kfifo;
		}
		disable_irq(client->irq);
		dev_dbg(&client->dev, "IRQ requested.\n");

		core->rds_fifo_depth = 20;
	} else {
		INIT_DELAYED_WORK(&core->status_monitor,
				  si473x_core_poll_loop);
		dev_info(&client->dev,
			 "No IRQ number specified, will use polling\n");

		core->rds_fifo_depth = 5;
	}

	core->chip_id = id->driver_data;

	rval = si473x_core_get_revision_info(core);
	if (rval < 0) {
		rval = -ENODEV;
		goto free_kfifo;
	}

	cell_num = 0;

	cell = &core->cells[SI473X_RADIO_CELL];
	cell->name = "si473x-radio";
	cell->of_compatible = "si,si473x-radio";
	cell_num++;
	cell = &core->cells[SI473X_CODEC_CELL];
	cell->name = "si473x-codec";
	cell->of_compatible = "si,si473x-codec";
	cell_num++;

	rval = mfd_add_devices(&client->dev,
				   (client->adapter->nr << 8) + client->addr,
				   core->cells, cell_num,
				   NULL, 0, NULL);
	if (!rval)
		return 0;

free_kfifo:
	kfifo_free(&core->rds_fifo);

free_gpio:
	if (gpio_is_valid(core->gpio_reset))
		gpio_free(core->gpio_reset);

	return rval;
}

static int si473x_i2c_remove(struct i2c_client *client)
{
	struct si473x_core *core = i2c_get_clientdata(client);

	si473x_core_pronounce_dead(core);
	mfd_remove_devices(&client->dev);

	if (client->irq)
		disable_irq(client->irq);
	else
		cancel_delayed_work_sync(&core->status_monitor);

	kfifo_free(&core->rds_fifo);

	if (gpio_is_valid(core->gpio_reset))
		gpio_free(core->gpio_reset);

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id si473x_dt_ids[] = {
	{ .compatible = "si,si4731", },
	{ }
};
MODULE_DEVICE_TABLE(of, si473x_dt_ids);
#endif


static const struct i2c_device_id si473x_i2c_id[] = {
	{ "si4731", SI473X_CHIP_SI4731 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, si473x_i2c_id);

static struct i2c_driver si473x_core_driver = {
	.driver		= {
		.name	= "si473x-core",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(si473x_dt_ids),
	},
	.probe			= si473x_i2c_probe,
	.remove			= si473x_i2c_remove,
	.id_table		= si473x_i2c_id,
};
module_i2c_driver(si473x_core_driver);


MODULE_AUTHOR("rpi Receiver <rpi-receiver@htl-steyr.ac.at>");
MODULE_DESCRIPTION("Si473x AM/FM device driver");
MODULE_LICENSE("GPL");
