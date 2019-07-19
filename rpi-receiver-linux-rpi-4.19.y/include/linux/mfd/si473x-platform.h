/*
 * include/media/si473x-platform.h -- Platform data specific definitions
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

#ifndef __SI473X_PLATFORM_H__
#define __SI473X_PLATFORM_H__

/**
 * @func:   selects the boot function of the device. I.e.
 *          SI473X_FUNC_FM_RECEIVER - FM receiver
 *          SI473X_FUNC_AM_RECEIVER - AM receiver
 * @mode:   Digital and/or analog Audio output:
 *          SI473X_MODE_DIGITAL_AUDIO_OUTPUTS
 *          SI473X_MODE_ANALOG_AUDIO_OUTPUTS
 */

enum si473x_func {
	SI473X_FUNC_FM_RECEIVER = 0,
	SI473X_FUNC_AM_RECEIVER = 1,
};

enum si473x_mode { // for DIGITAL_AUDIO_OUTPUTS set XOSCEN = 0
	SI473X_MODE_ANALOG_AUDIO_OUTPUTS	= 0x05,
	SI473X_MODE_DIGITAL_AUDIO_OUTPUTS	= 0xB0,
};

struct si473x_power_up_args {
	enum si473x_func    func;
	enum si473x_mode    mode;
};

#endif /* __SI473X_PLATFORM_H__ */
