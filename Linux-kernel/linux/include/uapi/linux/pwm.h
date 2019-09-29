// SPDX-License-Identifier: GPL-2.0
/*
 * Definitions for character device interface for the generic PWM framework
 *
 * Copyright (c) 2019 Microsoft Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */

#ifndef _UAPI__LINUX_PWM_H
#define _UAPI__LINUX_PWM_H

#include <linux/ioctl.h>
#include <linux/types.h>

/**
 * enum pwm_polarity - polarity of a PWM signal
 * @PWM_POLARITY_NORMAL: a high signal for the duration of the duty-
 * cycle, followed by a low signal for the remainder of the pulse
 * period
 * @PWM_POLARITY_INVERSED: a low signal for the duration of the duty-
 * cycle, followed by a high signal for the remainder of the pulse
 * period
 */
enum pwm_polarity {
	PWM_POLARITY_NORMAL,
	PWM_POLARITY_INVERSED,
};

/*
 * struct pwm_state - state of a PWM channel
 * @period: PWM period (in nanoseconds)
 * @duty_cycle: PWM duty cycle (in nanoseconds)
 * @polarity: PWM polarity
 * @enabled: PWM enabled status
 * @extended_state: optional driver-specific state data
 * @extended_state_size: size of data pointed to by extended_state
 */
struct pwm_state {
	unsigned int period;
	unsigned int duty_cycle;
	enum pwm_polarity polarity;
	bool enabled;

	void *extended_state;
	size_t extended_state_size;
};

struct pwm_chardev_params {
	unsigned int pwm_index;
	struct pwm_state state;
};

#define PWM_APPLY_STATE _IOW(0xf7, 0x01, struct pwm_chardev_params)
#define PWM_GET_STATE _IOWR(0xf7, 0x02, struct pwm_chardev_params)
#define PWM_EXPORT _IOW(0xf7, 0x03, unsigned int)
#define PWM_UNEXPORT _IOW(0xf7, 0x04, unsigned int)

#endif /* _UAPI__LINUX_PWM_H */
