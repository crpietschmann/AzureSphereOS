// SPDX-License-Identifier: GPL-2.0
/*
 * Custom IOCTL for IIO without sysfs
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
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc., 59 Temple
 * Place - Suite 330, Boston, MA 02111-1307 USA.
 *
 */

#ifndef _UAPI_IIO_IOCTL_H_
#define _UAPI_IIO_IOCTL_H_

#include <linux/ioctl.h>

#define IIO_IOCTL_VAL_INT 1
#define IIO_IOCTL_INT_PLUS_MICRO 2
#define IIO_IOCTL_INT_PLUS_NANO 3
#define IIO_IOCTL_INT_PLUS_MICRO_DB 4
#define IIO_IOCTL_INT_MULTIPLE 5
#define IIO_IOCTL_VAL_FRACTIONAL 10
#define IIO_IOCTL_VAL_FRACTIONAL_LOG2 11

/**
 * struct iio_ioctl_dev_info
 * @cbSize:	sizeof(iid_dev_info), used to version the struct
 * @cbTotalSize: Total size of strucure including variable sizedfield
 * @modes: operating modes supported by device
 * @currentMode: current operating mode
 * @masklength: the length of the mask established from
 *			channels
 * @num_chanels: number of channels
 * @name: device name
 **/
struct iio_ioctl_dev_info {
	unsigned int size;
	int modes;
	int currentmode;
	unsigned int masklength;
	int num_channels;
	const char *name;
};

/**
 * struct iio_ioctl_dev_info_buffer
 * @cbSize: set to sizeof(iio_ioctl_dev_info_buffer) used to version the struct
 * @cbTotal: the total size, including sizeof(iio_ioctl_dev_info_buffer) +
 *     the size of all extra data pointed to by iio_ioctl_dev_info
 * @dev_info:
 */
struct iio_ioctl_dev_info_buffer {
	unsigned int size;
	unsigned int total_size;
	struct iio_ioctl_dev_info *dev_info;
};

enum iio_ioctl_chan_info_enum {
	IIO_IOCTL_CHAN_INFO_RAW = 0,
	IIO_IOCTL_CHAN_INFO_PROCESSED,
	IIO_IOCTL_CHAN_INFO_SCALE,
	IIO_IOCTL_CHAN_INFO_OFFSET,
	IIO_IOCTL_CHAN_INFO_CALIBSCALE,
	IIO_IOCTL_CHAN_INFO_CALIBBIAS,
	IIO_IOCTL_CHAN_INFO_PEAK,
	IIO_IOCTL_CHAN_INFO_PEAK_SCALE,
	IIO_IOCTL_CHAN_INFO_QUADRATURE_CORRECTION_RAW,
	IIO_IOCTL_CHAN_INFO_AVERAGE_RAW,
	IIO_IOCTL_CHAN_INFO_LOW_PASS_FILTER_3DB_FREQUENCY,
	IIO_IOCTL_CHAN_INFO_HIGH_PASS_FILTER_3DB_FREQUENCY,
	IIO_IOCTL_CHAN_INFO_SAMP_FREQ,
	IIO_IOCTL_CHAN_INFO_FREQUENCY,
	IIO_IOCTL_CHAN_INFO_PHASE,
	IIO_IOCTL_CHAN_INFO_HARDWAREGAIN,
	IIO_IOCTL_CHAN_INFO_HYSTERESIS,
	IIO_IOCTL_CHAN_INFO_INT_TIME,
	IIO_IOCTL_CHAN_INFO_ENABLE,
	IIO_IOCTL_CHAN_INFO_CALIBHEIGHT,
	IIO_IOCTL_CHAN_INFO_CALIBWEIGHT,
	IIO_IOCTL_CHAN_INFO_DEBOUNCE_COUNT,
	IIO_IOCTL_CHAN_INFO_DEBOUNCE_TIME,
	IIO_IOCTL_CHAN_INFO_CALIBEMISSIVITY,
	IIO_IOCTL_CHAN_INFO_OVERSAMPLING_RATIO,
};

/**
 * struct iio_ioctl_raw_channel_info
 * @size: sizeof(struct iio_ioctl_raw_channel_info) used to version
 * @channel_index: the index of the channel to operat on.
 * @val: The first (most signifcant) bits in the answer.
 * @val2: The 2nd part of the input/output.
 * @mask: Which properoty we are operating on.  Use iio_ioctl_chan_info_enum
 *
 * The interpretation of val1 and val2 depend on the return value of the
 * IOCTL. THe menings are captured by the IIO_IOCTL_VAL enums at the beginning
 * of this file.
 * IIO_IOCTL_INT means that only val1 is used, and it is an integer
 * IIO_IOCTL_INT_PLUS_MICRO means that the integer value is stored in val1 and
 *      val2 contains the fractional part in 10e-6 units.
 * IIO_IOCTL_INT_PLUS_NANO means that the integer part is stored in val2 and
 *      val3 contains the franctional part in 10e-9 units.
 * and so on.
 */
struct iio_ioctl_raw_channel_info {
	unsigned int size;
	int index;
	int val;
	int val2;
	long mask;
};

/**
 * struct iio_ioctl_chan_spec_buffer_size
 * @size: sizeof(struct iio_ioctl_chan_spec_buffer_size) used to version struct
 * @index: the index of the channel, filled by the caller
 * @total_size: total size in bytes of struct and all embedded data
 * filled by the driver when the IOCTL is called.
 */
struct iio_ioctl_chan_spec_buffer_size {
	unsigned int size;
	int index;
	unsigned int total_size;
};

enum iio_ioctl_shared_by {
	IIO_IOCTL_SEPARATE,
	IIO_IOCTL_SHARED_BY_TYPE,
	IIO_IOCTL_SHARED_BY_DIR,
	IIO_IOCTL_SHARED_BY_ALL
};

enum iio_ioctl_endian {
	IIO_IOCTL_CPU,
	IIO_IOCTL_BE,
	IIO_IOCTL_LE,
};

enum iio_ioctl_chan_type {
	IIO_IOCTL_VOLTAGE,
	IIO_IOCTL_CURRENT,
	IIO_IOCTL_POWER,
	IIO_IOCTL_ACCEL,
	IIO_IOCTL_ANGL_VEL,
	IIO_IOCTL_MAGN,
	IIO_IOCTL_LIGHT,
	IIO_IOCTL_INTENSITY,
	IIO_IOCTL_PROXIMITY,
	IIO_IOCTL_TEMP,
	IIO_IOCTL_INCLI,
	IIO_IOCTL_ROT,
	IIO_IOCTL_ANGL,
	IIO_IOCTL_TIMESTAMP,
	IIO_IOCTL_CAPACITANCE,
	IIO_IOCTL_ALTVOLTAGE,
	IIO_IOCTL_CCT,
	IIO_IOCTL_PRESSURE,
	IIO_IOCTL_HUMIDITYRELATIVE,
	IIO_IOCTL_ACTIVITY,
	IIO_IOCTL_STEPS,
	IIO_IOCTL_ENERGY,
	IIO_IOCTL_DISTANCE,
	IIO_IOCTL_VELOCITY,
	IIO_IOCTL_CONCENTRATION,
	IIO_IOCTL_RESISTANCE,
	IIO_IOCTL_PH,
	IIO_IOCTL_UVINDEX,
	IIO_IOCTL_ELECTRICALCONDUCTIVITY,
};

enum iio_ioctl_modifier {
	IIO_IOCTL_NO_MOD,
	IIO_IOCTL_MOD_X,
	IIO_IOCTL_MOD_Y,
	IIO_IOCTL_MOD_Z,
	IIO_IOCTL_MOD_X_AND_Y,
	IIO_IOCTL_MOD_X_AND_Z,
	IIO_IOCTL_MOD_Y_AND_Z,
	IIO_IOCTL_MOD_X_AND_Y_AND_Z,
	IIO_IOCTL_MOD_X_OR_Y,
	IIO_IOCTL_MOD_X_OR_Z,
	IIO_IOCTL_MOD_Y_OR_Z,
	IIO_IOCTL_MOD_X_OR_Y_OR_Z,
	IIO_IOCTL_MOD_LIGHT_BOTH,
	IIO_IOCTL_MOD_LIGHT_IR,
	IIO_IOCTL_MOD_ROOT_SUM_SQUARED_X_Y,
	IIO_IOCTL_MOD_SUM_SQUARED_X_Y_Z,
	IIO_IOCTL_MOD_LIGHT_CLEAR,
	IIO_IOCTL_MOD_LIGHT_RED,
	IIO_IOCTL_MOD_LIGHT_GREEN,
	IIO_IOCTL_MOD_LIGHT_BLUE,
	IIO_IOCTL_MOD_QUATERNION,
	IIO_IOCTL_MOD_TEMP_AMBIENT,
	IIO_IOCTL_MOD_TEMP_OBJECT,
	IIO_IOCTL_MOD_NORTH_MAGN,
	IIO_IOCTL_MOD_NORTH_TRUE,
	IIO_IOCTL_MOD_NORTH_MAGN_TILT_COMP,
	IIO_IOCTL_MOD_NORTH_TRUE_TILT_COMP,
	IIO_IOCTL_MOD_RUNNING,
	IIO_IOCTL_MOD_JOGGING,
	IIO_IOCTL_MOD_WALKING,
	IIO_IOCTL_MOD_STILL,
	IIO_IOCTL_MOD_ROOT_SUM_SQUARED_X_Y_Z,
	IIO_IOCTL_MOD_I,
	IIO_IOCTL_MOD_Q,
	IIO_IOCTL_MOD_CO2,
	IIO_IOCTL_MOD_VOC,
	IIO_IOCTL_MOD_LIGHT_UV,
};

enum iio_ioctl_event_type {
	IIO_IOCTL_EV_TYPE_THRESH,
	IIO_IOCTL_EV_TYPE_MAG,
	IIO_IOCTL_EV_TYPE_ROC,
	IIO_IOCTL_EV_TYPE_THRESH_ADAPTIVE,
	IIO_IOCTL_EV_TYPE_MAG_ADAPTIVE,
	IIO_IOCTL_EV_TYPE_CHANGE,
};

enum iio_ioctl_event_direction {
	IIO_IOCTL_EV_DIR_EITHER,
	IIO_IOCTL_EV_DIR_RISING,
	IIO_IOCTL_EV_DIR_FALLING,
	IIO_IOCTL_EV_DIR_NONE,
};

/**
 * struct iio_ioctl_event_spec - specification for a channel event
 * @type: Type of the event
 * @dir: Direction of the event
 * @mask_separate: Bit mask of enum iio_ioctl_event_info values. Attributes
 *    set in this mask will be registered per channel.
 * @mask_shared_by_type: Bit mask of enum iio_ioctl_event_info values.
 *     Attributes set in this mask will be shared by channel type.
 * @mask_shared_by_dir: Bit mask of enum iio_ioctl_event_info values. Attributes
 *     set in this mask will be shared by channel type and direction.
 * @mask_shared_by_all: Bit mask of enum iio_ioctl_event_info values.
 *     Attributes
 *			    set in this mask will be shared by all channels.
 */
struct iio_ioctl_event_spec {
	unsigned int size; 
	struct iio_ioctl_event_spec *next; 
	enum iio_ioctl_event_type type;
	enum iio_ioctl_event_direction dir;
	unsigned long mask_separate;
	unsigned long mask_shared_by_type;
	unsigned long mask_shared_by_dir;
	unsigned long mask_shared_by_all;
};

/**
 * struct iio_ioctl_chan_spec_ext_info - Extended channel info attribute
 * @name:	Info attribute name
 * @shared:	Whether this attribute is shared between all channels.
 */
struct iio_ioctl_chan_spec_ext_info {
	unsigned int size; 
	struct iio_ioctl_chan_spec_ext_info *next; 
	const char *name;
	enum iio_ioctl_shared_by shared;
};

/**
 * struct iio_ioctl_chan_spec - specification of a single channel
 * @type:		What type of measurement is the channel making.
 * @channel:		What number do we wish to assign the channel.
 * @channel2:		If there is a second number for a differential
 *			channel then this is it. If modified is set then the
 *			value here specifies the modifier.
 * @address:		Driver specific identifier.
 * @scan_index:		Monotonic index to give ordering in scans when read
 *			from a buffer.
 * @scan_type:		sign:		's' or 'u' to specify signed or unsigned
 *			realbits:	Number of valid bits of data
 *			storagebits:	Realbits + padding
 *			shift:		Shift right by this before masking out
 *					realbits.
 *			repeat:		Number of times real/storage bits
 *					repeats. When the repeat element is
 *					more than 1, then the type element in
 *					sysfs will show a repeat value.
 *					Otherwise, the number of repetitions is
 *					omitted.
 *			endianness:	little or big endian
 * @info_mask_separate: What information is to be exported that is specific to
 *			this channel.
 * @info_mask_shared_by_type: What information is to be exported that is shared
 *			by all channels of the same type.
 * @info_mask_shared_by_dir: What information is to be exported that is shared
 *			by all channels of the same direction.
 * @info_mask_shared_by_all: What information is to be exported that is shared
 *			by all channels.
 * @event_spec:		Array of events which should be registered for this
 *			channel.
 * @num_event_specs:	Size of the event_spec array.
 * @ext_info:		Array of extended info attributes for this channel.
 *			The array is NULL terminated, the last element should
 *			have its name field set to NULL.
 * @extend_name:	Allows labeling of channel attributes with an
 *			informative name. Note this has no effect codes etc,
 *			unlike modifiers.
 * @datasheet_name:	A name used in in-kernel mapping of channels. It should
 *			correspond to the first name that the channel is referred
 *			to by in the datasheet (e.g. IND), or the nearest
 *			possible compound name (e.g. IND-INC).
 * @modified:		Does a modifier apply to this channel. What these are
 *			depends on the channel type.  Modifier is set in
 *			channel2. Examples are IIO_IOCTL_MOD_X for axial sensors
 *			about the 'x' axis.
 * @indexed:		Specify the channel has a numerical index. If not,
 *			the channel index number will be suppressed for sysfs
 *			attributes but not for event codes.
 * @output:		Channel is output.
 * @differential:	Channel is differential.
 */
struct iio_ioctl_chan_spec {
	unsigned int size; 
	enum iio_ioctl_chan_type type;
	int channel;
	int channel2;
	unsigned long address;
	int scan_index;
	struct {
		char sign;
		__u8 realbits;
		__u8 storagebits;
		__u8 shift;
		__u8 repeat;
		enum iio_ioctl_endian endianness;
	} scan_type;
	long info_mask_separate;
	long info_mask_shared_by_type;
	long info_mask_shared_by_dir;
	long info_mask_shared_by_all;
	const struct iio_ioctl_event_spec *event_spec;
	unsigned int num_event_specs;
	const struct iio_ioctl_chan_spec_ext_info *ext_info;
	const char *extend_name;
	const char *datasheet_name;
	unsigned int modified : 1;
	unsigned int indexed : 1;
	unsigned int output : 1;
	unsigned int differential : 1;
};

/**
 * struct iio_ioctl_chan_spec_buffer
 * @size: sizeof(struct iio_ioctl_chan_spec_buffer) used to version struct
 * @total_size: total size in bytes of struct and all embedded data
 * @index: the index of the channel
 * @chan_spec: The iio_ioctl_chan_spec for the device.
 */
struct iio_ioctl_chan_spec_buffer {
	unsigned long size;
	unsigned long total_size;
	int index;
	struct iio_ioctl_chan_spec *channel;
};

/**
 * struct iio_ioctl_read_chan_ext_info
 * @size: sizeof(struct iio_ioctl_read_chan_ext_info) used to version
 * @channel_index: the index of the channel
 * @info_index: the index of the extended property to set in the ext_info array 
 * in the channel metadata char *buffer: the value to read the property into. 
 * @buffer the buffer to read the value into
 * @length the length of the buffer to write the value into.
 * 
 * The IOCTL will return the number of characters it wrote to the buffer. The 
 * IOCTL will truncate the value to fit into the buffer.  The IOCTL will always
 * ensure there is a terminating NULL on successful completion.
 *
 */ 
struct iio_ioctl_read_chan_ext_info{
	unsigned int size;
	unsigned int channel_index;
	unsigned int info_index;
	char *buffer;
	size_t length; 
};

/**
 * struct iio_ioctl_write_chan_ext_info
 * @size: sizeof(struct iio_ioctl_write_chan_ext_info) used to version
 * @channel_index: the index of the channel
 * @info_index: the index of the extended property to set in the ext_info array 
 * in the channel metadata
 * @buffer: the value to set the property into.  The string must be null 
 * terminated. The length of the buffer including the NULL terminator must be
 * set in the length field. 
 * @len: the length of the buffer to write. The buffer must be null terminated 
 * and the length must include the null terminator. The length cannot zero and 
 * cannot be more than PAGE_SIZE.
 *
 */ 


struct iio_ioctl_write_chan_ext_info{
	unsigned int size;
	unsigned int channel_index;
	unsigned int info_index;
	const char *buffer;
	size_t length;
};

#define IIO_GET_DEVICE_INFO_BUFFER_TOTAL_SIZE_IOCTL _IOR('i', 0xD0, unsigned int)
#define IIO_GET_DEVICE_INFO_BUFFER_IOCTL _IOWR('i', 0xD1, struct iio_ioctl_dev_info_buffer)
#define IIO_GET_CHANNEL_SPEC_BUFFER_TOTAL_SIZE_IOCTL _IOWR('i', 0xD2, struct iio_ioctl_chan_spec_buffer_size)
#define IIO_GET_CHANNEL_SPEC_BUFFER_IOCTL _IOWR('i', 0xD3, struct iio_ioctl_chan_spec_buffer)
#define IIO_READ_RAW_CHANNEL_INFO_IOCTL _IOWR('i', 0xD4, struct iio_ioctl_raw_channel_info)
#define IIO_WRITE_RAW_CHANNEL_INFO_IOCTL _IOWR('i', 0xD5, struct iio_ioctl_raw_channel_info)
#define IIO_READ_CHANNEL_EXT_INFO_IOCTL _IOWR('i', 0xD6, struct iio_ioctl_read_chan_ext_info)
#define IIO_WRITE_CHANNEL_EXT_INFO_IOCTL _IOWR('i', 0xD7, struct iio_ioctl_write_chan_ext_info)

#ifdef CONFIG_IIO_BUFFER

#define IIO_SCAN_MASK_QUERY_BIT_IOCTL _IOW('i', 0xD8, unsigned int)
#define IIO_SCAN_MASK_SET_BIT_IOCTL	_IOW('i', 0xD9, unsigned int)
#define IIO_SCAN_MASK_CLEAR_BIT_IOCTL _IOW('i', 0xDA, unsigned int)
#define IIO_BUFFER_GET_ENABLE_IOCTL _IO('i', 0xDB)
#define IIO_BUFFER_SET_ENABLE_IOCTL _IOW('i', 0xDC, unsigned int)
#define IIO_BUFFER_GET_LENGTH_IOCTL	_IO('i', 0xDD)
#define IIO_BUFFER_SET_LENGTH_IOCTL _IOW('i', 0xDE, unsigned int)
#define IIO_BUFFER_GET_WATERMARK_IOCTL _IO('i', 0xDF)
#define IIO_BUFFER_SET_WATERMARK_IOCTL _IOW('i', 0xE0, unsigned int)

#endif

#endif
