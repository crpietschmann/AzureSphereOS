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
 */

#include <linux/iio/iio.h>
#include <linux/iio/ioctl.h>
#include <uapi/linux/iio/ioctl.h>
#include <linux/uaccess.h>
#include <linux/slab.h>

#ifdef CONFIG_IIO_BUFFER

#include <linux/iio/buffer.h>

#endif


#define alignof(type) ((size_t)(offsetof(struct { char c; type d; }, d)))


/*
 * Struct Copy Functions to iio_ioctl_xxx from iio_xxx
 */

static int iio_convert_chan_type(enum iio_chan_type from,
		enum iio_ioctl_chan_type *to)
{
	switch (from) {
	case IIO_VOLTAGE:
		*to = IIO_IOCTL_VOLTAGE;
		break;
	case IIO_CURRENT:
		*to = IIO_IOCTL_CURRENT;
		break;
	case IIO_POWER:
		*to = IIO_IOCTL_POWER;
		break;
	case IIO_ACCEL:
		*to = IIO_IOCTL_ACCEL;
		break;
	case IIO_ANGL_VEL:
		*to = IIO_IOCTL_ANGL_VEL;
		break;
	case IIO_MAGN:
		*to = IIO_IOCTL_MAGN;
		break;
	case IIO_LIGHT:
		*to = IIO_IOCTL_LIGHT;
		break;
	case IIO_INTENSITY:
		*to = IIO_IOCTL_INTENSITY;
		break;
	case IIO_PROXIMITY:
		*to = IIO_IOCTL_PROXIMITY;
		break;
	case IIO_TEMP:
		*to = IIO_IOCTL_TEMP;
		break;
	case IIO_INCLI:
		*to = IIO_IOCTL_INCLI;
		break;
	case IIO_ROT:
		*to = IIO_IOCTL_ROT;
		break;
	case IIO_ANGL:
		*to = IIO_IOCTL_ANGL;
		break;
	case IIO_TIMESTAMP:
		*to = IIO_IOCTL_TIMESTAMP;
		break;
	case IIO_CAPACITANCE:
		*to = IIO_IOCTL_CAPACITANCE;
		break;
	case IIO_ALTVOLTAGE:
		*to = IIO_IOCTL_ALTVOLTAGE;
		break;
	case IIO_CCT:
		*to = IIO_IOCTL_CCT;
		break;
	case IIO_PRESSURE:
		*to = IIO_IOCTL_PRESSURE;
		break;
	case IIO_HUMIDITYRELATIVE:
		*to = IIO_IOCTL_HUMIDITYRELATIVE;
		break;
	case IIO_ACTIVITY:
		*to = IIO_IOCTL_ACTIVITY;
		break;
	case IIO_STEPS:
		*to = IIO_IOCTL_STEPS;
		break;
	case IIO_ENERGY:
		*to = IIO_IOCTL_ENERGY;
		break;
	case IIO_DISTANCE:
		*to = IIO_IOCTL_DISTANCE;
		break;
	case IIO_VELOCITY:
		*to = IIO_IOCTL_VELOCITY;
		break;
	case IIO_CONCENTRATION:
		*to = IIO_IOCTL_CONCENTRATION;
		break;
	case IIO_RESISTANCE:
		*to = IIO_IOCTL_RESISTANCE;
		break;
	case IIO_PH:
		*to = IIO_IOCTL_PH;
		break;
	case IIO_UVINDEX:
		*to = IIO_IOCTL_UVINDEX;
		break;
	case IIO_ELECTRICALCONDUCTIVITY:
		*to = IIO_IOCTL_ELECTRICALCONDUCTIVITY;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int iio_convert_chan_info(enum iio_chan_info_enum from,
		enum iio_ioctl_chan_info_enum *to)
{
	switch (from) {
	case IIO_CHAN_INFO_RAW:
		*to = IIO_IOCTL_CHAN_INFO_RAW;
		break;
	case IIO_CHAN_INFO_PROCESSED:
		*to = IIO_IOCTL_CHAN_INFO_PROCESSED;
		break;
	case IIO_CHAN_INFO_SCALE:
		*to = IIO_IOCTL_CHAN_INFO_SCALE;
		break;
	case IIO_CHAN_INFO_OFFSET:
		*to = IIO_IOCTL_CHAN_INFO_OFFSET;
		break;
	case IIO_CHAN_INFO_CALIBSCALE:
		*to = IIO_IOCTL_CHAN_INFO_CALIBSCALE;
		break;
	case IIO_CHAN_INFO_CALIBBIAS:
		*to = IIO_IOCTL_CHAN_INFO_CALIBBIAS;
		break;
	case IIO_CHAN_INFO_PEAK:
		*to = IIO_IOCTL_CHAN_INFO_PEAK;
		break;
	case IIO_CHAN_INFO_PEAK_SCALE:
		*to = IIO_IOCTL_CHAN_INFO_PEAK_SCALE;
		break;
	case IIO_CHAN_INFO_QUADRATURE_CORRECTION_RAW:
		*to = IIO_IOCTL_CHAN_INFO_QUADRATURE_CORRECTION_RAW;
		break;
	case IIO_CHAN_INFO_AVERAGE_RAW:
		*to = IIO_IOCTL_CHAN_INFO_AVERAGE_RAW;
		break;
	case IIO_CHAN_INFO_LOW_PASS_FILTER_3DB_FREQUENCY:
		*to = IIO_IOCTL_CHAN_INFO_LOW_PASS_FILTER_3DB_FREQUENCY;
		break;
	case IIO_CHAN_INFO_HIGH_PASS_FILTER_3DB_FREQUENCY:
		*to = IIO_IOCTL_CHAN_INFO_HIGH_PASS_FILTER_3DB_FREQUENCY;
		break;
	case IIO_CHAN_INFO_SAMP_FREQ:
		*to = IIO_IOCTL_CHAN_INFO_SAMP_FREQ;
		break;
	case IIO_CHAN_INFO_FREQUENCY:
		*to = IIO_IOCTL_CHAN_INFO_FREQUENCY;
		break;
	case IIO_CHAN_INFO_PHASE:
		*to = IIO_IOCTL_CHAN_INFO_PHASE;
		break;
	case IIO_CHAN_INFO_HARDWAREGAIN:
		*to = IIO_IOCTL_CHAN_INFO_HARDWAREGAIN;
		break;
	case IIO_CHAN_INFO_HYSTERESIS:
		*to = IIO_IOCTL_CHAN_INFO_HYSTERESIS;
		break;
	case IIO_CHAN_INFO_INT_TIME:
		*to = IIO_IOCTL_CHAN_INFO_INT_TIME;
		break;
	case IIO_CHAN_INFO_ENABLE:
		*to = IIO_IOCTL_CHAN_INFO_ENABLE;
		break;
	case IIO_CHAN_INFO_CALIBHEIGHT:
		*to = IIO_IOCTL_CHAN_INFO_CALIBHEIGHT;
		break;
	case IIO_CHAN_INFO_CALIBWEIGHT:
		*to = IIO_IOCTL_CHAN_INFO_CALIBWEIGHT;
		break;
	case IIO_CHAN_INFO_DEBOUNCE_COUNT:
		*to = IIO_IOCTL_CHAN_INFO_DEBOUNCE_COUNT;
		break;
	case IIO_CHAN_INFO_DEBOUNCE_TIME:
		*to = IIO_IOCTL_CHAN_INFO_DEBOUNCE_TIME;
		break;
	case IIO_CHAN_INFO_CALIBEMISSIVITY:
		*to = IIO_IOCTL_CHAN_INFO_CALIBEMISSIVITY;
		break;
	case IIO_CHAN_INFO_OVERSAMPLING_RATIO:
		*to = IIO_IOCTL_CHAN_INFO_OVERSAMPLING_RATIO;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int iio_convert_chan_info_inverse(enum iio_ioctl_chan_info_enum from,
		enum iio_chan_info_enum *to)
{
	switch (from) {
	case IIO_IOCTL_CHAN_INFO_RAW:
		*to = IIO_CHAN_INFO_RAW;
		break;
	case IIO_IOCTL_CHAN_INFO_PROCESSED:
		*to = IIO_CHAN_INFO_PROCESSED;
		break;
	case IIO_IOCTL_CHAN_INFO_SCALE:
		*to = IIO_CHAN_INFO_SCALE;
		break;
	case IIO_IOCTL_CHAN_INFO_OFFSET:
		*to = IIO_CHAN_INFO_OFFSET;
		break;
	case IIO_IOCTL_CHAN_INFO_CALIBSCALE:
		*to = IIO_CHAN_INFO_CALIBSCALE;
		break;
	case IIO_IOCTL_CHAN_INFO_CALIBBIAS:
		*to = IIO_IOCTL_CHAN_INFO_CALIBBIAS;
		break;
	case IIO_IOCTL_CHAN_INFO_PEAK:
		*to = IIO_IOCTL_CHAN_INFO_PEAK;
		break;
	case IIO_IOCTL_CHAN_INFO_PEAK_SCALE:
		*to = IIO_CHAN_INFO_PEAK_SCALE;
		break;
	case IIO_IOCTL_CHAN_INFO_QUADRATURE_CORRECTION_RAW:
		*to = IIO_CHAN_INFO_QUADRATURE_CORRECTION_RAW;
		break;
	case IIO_IOCTL_CHAN_INFO_AVERAGE_RAW:
		*to = IIO_CHAN_INFO_AVERAGE_RAW;
		break;
	case IIO_IOCTL_CHAN_INFO_LOW_PASS_FILTER_3DB_FREQUENCY:
		*to = IIO_CHAN_INFO_LOW_PASS_FILTER_3DB_FREQUENCY;
		break;
	case IIO_IOCTL_CHAN_INFO_HIGH_PASS_FILTER_3DB_FREQUENCY:
		*to = IIO_CHAN_INFO_HIGH_PASS_FILTER_3DB_FREQUENCY;
		break;
	case IIO_IOCTL_CHAN_INFO_SAMP_FREQ:
		*to = IIO_CHAN_INFO_SAMP_FREQ;
		break;
	case IIO_IOCTL_CHAN_INFO_FREQUENCY:
		*to = IIO_CHAN_INFO_FREQUENCY;
		break;
	case IIO_IOCTL_CHAN_INFO_PHASE:
		*to = IIO_CHAN_INFO_PHASE;
		break;
	case IIO_IOCTL_CHAN_INFO_HARDWAREGAIN:
		*to = IIO_CHAN_INFO_HARDWAREGAIN;
		break;
	case IIO_IOCTL_CHAN_INFO_HYSTERESIS:
		*to = IIO_CHAN_INFO_HYSTERESIS;
		break;
	case IIO_IOCTL_CHAN_INFO_INT_TIME:
		*to = IIO_CHAN_INFO_INT_TIME;
		break;
	case IIO_IOCTL_CHAN_INFO_ENABLE:
		*to = IIO_CHAN_INFO_ENABLE;
		break;
	case IIO_IOCTL_CHAN_INFO_CALIBHEIGHT:
		*to = IIO_CHAN_INFO_CALIBHEIGHT;
		break;
	case IIO_IOCTL_CHAN_INFO_CALIBWEIGHT:
		*to = IIO_CHAN_INFO_CALIBWEIGHT;
		break;
	case IIO_IOCTL_CHAN_INFO_DEBOUNCE_COUNT:
		*to = IIO_CHAN_INFO_DEBOUNCE_COUNT;
		break;
	case IIO_IOCTL_CHAN_INFO_DEBOUNCE_TIME:
		*to = IIO_CHAN_INFO_DEBOUNCE_TIME;
		break;
	case IIO_IOCTL_CHAN_INFO_CALIBEMISSIVITY:
		*to = IIO_CHAN_INFO_CALIBEMISSIVITY;
		break;
	case IIO_IOCTL_CHAN_INFO_OVERSAMPLING_RATIO:
		*to = IIO_CHAN_INFO_OVERSAMPLING_RATIO;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int iio_convert_shared_by(enum iio_shared_by from,
		enum iio_ioctl_shared_by *to)
{
	switch (from) {
	case IIO_SEPARATE:
		*to = IIO_IOCTL_SEPARATE;
		break;
	case IIO_SHARED_BY_TYPE:
		*to = IIO_IOCTL_SHARED_BY_TYPE;
		break;
	case IIO_SHARED_BY_DIR:
		*to = IIO_IOCTL_SHARED_BY_DIR;
		break;
	case IIO_SHARED_BY_ALL:
		*to = IIO_IOCTL_SHARED_BY_ALL;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int iio_convert_endian(enum iio_endian from, enum iio_ioctl_endian *to)
{
	switch (from) {
	case IIO_CPU:
		*to = IIO_IOCTL_CPU;
		break;
	case IIO_BE:
		*to = IIO_IOCTL_BE;
		break;
	case IIO_LE:
		*to = IIO_IOCTL_BE;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int iio_convert_modifier(enum iio_modifier from,
		enum iio_ioctl_modifier *to)
{
	switch (from) {
	case IIO_NO_MOD:
		*to = IIO_IOCTL_NO_MOD;
		break;
	case IIO_MOD_X:
		*to = IIO_IOCTL_MOD_X;
		break;
	case IIO_MOD_Y:
		*to = IIO_IOCTL_MOD_Y;
		break;
	case IIO_MOD_Z:
		*to = IIO_IOCTL_MOD_Z;
		break;
	case IIO_MOD_X_AND_Y:
		*to = IIO_IOCTL_MOD_X_AND_Y;
		break;
	case IIO_MOD_X_AND_Z:
		*to = IIO_IOCTL_MOD_X_AND_Z;
		break;
	case IIO_MOD_Y_AND_Z:
		*to = IIO_IOCTL_MOD_Y_AND_Z;
		break;
	case IIO_MOD_X_AND_Y_AND_Z:
		*to = IIO_IOCTL_MOD_X_AND_Y_AND_Z;
		break;
	case IIO_MOD_X_OR_Y:
		*to = IIO_IOCTL_MOD_X_OR_Y;
		break;
	case IIO_MOD_X_OR_Z:
		*to = IIO_IOCTL_MOD_X_OR_Z;
		break;
	case IIO_MOD_Y_OR_Z:
		*to = IIO_IOCTL_MOD_Y_OR_Z;
		break;
	case IIO_MOD_X_OR_Y_OR_Z:
		*to = IIO_IOCTL_MOD_X_OR_Y_OR_Z;
		break;
	case IIO_MOD_LIGHT_BOTH:
		*to = IIO_IOCTL_MOD_LIGHT_BOTH;
		break;
	case IIO_MOD_LIGHT_IR:
		*to = IIO_IOCTL_MOD_LIGHT_IR;
		break;
	case IIO_MOD_ROOT_SUM_SQUARED_X_Y:
		*to = IIO_IOCTL_MOD_ROOT_SUM_SQUARED_X_Y;
		break;
	case IIO_MOD_SUM_SQUARED_X_Y_Z:
		*to = IIO_IOCTL_MOD_SUM_SQUARED_X_Y_Z;
		break;
	case IIO_MOD_LIGHT_CLEAR:
		*to = IIO_IOCTL_MOD_LIGHT_CLEAR;
		break;
	case IIO_MOD_LIGHT_RED:
		*to = IIO_IOCTL_MOD_LIGHT_RED;
		break;
	case IIO_MOD_LIGHT_GREEN:
		*to = IIO_IOCTL_MOD_LIGHT_GREEN;
		break;
	case IIO_MOD_LIGHT_BLUE:
		*to = IIO_IOCTL_MOD_LIGHT_BLUE;
		break;
	case IIO_MOD_QUATERNION:
		*to = IIO_IOCTL_MOD_QUATERNION;
		break;
	case IIO_MOD_TEMP_AMBIENT:
		*to = IIO_IOCTL_MOD_TEMP_AMBIENT;
		break;
	case IIO_MOD_TEMP_OBJECT:
		*to = IIO_IOCTL_MOD_TEMP_OBJECT;
		break;
	case IIO_MOD_NORTH_MAGN:
		*to = IIO_IOCTL_MOD_NORTH_MAGN;
		break;
	case IIO_MOD_NORTH_TRUE:
		*to = IIO_IOCTL_MOD_NORTH_TRUE;
		break;
	case IIO_MOD_NORTH_MAGN_TILT_COMP:
		*to = IIO_IOCTL_MOD_NORTH_MAGN_TILT_COMP;
		break;
	case IIO_MOD_NORTH_TRUE_TILT_COMP:
		*to = IIO_IOCTL_MOD_NORTH_TRUE_TILT_COMP;
		break;
	case IIO_MOD_RUNNING:
		*to = IIO_IOCTL_MOD_RUNNING;
		break;
	case IIO_MOD_JOGGING:
		*to = IIO_IOCTL_MOD_JOGGING;
		break;
	case IIO_MOD_WALKING:
		*to = IIO_IOCTL_MOD_WALKING;
		break;
	case IIO_MOD_STILL:
		*to = IIO_IOCTL_MOD_STILL;
		break;
	case IIO_MOD_ROOT_SUM_SQUARED_X_Y_Z:
		*to = IIO_IOCTL_MOD_ROOT_SUM_SQUARED_X_Y_Z;
		break;
	case IIO_MOD_I:
		*to = IIO_IOCTL_MOD_I;
		break;
	case IIO_MOD_Q:
		*to = IIO_IOCTL_MOD_Q;
		break;
	case IIO_MOD_CO2:
		*to = IIO_IOCTL_MOD_CO2;
		break;
	case IIO_MOD_VOC:
		*to = IIO_IOCTL_MOD_VOC;
		break;
	case IIO_MOD_LIGHT_UV:
		*to = IIO_IOCTL_MOD_LIGHT_UV;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int iio_convert_event_type(enum iio_event_type from,
		enum iio_ioctl_event_type *to)
{
	switch (from) {
	case IIO_EV_TYPE_THRESH:
		*to = IIO_IOCTL_EV_TYPE_THRESH;
		break;
	case IIO_EV_TYPE_MAG:
		*to = IIO_IOCTL_EV_TYPE_MAG;
		break;
	case IIO_EV_TYPE_ROC:
		*to = IIO_IOCTL_EV_TYPE_ROC;
		break;
	case IIO_EV_TYPE_THRESH_ADAPTIVE:
		*to = IIO_IOCTL_EV_TYPE_THRESH_ADAPTIVE;
		break;
	case IIO_EV_TYPE_MAG_ADAPTIVE:
		*to = IIO_IOCTL_EV_TYPE_MAG_ADAPTIVE;
		break;
	case IIO_EV_TYPE_CHANGE:
		*to = IIO_IOCTL_EV_TYPE_CHANGE;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int iio_convert_event_direction(enum iio_event_direction from,
		enum iio_ioctl_event_direction *to)
{
	switch (from) {
	case IIO_EV_DIR_EITHER:
		*to = IIO_IOCTL_EV_DIR_EITHER;
		break;
	case IIO_EV_DIR_RISING:
		*to = IIO_IOCTL_EV_DIR_RISING;
		break;
	case IIO_EV_DIR_FALLING:
		*to = IIO_IOCTL_EV_DIR_FALLING;
		break;
	case IIO_EV_DIR_NONE:
		*to = IIO_IOCTL_EV_DIR_NONE;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

/*
 * convert the shared mask. The shared mask is a bit field.  The ordinal postion
 * of each bit matches the enum value in enum iio_chan_info_enum.  The
 * algorithm here will be to loop through the source mask and set the
 * "converted" bit in the destination.
 */
static int iio_convert_mask(long int from, long int *to)
{
	int ret;
	int bitcount = 8 * sizeof(from);
	int i;
	enum iio_ioctl_chan_info_enum chan_info;

	/* clear all the bits in the output mask */
	*to = 0;

	for (i = 0; i < bitcount; ++i) {
		if (from & 1) {
			ret = iio_convert_chan_info((enum iio_chan_info_enum)i,
					&chan_info);
			if (ret < 0)
				return ret;

			*to = *to | BIT(chan_info);
		}

		from = from >> 1;
	}

	return 0;
}

/*
 * Does a shallow copy an iio_chan_spec (the kernel version of the struct)
 * to an iio_ioctl_chan_spec (the version exposed through UAPI).  Copies
 * field by field since the binary compatibility cannot be ensured.
 * The kernel might get updated but the user API must remain a contract
 *
 */
static int iio_shallow_copy_chan_spec(const struct iio_chan_spec *from,
		struct iio_ioctl_chan_spec *to)
{
	int ret;

	ret = iio_convert_chan_type(from->type, &to->type);
	if (ret < 0)
		return -EINVAL;

	to->channel = from->channel;

	/* the meaning of channel2 depends on the modified flag */
	if (from->modified == 1) {
		ret = iio_convert_modifier(from->channel2,
				(enum iio_ioctl_modifier *)&to->channel2);
		if (ret < 0)
			return ret;
	} else
		to->channel2 = from->channel2;

	to->address = from->address;
	to->scan_index = from->scan_index;
	to->scan_type.sign = from->scan_type.sign;
	to->scan_type.realbits = from->scan_type.realbits;
	to->scan_type.storagebits = from->scan_type.storagebits;
	to->scan_type.shift = from->scan_type.shift;
	to->scan_type.repeat = from->scan_type.repeat;

	ret = iio_convert_endian(from->scan_type.endianness,
			&(to->scan_type.endianness));
	if (ret < 0)
		return ret;

	ret = iio_convert_mask(from->info_mask_separate,
			&to->info_mask_separate);
	if (ret < 0)
		return ret;

	ret = iio_convert_mask(from->info_mask_shared_by_type,
			&to->info_mask_shared_by_type);
	if (ret < 0)
		return ret;

	ret = iio_convert_mask(from->info_mask_shared_by_dir,
			&to->info_mask_shared_by_dir);
	if (ret < 0)
		return ret;

	ret = iio_convert_mask(from->info_mask_shared_by_all,
			&to->info_mask_shared_by_all);
	if (ret < 0)
		return ret;

	to->event_spec = NULL;
	to->num_event_specs = from->num_event_specs;
	to->extend_name = NULL;
	to->datasheet_name = NULL;
	to->modified = from->modified;
	to->indexed = from->indexed;
	to->output = from->output;
	to->differential = from->differential;

	return 0;
}

static int iio_convert_event_spec(const struct iio_event_spec *from,
		struct iio_ioctl_event_spec *to)
{
	int ret;

	ret = iio_convert_event_type(from->type, &to->type);
	if (ret < 0)
		return -EINVAL;
	ret = iio_convert_event_direction(from->dir, &to->dir);
	if (ret < 0)
		return -EINVAL;
	to->mask_separate = from->mask_separate;
	to->mask_shared_by_type = from->mask_shared_by_type;
	to->mask_shared_by_dir = from->mask_shared_by_dir;
	to->mask_shared_by_all = from->mask_shared_by_all;

	return 0;
}



/*
 * calculates the padding to get to the next offset for the given
 * alignment
 * @offset the offset from the base. The base is assumed to be aligned
 * @alignment the alignment, i.e. 8 for 8 byte alignment
 */
static unsigned int pad(unsigned int offset, unsigned int alignment)
{
	unsigned int rem;
	unsigned int p;

	rem = offset % alignment;
	if (rem == 0)
		p = 0;
	else
		p = alignment - rem;

	return p;
}

/*
 * Many of the API's for this set of IOCTLS work by having a structure at
 * the beginning of a memory buffer.  This memory buffer is bigger than the
 * strucure itself. The memory after the structure is used to hold the targets
 * of any pointers that are embedded in the strucure.
 *
 * This is a utility function to copy a string that is embedded in a strucure
 * into the buffer after the strucure while at the same time updating the
 * containing string pointer to the location the string is copied to. It also
 * manages the offset.
 *
 * @base: the base user mode pointer of the target.
 * @offset: the offset from the base pointer for the string to be copied to
 * @sz: the source to copy
 * @container: the address in the containing strucure that holds the string
 *     pointer
 */
static long iio_copy_embedded_string_to_user(char __user *base,
		unsigned int *offset, unsigned int size, const char *sz, 
		char **container)
{
	unsigned int len;
	unsigned int align;
	unsigned int o = *offset;

	if (sz != NULL) {
		len = strlen(sz) + 1;
		align = alignof(char);
		o += pad(o, align);
		*container = base + o;
		
		/* make sure we are never copying past the end */ 
		BUG_ON(o + len > size);
		
		if (copy_to_user(base + o, sz, len))
			return -EFAULT;
		o += len;
	}

	*offset = o;
	return 0;
}

static unsigned int iio_get_device_info_buffer_total_size(
		struct iio_dev *indio_dev)
{
	unsigned int size = 0;
	unsigned int alignment;

	/* validate parameters */
	if (indio_dev == NULL)
		return size;

	size += sizeof(struct iio_ioctl_dev_info_buffer);

	/* iio_ioctl_dev_info */
	alignment =  alignof(struct iio_ioctl_dev_info);
	size += pad(size, alignment);
	size += sizeof(struct iio_ioctl_dev_info);  

	/* name */
	if (indio_dev->name != NULL) {
		alignment = alignof(char);
		size += pad(size, alignment);
		size += strlen(indio_dev->name) + 1;
	}

	return size;
}

static long iio_get_device_info_buffer(struct iio_dev *indio_dev,
				unsigned long arg)
{
	struct iio_ioctl_dev_info_buffer __user *dibuff =
		(struct iio_ioctl_dev_info_buffer *)arg;
	struct iio_ioctl_dev_info_buffer mydibuf;
	struct iio_ioctl_dev_info mydi; 
	unsigned int total_size = 0;
	unsigned int offset;
	unsigned int dev_info_offset;
	unsigned int alignment; 

	/*
	 * validate arguments
	 */
	if (dibuff == NULL)
		return -EINVAL;

	/* check that we can read and write the fields of the structure */
	if (access_ok(VERIFY_WRITE, dibuff, sizeof(*dibuff)) == 0)
		return -EFAULT;

	/* copy in the data from user-mode and validate the members */
	if (copy_from_user(&mydibuf, dibuff, sizeof(*dibuff)))
		return -EFAULT;

	/* check that we recognize the version of the iio_ioctl_dev_info_buffer
	 */
	if (mydibuf.size != sizeof(struct iio_ioctl_dev_info_buffer)) {
		dev_err(indio_dev->dev.parent,
			"iio_ioctl_dev_info_buffer.size value is unexpected.  Expected %i got %i",
			sizeof(struct iio_ioctl_dev_info_buffer),
			mydibuf.size);
		return -EINVAL;
	}

	/* check that we have enough memory in the user buffer */
	total_size = iio_get_device_info_buffer_total_size(indio_dev);
	if (mydibuf.total_size < total_size) {
		dev_err(indio_dev->dev.parent,
			"Not enough space for device info.  needed %i got %i",
			total_size, mydibuf.total_size);
		return -EINVAL;
	}

	/* now check access to the entire thing including the memory  after the
	 * buffer used to hold the data pointed to by the embedded pointers.
	 */
	if (access_ok(VERIFY_WRITE, dibuff, mydibuf.total_size) == 0)
		return -EFAULT;
	/*
	 * Copy over the data
	 */

	/* fill in the data we know, putting in NULL for the embedded
	 * pointers. We'll calculate those later.
	 */
	mydi.size = sizeof(struct iio_ioctl_dev_info); 
	mydi.modes = indio_dev->modes;
	mydi.currentmode = indio_dev->currentmode;
	mydi.masklength = indio_dev->masklength;
	mydi.num_channels = indio_dev->num_channels;
	mydi.name = NULL;

	/* calculate the offsets for the dev_info structure  */
	offset = sizeof(struct iio_ioctl_dev_info_buffer);
	alignment =  alignof(struct iio_ioctl_dev_info);
	offset += pad(offset, alignment);
	dev_info_offset = offset; 

	/* name */
	if (indio_dev->name != NULL) {
		offset += sizeof(mydi);
		alignment = alignof(char);
		offset += pad(offset, alignment);
		if (iio_copy_embedded_string_to_user((char *)dibuff, &offset,
				total_size, indio_dev->name,
				(char **)&(mydi.name)) < 0)
			return -EFAULT;
	}

	/* struct iio_ioctl_dev_info */ 
	if (copy_to_user((char *)dibuff + dev_info_offset, &mydi, sizeof(mydi)))
		return -EFAULT;

	/* now that the embedded data has been copied and the
	 * offsets calculated, we can copy the main structure
	 * back to user mode
	 */
	mydibuf.dev_info = 
			(struct iio_ioctl_dev_info *)((char *)dibuff + dev_info_offset);
	if (copy_to_user(dibuff, &mydibuf, sizeof(*dibuff)))
		return -EFAULT;


	return 0;
}


/**
 * iio_check_valid_channel
 *
 * precondition:  indio_dev is not NULL
 * postcondition: return is non-negative on succes or negative on failure.
 */
static long iio_check_valid_channel(const struct iio_dev *indio_dev, int index)
{

	/* check that the channel number is valid */
	if ((index < 0) || (index >= indio_dev->num_channels)) {
		dev_err(indio_dev->dev.parent,
				"channel index [%i] is out of range", index);
		return -EFAULT;
	}

	return 0;
}


static int iio_count_ext_info(const struct iio_chan_spec_ext_info *ext_info)
{
	int index = 0; 

	if (ext_info == NULL)
		return 0; 

	/* interate till you find the NULL name.  At that point you will have 
	 * index as the index of the NULL. That index is one past the last 
	 * valid index, which is our count. 
	 */
	while (ext_info[index].name != NULL)
		index++;

	return index; 
}


/*
 * Gets the total size of the buffer and assumes that all of the parameters
 * have already been validated.
 *
 * Warning: The implementation of this function is tightly coupled to
 * iio_get_channel_spec_buffer_ioctl as the size calculated depends on the
 * padding needed and that depends exactly on how the fields are laid out.
 */
unsigned int iio_get_channel_spec_buffer_total_size(struct iio_dev *indio_dev,
		int index)
{
	unsigned int size = 0;
	struct iio_chan_spec const *spec;
	unsigned int alignment;
	struct iio_chan_spec_ext_info const *ext_info;
	int count; 

	spec = &(indio_dev->channels[index]);

	size += sizeof(struct iio_ioctl_chan_spec_buffer);

	/* iio_ioctl_chan_spec */
	alignment = alignof(struct iio_ioctl_chan_spec);
	size += pad(size, alignment);
	size += sizeof(struct iio_ioctl_chan_spec);

	/* event_spec */
	alignment = alignof(struct iio_ioctl_event_spec);
	size += pad(size, alignment);
	size += spec->num_event_specs * sizeof(struct iio_ioctl_event_spec);

	/* ext_info
	 * The array is NULL terminated, the last element should
	 * have its name field set to NULL
	 *
	 */
	if (spec->ext_info != NULL) {
		alignment = alignof(struct iio_ioctl_chan_spec_ext_info);
		size += pad(size, alignment);

		/* get the count of ext-info structs */ 
		count = iio_count_ext_info(spec->ext_info);

		/* include space for all of them, plus one for the null */ 
		size += (count) * 
				sizeof(struct iio_ioctl_chan_spec_ext_info);


		/* iterate through the array again and capture the length of
		 * each name
		 */
		ext_info = spec->ext_info;
		while (ext_info->name != NULL) {
			alignment = alignof(char);
			size += pad(size, alignment);
			size += strlen(ext_info->name) + 1;
			ext_info += 1;
		}
	}

	/* extend_name */
	if (spec->extend_name != NULL) {
		alignment = alignof(char);
		size += pad(size, alignment);
		size += strlen(spec->extend_name) + 1;
	}

	/* datasheet_name */
	if (spec->datasheet_name != NULL) {
		alignment = alignof(char);
		size += pad(size, alignment);
		size += strlen(spec->datasheet_name) + 1;
	}

	return size;
}


static long iio_get_channel_spec_buffer_total_size_ioctl(
		struct iio_dev *indio_dev, unsigned long arg)
{
	struct iio_ioctl_chan_spec_buffer_size __user *buff_size =
			(struct iio_ioctl_chan_spec_buffer_size *)arg;
	struct iio_ioctl_chan_spec_buffer_size mybuffsize;
	long ret = 0;

	/* validate arguments */
	if (buff_size == NULL)
		return -EINVAL;

	/* check that channels is not null */
	if (indio_dev->channels == NULL)
		return -EFAULT;

	/* check that we can read and write the user data */
	if (access_ok(VERIFY_WRITE, buff_size, sizeof(*buff_size)) == 0)
		return -EFAULT;

	if (__copy_from_user(&mybuffsize, buff_size, sizeof(*buff_size)))
		return -EFAULT;

	/* check that we recognize the version of the
	 * iio_ioctl_chan_spec_buffer_size
	 */
	if (mybuffsize.size != sizeof(struct iio_ioctl_chan_spec_buffer_size)) {
		dev_err(indio_dev->dev.parent,
				"iio_ioctl_chan_buffer_size.size value is unexpected.  Expected %u got %u",
				sizeof(struct iio_ioctl_chan_spec_buffer_size),
				mybuffsize.size);
		return -EINVAL;
	}

	/* check that the channel number is valid */
	ret = iio_check_valid_channel(indio_dev, mybuffsize.index);
	if (ret < 0)
		return ret;

	/* calculate the size and the data back to user mode */
	mybuffsize.total_size =
			iio_get_channel_spec_buffer_total_size(indio_dev,
			mybuffsize.index);

	if (__copy_to_user(buff_size, &mybuffsize, sizeof(*buff_size)))
		return -EFAULT;

	return 0;
}

static int iio_copy_event_spec(const struct iio_event_spec *from,
		int num_event_specs,
		struct iio_ioctl_chan_spec_buffer __user *base,
		unsigned int *offset, unsigned int size)
{
	int ret = 0;
	int i = 0;
	struct iio_ioctl_event_spec event_spec;
	int align = 0;

	/* event_spec: calculate offset and set embedded pointer */
	if (from != NULL) {
		align =  alignof(struct iio_ioctl_event_spec);
		*offset += pad(*offset, align);

		/* go through one by one and convert each event spec */
		event_spec.size = sizeof(event_spec);
		event_spec.next = NULL;
		for (i = 0; i < num_event_specs; i++) {
			
			ret = iio_convert_event_spec(&from[i], &event_spec);
			if (ret < 0)
				return ret;

			/* check that we have room */ 
			BUG_ON(*offset + sizeof(event_spec) > size);

			/* set the next pointer for all but the last one */ 
			if (num_event_specs > 1 && i < num_event_specs - 1){
				event_spec.next = (struct iio_ioctl_event_spec *)
						((char *)base + *offset + 
						sizeof(struct iio_ioctl_event_spec));
			}
			else {
				event_spec.next = NULL; 
			}

			if (copy_to_user(((char *)base + *offset), &event_spec,
					sizeof(event_spec)))
				return -EFAULT;

			*offset += sizeof(event_spec);
		}
	}

	return 0;
}




/* this one is a bit tricky.  This is a "null terminated array". So the pointer
 * is an array but the name is NULL for the last entry. The algorithm does the
 * conversion in two passes. One pass to copy over the array without fixing
 * up the pointers. And a second pass to fix the pointers.
 */
static int iio_copy_ext_info(const struct iio_chan_spec_ext_info *from,
		struct iio_ioctl_chan_spec* channel,
		char __user *base, unsigned int *offset, unsigned int size)
{
	struct iio_ioctl_chan_spec_ext_info __user *to;
	int ret;
	struct iio_ioctl_chan_spec_ext_info ext_info;
	int index; /* index into "from" */
	unsigned int align;
	unsigned int count;
	unsigned int my_offset = *offset;

	if (from != NULL) {
		align = alignof(struct iio_ioctl_chan_spec_ext_info);
		my_offset += pad(my_offset, align);

		/* copy over the pointer to the array into the containing 
		 * structure
		 */
		to = (struct iio_ioctl_chan_spec_ext_info *)(base + my_offset);
		channel->ext_info = (struct iio_ioctl_chan_spec_ext_info *)(base + 
				my_offset); 

		/* count the number of elements in the NULL termintaed array.
		 * The last element of the array has a NULL name member.
		 */
		count = iio_count_ext_info(from);

		/* now copy over the embedded string and the matching element */

		/* get the offset past the variable length array, do not include one 
		 * extra for the null array.  We are copying into a list and the NULL
		 * next pointer serves the purpose of the null array element. 
		 */
		my_offset += (count) *
				sizeof(struct iio_ioctl_chan_spec_ext_info);

		/* now walk through ext_info and copy over the embedded
		 * string and capture the addresses into the containing
		 * structure. Once done, copy over the containing strucure.
		 * Note: we do not add the null strucure to the list. 
		 */
		for (index = 0; index < count; index++) {
			align = alignof(char);
			my_offset += pad(my_offset, align);
			memset(&ext_info, 0, sizeof(ext_info));
			ext_info.size = sizeof(ext_info); 
			ext_info.next = NULL; 

			ret = iio_convert_shared_by(
					from[index].shared, &ext_info.shared);
			if (ret < 0)
				return ret;

			/* handle the next pointer */
			if (count > 1 && index < count - 1){
				ext_info.next = &to[index + 1];
			}
			else {
				ext_info.next = NULL; 
			}

			ret = iio_copy_embedded_string_to_user(
					(char *)base, &my_offset, size,
					from[index].name,
					(char **)&ext_info.name);
			if (ret < 0)
				return ret;

			/* copy the containing ext_info structure */
			if (copy_to_user(&to[index],
					&ext_info, sizeof(ext_info)))
				return -EFAULT;

		}

		*offset = my_offset;
	}


	return 0;
}


static long iio_get_channel_spec_buffer(struct iio_dev *indio_dev,
		unsigned long arg)
{
	struct iio_ioctl_chan_spec_buffer __user *csbuff =
		(struct iio_ioctl_chan_spec_buffer *)arg;
	struct iio_ioctl_chan_spec_buffer mycsbuff;
	struct iio_ioctl_chan_spec mycs; 
	struct iio_chan_spec const *spec = NULL;
	long ret = 0;
	unsigned int offset = 0;
	unsigned int chan_spec_offset;
	unsigned int alignment; 
	unsigned int size = 0;

	/*
	 * validate arguments
	 */
	if (csbuff == NULL)
		return -EINVAL;

	/* check that we can read and write the user data */
	if (access_ok(VERIFY_WRITE, csbuff, sizeof(*csbuff)) == 0)
		return -EFAULT;

	if (copy_from_user(&mycsbuff, csbuff, sizeof(*csbuff)))
		return -EFAULT;

	/* check that we recognize the size of the spec buffer */
	if (mycsbuff.size != sizeof(struct iio_ioctl_chan_spec_buffer)) {
		dev_err(indio_dev->dev.parent,
			"iio_ioctl_chan_spec_buffer.size value is unexpected.  Expected %u got %lu",
			sizeof(struct iio_ioctl_chan_spec_buffer),
			mycsbuff.size);
		return -EINVAL;
	}

	/* check that channels is not null */
	if (indio_dev->channels == NULL)
		return -EFAULT;

	/* check that the channel number is valid */
	ret = iio_check_valid_channel(indio_dev, mycsbuff.index);
	if (ret < 0)
		return ret;

	/* check that we have enough room */
	size = iio_get_channel_spec_buffer_total_size(indio_dev,
			mycsbuff.index);
	if (mycsbuff.total_size < size) {
		dev_err(indio_dev->dev.parent,
				"Not enough space for iio_ioctl_channel_spec_buffer. needed %u got %lu",
				size, mycsbuff.total_size);
		return -EINVAL;
	}

	spec = &(indio_dev->channels[mycsbuff.index]);

	/*
	 *  Grab the data, copy it in as we calculate Offsets
	 */

	/* copy over the static values to our local buffer */
	mycs.size = sizeof(mycs); 
	ret = iio_shallow_copy_chan_spec(spec, &mycs);
	if (ret < 0) {
		/* this can only happen if the kernel definition has changed and
		 * the conversion function was not updated
		 */
		dev_err(indio_dev->dev.parent,
				"Failed to convert iio_ioctl_chan_spec");
		return ret;
	}

	/* now we have to fixup the 5 pointers, start by getting the offset past
	 *  the end of the main structureunsigned int
	 */
	offset += sizeof(struct iio_ioctl_chan_spec_buffer);
	alignment = alignof(struct iio_ioctl_chan_spec);
	offset += pad(offset, alignment);
	chan_spec_offset = offset; 
	mycsbuff.channel = 
			(struct iio_ioctl_chan_spec *)((char *)csbuff + chan_spec_offset); 
	offset += sizeof(struct iio_ioctl_chan_spec);

	/* event_spec: calculate offset and set embedded pointer */
	ret = iio_copy_event_spec(spec->event_spec, spec->num_event_specs,
			csbuff, &offset, size);
	if (ret < 0) {
		dev_err(indio_dev->dev.parent,
				"Failed to convert iio_ioctl_event_spec");
		return ret;
	}

	/* copy over ext_info. Note this is a variable length array */
	ret = iio_copy_ext_info(spec->ext_info, &mycs, 
			(char __user *)csbuff, &offset, size);
	if (ret < 0) {
		dev_err(indio_dev->dev.parent,
				"Failed to convert iio_ioctl_ext_info");
		return ret;
	}

	ret = iio_copy_embedded_string_to_user((char *)csbuff, &offset, size,
			spec->extend_name,
			(char **)&(mycs.extend_name));
	if (ret < 0)
		return ret;

	ret = iio_copy_embedded_string_to_user((char *)csbuff, &offset, size,
			spec->datasheet_name,
			(char **)&(mycs.datasheet_name));
	if (ret < 0)
		return ret;

	if (copy_to_user((char *)csbuff + chan_spec_offset, &mycs, 
			sizeof(mycs)))
		return -EFAULT;

	/* copy over the main structure.  We have captured the address of all of
	 * the embedded pointers
	 */

	if (copy_to_user(csbuff, &mycsbuff, sizeof(mycsbuff)))
		return -EFAULT;

	return 0;
}

static int iio_nosysfs_validate_raw(struct iio_dev *indio_dev,
		struct iio_ioctl_raw_channel_info __user *info,
		struct iio_ioctl_raw_channel_info *myinfo)
{
	int ret;

	if (copy_from_user(myinfo, info, sizeof(*info)))
		return -EFAULT;

	/* check that the size of the strucure is recognized */
	if (myinfo->size != sizeof(struct iio_ioctl_raw_channel_info))
		return -EINVAL; 

	/* check that the channel number is valid */
	ret = iio_check_valid_channel(indio_dev, myinfo->index);
	if (ret < 0)
		return ret;

	return 0;
}

static long iio_nosysfs_read_raw(struct iio_dev *indio_dev, unsigned long arg)
{
	struct iio_ioctl_raw_channel_info __user *info =
			(struct iio_ioctl_raw_channel_info __user *)arg;
	struct iio_ioctl_raw_channel_info myinfo;
	long ret = 0;
	long mask;

	/* check that the driver has a read_raw function */
	if (indio_dev->info->read_raw == NULL)
		return -EOPNOTSUPP;

	/* validate arguments */
	ret = iio_nosysfs_validate_raw(indio_dev, info, &myinfo);
	if (ret < 0)
		return ret;

	ret = iio_convert_chan_info_inverse(myinfo.mask,
			(enum iio_chan_info_enum *)&mask);
	if (ret < 0)
		return ret;

	ret = indio_dev->info->read_raw(indio_dev,
			&(indio_dev->channels[myinfo.index]),
			&(myinfo.val), &(myinfo.val2), mask);
	if (ret > 0) {
		if (__copy_to_user(info, &myinfo, sizeof(*info)))
			return -EFAULT;
	}

	return ret;
}

static long iio_nosysfs_write_raw(struct iio_dev *indio_dev, unsigned long arg)
{
	struct iio_ioctl_raw_channel_info __user *info =
			(struct iio_ioctl_raw_channel_info __user *)arg;
	struct iio_ioctl_raw_channel_info myinfo;
	long ret = 0;
	long mask;

	/* check that the driver has a write_raw function */
	if (indio_dev->info->write_raw == NULL)
		return -EOPNOTSUPP;

	/* validate arguments */
	ret = iio_nosysfs_validate_raw(indio_dev, info, &myinfo);
	if (ret < 0)
		return ret;

	ret = iio_convert_chan_info_inverse(myinfo.mask,
			(enum iio_chan_info_enum *)&mask);
	if (ret < 0)
		return ret;

	ret = indio_dev->info->write_raw(indio_dev,
			&(indio_dev->channels[myinfo.index]),
			myinfo.val, myinfo.val2, mask);

	return ret;
}

static long iio_validate_ext_info(struct iio_dev *indio_dev, 
		unsigned int channel_index, unsigned int info_index)
{
	const struct iio_chan_spec *channel;
	unsigned int count; 

	if (channel_index >= indio_dev->num_channels)
		return -EINVAL;
	
	channel = &indio_dev->channels[channel_index];
	count = iio_count_ext_info(channel->ext_info);
	if (info_index >= count)
		return -EINVAL; 

	return 0;
}

static long iio_nosysfs_read_ch_ext_inf(struct iio_dev *indio_dev,  
		unsigned long arg)
{
	struct iio_ioctl_read_chan_ext_info __user *info =
			(struct iio_ioctl_read_chan_ext_info __user *)arg;
	struct iio_ioctl_read_chan_ext_info myinfo;
	long ret = 0;
	const struct iio_chan_spec_ext_info *ext_info;
	const struct iio_chan_spec *channel;

	char *my_buffer;

	if (copy_from_user(&myinfo, info, sizeof(*info)))
		return -EFAULT;
	
	if (myinfo.size != sizeof(struct iio_ioctl_read_chan_ext_info))
			return -EINVAL; 

	ret = iio_validate_ext_info(indio_dev, myinfo.channel_index, myinfo.info_index);
	if (ret < 0) 
		return ret; 

	channel = &indio_dev->channels[myinfo.channel_index];
	ext_info = &channel->ext_info[myinfo.info_index];
	if (ext_info->read == NULL)
		return -EOPNOTSUPP; 

	my_buffer = (char *)kzalloc(PAGE_SIZE, GFP_KERNEL);
	if (my_buffer == NULL)
		return -ENOMEM;

	// ret does not include the null character, 
	ret = ext_info->read(indio_dev, ext_info->private, channel, 
			my_buffer);

	if (ret >= 0){
		if(copy_to_user(myinfo.buffer, my_buffer, 
				(ret + 1) > myinfo.length ? myinfo.length : (ret + 1)))
			ret = -EFAULT;
	}
	
	kfree(my_buffer);
	return ret; 
}

static long iio_nosysfs_write_ch_ext_inf(struct iio_dev *indio_dev,
		unsigned long arg)
{
	struct iio_ioctl_write_chan_ext_info __user *info =
			(struct iio_ioctl_write_chan_ext_info __user *)arg;
	struct iio_ioctl_write_chan_ext_info myinfo;
	long ret = 0;
	const struct iio_chan_spec_ext_info *ext_info;
	const struct iio_chan_spec *channel;
	char *my_buffer; 

	if (copy_from_user(&myinfo, info, sizeof(*info)))
		return -EFAULT;

	if (myinfo.size != sizeof(myinfo))
		return -EINVAL;

	ret = iio_validate_ext_info(indio_dev, myinfo.channel_index, 
			myinfo.info_index);
	if (ret < 0)
		return ret; 
		

	/* check that the length is sane. Better be less than a page and not
	zero so that our allocation is sane */
	if (myinfo.length == 0 || myinfo.length > PAGE_SIZE)
		return -EINVAL; 

	channel = &indio_dev->channels[myinfo.channel_index];
	ext_info = &channel->ext_info[myinfo.info_index]; 
	if (ext_info->write == NULL)
		return -EOPNOTSUPP;

	my_buffer = (char *)kzalloc(myinfo.length, GFP_KERNEL);
	if(my_buffer == NULL)
		return -ENOMEM;

	if(copy_from_user(my_buffer, myinfo.buffer, myinfo.length)){
		ret = -EFAULT;
		goto err_free;
	}
	/* ensure we are null terminated, this is safe because length is not 
	 * zero and not more than a page, and we did it in our own memory. 
	 */
	my_buffer[myinfo.length -1] = '\0'; 

	ret = ext_info->write(indio_dev, ext_info->private, 
			channel, my_buffer, myinfo.length);

err_free: 
	kfree(my_buffer);
	return ret;  
}

#ifdef CONFIG_IIO_BUFFER

static long iio_process_buffer_ioctls(struct iio_dev *indio_dev,
		unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	int i;
	bool enabled; 

	/* check that we even have a buffer.  No need to continue if the
	 * underlying driver doesn't even have a buffer.
	 */
	if (indio_dev->buffer == NULL)
		return -EINVAL;

	switch (cmd) {
	case IIO_SCAN_MASK_QUERY_BIT_IOCTL:
		if (arg >= indio_dev->masklength)
			return -EINVAL;
		ret = iio_scan_mask_query(indio_dev, indio_dev->buffer, arg);
		return ret;

	case IIO_SCAN_MASK_SET_BIT_IOCTL:
		if (arg >= indio_dev->masklength)
			return -EINVAL;
		ret = iio_scan_mask_set(indio_dev, indio_dev->buffer, arg);
		return ret;

	case IIO_SCAN_MASK_CLEAR_BIT_IOCTL:
		if (arg >= indio_dev->masklength)
			return -EINVAL;
		ret = iio_scan_mask_clear(indio_dev->buffer, arg);
		return ret;

	case IIO_BUFFER_GET_ENABLE_IOCTL:
		enabled = iio_buffer_get_enable(indio_dev);
		return enabled ? 1 : 0;

	case IIO_BUFFER_SET_ENABLE_IOCTL:
		i = (arg != 0);
		ret = iio_buffer_set_enable(indio_dev, i);
		return ret;

	case IIO_BUFFER_GET_LENGTH_IOCTL:
		ret = iio_buffer_get_length(indio_dev);
		return ret;

	case IIO_BUFFER_SET_LENGTH_IOCTL:
		// limit the buffer size to 512 scan-lines.
		if (arg > 512){
			dev_err(indio_dev->dev.parent,
					"Maximum Buffer Length is 512, requested length is: %lu\n",
					arg);
			return -EINVAL; 
		}

		ret = iio_buffer_set_length(indio_dev, arg);
		return ret;

	case IIO_BUFFER_SET_WATERMARK_IOCTL:
		ret = iio_buffer_set_watermark(indio_dev, arg);
		return ret;

	case IIO_BUFFER_GET_WATERMARK_IOCTL:
		ret = iio_buffer_get_watermark(indio_dev);
		return ret;
	}

	return -EINVAL;
}

#endif


long iio_nosysfs_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct iio_dev *indio_dev = filp->private_data;
	int __user *ip = (int __user *)arg;
	unsigned int size = 0;
	int ret = 0;

	switch (cmd) {
	case IIO_GET_DEVICE_INFO_BUFFER_TOTAL_SIZE_IOCTL:
		size = iio_get_device_info_buffer_total_size(indio_dev);
		ret = put_user(size, ip);
		if (ret < 0)
			return -EFAULT;
		return 0;

	case IIO_GET_DEVICE_INFO_BUFFER_IOCTL:
		return iio_get_device_info_buffer(indio_dev, arg);

	case IIO_GET_CHANNEL_SPEC_BUFFER_TOTAL_SIZE_IOCTL:
		return iio_get_channel_spec_buffer_total_size_ioctl(indio_dev,
				arg);

	case IIO_GET_CHANNEL_SPEC_BUFFER_IOCTL:
		return iio_get_channel_spec_buffer(indio_dev, arg);

	case IIO_READ_RAW_CHANNEL_INFO_IOCTL:
		return iio_nosysfs_read_raw(indio_dev, arg);

	case IIO_WRITE_RAW_CHANNEL_INFO_IOCTL:
		return iio_nosysfs_write_raw(indio_dev, arg);

	case IIO_READ_CHANNEL_EXT_INFO_IOCTL:
		return iio_nosysfs_read_ch_ext_inf(indio_dev, arg);

	case IIO_WRITE_CHANNEL_EXT_INFO_IOCTL:
		return iio_nosysfs_write_ch_ext_inf(indio_dev, arg);

#ifdef CONFIG_IIO_BUFFER

	case IIO_SCAN_MASK_QUERY_BIT_IOCTL:
	case IIO_SCAN_MASK_SET_BIT_IOCTL:
	case IIO_SCAN_MASK_CLEAR_BIT_IOCTL:
	case IIO_BUFFER_GET_ENABLE_IOCTL:
	case IIO_BUFFER_SET_ENABLE_IOCTL:
	case IIO_BUFFER_GET_LENGTH_IOCTL:
	case IIO_BUFFER_SET_LENGTH_IOCTL:
	case IIO_BUFFER_SET_WATERMARK_IOCTL:
	case IIO_BUFFER_GET_WATERMARK_IOCTL:
		return iio_process_buffer_ioctls(indio_dev, cmd, arg);

#endif
	}

	/* If we got here, then we don't handle the IOCTL */
	dev_err(indio_dev->dev.parent,"IIOCTL 0x%02x not recognized\n", cmd);
	return -EINVAL;
}
