// SPDX-License-Identifier: GPL-2.0
/*
 * Character device interface for the generic PWM framework
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

#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pwm.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <uapi/linux/pwm.h>

#define DEVICE_NAME "pwm%u"
#define CLASS_NAME "pwm-chardev"
#define MAX_DEVS 256

static int major_dev;
static struct class *pwm_chardev_class;

struct pwm_dev {
	struct cdev cdev;
	struct device *device;
	struct pwm_chip *chip;
	struct mutex lock;
	bool valid;
};

static int pwm_chardev_show(struct seq_file *m, void *p)
{
	struct pwm_dev *dev = m->private;

	seq_printf(m, "%u\n", dev->chip->npwm);
	return 0;
}

static int pwm_chardev_open(struct inode *inodep, struct file *filp)
{
	struct pwm_dev *dev =
		container_of(inodep->i_cdev, struct pwm_dev, cdev);

	return single_open(filp, pwm_chardev_show, dev);
}

static int pwm_ioctl_apply_state(void __user *arg, struct pwm_dev *data)
{
	int ret = 0;
	struct pwm_chardev_params input_data;
	void __user *user_extended_state;

	memset(&input_data, 0, sizeof(input_data));

	if (copy_from_user(&input_data, arg, sizeof(input_data))) {
		ret = -EFAULT;
		goto out;
	}

	user_extended_state = input_data.state.extended_state;

	if (user_extended_state) {
		input_data.state.extended_state = kzalloc(
			input_data.state.extended_state_size, GFP_KERNEL);

		if (!input_data.state.extended_state) {
			ret = -ENOMEM;
			goto out;
		}
		if (copy_from_user(input_data.state.extended_state,
				   user_extended_state,
				   input_data.state.extended_state_size)) {
			ret = -EFAULT;
			goto out;
		}
	}

	if (input_data.pwm_index >= data->chip->npwm) {
		dev_err(data->device, "pwm_index %u does not exist, npwm: %u\n",
			input_data.pwm_index, data->chip->npwm);
		ret = -ENODEV;
		goto out;
	}

	ret = pwm_apply_state(&data->chip->pwms[input_data.pwm_index],
			      &input_data.state);
	if (ret) {
		dev_err(data->device, "pwm_apply_state error: %d\n", ret);
		goto out;
	}

out:
	// pwm_apply_state keeps a copy of extended_state, so free this
	kfree(input_data.state.extended_state);
	return ret;
}

static int pwm_ioctl_get_state(void __user *arg, struct pwm_dev *data)
{
	int ret = 0;
	struct pwm_chardev_params input_data;
	struct pwm_chardev_params output_data;
	void __user *user_extended_state;
	void *output_extended_state = NULL;

	memset(&output_data, 0, sizeof(output_data));

	if (copy_from_user(&input_data, arg, sizeof(input_data))) {
		ret = -EFAULT;
		goto out;
	}

	user_extended_state = input_data.state.extended_state;

	if (input_data.pwm_index >= data->chip->npwm) {
		dev_err(data->device, "pwm_index %u does not exist, npwm: %u\n",
			input_data.pwm_index, data->chip->npwm);
		ret = -ENODEV;
		goto out;
	}

	output_data.pwm_index = input_data.pwm_index;

	if (user_extended_state) {
		output_data.state.extended_state = kzalloc(
			input_data.state.extended_state_size, GFP_KERNEL);
		output_extended_state = output_data.state.extended_state;

		if (!output_data.state.extended_state) {
			ret = -ENOMEM;
			goto out;
		}

		output_data.state.extended_state_size =
			input_data.state.extended_state_size;
	}

	ret = pwm_get_state_extended(&data->chip->pwms[input_data.pwm_index],
				     &output_data.state);
	if (ret)
		goto out;

	if (user_extended_state && output_data.state.extended_state) {
		if (copy_to_user(user_extended_state,
				 output_data.state.extended_state,
				 output_data.state.extended_state_size)) {
			ret = -EFAULT;
			goto out;
		}
		output_data.state.extended_state = user_extended_state;
	}
	if (copy_to_user(arg, &output_data, sizeof(output_data))) {
		ret = -EFAULT;
		goto out_ext_state_user_ptr;
	}

out_ext_state_user_ptr:
	output_data.state.extended_state = NULL;
out:
	kfree(output_extended_state);
	return ret;
}

static int pwm_ioctl_export(void __user *arg, struct pwm_dev *data)
{
	int ret = 0;
	unsigned int hwpwm = 0;
	struct pwm_device *pwm;

	ret = get_user(hwpwm, (unsigned int __user *)arg);
	if (ret)
		goto out;

	if (hwpwm >= data->chip->npwm) {
		dev_err(data->device, "pwm_index %u does not exist, npwm: %u\n",
			hwpwm, data->chip->npwm);
		ret = -ENODEV;
		goto out;
	}

	pwm = pwm_request_from_chip(data->chip, hwpwm, "chardev");
	if (IS_ERR(pwm)) {
		ret = PTR_ERR(pwm);
		goto out;
	}

	if (test_and_set_bit(PWMF_CHARDEV_EXPORTED, &pwm->flags)) {
		ret = -EBUSY;
		goto out_busy;
	}

	return 0;

out_busy:
	pwm_put(pwm);
out:
	return ret;
}

static int pwm_unexport_child(struct pwm_device *pwm)
{
	int ret = 0;

	if (!test_and_clear_bit(PWMF_CHARDEV_EXPORTED, &pwm->flags)) {
		ret = -ENODEV;
		goto out;
	}

	pwm_put(pwm);

out:
	return ret;
}

static int pwm_ioctl_unexport(void __user *arg, struct pwm_dev *data)
{
	int ret = 0;
	unsigned int hwpwm = 0;
	struct pwm_device *pwm;

	ret = get_user(hwpwm, (unsigned int __user *)arg);
	if (ret)
		goto out;

	if (hwpwm >= data->chip->npwm) {
		dev_err(data->device, "pwm_index %u does not exist, npwm: %u\n",
			hwpwm, data->chip->npwm);
		ret = -ENODEV;
		goto out;
	}

	pwm = &data->chip->pwms[hwpwm];

	ret = pwm_unexport_child(pwm);
	if (ret)
		goto out;

out:
	return ret;
}

static long pwm_chardev_ioctl(struct file *filp, unsigned int cmd,
			      unsigned long arg_)
{
	void __user *arg = (void __user *)arg_;
	struct pwm_dev *data = ((struct seq_file *)filp->private_data)->private;
	long res = 0;

	mutex_lock(&data->lock);
	if (!data->valid) {
		dev_err(data->device, "accessing deinitialized pwm\n");
		res = -ENODEV;
		goto out;
	}

	switch (cmd) {
	case PWM_APPLY_STATE:
		res = pwm_ioctl_apply_state(arg, data);
		break;
	case PWM_GET_STATE:
		res = pwm_ioctl_get_state(arg, data);
		break;
	case PWM_EXPORT:
		res = pwm_ioctl_export(arg, data);
		break;
	case PWM_UNEXPORT:
		res = pwm_ioctl_unexport(arg, data);
		break;
	default:
		if (data->chip->ops->ioctl)
			res = data->chip->ops->ioctl(data->chip, cmd, arg_);
		else
			res = -ENOTTY;
		break;
	}

out:
	mutex_unlock(&data->lock);
	return res;
}

static const struct file_operations fops = { .open = pwm_chardev_open,
					     .read = seq_read,
					     .llseek = seq_lseek,
					     .release = single_release,
					     .unlocked_ioctl =
						     pwm_chardev_ioctl };

static int pwmchip_chardev_match(struct device *parent, const void *data)
{
	struct pwm_dev *pwm_dev = dev_get_drvdata(parent);

	return pwm_dev->chip == data;
}

void pwmchip_chardev_export(struct pwm_chip *chip)
{
	int err;
	struct device *pwm_chardev_device;
	struct pwm_dev *pwm_dev;
	int id = -1;
	dev_t devt;

	if (chip->dev->of_node) {
		// Stable allocation of id based on order in device tree
		id = of_alias_get_id(chip->dev->of_node, "pwm");
	}
	if (id < 0) {
		// Otherwise, mirror sysfs behavior of naming based on first pwm
		// device index on chip
		id = chip->base;
	}

	devt = MKDEV(major_dev, id);

	pwm_chardev_device = device_create(pwm_chardev_class, chip->dev, devt,
					   NULL, DEVICE_NAME, id);
	if (IS_ERR(pwm_chardev_device)) {
		err = PTR_ERR(pwm_chardev_device);
		dev_err(chip->dev, "Error %d creating device for pwm %d", err,
			id);
		goto out;
	}

	pwm_dev = devm_kzalloc(pwm_chardev_device, sizeof(struct pwm_dev),
			       GFP_KERNEL);
	if (!pwm_dev)
		goto out_dev_allocated;

	dev_set_drvdata(pwm_chardev_device, pwm_dev);

	mutex_init(&pwm_dev->lock);
	// Need to lock since the character device will become active in
	// cdev_add
	mutex_lock(&pwm_dev->lock);

	cdev_init(&pwm_dev->cdev, &fops);
	pwm_dev->cdev.owner = THIS_MODULE;
	pwm_dev->cdev.ops = &fops;
	// cdev will hold a reference to its parent and release it when all
	// handles are closed
	pwm_dev->cdev.kobj.parent = &pwm_chardev_device->kobj;

	err = cdev_add(&pwm_dev->cdev, devt, 1);
	if (err) {
		dev_err(pwm_chardev_device, "Error %d adding pwm %d", err, id);
		kobject_put(&pwm_dev->cdev.kobj);
		goto out_locked;
	}

	pwm_dev->device = pwm_chardev_device;
	pwm_dev->chip = chip;

	pwm_dev->valid = true;

	mutex_unlock(&pwm_dev->lock);
	return;

out_locked:
	mutex_unlock(&pwm_dev->lock);
out_dev_allocated:
	device_del(pwm_chardev_device);
out:
	return;
}

void pwmchip_chardev_unexport(struct pwm_chip *chip)
{
	struct device *parent = class_find_device(pwm_chardev_class, NULL, chip,
						  pwmchip_chardev_match);
	struct pwm_dev *pwm_dev;

	if (!parent) {
		dev_err(chip->dev,
			"pwmchip_chardev_unexport: failed to find device\n");
		return;
	}

	pwm_dev = dev_get_drvdata(parent);

	cdev_del(&pwm_dev->cdev);
	// for class_find_device()
	put_device(parent);
	device_destroy(pwm_chardev_class, parent->devt);

	mutex_lock(&pwm_dev->lock);
	pwm_dev->valid = false;
	mutex_unlock(&pwm_dev->lock);
}

void pwmchip_chardev_unexport_children(struct pwm_chip *chip)
{
	int i;
	struct device *parent = class_find_device(pwm_chardev_class, NULL, chip,
						  pwmchip_chardev_match);

	if (!parent) {
		dev_err(chip->dev,
			"pwmchip_chardev_unexport_children: failed to find device\n");
		return;
	}

	for (i = 0; i < chip->npwm; i++) {
		struct pwm_device *pwm = &chip->pwms[i];

		if (test_bit(PWMF_EXPORTED, &pwm->flags))
			pwm_unexport_child(pwm);
	}

	// for class_find_device()
	put_device(parent);
}

static int __init pwm_chardev_init(void)
{
	int res;
	dev_t devt;

	pwm_chardev_class = class_create(THIS_MODULE, CLASS_NAME);
	if (IS_ERR(pwm_chardev_class)) {
		pr_err("pwm_chardev_init: failed to register device class\n");
		res = PTR_ERR(pwm_chardev_class);
		goto out;
	}

	res = alloc_chrdev_region(&devt, 0, MAX_DEVS, CLASS_NAME);
	if (res) {
		pr_err("pwm_chardev_init: unable to alloc chrdev region\n");
		goto out;
	}

	major_dev = MAJOR(devt);

	return 0;

out:
	class_destroy(pwm_chardev_class);
	return res;
}

subsys_initcall(pwm_chardev_init);
