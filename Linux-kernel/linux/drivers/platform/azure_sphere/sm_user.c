// SPDX-License-Identifier: GPL-2.0
/*
 * Azure Sphere Security Monitor user interface driver
 *
 * Copyright (c) 2018 Microsoft Corporation. All rights reserved.
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

#ifdef CONFIG_AZURE_SPHERE_SECURITY_MONITOR_DEBUG
// Set DEBUG to 1 to enable debug log output
#define DEBUG 1
#endif

#include <linux/device.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <azure-sphere/security_monitor.h>
#include <azure-sphere/pluton_remoteapi.h>

#include <linux/fs.h>
#include <linux/miscdevice.h>

#include "sm_user.h"
#include "caller_security.h"
#include "security_state.h"
#include "pluton.h"
#include "attestation_runtime_operations.h"

#define DRIVER_NAME "azure-sphere-security-monitor-user"

// Global pointer to skuser state.
struct azure_sphere_sm_user_state *g_sm_user = NULL;

static const struct file_operations pluton_fops = {
	.owner = THIS_MODULE,
	.open = pluton_open,
	.release = pluton_release,
	.poll = pluton_poll,
	.read = pluton_read,
	.unlocked_ioctl = pluton_ioctl,
};

static struct miscdevice pluton_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "pluton",
	.fops = &pluton_fops,
	.mode = S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP|S_IROTH|S_IWOTH,
};

static const struct file_operations security_monitor_fops = {
		.owner = THIS_MODULE,
		.unlocked_ioctl = security_monitor_ioctl,
};

static struct miscdevice security_monitor_dev = {
		.minor = MISC_DYNAMIC_MINOR,
		.name = "security-monitor",
		.fops = &security_monitor_fops,
		.mode = S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP|S_IROTH|S_IWOTH,
};

///
/// Initializes our Security Monitor user interface driver
///
/// @pdev - Platform device pointer for this module
/// @returns - 0 for success
static int azure_sphere_sm_probe(struct platform_device *pdev)
{
	int ret;
	struct azure_sphere_sm_user_state *state = NULL;

	ret = pluton_remote_api_is_ready();
	if (ret != 0) {
		// Our dependencies haven't started yet, tell the kernel to try
		// again later
		return ret;
	}

	dev_info(&pdev->dev,
		 "Starting Azure Sphere Security Monitor user interface driver");

	state = devm_kzalloc(&pdev->dev, sizeof(*state), GFP_KERNEL);
	if (!state) {
		return -ENOMEM;
	}

	state->dev = &pdev->dev;

	platform_set_drvdata(pdev, state);

	// Register security monitor endpoint
	ret = misc_register(&security_monitor_dev);
	if (unlikely(ret)) {
		dev_err(&pdev->dev,
			"Error /dev/security-monitor registration failed with %d.\n",
			ret);
		return ret;
	}

	// Register pluton endpoint
	ret = misc_register(&pluton_dev);
	if (unlikely(ret)) {
		dev_err(&pdev->dev,
			"Error /dev/pluton registration failed with %d.\n",
			ret);
		return ret;
	}

	// Stash away the device pointer in a global so we can access it from
	// the APIs we export from this module.
	BUG_ON(g_sm_user != NULL);
	g_sm_user = state;

	return 0;
}

///
/// Teardown our Security Monitor user interface driver
///
/// @pdev - Platform device pointer for this module
/// @returns - 0 for success
static int azure_sphere_sm_remove(struct platform_device *pdev)
{
	BUG_ON(g_sm_user == NULL || g_sm_user->dev != &pdev->dev);

	misc_deregister(&security_monitor_dev);
	misc_deregister(&pluton_dev);

	g_sm_user = NULL;

	return 0;
}

// Driver metadata.
static const struct of_device_id azure_sphere_sm_match[] = {
    {.compatible = "microsoft,security-monitor"}, {/* Sentinel */}};


static struct platform_driver azure_sphere_sm_driver = {
    .probe = azure_sphere_sm_probe,
    .remove = azure_sphere_sm_remove,
    .driver = 
	{
		.name = DRIVER_NAME, .of_match_table = azure_sphere_sm_match
	}};

MODULE_DEVICE_TABLE(of, azure_sphere_sm_match);

module_platform_driver(azure_sphere_sm_driver);

MODULE_LICENSE("GPLv2");
MODULE_DESCRIPTION("Azure Sphere Security Monitor user interface driver");
MODULE_AUTHOR("Azure Sphere Team <azuresphereoss@microsoft.com>");
MODULE_ALIAS("platform:azure-sphere-security-monitor-user");
