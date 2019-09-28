// SPDX-License-Identifier: GPL-2.0
/*
 * MT362X RESET driver
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

#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/reboot.h>
#include <linux/stat.h>

#include <azure-sphere/security_monitor.h>

// The underlying linux kernel device
static struct device *mt362x_reset_dev = NULL;

///
/// Called by Kernel to power off the H/W
///
static void mt362x_power_off(void)
{
    int err = -ENOENT;

    // TODO: Add H/W shutdown code here

    if (err != 0) {
        dev_emerg(mt362x_reset_dev, "Unable to power off (%d)\n", err);
    }
}

///
/// Event handler registered by register_restart_handler() to handle event from SyS_reboot() in reboot.c
///
/// @this - notifier block for this event
/// @mode - reboot mode
/// @cmd  - reboot command such as 'reboot -f' or 'shutdown -r'
/// @returns -  NOTIFY_DONE for success
static int mt362x_restart(struct notifier_block *this, unsigned long mode,
                 void *cmd)
{
    int ret;
    dev_info(mt362x_reset_dev, "Entering H/W reboot...");
    ret = azure_sphere_sm_reset();
    dev_emerg(mt362x_reset_dev, "Restart failed (%d)\n", ret);

    return NOTIFY_DONE;
}

static struct notifier_block mt362x_restart_nb = {
    .notifier_call = mt362x_restart,
    .priority = 128,
};

static const struct of_device_id mt362x_reset_of_match[] =
{
    {
      // We use reset driver only which does both reboot and power-off
      .compatible = "mediatek,mt362x-reset",
    },
    {}
};

///
/// Registering mt362x_restart function to restart handler list
///
/// @returns -  0 for success
static int mt362x_register_restart_handler(void)
{
    int err;

    err = register_restart_handler(&mt362x_restart_nb);
    if (err) {
        dev_err(mt362x_reset_dev, "cannot register restart handler (err=%d)", err);
        return err;
    }

    return 0;
}

///
/// Initialize the reset driver
///
/// @pdev - Platform device for this module
/// @returns -  0 for success
static int mt362x_reset_probe(struct platform_device *pdev)
{
    int ret;

    // Allocate memory for our driver state
    mt362x_reset_dev = &pdev->dev;

    dev_info(mt362x_reset_dev, "Starting MT362X RESET driver");

    pm_power_off = mt362x_power_off;
    ret = mt362x_register_restart_handler();
    if (ret != 0) {
        return ret;
    }

    dev_dbg(mt362x_reset_dev, "Reset driver setup completed");

    return ret;
}

///
/// Teardown our reset driver
///
/// @pdev - Platform device pointer for this module
/// @returns - 0 for success
static int mt362x_reset_remove(struct platform_device *pdev)
{
    return 0;
}

static struct platform_driver mt362x_reset_driver = {
    .probe = mt362x_reset_probe,
    .remove = mt362x_reset_remove,
    .driver = {
        .name = "mt362x-reset",
        .of_match_table = mt362x_reset_of_match,
    },
};

module_platform_driver(mt362x_reset_driver);

MODULE_LICENSE("GPLv2");
MODULE_DESCRIPTION("MT362X reset controller specific functions");
MODULE_AUTHOR("Azure Sphere Team <azuresphereoss@microsoft.com>");
MODULE_ALIAS("platform:mt362x-reset");

