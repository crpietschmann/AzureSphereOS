// SPDX-License-Identifier: GPL-2.0
/*
 * AZURE SPHERE RESET driver
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

#include <linux/dma-mapping.h>
#include <linux/kdebug.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/reboot.h>
#include <linux/slab.h>
#include <linux/stat.h>

#include <asm/uaccess.h>
#include "security_monitor.h"
#include <azure-sphere/security_monitor.h>

//
// The error reporting markers must be handled manually here because the auto generated tools require cmake.
//
#define AggregationKind_Count 0
#define AggregationKind_Sum 1
#define AggregationKind_Average 2
#define AggregationKind_LastValue 3
#define AggregationKind_FirstValue 4
#define AggregationType_None 0
#define AggregationType_UINT16 1
#define AggregationType_UINT32 2
#define AggregationType_Variable 3
#define MAKE_MARKER_ID(Value, AggregationKind, ValueLength) (((uint16_t)(Value) << 6) | ((AggregationKind) << 3) | (ValueLength))

// Kernel error reporting markers
#define NW_KERNEL_KernelPanic MAKE_MARKER_ID(100, AggregationKind_FirstValue, AggregationType_Variable)
#define NW_KERNEL_KernelDie MAKE_MARKER_ID(101, AggregationKind_FirstValue, AggregationType_Variable)

// The underlying linux kernel device
static struct device *azsphere_reset_dev = NULL;

///
/// Called by Kernel to power off the H/W
///
static void azsphere_power_off(void)
{
    int err = -ENOENT;

    // TODO: Add H/W shutdown code here

    if (err != 0)
    {
        dev_emerg(azsphere_reset_dev, "Unable to power off (%d)\n", err);
    }
}

///
/// Event handler registered by register_restart_handler() to handle event from SyS_reboot() in reboot.c
///
/// @this - notifier block for this event
/// @mode - reboot mode
/// @cmd  - reboot command such as 'reboot -f' or 'shutdown -r'
/// @returns -  NOTIFY_DONE for success
static int azsphere_restart(struct notifier_block *this, unsigned long mode,
                            void *cmd)
{
    int ret;
    dev_info(azsphere_reset_dev, "Entering H/W reboot...");
    ret = azure_sphere_sm_reset();
    dev_emerg(azsphere_reset_dev, "Restart failed (%d)\n", ret);

    return NOTIFY_DONE;
}

static struct notifier_block azsphere_restart_nb = {
    .notifier_call = azsphere_restart,
    .priority = 128,
};

static const struct of_device_id azsphere_reset_of_match[] =
    {
        {
            // We use reset driver only which does both reboot and power-off
            .compatible = "microsoft,azsphere-reset",
        },
        {}};

// Global variables used in panic/oops handler
static uint8_t has_panicked = 0;
static uint8_t has_died = 0;
// Contiguous physical memory buffer allocated on reset driver probe and used for error
// reporting. The buffer is freed when the driver is removed.
static char *azsphere_fatal_error_data = NULL;
// The maximum alllowed size for error report data
#define MAX_ERROR_REPORT_BUFFER_SIZE 255
static size_t azsphere_fatal_error_data_size = MAX_ERROR_REPORT_BUFFER_SIZE;

//
// Kernel specific call to SMC call for logging telemetry.
//
static int azure_sphere_sm_record_telemetry_event_data_internal(uint16_t id, uint32_t event_timestamp, uint8_t payload_length, const void *payload)
{
    int err = 0;
    struct arm_smccc_res res;
    dma_addr_t dma_addr;

    dev_info(azsphere_reset_dev, "Security Monitor call: RECORD_TELEMETRY_EVENT_DATA: 0x%X", id);

    if (payload_length == 0 || payload == NULL)
    {
        return -EINVAL;
    }

    dma_addr = dma_map_single(azsphere_reset_dev, (void *)payload, payload_length, DMA_FROM_DEVICE);
    if (dma_mapping_error(azsphere_reset_dev, dma_addr))
    {
        dev_err(azsphere_reset_dev, "Failed dma_map_single");
        return -ENOMEM;
    }

    dma_sync_single_for_device(azsphere_reset_dev, dma_addr, payload_length, DMA_FROM_DEVICE);

    arm_smccc_smc(SECURITY_MONITOR_API_RECORD_TELEMETRY_EVENT_DATA, id, event_timestamp, payload_length, dma_addr, 0, 0, 0, &res);
    err = res.a0;

    if (err != 0)
    {
        dev_err(azsphere_reset_dev,
                "Security Monitor call: RECORD_TELEMETRY_EVENT_DATA returned %#x", err);
    }

    dma_unmap_single(azsphere_reset_dev, dma_addr, payload_length, DMA_FROM_DEVICE);

    return err;
}

///
/// Panic Event handler registered by azsphere_reset_probe() to handle kernel panic events.
///
/// @this - notifier block for this event
/// @event - the panic event
/// @ptr - the panic string
/// @returns -  NOTIFY_DONE for success
static int panic_event(struct notifier_block *this, unsigned long event,
                       void *ptr)
{
    if (has_panicked || has_died)
    {
        return NOTIFY_DONE;
    }
    has_panicked = 1;

    if (azsphere_fatal_error_data != NULL && ptr != NULL)
    {
        int res = 0;

        /* Log Error Report for Panic */
        strncpy(azsphere_fatal_error_data, (const char *)ptr, azsphere_fatal_error_data_size);
        azsphere_fatal_error_data[azsphere_fatal_error_data_size - 1] = 0;
        res = azure_sphere_sm_record_telemetry_event_data_internal(NW_KERNEL_KernelPanic, 0, strlen(azsphere_fatal_error_data), azsphere_fatal_error_data);
        if (0 != res)
        {
            dev_err(azsphere_reset_dev, "Failed to record kernel panic error report: %d - %s", res, (const char *)ptr);
        }
    }
    else
    {
        dev_err(azsphere_reset_dev, "FailRep: No buffer %p : %p", azsphere_fatal_error_data, ptr);
    }

    return NOTIFY_DONE;
}

static struct notifier_block panic_block = {
    .notifier_call = panic_event,
};

//
// Kernel oops telemetry data (must be under 256 bytes).
//
struct oops_telemetry_data
{
    long err;
    int trapnr;
    int signr;
    long reg_sp;
    long reg_lr;
    long reg_pc;
    long reg_cpsr;
};

static_assert((sizeof(struct oops_telemetry_data) <= MAX_ERROR_REPORT_BUFFER_SIZE), "Oops error data is larger than buffer");

///
/// DIE Event handler registered by azsphere_reset_probe() to handle kernel oops/die events.
///
/// @this - notifier block for this event
/// @event - the die event
/// @ptr - the die arguments
/// @returns -  NOTIFY_DONE for success
static int die_event(struct notifier_block *this, unsigned long event,
                     void *ptr)
{
    struct die_args *args = (struct die_args *)ptr;
    struct pt_regs *regs = args->regs;
    struct oops_telemetry_data *err_data = (struct oops_telemetry_data *)azsphere_fatal_error_data;

    if (has_died)
    {
        return NOTIFY_DONE;
    }

    has_died = 1;

    switch (event)
    {
    case DIE_OOPS:
        if (args != NULL && err_data != NULL)
        {
            int res = 0;
            //
            // Copy relevant register values for error reporting structure
            //
            err_data->err = args->err;
            err_data->trapnr = args->trapnr;
            err_data->signr = args->signr;
            err_data->reg_pc = regs->ARM_pc;
            err_data->reg_sp = regs->ARM_sp;
            err_data->reg_lr = regs->ARM_lr;
            err_data->reg_cpsr = regs->ARM_cpsr;
            res = azure_sphere_sm_record_telemetry_event_data_internal(NW_KERNEL_KernelDie, 0, sizeof(*err_data), err_data);
            if (0 != res)
            {
                dev_err(azsphere_reset_dev, "Failed to record die error report: %d - %s", res, args->str);
            }
        }
        else
        {
            dev_err(azsphere_reset_dev, "FailRep: No buffer %p : %p", err_data, args);
        }
        break;
    default:
        break;
    }

    return NOTIFY_DONE;
}

static struct notifier_block die_block = {
    .notifier_call = die_event,
};

///
/// Registering azsphere_restart function to restart handler list
///
/// @returns -  0 for success
static int
azsphere_register_restart_handler(void)
{
    int err;

    err = register_restart_handler(&azsphere_restart_nb);
    if (err)
    {
        dev_err(azsphere_reset_dev, "cannot register restart handler (err=%d)", err);
        return err;
    }

    return 0;
}

///
/// Initialize the reset driver
///
/// @pdev - Platform device for this module
/// @returns -  0 for success
static int azsphere_reset_probe(struct platform_device *pdev)
{
    int ret;

    // Allocate memory for our driver state
    azsphere_reset_dev = &pdev->dev;

    dev_info(azsphere_reset_dev, "Starting azsphere RESET driver");

    pm_power_off = azsphere_power_off;
    ret = azsphere_register_restart_handler();
    if (ret != 0)
    {
        return ret;
    }

    azsphere_fatal_error_data = kzalloc(azsphere_fatal_error_data_size, GFP_KERNEL);
    if (azsphere_fatal_error_data == NULL)
    {
        dev_err(azsphere_reset_dev, "Failed to alloc fatal error data");
    }

    atomic_notifier_chain_register(&panic_notifier_list, &panic_block);
    register_die_notifier(&die_block);

    dev_dbg(azsphere_reset_dev, "Reset driver setup completed");

    return ret;
}

///
/// Teardown our reset driver
///
/// @pdev - Platform device pointer for this module
/// @returns - 0 for success
static int azsphere_reset_remove(struct platform_device *pdev)
{
    if (azsphere_fatal_error_data != NULL)
    {
        kzfree(azsphere_fatal_error_data);
        azsphere_fatal_error_data = NULL;
    }

    return 0;
}

static struct platform_driver azsphere_reset_driver = {
    .probe = azsphere_reset_probe,
    .remove = azsphere_reset_remove,
    .driver = {
        .name = "azsphere-reset",
        .of_match_table = azsphere_reset_of_match,
    },
};

module_platform_driver(azsphere_reset_driver);

MODULE_LICENSE("GPLv2");
MODULE_DESCRIPTION("AZURE SPHERE reset controller specific functions");
MODULE_AUTHOR("Azure Sphere Team <azuresphereoss@microsoft.com>");
MODULE_ALIAS("platform:azsphere-reset");
