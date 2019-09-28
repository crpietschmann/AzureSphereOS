// SPDX-License-Identifier: GPL-2.0
/*
 * MT3620 Pluton driver
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

#ifdef CONFIG_MT3620_PLUTON_DEBUG

// Set DEBUG to 1 to enable debug log output
#define DEBUG 1

#endif

#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mailbox_client.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/skbuff.h>
#include <linux/spinlock_types.h>
#include <linux/timer.h>
#include <linux/types.h>
#include <azure-sphere/pluton_remoteapi.h>
#include <mt3620/mt3620_mailbox.h>

#include "pluton_remoteapi_provider.h"

#define DRIVER_NAME "mt3620-pluton"

// Our basic device state structure
struct mt3620_pluton {
	// Our client to the mailbox controller
	struct mbox_client client;
	// Our mailbox channel for events
	struct mbox_chan *event_channel;
	// The underlying linux kernel device
	struct device *dev;
};

static struct mt3620_pluton *g_pluton;

///
/// Callback fired when the mailbox has data
///
/// @cl - mbox_client for the mailbox
/// @mesg - Data from the mailbox in the form of a mt3620_mailbox_data struct
static void mt3620_pluton_rx_callback(struct mbox_client *cl, void *mesg)
{
	struct mt3620_mailbox_data *mailbox_data =
	    (struct mt3620_mailbox_data *)mesg;
	
	union pluton_remoteapi_command_encoded cmd;
	cmd.as_u32 = mailbox_data->cmd;

	// Proxy off to remote API for processing
	pluton_remoteapi_incoming_message(cmd);
}

static int mt3620_send_ring_buffers(dma_addr_t buffers_addr, union pluton_remoteapi_command_encoded cmd)
{
	int ret = SUCCESS;
	struct mt3620_mailbox_data *mailboxData;

	// Wrap it all into our final mailbox structure
	mailboxData = kmalloc(sizeof(*mailboxData), GFP_KERNEL);
	if (mailboxData == NULL) {
		ret = -ENOMEM;
		goto exit;
	}

	mailboxData->data = buffers_addr;
	mailboxData->cmd = cmd.as_u32;

	dev_dbg(g_pluton->dev,
		"Sending event buffers with buffer at %x", buffers_addr);

	// Send the message
	ret = mbox_send_message(g_pluton->event_channel, mailboxData);

	if (ret > 0) {
		// Returns > 0 for success, map it back to 0 so the caller isn't
		// confused
		ret = SUCCESS;
	}

exit:
	if (ret != SUCCESS) {
		if (mailboxData != NULL) {
			kfree(mailboxData);
		}
	}
	return ret;
}

static int mt3620_send_message(union pluton_remoteapi_command_encoded cmd)
{
	int ret = SUCCESS;
	struct mt3620_mailbox_data *mailboxData = NULL;

	// Wrap it all into our final mailbox structure
	mailboxData = kmalloc(sizeof(*mailboxData), GFP_KERNEL);
	if (mailboxData == NULL) {
		ret = -ENOMEM;
		goto exit;
	}

	mailboxData->data = 0;
	mailboxData->cmd = cmd.as_u32;

	// Send the message
	ret = mbox_send_message(g_pluton->event_channel, mailboxData);
	if (ret > 0) {
		// Returns > 0 for success, map it back to 0 so the caller isn't
		// confused
		ret = SUCCESS;
	}

exit:
	if (ret != SUCCESS) {
		if (mailboxData != NULL) {
			kfree(mailboxData);
		}
	}

	return ret;
}

static struct pluton_remoteapi_provider mt3620_pluton_provider = {
	.send_ring_buffers = mt3620_send_ring_buffers,
	.send_message = mt3620_send_message
};

///
/// Initialize the pluton driver
///
/// @pdev - Platform device for this module
/// @returns -  0 for success
static int mt3620_pluton_probe(struct platform_device *pdev)
{
	struct mt3620_pluton *pluton;
	int ret = SUCCESS;

	// Allocate memory for our driver state
	pluton = devm_kzalloc(&pdev->dev, sizeof(*pluton), GFP_KERNEL);
	if (!pluton) {
		return -ENOMEM;
	}

	pluton->dev = &pdev->dev;

	pluton->client.dev = pluton->dev;
	pluton->client.tx_block = false;
	pluton->client.knows_txdone = false;
	pluton->client.rx_callback = mt3620_pluton_rx_callback;

	// Get our mailbox channels
	pluton->event_channel = mbox_request_channel(&pluton->client, 0);

	// Note these can and will fail early on with -EPROBE_DEFER if the mailbox isn't ready
	if (IS_ERR(pluton->event_channel)) {
		ret = PTR_ERR(pluton->event_channel);
		goto exit;
	}

	platform_set_drvdata(pdev, pluton);

	// Set up our global pointer, this will be used by the remote API
	g_pluton = pluton;

	dev_info(pluton->dev, "Starting mt3620 Pluton driver");

	mt3620_pluton_provider.dev = pluton->dev;

	ret = pluton_remoteapi_register_provider(&mt3620_pluton_provider);

exit:
	if (ret != SUCCESS) {
		g_pluton = NULL;
		platform_set_drvdata(pdev, NULL);

		if (pluton != NULL) {
			devm_kfree(&pdev->dev, pluton);
		}
	}

	return ret;
}

///
/// Teardown the pluton driver
///
/// @pdev - Platform device for this module
/// @returns -  0 for success
static int mt3620_pluton_remove(struct platform_device *pdev)
{
	struct mt3620_pluton *pluton = platform_get_drvdata(pdev);

	if (!pluton) {
		return -EINVAL;
	}

	pluton_remoteapi_unregister_provider(&mt3620_pluton_provider);

	mbox_free_channel(pluton->event_channel);

	return SUCCESS;
}

static const struct of_device_id mt3620_pluton_match[] = {
    {.compatible = "mediatek,mt3620-pluton"}, {/* Sentinel */},
};

MODULE_DEVICE_TABLE(of, mt3620_pluton_match);

static struct platform_driver mt3620_pluton_driver = {
    .probe = mt3620_pluton_probe,
    .remove = mt3620_pluton_remove,
    .driver =
	{
	    .name = DRIVER_NAME, .of_match_table = mt3620_pluton_match,
	},
};

module_platform_driver(mt3620_pluton_driver);

MODULE_LICENSE("GPLv2");
MODULE_DESCRIPTION("MT3620 Pluton driver");
MODULE_AUTHOR("Azure Sphere Team <azuresphereoss@microsoft.com>");
MODULE_ALIAS("platform:mt3620-pluton");
