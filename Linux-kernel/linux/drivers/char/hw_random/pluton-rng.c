// SPDX-License-Identifier: GPL-2.0
/*
 * Pluton hardware random number generation driver
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/hw_random.h>
#include <linux/platform_device.h>
#include <azure-sphere/pluton_remoteapi.h>

#define DRIVER_NAME "pluton-rng"
#define SUCCESS 0

struct pluton_rng {	
	struct hwrng rng;
	struct device *dev;
};

static int pluton_rng_read(struct hwrng *rng, void *buf, 
    size_t max, bool wait)
{
	struct pluton_rng *trng = container_of(rng, struct pluton_rng, rng);	
	int ret;

	ret = pluton_remote_api_send_command_to_m4_sync(
	    READ_RNG, 
		NULL, 0, 
		buf, 4);
	if (ret != SUCCESS) {	
		dev_err(trng->dev, "RNG: pluton_remote_api_send_command_to_m4_sync(..., READ_RNG, ...) failed!\n");		
		return 0;
	}

    return 4/*4 bytes of random data*/;
}

///
/// Initialize the rng driver
///
/// @pdev - Platform device for this module
/// @returns -  0 for success
static int pluton_rng_probe(struct platform_device *pdev)
{
	struct pluton_rng *rng;	
	int ret = SUCCESS;

	ret = pluton_remote_api_is_ready();
	if (ret != 0) {
		return ret;
	}

	// Allocate memory for our driver state
	rng = devm_kzalloc(&pdev->dev, sizeof(*rng), GFP_KERNEL);
	if (!rng) {
		return -ENOMEM;
	}

	rng->dev = &pdev->dev;

	// Register as hwrng
	rng->rng.name = pdev->name;
	rng->rng.read = pluton_rng_read;
	ret = hwrng_register(&rng->rng);
	if (ret != SUCCESS) {		
		return ret;
	}

	platform_set_drvdata(pdev, rng);

	return ret;
}

///
/// Teardown the rng driver
///
/// @pdev - Platform device for this module
/// @returns -  0 for success
static int pluton_rng_remove(struct platform_device *pdev)
{
	struct pluton_rng *rng = platform_get_drvdata(pdev);

	if (!rng) {
		return -EINVAL;
	}

    hwrng_unregister(&rng->rng);

	return SUCCESS;
}

static const struct of_device_id pluton_rng_match[] = {
    {.compatible = "microsoft,pluton-rng"},
    {/* Sentinel */}};

MODULE_DEVICE_TABLE(of, pluton_rng_match);

static struct platform_driver pluton_rng_driver = {
    .probe = pluton_rng_probe,
    .remove = pluton_rng_remove,
    .driver =
	{
	    .name = DRIVER_NAME, .of_match_table = pluton_rng_match,
	},
};

module_platform_driver(pluton_rng_driver);

MODULE_LICENSE("GPLv2");
MODULE_DESCRIPTION("Pluton rng driver");
MODULE_AUTHOR("Azure Sphere Team <azuresphereoss@microsoft.com>");
MODULE_ALIAS("platform:pluton-rng");
