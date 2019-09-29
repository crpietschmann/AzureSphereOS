// SPDX-License-Identifier: GPL-2.0
/*
 * Azure Sphere Security Monitor API
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
#define DEBUG 1
#endif

#include <linux/device.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/arm-smccc.h>
#include <asm/uaccess.h>
#include <azure-sphere/security_monitor.h>
#include <azure-sphere/security.h>
#include <uapi/linux/azure-sphere/security_monitor.h>

#include "security_monitor.h"

#define DRIVER_NAME "azure-sphere-security-monitor"

#define SM_MINIMUM_CACHE_COHERENCY_BUFFER_SIZE 4

// Global pointers to device.
static struct platform_device *g_sm_pdev = NULL;
static struct device *g_sm_dev = NULL;

///
/// Retrieves the version of the security monitor.
///
/// @returns - version number
u32 azure_sphere_sm_get_version(void)
{
	u32 version;
	struct arm_smccc_res res;

	dev_dbg(g_sm_dev, "Security Monitor call: GET VERSION");

	arm_smccc_smc(SECURITY_MONITOR_API_GET_VERSION, 0, 0, 0, 0, 0, 0, 0, &res);
	version = res.a0;

	dev_dbg(g_sm_dev,
			"Security Monitor call: GET VERSION returned %#x", version);

	return version;
}

EXPORT_SYMBOL_GPL(azure_sphere_sm_get_version);

///
/// Structure for tracking coherent memory for SMC calls.
///
/// @size                 - Size (in bytes) of the memory buffer
/// @memory_buffer        - The DMA addressable memory buffer.
/// @coherent_memory_addr - The cache coherent memory address of the buffer.
/// @owns_buffer          - Determines if the coherent APIs owns the buffer.
/// @is_dma_alloc         - Determines if dma_alloc_coherent was used to create the buffer.
/// @is_initialized       - Determines if the structure is initialized.
typedef struct _azure_sphere_sm_coherent_memory_params {
	size_t size;
	const void* memory_buffer;
	dma_addr_t coherent_memory_addr;
	bool owns_buffer;
	bool is_dma_alloc;
	bool is_initialized;
} azure_sphere_sm_coherent_memory_params;

///
/// Allocates either a kernel buffer or a dma coherent buffer depending on the requested size. If
/// the requested buffer size is less than a page, this function will use kzmalloc and map the coherent
/// address, otherwise, it will use dma_alloc_coherent since its min size is one page. The output
/// parameter coherency_params can be used with other azure_sphere_sm_coherent_memory_* functions to
/// perform cache coherent transactions across the secure world boundary.
///
/// @size               - The size of the provided buffer (in bytes).
/// @coherency_params   - The output parameter that contains the coherency information for other calls
///                       to azure_sphere_sm_coherent_memory_*.
/// @out_buffer         - An optional out parameter that will point to the allocated buffer.
/// @returns            - Returns 0 if successfull; otherwise, returns a negative value.
static int azure_sphere_sm_coherent_memory_alloc(
	size_t size, azure_sphere_sm_coherent_memory_params *coherency_params, void** out_buffer)
{
	void *alloc_buffer = NULL;
	dma_addr_t dma_addr = DMA_ERROR_CODE;
	bool use_dma_alloc = size >= PAGE_SIZE;

	if (coherency_params == NULL) {
		return -EINVAL;
	}
	
	if (out_buffer != NULL) {
		*out_buffer = NULL;
	}

	size = ALIGN(size, SM_MINIMUM_CACHE_COHERENCY_BUFFER_SIZE);

	///
	/// Allocate using dma_alloc_coherent if the size of the buffer is bigger
	/// than one page size. dma_alloc_coherent will allocate a minimum size of
	/// one page, so for smaller objects use kzmalloc and use dma_map_single
	/// to get the coherent memory address.
	///
	if (use_dma_alloc) {
		alloc_buffer = dma_alloc_coherent(g_sm_dev, size, &dma_addr,
				       GFP_KERNEL);

		if (dma_mapping_error(g_sm_dev, dma_addr) || alloc_buffer == NULL) {
			dev_err(g_sm_dev, "Failed dma_alloc_coherent");
			return -ENOMEM;
		}
	} else {
		alloc_buffer = kzalloc(size, GFP_KERNEL);

		if (alloc_buffer == NULL) {
			dev_err(g_sm_dev, "Failed to allocate GFP_KERNEL memory");
			return -ENOMEM;
		}
	}

	// update the output parameter values
	coherency_params->is_dma_alloc = use_dma_alloc;
	coherency_params->size = size;
	coherency_params->coherent_memory_addr = dma_addr;
	coherency_params->owns_buffer = true;	
	coherency_params->memory_buffer = alloc_buffer;
	coherency_params->is_initialized = true;

	if (out_buffer != NULL) {
		*out_buffer = alloc_buffer;
	}

	return 0;
}

///
/// For non-DMA allocated buffers, this function will map the cache coherent memory address for
/// the buffer. After this call, the memory buffer must not be modified by NW until
/// azure_sphere_sm_coherent_memory_unmap is called. For DMA allocated buffers, this is a no-op.
///
/// @coherency_params   - The parameter that contains the coherency information for the an
///                       allocation used in the SMC calls.
/// @returns            - Returns 0 if successfull; otherwise, returns a negative value.
static int azure_sphere_sm_coherent_memory_map(
	azure_sphere_sm_coherent_memory_params *coherency_params)
{
	if (coherency_params == NULL || !coherency_params->is_initialized) {
		return -EINVAL;
	}

	///
	/// DMA allocated buffers do not need to be mapped.
	///
	if (!coherency_params->is_dma_alloc) {
		dma_addr_t dma_addr =
			dma_map_single(g_sm_dev, (void**)coherency_params->memory_buffer, coherency_params->size, DMA_FROM_DEVICE);

		if (dma_mapping_error(g_sm_dev, dma_addr)) {
			dev_err(g_sm_dev, "Failed dma_map_single");
			return -ENOMEM;
		}

		dma_sync_single_for_device(g_sm_dev, dma_addr, coherency_params->size, DMA_FROM_DEVICE);
		coherency_params->coherent_memory_addr = dma_addr;
	}

	return 0;
}

///
/// For non-DMA allocated buffers, this function will un-map the cache coherent memory address for
/// the buffer. This is required to be done before the buffer memory is accessed. For DMA allocated
/// buffers, this is a no-op.
///
/// @coherency_params - The parameter that contains the coherency information for the an allocation
///                     used in the SMC calls.
/// @returns          - Returns 0 if successfull; otherwise, returns a negative value.
static void azure_sphere_sm_coherent_memory_unmap(
	azure_sphere_sm_coherent_memory_params *coherency_params)
{
	if (coherency_params == NULL || !coherency_params->is_initialized) {
		return;
	}

	if (!coherency_params->is_dma_alloc &&
		coherency_params->coherent_memory_addr != DMA_ERROR_CODE) {

 		dma_unmap_single(g_sm_dev, coherency_params->coherent_memory_addr, coherency_params->size, DMA_FROM_DEVICE);
		coherency_params->coherent_memory_addr = DMA_ERROR_CODE;
	}
}

///
/// Unmaps and frees (if owned) the allocated buffer used by the coherency_params structure.
///
/// @coherency_params - The parameter that contains the coherency information for the an
///                     allocation used in the SMC calls.
/// @returns          - Returns 0 if successfull; otherwise, returns a negative value.
static void azure_sphere_sm_coherent_memory_free(
	azure_sphere_sm_coherent_memory_params *coherency_params)
{
	if (coherency_params == NULL || !coherency_params->is_initialized) {
		return;
	}

	if (coherency_params->is_dma_alloc) {
		if (coherency_params->owns_buffer) {
			dma_free_coherent(g_sm_dev,
							coherency_params->size,
							(void**)coherency_params->memory_buffer,
							coherency_params->coherent_memory_addr);
		}
	} else {
		if (coherency_params->coherent_memory_addr != DMA_ERROR_CODE) {
	 		dma_unmap_single(g_sm_dev,
			 				 coherency_params->coherent_memory_addr,
							 coherency_params->size,
							 DMA_FROM_DEVICE);
		}
		if (coherency_params->owns_buffer) {
			kfree(coherency_params->memory_buffer);
			coherency_params->memory_buffer = NULL;
		}
	}

	coherency_params->coherent_memory_addr = DMA_ERROR_CODE;
}

///
/// Queries info about flash.
///
/// @flash_info - On success, receives info.
/// @returns - 0 for success
int azure_sphere_sm_query_flash(struct azure_sphere_sm_flash_info *flash_info)
{
	int err = 0;
	struct arm_smccc_res res;
	struct security_monitor_query_flash_result *result = NULL;
	azure_sphere_sm_coherent_memory_params params = {0};
	size_t length = sizeof(*result);

	dev_dbg(g_sm_dev, "Security Monitor call: QUERY_FLASH");

	if (flash_info == NULL) {
		return -EINVAL;
	}

	err = azure_sphere_sm_coherent_memory_alloc(length, &params, (void**)&result);
	if (err != 0) {
		goto cleanup;
	}

	result->info_length = length;

	err = azure_sphere_sm_coherent_memory_map(&params);
	if (err != 0) {
		goto cleanup;
	}

	/*
	 * Call into the security monitor.
	 */

	arm_smccc_smc(SECURITY_MONITOR_API_QUERY_FLASH, (u32)params.coherent_memory_addr, 0, 0, 0, 0, 0, 0, &res);
	err = res.a0;

	dev_dbg(g_sm_dev,
			"Security Monitor call: QUERY_FLASH returned %#x", err);

	azure_sphere_sm_coherent_memory_unmap(&params);

	if (err != 0)
		goto cleanup;

	/*
	 * Minimally validate the data. Support compatibilty as long as the
	 * size is at least big enough to fill in the appropriate flash
	 * information. Do not use sizeof(security_monitor_query_flash_result) because
	 * the structure may grown in the future and compatbility with the
	 * current implementation is only guaranteed to work with the size up
	 * to max_write_length. Any additional fields added in the future will
	 * need to be handled after validating info_length.
	 */

	if (result->info_length < (offsetof(struct security_monitor_query_flash_result, max_write_length) +
							   sizeof(result->max_write_length))) {
		dev_err(g_sm_dev, "Security Monitor call: QUERY_FLASH "
									  "returned invalid data (len=%#x)",
				result->info_length);

		err = -ENODEV;
		goto cleanup;
	}

	/*
	 * Fill out the provided structure with the information we received
	 * from the security monitor.
	 */

	memset(flash_info, 0, sizeof(*flash_info));

	flash_info->length = result->length;
	flash_info->erase.min_length = result->min_erase_length;
	flash_info->erase.preferred_length = result->preferred_erase_length;
	flash_info->erase.max_length = result->max_erase_length;
	flash_info->write.min_length = result->min_write_length;
	flash_info->write.preferred_length = result->preferred_write_length;
	flash_info->write.max_length = result->max_write_length;

	err = 0;

cleanup:
	azure_sphere_sm_coherent_memory_free(&params);

	return err;
}

EXPORT_SYMBOL_GPL(azure_sphere_sm_query_flash);

///
/// Erases a portion of flash.
///
/// @start_offset - Starting offset to start erasing at.
/// @length - Number of bytes to erase.
/// @returns - 0 for success
int azure_sphere_sm_erase_flash(u32 start_offset, u32 length)
{
	int err;
	struct arm_smccc_res res;

	dev_dbg(g_sm_dev, "Security Monitor call: ERASE_FLASH");

	arm_smccc_smc(SECURITY_MONITOR_API_ERASE_FLASH, start_offset, length, 0, 0, 0, 0, 0, &res);
	err = res.a0;

	dev_dbg(g_sm_dev,
			"Security Monitor call: ERASE_FLASH returned %#x", err);

	return err;
}

EXPORT_SYMBOL_GPL(azure_sphere_sm_erase_flash);

///
/// Writes to flash.
///
/// @start_offset - Starting offset to start writing at.
/// @length - Number of bytes to write.
/// @data - Pointer to the data to write.
/// @returns - 0 for success
int azure_sphere_sm_write_flash(u32 start_offset, u32 length, const void *data)
{
	int err;
	struct arm_smccc_res res;
	void *coherent_data;
	azure_sphere_sm_coherent_memory_params params = {0};

	dev_dbg(g_sm_dev, "Security Monitor call: WRITE_FLASH -->");

	if (data == NULL) {
		dev_err(g_sm_dev, "Invalid data buffer");
		return -EINVAL;
	}

	err = azure_sphere_sm_coherent_memory_alloc(length, &params, (void**)&coherent_data);
	if (err != 0) {
		goto cleanup;
	}

	memcpy(coherent_data, data, length);

	err = azure_sphere_sm_coherent_memory_map(&params);
	if (err != 0) {
		goto cleanup;
	}

	wmb();

	/*
	 * Call into security monitor.
	 */

	arm_smccc_smc(SECURITY_MONITOR_API_WRITE_FLASH, start_offset, length, (u32)params.coherent_memory_addr, 0, 0, 0, 0, &res);
	err = res.a0;

	dev_dbg(g_sm_dev,
			"Security Monitor call: WRITE_FLASH returned %#x", err);

	azure_sphere_sm_coherent_memory_unmap(&params);

cleanup:
	azure_sphere_sm_coherent_memory_free(&params);

	return err;
}

EXPORT_SYMBOL_GPL(azure_sphere_sm_write_flash);

///
/// Send generic command to Security Monitor
///
/// @cmd - Command id.
/// @input_param - Input param structure.
/// @input_length - The length of input param in bytes
/// @output_param - Output param structure.
/// @output_length - The length of output param in bytes
/// @returns - 0 for success
int azure_sphere_sm_generic_command_from_user(u32 cmd, void __user *input_param, u32 input_length,
											 void __user *output_param, u32 output_length)
{
	int err;
	struct arm_smccc_res res;
	uint8_t *params_va = NULL;
	int total_length = 0;
	u32 input_param_addr = 0;
	u32 output_param_addr = 0;
	u32 aligned_input_length = 0;
	u32 sm_smc_function = 0;
	azure_sphere_sm_coherent_memory_params params = {0};

	dev_dbg(g_sm_dev, "Security Monitor call: GENERIC_IMAGE_CMD: %d", cmd);

	// input_length can be 1 for the empty input param. So we have to align input_length
	// according to Linux align bytes, 4 otherwise security monitor will have trouble to access
	// output param which address is input_param + input_length.
	aligned_input_length = ALIGN(input_length, 4);
	total_length = aligned_input_length + output_length;

	// validate user-provided data is in bounds
	if ((aligned_input_length < input_length) ||
		(total_length < aligned_input_length)) {
		err = -EINVAL;
		goto cleanup;
	}

	if (total_length) {
		err = azure_sphere_sm_coherent_memory_alloc(total_length, &params, (void**)&params_va);
		if (err != 0) {
			goto cleanup;
		}

		if (input_length) {
			err = copy_from_user(params_va, input_param, input_length);
			if (unlikely(err)) {
				goto cleanup;
			}
		}

		err = azure_sphere_sm_coherent_memory_map(&params);
		if (err != 0) {
			goto cleanup;
		}

		if (input_length) {
			input_param_addr = (u32)params.coherent_memory_addr;
		}
		if (output_length) {
			output_param_addr = (u32)params.coherent_memory_addr + aligned_input_length;
		}
	}

	switch(cmd)
 {
	case AZURE_SPHERE_SMAPI_GET_APPLICATION_IMAGE_COUNT:
		sm_smc_function = SECURITY_MONITOR_API_GET_APPLICATION_IMAGE_COUNT;
		break;
	case AZURE_SPHERE_SMAPI_LIST_ALL_APPLICATION_IMAGES:
		sm_smc_function = SECURITY_MONITOR_API_LIST_ALL_APPLICATION_IMAGES;
		break;
	case AZURE_SPHERE_SMAPI_SHOULD_IMAGE_BE_UPDATED:
		sm_smc_function = SECURITY_MONITOR_API_SHOULD_IMAGE_BE_UPDATED;
		break;
	case AZURE_SPHERE_SMAPI_INVALIDATE_IMAGE:
		sm_smc_function = SECURITY_MONITOR_API_INVALIDATE_IMAGE;
		break;
	case AZURE_SPHERE_SMAPI_OPEN_IMAGE_FOR_STAGING:
		sm_smc_function = SECURITY_MONITOR_API_OPEN_IMAGE_FOR_STAGING;
		break;
	case AZURE_SPHERE_SMAPI_WRITE_BLOCK_TO_STAGE_IMAGE:
		sm_smc_function = SECURITY_MONITOR_API_WRITE_BLOCK_TO_STAGE_IMAGE;
		break;
	case AZURE_SPHERE_SMAPI_COMMIT_IMAGE_STAGING:
		sm_smc_function = SECURITY_MONITOR_API_COMMIT_IMAGE_STAGING;
		break;
	case AZURE_SPHERE_SMAPI_ABORT_IMAGE_STAGING:
		sm_smc_function = SECURITY_MONITOR_API_ABORT_IMAGE_STAGING;
		break;
	case AZURE_SPHERE_SMAPI_INSTALL_STAGED_IMAGES:
		sm_smc_function = SECURITY_MONITOR_API_INSTALL_STAGED_IMAGES;
		break;
	case AZURE_SPHERE_SMAPI_GET_COMPONENT_COUNT:
		sm_smc_function = SECURITY_MONITOR_API_GET_COMPONENT_COUNT;
		break;
	case AZURE_SPHERE_SMAPI_GET_COMPONENT_SUMMARY:
		sm_smc_function = SECURITY_MONITOR_API_GET_COMPONENT_SUMMARY;
		break;
	case AZURE_SPHERE_SMAPI_GET_COMPONENT_IMAGES:
		sm_smc_function = SECURITY_MONITOR_API_GET_COMPONENT_IMAGES;
		break;
	case AZURE_SPHERE_SMAPI_STAGE_COMPONENT_MANIFESTS:
		sm_smc_function = SECURITY_MONITOR_API_STAGE_COMPONENT_MANIFESTS;
		break;
	case AZURE_SPHERE_SMAPI_COUNT_OF_MISSING_IMAGES_TO_DOWNLOAD:
		sm_smc_function = SECURITY_MONITOR_API_COUNT_OF_MISSING_IMAGES_TO_DOWNLOAD;
		break;
	case AZURE_SPHERE_SMAPI_GET_MISSING_IMAGES_TO_DOWNLOAD:
		sm_smc_function = SECURITY_MONITOR_API_GET_MISSING_IMAGES_TO_DOWNLOAD;
		break;
	case AZURE_SPHERE_SMAPI_SET_PERIPHERAL_MAPPING:
		sm_smc_function = SECURITY_MONITOR_API_SET_PERIPHERAL_MAPPING;
		break;
	case AZURE_SPHERE_SMAPI_SET_PIN_MAPPING:
		sm_smc_function = SECURITY_MONITOR_API_SET_PIN_MAPPING;
		break;
	case AZURE_SPHERE_SMAPI_GET_ABI_TYPE_COUNT:
		sm_smc_function = SECURITY_MONITOR_API_GET_ABI_TYPE_COUNT;
		break;
	case AZURE_SPHERE_SMAPI_GET_ABI_VERSIONS:
		sm_smc_function = SECURITY_MONITOR_API_GET_ABI_VERSIONS;
		break;
	case AZURE_SPHERE_SMAPI_GET_UPDATE_CERT_STORE_IMAGE_INFO:
		sm_smc_function = SECURITY_MONITOR_API_GET_UPDATE_CERT_STORE_IMAGE_INFO;
		break;
	case AZURE_SPHERE_SMAPI_STAGE_BASE_MANIFESTS:
		sm_smc_function = SECURITY_MONITOR_API_STAGE_BASE_MANIFESTS;
		break;
	case AZURE_SPHERE_SMAPI_COUNT_OF_MISSING_BASE_IMAGES_TO_DOWNLOAD:
		sm_smc_function = SECURITY_MONITOR_API_COUNT_OF_MISSING_BASE_IMAGES_TO_DOWNLOAD;
		break;
	case AZURE_SPHERE_SMAPI_GET_MISSING_BASE_IMAGES_TO_DOWNLOAD:
		sm_smc_function = SECURITY_MONITOR_API_GET_MISSING_BASE_IMAGES_TO_DOWNLOAD;
		break;
	case AZURE_SPHERE_SMAPI_GET_SOFTWARE_ROLLBACK_INFO:
		sm_smc_function = SECURITY_MONITOR_API_GET_ROLLBACK_INFO;
		break;
	default:
		dev_err(g_sm_dev,
				"Invalid command: %d", cmd);
		err = -EINVAL;
		goto cleanup;
	}

	arm_smccc_smc(sm_smc_function, input_param_addr, input_length, output_param_addr, output_length, 0, 0, 0, &res);
	err = res.a0;

	dev_dbg(
		g_sm_dev,
		"Security Monitor call: %d returned %#x",
		cmd, err);

	azure_sphere_sm_coherent_memory_unmap(&params);

	if (err != 0) {
		goto cleanup;
	}

	if (output_length) {
		err = copy_to_user(output_param, params_va + aligned_input_length, output_length);
	}

cleanup:
	azure_sphere_sm_coherent_memory_free(&params);

	return err;
}

EXPORT_SYMBOL_GPL(azure_sphere_sm_generic_command_from_user);

///
/// Verify linear-cramfs image.
///
/// @flash_address - the flash address of the image to verify.
/// @returns - 0 on success; a negative error code otherwise.
int azure_sphere_sm_verify_image_by_flash_address(u32 flash_address)
{
	struct arm_smccc_res res;
	int err;

	arm_smccc_smc(SECURITY_MONITOR_API_VERIFY_IMAGE_BY_FLASH_ADDRESS_CMD, flash_address, 0, 0, 0, 0, 0, 0, &res);
	err = res.a0;
	dev_dbg(g_sm_dev, "Security Monitor call: VERIFY_IMAGE_BY_FLASH_ADDRESS returned %#x",
			err);

	return err;
}

///
/// Derive Key
///
/// @returns - noreturn on success; a negative error code otherwise.
int azure_sphere_sm_derive_key(void *client_uid, u32 generation_delta, void *key, u32 *instance, u32 *generation)
{
	struct arm_smccc_res res;
	int err;
	struct security_monitor_derive_key_data *buffer = NULL;
	size_t length = sizeof(*buffer);
	azure_sphere_sm_coherent_memory_params params = {0};
	u32 input_addr = 0;
	u32 output_addr = 0;

	/*
	 * Allocate and initialize the buffer containing the parameters and
	 * space for the result..
	 */

	err = azure_sphere_sm_coherent_memory_alloc(length, &params, (void**)&buffer);
	if (err != 0) {
		goto cleanup;
	}

	memcpy(buffer->input_params.client_uid, client_uid, sizeof(buffer->input_params.client_uid));
	buffer->input_params.generation_delta = generation_delta;

	err = azure_sphere_sm_coherent_memory_map(&params);
	if (err != 0) {
		goto cleanup;
	}

	input_addr = params.coherent_memory_addr;
	output_addr = input_addr + offsetof(struct security_monitor_derive_key_data, output_params);

	/*
	 * Call into the security monitor.
	 */

	arm_smccc_smc(SECURITY_MONITOR_API_DERIVE_KEY_CMD, input_addr, sizeof(buffer->input_params), output_addr, sizeof(buffer->output_params), 0, 0, 0, &res);
	err = res.a0;

	dev_dbg(g_sm_dev,
			"Security Monitor call: DERIVE_KEY returned %#x", err);

	azure_sphere_sm_coherent_memory_unmap(&params);

	if (err != 0)
		goto cleanup;

	memcpy(key, buffer->output_params.key, sizeof(buffer->output_params.key));
	*instance = buffer->output_params.instance;
	*generation = buffer->output_params.generation;

cleanup:
	if (buffer != NULL) {
		memzero_explicit(buffer->output_params.key, sizeof(buffer->output_params.key));
	}
	azure_sphere_sm_coherent_memory_free(&params);

	return err;
}

///
/// Reset device
///
/// @returns - noreturn on success; a negative error code otherwise.
int azure_sphere_sm_reset(void)
{
	struct arm_smccc_res res;
	int ret;
	arm_smccc_smc(SECURITY_MONITOR_API_RESET_CMD, 0, 0, 0, 0, 0, 0, 0, &res);
	ret = res.a0;
	dev_err(g_sm_dev, "Security Monitor call: SECURITY_MONITOR_API_RESET_CMD returned unexpectedly");

	return ret;
}

#define WRITE_LOG_MAX_LENGTH (8 * 1024)

///
/// Write log data to security monitor log storage
///
/// @log_data - pointer to the log data
/// @log_data_size - size of the log data
/// @returns - WriteLogReturnCode
int azure_sphere_sm_write_log_from_user(const void __user *log_data, u32 length)
{
	int err;
	struct arm_smccc_res res;
	uint8_t *params_va = NULL;
	azure_sphere_sm_coherent_memory_params params = {0};

	dev_dbg(g_sm_dev, "Security Monitor call: WRITE_LOG");

	if (length == 0) {
		return -EINVAL;
	}

	if (length > WRITE_LOG_MAX_LENGTH) {
		return -ENOMEM;
	}

	err = azure_sphere_sm_coherent_memory_alloc(length, &params, (void**)&params_va);
	if (err != 0) {
		goto cleanup;
	}

	err = copy_from_user(params_va, log_data, length);
	if (unlikely(err)) {
		goto cleanup;
	}

	err = azure_sphere_sm_coherent_memory_map(&params);
	if (err != 0) {
		goto cleanup;
	}

	arm_smccc_smc(SECURITY_MONITOR_API_WRITE_LOG, params.coherent_memory_addr, length, 0, 0, 0, 0, 0, &res);
	err = res.a0;

	dev_dbg(
		g_sm_dev,
		"Security Monitor call: SECURITY_MONITOR_API_WRITE_LOG returned %#x",
		err);

	azure_sphere_sm_coherent_memory_unmap(&params);

cleanup:
	azure_sphere_sm_coherent_memory_free(&params);

	return err;
}

EXPORT_SYMBOL_GPL(azure_sphere_sm_write_log_from_user);

#define GET_LOG_DATA_MAX_LENGTH (8 * 1024)

///
/// Get log data
///
/// @storage_type - storage to get the data of
/// @offset - the start offset for the data to receive in the buffer
/// @length - size of the log data within the buffer to get
/// @log_buffer - the log buffer
/// @returns - GetLogDataReturnCode
int azure_sphere_sm_get_log_data(u32 storage_type, u32 offset, u32 length, void __user* log_buffer)
{
	int err;
	struct arm_smccc_res res;
	uint8_t *params_va = NULL;
	azure_sphere_sm_coherent_memory_params params = {0};

	dev_dbg(g_sm_dev, "Security Monitor call: GET_LOG_DATA");

	if (!azure_sphere_capable(AZURE_SPHERE_CAP_MANAGE_LOG_AND_TELEMETRY)) {
		dev_err(g_sm_dev, "%s sender not allowed to manage log data\n", __FUNCTION__);
		return -EPERM;
	}

	if (length == 0 || log_buffer == NULL) {
		return -EINVAL;
	}

	if (length > GET_LOG_DATA_MAX_LENGTH) {
		return -ENOMEM;
	}

	err = azure_sphere_sm_coherent_memory_alloc(length, &params, (void**)&params_va);
	if (err != 0) {
		goto cleanup;
	}

	err = azure_sphere_sm_coherent_memory_map(&params);
	if (err != 0) {
		goto cleanup;
	}

	arm_smccc_smc(SECURITY_MONITOR_API_GET_LOG_DATA, offset, length, params.coherent_memory_addr, storage_type, 0, 0, 0, &res);
	err = res.a0;

	dev_dbg(
	    g_sm_dev,
	    "Security Monitor call: GET_LOG_DATA returned %#x",
	    err);

	azure_sphere_sm_coherent_memory_unmap(&params);

	err = copy_to_user(log_buffer, params_va, length);
	if (unlikely(err)) {
		goto cleanup;
	}

cleanup:
	azure_sphere_sm_coherent_memory_free(&params);

	return err;
}

EXPORT_SYMBOL_GPL(azure_sphere_sm_get_log_data);


///
/// Get log data size
///
/// @storage_type - storage to get the size of
/// @returns - log data size in bytes
int azure_sphere_sm_get_log_data_size(u32 storage_type)
{
	int err;
	struct arm_smccc_res res;
	dev_dbg(g_sm_dev, "Security Monitor call: GET_LOG_DATA_SIZE");

	if (!azure_sphere_capable(AZURE_SPHERE_CAP_MANAGE_LOG_AND_TELEMETRY)) {
		dev_err(g_sm_dev, "%s sender not allowed to manage log data\n", __FUNCTION__);
		return -EPERM;
	}

	arm_smccc_smc(SECURITY_MONITOR_API_GET_LOG_DATA_SIZE, storage_type, 0, 0, 0, 0, 0, 0, &res);
	err = res.a0;

	dev_dbg(
		g_sm_dev,
		"Security Monitor call: GET_LOG_DATA_SIZE returned %#x",
		err);

	return err;
}

EXPORT_SYMBOL_GPL(azure_sphere_sm_get_log_data_size);

int azure_sphere_sm_get_peripheral_count(uint16_t peripheral_type) {
	int err;
	struct arm_smccc_res res;
	dev_dbg(g_sm_dev, "Security Monitor call: SECURITY_MONITOR_API_GET_PERIPHERAL_COUNT");

	arm_smccc_smc(SECURITY_MONITOR_API_GET_PERIPHERAL_COUNT, peripheral_type, 0, 0, 0, 0, 0, 0, &res);
	err = res.a0;

	dev_dbg(
		g_sm_dev,
		"Security Monitor call: SECURITY_MONITOR_API_GET_PERIPHERAL_COUNT returned %#x",
		err);

	return err;
}

EXPORT_SYMBOL_GPL(azure_sphere_sm_get_peripheral_count);

// Max 8KB
#define MAX_LIST_PERIPHERALS_BUFFER_SIZE (8 * 1024)

int azure_sphere_sm_list_peripherals(uint16_t peripheral_type, void __user* uart_data, u32 length, u32 entry_length) {
	int err;
	struct arm_smccc_res res;
	uint8_t *params_va = NULL;
	u32 params_addr = 0;
	azure_sphere_sm_coherent_memory_params params = {0};

	u32 aligned_length = ALIGN(length, 4);

	dev_dbg(g_sm_dev, "Security Monitor call: LIST_PERIPHERALS");

	if (aligned_length > MAX_LIST_PERIPHERALS_BUFFER_SIZE) {
		err = -ENOMEM;
		goto cleanup;
	}

	if (aligned_length) {
		err = azure_sphere_sm_coherent_memory_alloc(aligned_length, &params, (void**)&params_va);
		if (err != 0) {
			goto cleanup;
		}

		err = azure_sphere_sm_coherent_memory_map(&params);
		if (err != 0) {
			goto cleanup;
		}

		params_addr = params.coherent_memory_addr;
	}

	arm_smccc_smc(SECURITY_MONITOR_API_LIST_PERIPHERALS, peripheral_type, params_addr, aligned_length, entry_length, 0, 0, 0, &res);
	err = res.a0;

	dev_dbg(
		g_sm_dev,
		"Security Monitor call: SECURITY_MONITOR_API_LIST_PERIPHERALS returned %#x",
		err);

	azure_sphere_sm_coherent_memory_unmap(&params);

	// The SMC returns a negative value on error, and otherwise the number of entries filled out.
	if (err < 0) {
		goto cleanup;
	}

        // We clobber the return value and return back 0, as that's what current clients are expecting.
	err = 0;

	if (length) {
		err = copy_to_user(uart_data, params_va, length);
	}

cleanup:
	azure_sphere_sm_coherent_memory_free(&params);
	return err;
}

///
/// Get the N9 Wi-fi firmware address and length
///
/// @wifi_firmware_address - The address of the n9 Wi-Fi firmware image.
/// @wifi_firmware_length  - The length of the Wi-Fi firmware image.
/// @returns - 0 on success; a negative error code otherwise.
int azure_sphere_sm_get_n9_firmware_location(u32 *wifi_firmware_address, u32 *wifi_firmware_length)
{
	struct arm_smccc_res res;
	struct security_monitor_get_wifi_firmware_params *wifi_fw_args = NULL;
	u32 args_length = 0;
	u32 input_length = 0;
	u32 output_length = 0;
	dma_addr_t input;
	dma_addr_t output;
	azure_sphere_sm_coherent_memory_params params = {0};

	int err;

	if (wifi_firmware_address == NULL || wifi_firmware_length == NULL){
		return -EINVAL;
	}

	args_length = sizeof(*wifi_fw_args);
	err = azure_sphere_sm_coherent_memory_alloc(args_length, &params, (void**)&wifi_fw_args);
	if (err != 0) {
		goto cleanup;
	}

	err = azure_sphere_sm_coherent_memory_map(&params);
	if (err != 0) {
		goto cleanup;
	}

	input_length = sizeof(wifi_fw_args->input);
	output_length = sizeof(wifi_fw_args->output);

	input = params.coherent_memory_addr + offsetof(struct security_monitor_get_wifi_firmware_params, input);
	output = params.coherent_memory_addr + offsetof(struct security_monitor_get_wifi_firmware_params, output);

	arm_smccc_smc(SECURITY_MONITOR_API_GET_WIFI_FIRMWARE_LOCATION, input, input_length, output, output_length, 0, 0, 0, &res);
	err = res.a0;

	dev_dbg(g_sm_dev, "Security Monitor call: SECURITY_MONITOR_API_GET_WIFI_FIRMWARE_LOCATION returned %#x",
			err);

	azure_sphere_sm_coherent_memory_unmap(&params);

	if (err != 0) {
		dev_err(g_sm_dev, "Failed to find Wi-Fi firmware");
		goto cleanup;
	}

	*wifi_firmware_address = wifi_fw_args->output.wifi_firmware_address;
	*wifi_firmware_length = wifi_fw_args->output.wifi_firmware_length;

cleanup:
	azure_sphere_sm_coherent_memory_free(&params);

	return err;
}

EXPORT_SYMBOL_GPL(azure_sphere_sm_get_n9_firmware_location);

///
/// Sets the current time in the RTC
///
/// @time - current date & time to set in the RTC
/// @returns - 0 for success, non-zero for failure
int azure_sphere_set_rtc_current_time(struct azure_sphere_rtc_time *time)
{
	int err;
	struct arm_smccc_res res;
	uint8_t *params_va = NULL;
	u32 params_addr = 0;
	azure_sphere_sm_coherent_memory_params params = {0};
	u32 length = sizeof(struct azure_sphere_rtc_time);

	dev_dbg(g_sm_dev, "Security Monitor call: SET_RTC_CURRENT_TIME");

	err = azure_sphere_sm_coherent_memory_alloc(length, &params, (void**)&params_va);
	if (err != 0) {
		goto cleanup;
	}

	memcpy(params_va, time, length);
	
	err = azure_sphere_sm_coherent_memory_map(&params);
	if (err != 0) {
		goto cleanup;
	}

	params_addr = params.coherent_memory_addr;

	arm_smccc_smc(SECURITY_MONITOR_API_SET_RTC_CURRENT_TIME, params_addr, length, 0, 0, 0, 0, 0, &res);
	err = res.a0;

	azure_sphere_sm_coherent_memory_unmap(&params);

	dev_dbg(g_sm_dev,
			"Security Monitor call: SET_RTC_CURRENT_TIME returned %#x", err);

cleanup:
	azure_sphere_sm_coherent_memory_free(&params);
	return err;
}

EXPORT_SYMBOL_GPL(azure_sphere_set_rtc_current_time);

///
/// Sets the alarm time and enable state in the RTC
///
/// @alarm - alarm date & time and enable state to set in the RTC
/// @returns - 0 for success, non-zero for failure
int azure_sphere_set_rtc_alarm(struct azure_sphere_rtc_wake_alarm *alarm)
{
	int err;
	struct arm_smccc_res res;
	uint8_t *params_va = NULL;
	u32 params_addr = 0;
	azure_sphere_sm_coherent_memory_params params = {0};
	u32 length = sizeof(struct azure_sphere_rtc_wake_alarm);


	dev_dbg(g_sm_dev, "Security Monitor call: SET_RTC_ALARM");

	err = azure_sphere_sm_coherent_memory_alloc(length, &params, (void**)&params_va);
	if (err != 0) {
		goto cleanup;
	}

	memcpy(params_va, alarm, length);
	
	err = azure_sphere_sm_coherent_memory_map(&params);
	if (err != 0) {
		goto cleanup;
	}

	params_addr = params.coherent_memory_addr;

	arm_smccc_smc(SECURITY_MONITOR_API_SET_RTC_ALARM, params_addr, length, 0, 0, 0, 0, 0, &res);
	err = res.a0;

	azure_sphere_sm_coherent_memory_unmap(&params);

	dev_dbg(g_sm_dev,
			"Security Monitor call: SET_RTC_ALARM returned %#x", err);

cleanup:
	azure_sphere_sm_coherent_memory_free(&params);
	return err;
}

EXPORT_SYMBOL_GPL(azure_sphere_set_rtc_alarm);

///
/// Controls an IO core
///
/// @action - Wether to start, stop or control debug of an IO Core
/// @core_id - The IO Core instance to control
/// @physical_address - The physical address of the executable to run (if this is a start request)
/// @size - The size of the executable
/// @flags - Debug flags (if this is a set_flags or start_core requst), type (for set communication buffer)
/// @returns - 0 for success, non-zero for failure
static int azure_sphere_io_core_control(enum security_monitor_io_core_control_action action, uint32_t core_id, uintptr_t physical_address, loff_t size, uint32_t flags)
{
	int err;
	struct arm_smccc_res res;
	struct security_monitor_io_core_control *buffer = NULL;
	u32 input_addr = 0;
	u32 output_addr = 0;
	azure_sphere_sm_coherent_memory_params params = {0};
	u32 length = sizeof(struct security_monitor_io_core_control);

	dev_dbg(g_sm_dev, "Security Monitor call: IO_CORE_CONTROL");

	err = azure_sphere_sm_coherent_memory_alloc(length, &params, (void**)&buffer);
	if (err != 0) {
		goto cleanup;
	}

	buffer->input_params.core_id = core_id;
	buffer->input_params.action = action;
	if (action == SECURITY_MONITOR_IO_CORE_CONTROL_ACTION_START_CORE) {
		buffer->input_params.start_core_params.address = physical_address;
		buffer->input_params.start_core_params.size = size;
		buffer->input_params.start_core_params.flags = flags;
	} else if (action == SECURITY_MONITOR_IO_CORE_CONTROL_ACTION_STOP_CORE) {
		// no further arguments
	} else if (action == SECURITY_MONITOR_IO_CORE_CONTROL_ACTION_SET_FLAGS) {
		buffer->input_params.set_flags_params.flags = flags;
	} else if (action == SECURITY_MONITOR_IO_CORE_CONTROL_ACTION_CONFIGURE_COMMUNICATION_BUFFER) {
		buffer->input_params.configure_communication_buffer.type = flags;
		buffer->input_params.configure_communication_buffer.address = physical_address;
		buffer->input_params.configure_communication_buffer.size = size;
	} else {
		err = -EINVAL;
		goto cleanup;
	}

	err = azure_sphere_sm_coherent_memory_map(&params);
	if (err != 0) {
		goto cleanup;
	}

	input_addr = params.coherent_memory_addr;
	output_addr = input_addr + offsetof(struct security_monitor_derive_key_data, output_params);

	/*
	 * Call into the security monitor.
	 */

	arm_smccc_smc(SECURITY_MONITOR_API_IO_CORE_CONTROL, input_addr, sizeof(buffer->input_params), output_addr, sizeof(buffer->output_params), 0, 0, 0, &res);
	err = res.a0;

	dev_dbg(g_sm_dev,
			"Security Monitor call: IO_CORE_CONTROL returned %#x", err);

	azure_sphere_sm_coherent_memory_unmap(&params);

	if (err != 0)
		goto cleanup;

	err = buffer->output_params.result;
cleanup:
	azure_sphere_sm_coherent_memory_free(&params);
	return err;
}

///
/// Starts an IO core
///
/// @core_id - The IO Core instance to control
/// @physical_address - The physical address of the executable to run
/// @size - The size of the executable
/// @returns - 0 for success, non-zero for failure
int azure_sphere_sm_io_core_start(int core, uintptr_t physical_address, loff_t size, uint32_t flags)
{
	return azure_sphere_io_core_control(SECURITY_MONITOR_IO_CORE_CONTROL_ACTION_START_CORE, core, physical_address, size, flags);
}

EXPORT_SYMBOL_GPL(azure_sphere_sm_io_core_start);

///
/// Stops an IO core
///
/// @core_id - The IO Core instance to control
/// @returns - 0 for success, non-zero for failure
int azure_sphere_sm_io_core_stop(int core)
{
	return azure_sphere_io_core_control(SECURITY_MONITOR_IO_CORE_CONTROL_ACTION_STOP_CORE, core, 0, 0, 0);
}

EXPORT_SYMBOL_GPL(azure_sphere_sm_io_core_stop);

///
/// Sets debug flags of an IO core
///
/// @core_id - The IO Core instance to control
/// @flags - Debug flags (if this is a set_flags request)
/// @returns - 0 for success, non-zero for failure
///
int azure_sphere_sm_io_core_set_flags(int core, uint32_t flags)
{
	return azure_sphere_io_core_control(SECURITY_MONITOR_IO_CORE_CONTROL_ACTION_SET_FLAGS, core, 0, 0, flags);
}

EXPORT_SYMBOL_GPL(azure_sphere_sm_io_core_set_flags);

#define RECORD_TELEMETRY_EVENT_DATA_MAX_LENGTH (8 * 1024)

///
/// Record a telemetry event
///
/// @id - Telemetry ID to record
/// @event_timestamp - Time of the telemetry event
/// @payload_length - Number of bytes of payload data.
/// @payload - Payload data.  May be null if payload_length is 0
/// @returns - 0 for success, non-zero for failure
int azure_sphere_sm_record_telemetry_event_data(uint16_t id, uint32_t event_timestamp, uint8_t payload_length, const void __user *payload)
{
	int err;
	struct arm_smccc_res res;
	uint8_t *params_va = NULL;
	azure_sphere_sm_coherent_memory_params params = {0};

	dev_dbg(g_sm_dev, "Security Monitor call: RECORD_TELEMETRY_EVENT_DATA");

	if (!azure_sphere_capable(AZURE_SPHERE_CAP_RECORD_TELEMETRY)) {
		dev_err(g_sm_dev, "%s sender not allowed to record telemetry\n", __FUNCTION__);
		return -EPERM;
	}

	if (payload_length > RECORD_TELEMETRY_EVENT_DATA_MAX_LENGTH) {
		return -ENOMEM;
	}

	if (payload_length) {
		err = azure_sphere_sm_coherent_memory_alloc(payload_length, &params, (void**)&params_va);
		if (err != 0) {
			goto cleanup;
		}

		// let copy_from_user handle a null or invalid pointer
		err = copy_from_user(params_va, payload, payload_length);
		if (unlikely(err)) {
			goto cleanup;
		}

		err = azure_sphere_sm_coherent_memory_map(&params);
		if (err != 0) {
			goto cleanup;
		}
	}
	arm_smccc_smc(SECURITY_MONITOR_API_RECORD_TELEMETRY_EVENT_DATA, id, event_timestamp, payload_length, params.coherent_memory_addr, 0, 0, 0, &res);
	err = res.a0;

	dev_dbg(g_sm_dev,
			"Security Monitor call: RECORD_TELEMETRY_EVENT_DATA returned %#x", err);

cleanup:
	azure_sphere_sm_coherent_memory_unmap(&params);
	azure_sphere_sm_coherent_memory_free(&params);
	return err;
}

EXPORT_SYMBOL_GPL(azure_sphere_sm_record_telemetry_event_data);

#define GET_TELEMETRY_MAX_LENGTH (8 * 1024)

///
/// Get telemetry data
///
/// @offset - Offset into aggregate to begin copying.  Pass 0 to create the aggregate.
/// @buffer - Buffer to store the aggregate in.  Should be exactly 4096.
/// @buffer_size - Length of the output_buffer, in bytes.
/// @returns - Length of data written to the output buffer, or <= 0 on failure
int azure_sphere_sm_get_telemetry(uint16_t offset, void __user *buffer, uint16_t buffer_size)
{
	int err;
	struct arm_smccc_res res;
	uint8_t *params_va = NULL;
	azure_sphere_sm_coherent_memory_params params = {0};

	dev_dbg(g_sm_dev, "Security Monitor call: GET_TELEMETRY_DATA");

	if (!azure_sphere_capable(AZURE_SPHERE_CAP_MANAGE_LOG_AND_TELEMETRY)) {
		dev_err(g_sm_dev, "%s sender not allowed to manage telemetry\n", __FUNCTION__);
		return -EPERM;
	}

	if (buffer_size == 0 || buffer == NULL) {
		return -EINVAL;
	}

	if (buffer_size > GET_TELEMETRY_MAX_LENGTH) {
		return -ENOMEM;
	}

	err = azure_sphere_sm_coherent_memory_alloc(buffer_size, &params, (void**)&params_va);
	if (err != 0) {
		goto cleanup;
	}

	err = azure_sphere_sm_coherent_memory_map(&params);
	if (err != 0) {
		goto cleanup;
	}

	arm_smccc_smc(SECURITY_MONITOR_API_GET_TELEMETRY_DATA, offset, (u32)params.coherent_memory_addr, buffer_size, 0, 0, 0, 0, &res);

	dev_dbg(
	    g_sm_dev,
	    "Security Monitor call: GET_TELEMETRY_DATA returned %#x",
	    (u32)res.a0);

	azure_sphere_sm_coherent_memory_unmap(&params);

    if (res.a0 != 0) {
		err = copy_to_user(buffer, params_va, buffer_size);
		if (unlikely(err)) {
			goto cleanup;
		}
		err = res.a0; // Return the length returned from the monitor call
	}

cleanup:
	azure_sphere_sm_coherent_memory_free(&params);

	dev_dbg(g_sm_dev,
			"Security Monitor call: GET_TELEMETRY_DATA returned %#x", err);

	return err;
}

EXPORT_SYMBOL_GPL(azure_sphere_sm_get_telemetry);

///
/// Reset or retain telemetry data
///
/// @retain - true to retain, false to reset
/// @returns - 0 for success, non-zero for failure
int azure_sphere_sm_reset_retain_telemetry(bool retain)
{
	int err;
	struct arm_smccc_res res;
	u32 sm_smc_function = 0;

	dev_dbg(g_sm_dev, "Security Monitor call: RESET or RETAIN _TELEMETRY");

	if (!azure_sphere_capable(AZURE_SPHERE_CAP_MANAGE_LOG_AND_TELEMETRY)) {
		dev_err(g_sm_dev, "%s sender not allowed to manage telemetry\n", __FUNCTION__);
		return -EPERM;
	}

	sm_smc_function = (retain) ? SECURITY_MONITOR_API_RETAIN_TELEMETRY :
	                             SECURITY_MONITOR_API_RESET_TELEMETRY;

	arm_smccc_smc(sm_smc_function, 0, 0, 0, 0, 0, 0, 0, &res);
	err = res.a0;

	dev_dbg(g_sm_dev,
			"Security Monitor call: RESET or RETAIN _TELEMETRY returned %#x", err);

	return err;
}
EXPORT_SYMBOL_GPL(azure_sphere_sm_reset_retain_telemetry);

/// Configures a communication buffer for an IO Core
///
/// @core - The IO Core instance to control
/// @type - The type of the communication buffer
/// @physical_address - The physical address of the communication buffer
/// @size - The size of the communication buffer
/// @returns - 0 for success, non-zero for failure
///
int azure_sphere_sm_io_core_configure_communication_buffer(int core, uint32_t type, uintptr_t physical_address, loff_t size)
{
	return azure_sphere_io_core_control(SECURITY_MONITOR_IO_CORE_CONTROL_ACTION_CONFIGURE_COMMUNICATION_BUFFER, core, physical_address, size, type);
}
EXPORT_SYMBOL_GPL(azure_sphere_sm_io_core_set_flags);

///
/// Initializes our Security Monitor client driver
///
/// @pdev - Platform device pointer for this module
/// @returns - 0 for success
static int azure_sphere_sm_probe(struct platform_device *pdev)
{
	int err;
	u32 version;

	// Stash away the device pointer in a global so we can access it from
	// the APIs we export from this module.
	g_sm_dev = &pdev->dev;

	dev_info(g_sm_dev,
			 "Starting Azure Sphere Security Monitor client driver");

	err = dma_set_coherent_mask(g_sm_dev, DMA_BIT_MASK(32));
	if (err) {
		dev_err(g_sm_dev,
				"dma_set_coherent_mask failed: %d", err);
		return err;
	}

	version = azure_sphere_sm_get_version();
	dev_info(g_sm_dev, "Detected security monitor version: %u",
			 version);

	// The security monitor will get updated prior to the NWOS and is always
	// backwards compatible; therefore, any version greater or equal to
	// the NW's current security monitor version is supported.
	if (version < SECURITY_MONITOR_CURRENT_VERSION) {
		dev_err(
			g_sm_dev,
			"Unsupported Security Monitor version: expected %u, saw %u",
			SECURITY_MONITOR_CURRENT_VERSION, version);

		g_sm_dev = NULL;
		return -ENODEV;
	}

	return 0;
}

///
/// Teardown our Security Monitor client driver
///
/// @pdev - Platform device pointer for this module
/// @returns - 0 for success
static int azure_sphere_sm_remove(struct platform_device *pdev)
{
	// We don't need to clean anything up as part of tear down.
	return 0;
}

// Driver metadata.
static struct platform_driver azure_sphere_sm_driver = {
	.probe = azure_sphere_sm_probe,
	.remove = azure_sphere_sm_remove,
	.driver = {.name = DRIVER_NAME}};

static int __init azure_sphere_sm_init(void)
{
	int err;

	err = platform_driver_register(&azure_sphere_sm_driver);
	if (err)
		return err;

	g_sm_pdev =
		platform_device_register_simple(DRIVER_NAME, -1, NULL, 0);
	if (IS_ERR(g_sm_pdev)) {
		err = PTR_ERR(g_sm_pdev);
		platform_driver_unregister(&azure_sphere_sm_driver);

		return err;
	}

	return 0;
}

static void __exit azure_sphere_sm_exit(void)
{
	if (g_sm_pdev)
		platform_device_unregister(g_sm_pdev);

	platform_driver_unregister(&azure_sphere_sm_driver);
}

postcore_initcall(azure_sphere_sm_init);
module_exit(azure_sphere_sm_exit);

MODULE_LICENSE("GPLv2");
MODULE_DESCRIPTION("Azure Sphere Security Monitor client driver");
MODULE_AUTHOR("Azure Sphere Team <azuresphereoss@microsoft.com>");
MODULE_ALIAS("platform:azure-sphere-security-monitor");
