/*
 * (c) 2025, Infineon Technologies AG, or an affiliate of Infineon
 * Technologies AG. All rights reserved.
 * This software, associated documentation and materials ("Software") is
 * owned by Infineon Technologies AG or one of its affiliates ("Infineon")
 * and is protected by and subject to worldwide patent protection, worldwide
 * copyright laws, and international treaty provisions. Therefore, you may use
 * this Software only as provided in the license agreement accompanying the
 * software package from which you obtained this Software. If no license
 * agreement applies, then any use, reproduction, modification, translation, or
 * compilation of this Software is prohibited without the express written
 * permission of Infineon.
 *
 * Disclaimer: UNLESS OTHERWISE EXPRESSLY AGREED WITH INFINEON, THIS SOFTWARE
 * IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING, BUT NOT LIMITED TO, ALL WARRANTIES OF NON-INFRINGEMENT OF
 * THIRD-PARTY RIGHTS AND IMPLIED WARRANTIES SUCH AS WARRANTIES OF FITNESS FOR A
 * SPECIFIC USE/PURPOSE OR MERCHANTABILITY.
 * Infineon reserves the right to make changes to the Software without notice.
 * You are responsible for properly designing, programming, and testing the
 * functionality and safety of your intended application of the Software, as
 * well as complying with any legal requirements related to its use. Infineon
 * does not guarantee that the Software will be free from intrusion, data theft
 * or loss, or other breaches ("Security Breaches"), and Infineon shall have
 * no liability arising out of any Security Breaches. Unless otherwise
 * explicitly approved by Infineon, the Software may not be used in any
 * application where a failure of the Product or any consequences of the use
 * thereof can reasonably be expected to result in personal injury.
 */

/**
 * file iflash.c
 *
 * Description: This file contains block device function implementations
 *              required by kv-store library for external serial flash
 *
 * Related Document: See README.md
 */

#ifdef USE_INTERNAL_FLASH

#include "cyhal_flash.h"
#include "mtb_kvstore.h"
#include "cy_retarget_io.h"
#include "app_terminal_trace.h"
cy_rslt_t result;

static cyhal_flash_t flash_obj;
static cyhal_flash_block_info_t flash_block_info;
static cyhal_flash_info_t flash_info;

/*******************************************************************************
 * Defines
 ******************************************************************************/
#define AUXILIARY_FLASH_BLOCK                (1)
#define AUXILIARY_FLASH_LENGTH               (16)

/*******************************************************************************
 * Private functions
 ******************************************************************************/

/**
 * Function to get the read size of the block device
 * for a specific address.
 */
static uint32_t bd_read_size(void* context, uint32_t addr)
{
    (void)context;
    (void)addr;
    return 1;
}

/**
 * Function to get the program size of the block device
 * for a specific address.
 */
static uint32_t bd_program_size(void* context, uint32_t addr)
{
    (void)context;
    if(flash_info.block_count != 0)
        return flash_block_info.page_size;
    else
        return CY_RSLT_TYPE_ERROR;
}

/**
 * Function prototype to get the erase size of the block device
 * for a specific address.
 */
static uint32_t bd_erase_size(void* context, uint32_t addr)
{
    (void)context;
    if(flash_info.block_count != 0)
        return flash_block_info.sector_size;
    else
        return CY_RSLT_TYPE_ERROR;
}

/**
 * Function for reading data from the block device.
 */
static cy_rslt_t bd_read(void* context, uint32_t addr,
                         uint32_t length, uint8_t* buf)
{
    (void)context;
    return cyhal_flash_read(&flash_obj, addr, buf, length);
}

/**
 * Function for programming of a block device for a specific address
 */
static cy_rslt_t bd_program(void* context, uint32_t addr,
                            uint32_t length, const uint8_t* buf)
{
    (void)context;
    uint32_t prog_size = bd_program_size(context, addr);
    CY_ASSERT(0 == (length % prog_size));
    volatile cy_rslt_t result = CY_RSLT_SUCCESS;
    for(uint32_t loc = addr; result == CY_RSLT_SUCCESS && loc < addr + length;
         loc += prog_size, buf += prog_size)
    {
        result = cyhal_flash_program(&flash_obj, loc, (const uint32_t*)buf);
    }
    return result;
}

/**
 * Function for erasing of a block device for a specific address
 */
static cy_rslt_t bd_erase(void* context, uint32_t addr, uint32_t length)
{
    (void)context;
    uint32_t erase_size = bd_erase_size(context, addr);
    CY_ASSERT(0 == (length % erase_size));
    cy_rslt_t result = CY_RSLT_SUCCESS;
    for(uint32_t loc = addr; result == CY_RSLT_SUCCESS && loc < addr + length; loc += erase_size)
    {
        result = cyhal_flash_erase(&flash_obj, loc);
    }
    return result;
}

/**
 * This function initializes the underlying block device (in this case internal flash).
 */
void nvram_device_init(void)
{
    result = cyhal_flash_init(&flash_obj);
    if(result != CY_RSLT_SUCCESS)
    {
        WICED_BT_TRACE("Internal flash initialization failed!");
        CY_ASSERT(0);
    }

    cyhal_flash_get_info(&flash_obj, &flash_info);
    flash_block_info = flash_info.blocks[AUXILIARY_FLASH_BLOCK];

    NVRAM_LIB_TRACE("cyhal_flash_init success");
}

/**
 * This function initializes the underlying block device (in this case internal flash).
 */
void nvram_get_init_params(uint32_t* length, uint32_t* start_addr)
{
    *length = flash_block_info.page_size * AUXILIARY_FLASH_LENGTH;

    *start_addr = flash_block_info.start_address +
                  flash_block_info.size - (*length);
}

/**
 * default mtb_kvstore_bd_t
 */
mtb_kvstore_bd_t default_nvram_device =
{
    .read         = bd_read,
    .program      = bd_program,
    .erase        = bd_erase,
    .read_size    = bd_read_size,
    .program_size = bd_program_size,
    .erase_size   = bd_erase_size,
    .context      = &flash_obj,
};

#endif
/* END OF FILE [] */
