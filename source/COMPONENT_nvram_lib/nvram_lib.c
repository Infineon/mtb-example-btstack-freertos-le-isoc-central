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
 * file nvram_lib.c
 *
 * This is the source code for nvram access using kv-store library
 *
 * Related Document: See README.md
 */

#include "wiced_bt_types.h"
#include "wiced_bt_trace.h"
#include "mtb_kvstore.h"
#include "app_terminal_trace.h"
#if NVRAM_TRACE
# define NVRAM_LIB_TRACE     WICED_BT_TRACE
#else
# define NVRAM_LIB_TRACE(...)
#endif

/*******************************************************************************
*        extern
*******************************************************************************/
extern mtb_kvstore_bd_t nvram_cb;
extern void nvram_get_init_params(uint32_t* length, uint32_t* start_addr);
extern void nvram_device_init(void);

/*******************************************************************************
* data
*******************************************************************************/
static mtb_kvstore_t   kvstore_obj;
static uint8_t initialized = FALSE;

/*******************************************************************************
* Functions
*******************************************************************************/

/**
 * This function performs a write to NVRAM
 */
cy_rslt_t nvram_write(const char* key, uint8_t * ptr, uint32_t size)
{
    cy_rslt_t result;

    if (!initialized)
    {
        return MTB_KVSTORE_ITEM_NOT_FOUND_ERROR;
    }

    NVRAM_LIB_TRACE("NVRAM write %s len=%d", key, size);
    NVRAM_LIB_TRACE("%A", ptr, size);

    result = mtb_kvstore_write(&kvstore_obj, key, (uint8_t *) ptr, size);
    if (result != CY_RSLT_SUCCESS)
    {
        NVRAM_LIB_TRACE("NVRAM write failed");
    }
    return result;
}

/**
 * This function performs a read from NVRAM
 */
cy_rslt_t nvram_read(const char* key, uint8_t * ptr, uint32_t len)
{
    cy_rslt_t result;

    NVRAM_LIB_TRACE("NVRAM read %s len=%d", key, len);

    if (!initialized)
    {
        return MTB_KVSTORE_ITEM_NOT_FOUND_ERROR;
    }

    result = mtb_kvstore_read(&kvstore_obj, key, (uint8_t *) ptr, &len);

#if NVRAM_TRACE
    if (result == CY_RSLT_SUCCESS)
    {
        NVRAM_LIB_TRACE("Success %A", ptr, len);
    }
    else if (result == MTB_KVSTORE_ITEM_NOT_FOUND_ERROR)
    {
        NVRAM_LIB_TRACE("'%s' not found in NVRAM", key);
    }
    else
    {
        NVRAM_LIB_TRACE("failed, result 0x%x, len=%d", result, len);
    }
#endif
    return result;
}

/**
* This function initializes the SMIF and kv-store library
*/
wiced_bool_t nvram_init(mtb_kvstore_bd_t * cfg)
{
    uint32_t start_addr, length;

    // if app didn't define sflash read/write functions, we use lib default operation
    if (cfg == NULL)
    {
#ifdef USE_INTERNAL_FLASH
        NVRAM_LIB_TRACE("nvram_init, using lib internal flash functions");
#else
        NVRAM_LIB_TRACE("nvram_init, using lib serial flash functions");
#endif
        cfg = &nvram_cb;
    }
    else
    {
        NVRAM_LIB_TRACE("nvram_init, using custom flash functions");
    }
    nvram_device_init();
    nvram_get_init_params(&length, &start_addr);

    NVRAM_LIB_TRACE("kv-store start: %08x, len: %08x", start_addr, length);

    /*Check if the kv-store initialization was successfull*/
    if (CY_RSLT_SUCCESS != mtb_kvstore_init(&kvstore_obj, start_addr, length, cfg))
    {
        WICED_BT_TRACE("*** failed to initialize kv-store");
        return FALSE;
    }

    NVRAM_LIB_TRACE("NVRAM init success");
    initialized = TRUE;
    return TRUE;
}
