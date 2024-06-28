/*
 * Copyright 2024, Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software") is owned by Cypress Semiconductor Corporation
 * or one of its affiliates ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products.  Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
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
