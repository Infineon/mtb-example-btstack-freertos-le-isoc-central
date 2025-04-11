/*
 * Copyright 2016-2025, Cypress Semiconductor Corporation (an Infineon company) or
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

/*******************************************************************************
*
* File Name: app_dev.c
*
* Abstract: This file implements the Host List storing/retrieving to/from NVRAM
*
* Functions:
*
*******************************************************************************/
#include "wiced_bt_trace.h"
#include "wiced_bt_cfg.h"
#include "wiced_timer.h"
#include "nvram_lib.h"
#include "app.h"

#include "app_terminal_trace.h"

#if APP_DEV_TRACE_EN
# define APP_DEV_TRACE        WICED_BT_TRACE
# if TRACE_APP_DEV > 1
#  define APP_DEV_TRACE2      WICED_BT_TRACE
# else
#  define APP_DEV_TRACE2(...)
# endif
#else
# define APP_DEV_TRACE(...)
# define APP_DEV_TRACE2(...)
#endif

#define COMMIT_DELAY 500     // 500 msec to commit

#define APP_DEVICE_LIST_MAX    MAX_SUPPORTED_DEV_COUNT

#define APP_DEVICE_LIST_SIZE (APP_DEVICE_LIST_MAX*APP_DEVICE_LIST_ELEMENT_SIZE)

#define VS_ID_APP_DEVICE_LIST "device_list"

static struct
{
    // device list NVRAM cache
    app_dev_info_t list[APP_DEVICE_LIST_MAX];

    // timer to commit NVRAM host list
    wiced_timer_t commitTimer;

} app_dev_cb = {0};

////////////////////////////////////////////////////////////////////////////////
///////////////////////////// Private functions ////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
static wiced_bool_t app_dev_null_dev( uint8_t idx )
{
    wiced_bt_device_address_t nullAddr = {0};
    uint8_t* p_bd_addr = app_dev_cb.list[idx].link_key.bd_addr;

    if (idx < APP_DEVICE_LIST_MAX)
    {
        if (!memcmp(nullAddr, p_bd_addr, BD_ADDR_LEN))
        {
            return TRUE;
        }
    }
    return FALSE;
}

////////////////////////////////////////////////////////////////////////////////
/// This function is the timeout handler for commit_timer
////////////////////////////////////////////////////////////////////////////////
static void app_dev_commit_timer_cb( TIMER_PARAM_TYPE arg )
{
    cy_rslt_t result;

    APP_DEV_TRACE("device info commit to NVRAM");
    result = nvram_write( VS_ID_APP_DEVICE_LIST, (uint8_t *) app_dev_cb.list,
                          APP_DEVICE_LIST_SIZE);
    // save host info to NVRAM
    if(result)
    {
        APP_DEV_TRACE("host info failed to commit to NVRAM");
    }
}

////////////////////////////////////////////////////////////////////////////////
// Save host info to NVRAM VS section. It provides delayed commit so multiple
// commits within delayed */
/// time period, it only make one commit to NVRAM. (To save flash lifespam)
///
/// \param
///    delay -- delay the NVRAM write
///
/// \return none
////////////////////////////////////////////////////////////////////////////////
static void app_dev_update_to_nvram(uint32_t delay)
{
    if (wiced_is_timer_in_use(&app_dev_cb.commitTimer))
    {
        wiced_stop_timer(&app_dev_cb.commitTimer);
    }

    if (delay)
    {
        wiced_start_timer(&app_dev_cb.commitTimer, delay);
    }
    else
    {
        app_dev_commit_timer_cb(0);
    }
}

////////////////////////////////////////////////////////////////////////////////
/// Returns the empty slot index
/// \param none
// \returns the index of the empty slot, DEV_INFO_NOT_FOUND if no empty slot
// available. */
////////////////////////////////////////////////////////////////////////////////
static uint8_t app_dev_alloc()
{
    wiced_bt_device_address_t bdAddr={0};
    return app_dev_findAddr(bdAddr);
}

////////////////////////////////////////////////////////////////////////////////
//////////////////////////// Public functions //////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
/// device_get_info
///
///   get paired host info.
///
/// param
///   buf  -- a pointer to a size of HOST_INFO_SIZE buffer to hold host info.
///
/// \return data length
////////////////////////////////////////////////////////////////////////////////
wiced_bool_t app_dev_get_info(uint8_t idx, uint8_t * buf)
{
    if(idx < APP_DEVICE_LIST_MAX)
    {
        if (!app_dev_null_dev(idx))
        {
            memcpy(buf, &app_dev_cb.list[idx], APP_DEVICE_LIST_ELEMENT_SIZE);
            return TRUE;
        }
    }

    return FALSE;
}

////////////////////////////////////////////////////////////////////////////////
/// app_dev_get_link_key:
////////////////////////////////////////////////////////////////////////////////
wiced_bool_t app_dev_get_link_key(const wiced_bt_device_address_t bdAddr,
                                  wiced_bt_device_link_keys_t * p_link_key)
{
    uint8_t index = app_dev_findAddr(bdAddr);

    if (index != DEV_INFO_NOT_FOUND)
    {
#ifdef ENABLE_BT_SPY_LOG
        APP_DEV_TRACE("Found link key for %B", bdAddr);
#endif
        memcpy(p_link_key, &app_dev_cb.list[index].link_key,
               sizeof(wiced_bt_device_link_keys_t));
        return TRUE;
    }
#ifdef ENABLE_BT_SPY_LOG
    APP_DEV_TRACE("No link key found for %B", bdAddr);
#endif
    return FALSE;
}

////////////////////////////////////////////////////////////////////////////////
/// device_set_link_key
/// This function is called when the host is bonded and data will be saved into
/// NVRAM database.
///
/// \param bdAddr host address to activate
/// \param link_key host link_key
///
////////////////////////////////////////////////////////////////////////////////
uint8_t app_dev_set_link_key(wiced_bt_device_link_keys_t * link_keys)
{
    if (link_keys)
    {
        uint8_t index = app_dev_findAddr(link_keys->bd_addr);

        // If this address is new, find an empty slot for saving data
        if (index == DEV_INFO_NOT_FOUND)
        {
            index = app_dev_alloc(); // get an empty slot
            if (index == DEV_INFO_NOT_FOUND)
            {
                WICED_BT_TRACE("Memeory full! Only %d devices are supported",
                               APP_DEVICE_LIST_MAX);
                return DEV_INFO_NOT_FOUND;
            }
#ifdef ENABLE_BT_SPY_LOG
            APP_DEV_TRACE("Allocated index %d for %B",index,link_keys->bd_addr);
#endif
        }
        else // link_key exists, we delete the old one
        {
#ifdef ENABLE_BT_SPY_LOG
            APP_DEV_TRACE("%B exists at index %d, reusing it",
                          link_keys->bd_addr, index);
#endif
            app_dev_delete_index(index);
        }

        /*If the address is not public address, update the resolution database*/
        if(link_keys->key_data.ble_addr_type &&   // not public address
           (wiced_bt_dev_add_device_to_address_resolution_db(link_keys)
           != WICED_BT_SUCCESS))
        {
            WICED_BT_TRACE("Err:"
                "wiced_bt_dev_add_device_to_address_resolution_db failed");
            return DEV_INFO_NOT_FOUND;
        }

        if(memcmp(link_keys->bd_addr, link_keys->conn_addr, BD_ADDR_LEN) != 0)
        {
            app_add_resolved_address(link_keys->bd_addr, link_keys->conn_addr);
        }
#ifdef ENABLE_BT_SPY_LOG
        // save the link_key info.
        APP_DEV_TRACE("Saving link key to device %B, index=%d",
                      link_keys->bd_addr, index);
#endif
        memcpy(&app_dev_cb.list[index].link_key, link_keys,
               sizeof(wiced_bt_device_link_keys_t));
        app_dev_update_to_nvram(COMMIT_DELAY);
        return index;
    }
    else
    {
        WICED_BT_TRACE("Invalid Link_key");
    }
    return DEV_INFO_NOT_FOUND;
}

////////////////////////////////////////////////////////////////////////////////
/// Read bonded devices information from NVRAM VS section and initialize
/// hostlist_List
///
/// \param none
///
/// \return none
////////////////////////////////////////////////////////////////////////////////
void app_dev_init(void)
{
    APP_DEV_TRACE("app_dev_init");
    memset(app_dev_cb.list, 0x00, APP_DEVICE_LIST_SIZE);

    //timer to allow commit nvram write
    wiced_init_timer( &app_dev_cb.commitTimer, app_dev_commit_timer_cb, 
                      0, WICED_MILLI_SECONDS_TIMER );

    if (nvram_read(VS_ID_APP_DEVICE_LIST, (uint8_t *)app_dev_cb.list,
                   APP_DEVICE_LIST_SIZE) == CY_RSLT_SUCCESS)
    {
        for (uint8_t index = 0; index < APP_DEVICE_LIST_MAX; index++)
        {
            if ((!app_dev_null_dev(index))
                && (app_dev_cb.list[index].link_key.key_data.ble_addr_type))
            {
                    wiced_bt_dev_add_device_to_address_resolution_db(
                                         &app_dev_cb.list[index].link_key);
            }
        }
    }
    else
    {
        APP_DEV_TRACE("Device info not found in NVRAM");
    }
}

////////////////////////////////////////////////////////////////////////////////
/// Delete host with given addr from NVRAM
///
/// \param none
///
////////////////////////////////////////////////////////////////////////////////
wiced_bool_t app_dev_remove_addr(wiced_bt_device_address_t bdAddr)
{
    return app_dev_delete_index(app_dev_findAddr(bdAddr));
}

////////////////////////////////////////////////////////////////////////////////
/// Delete all bonded devices from NVRAM VS section and reset BleHostList to 0
///
/// \param none
///
/// \return none
////////////////////////////////////////////////////////////////////////////////
void app_dev_remove_all(void)
{
    for (uint8_t index = 0; index < APP_DEVICE_LIST_MAX; index++)
    {
        if (!app_dev_null_dev(index))
        {
            app_dev_delete_index(index);
        }
    }

    /*The following call appears redundant but is apparently required to avoid
      the set PHY problem */
    wiced_bt_ble_address_resolution_list_clear_and_disable();

    memset(app_dev_cb.list, 0x00, APP_DEVICE_LIST_SIZE);
    app_dev_update_to_nvram(COMMIT_DELAY);
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
wiced_bool_t app_dev_exist(const wiced_bt_device_address_t device_bd_addr)
{
    return app_dev_findAddr(device_bd_addr) != DEV_INFO_NOT_FOUND;
}


////////////////////////////////////////////////////////////////////////////////
/// Returns the index of the element by giveing address
/// \param bdAddr BD address of the device to find
/// \return index of the device if it exists in the list, DEV_INFO_NOT_FOUND if
/// the host is not in the list
////////////////////////////////////////////////////////////////////////////////
uint8_t app_dev_findAddr(const wiced_bt_device_address_t bdAddr)
{
    uint8_t index;

    // Go through all the valid entries in the table
    for (index=0; index < APP_DEVICE_LIST_MAX; index++)
    {
        if (memcmp(&app_dev_cb.list[index].link_key.bd_addr, bdAddr,
            BD_ADDR_LEN) == 0)
        {
            // Got it! Return the index
            return index;
        }
    }

    // If we get here, the address doesn't exist.
    return DEV_INFO_NOT_FOUND;
}

////////////////////////////////////////////////////////////////////////////////
/// Returns true if the nvram space is full
/// \param none
////////////////////////////////////////////////////////////////////////////////
wiced_bool_t app_dev_full()
{
    return app_dev_alloc() == DEV_INFO_NOT_FOUND;
}

////////////////////////////////////////////////////////////////////////////////
// device_del: delete host
//     if host is LE and private address, remove it from resolving list also.
//
// return WICED_TRUE if host already exists
////////////////////////////////////////////////////////////////////////////////
wiced_bool_t app_dev_delete_index(uint8_t i)
{
    // make sure index is valid
    if (i < APP_DEVICE_LIST_MAX)
    {
         // not public address
        if (app_dev_cb.list[i].link_key.key_data.ble_addr_type)
        {
            wiced_bt_dev_remove_device_from_address_resolution_db(
                &app_dev_cb.list[i].link_key);
        }
        wiced_bt_dev_delete_bonded_device(app_dev_cb.list[i].link_key.bd_addr);

        // delete current host element
        memset(&app_dev_cb.list[i], 0, sizeof(app_dev_info_t));
        app_dev_update_to_nvram(COMMIT_DELAY);
        return TRUE;
    }
    return FALSE;
}

////////////////////////////////////////////////////////////////////////////////
/// app_dev_set_info
/// \param index
////////////////////////////////////////////////////////////////////////////////
wiced_bool_t app_dev_set_info(app_dev_info_t * p_data)
{
    uint8_t index = app_dev_set_link_key(&p_data->link_key);
    if( (index != DEV_INFO_NOT_FOUND) && (index < APP_DEVICE_LIST_MAX) )
    {
        memcpy(&app_dev_cb.list[index], p_data, APP_DEVICE_LIST_ELEMENT_SIZE);
        app_dev_update_to_nvram(COMMIT_DELAY);
        return TRUE;
    }
    return FALSE;
}

wiced_bt_ble_address_type_t app_dev_get_address_type(
                                wiced_bt_device_address_t bdaddr)
{
    uint8_t index = app_dev_findAddr(bdaddr);

    if (index == DEV_INFO_NOT_FOUND)
    {
#ifdef ENABLE_BT_SPY_LOG
        APP_DEV_TRACE("Err: %B is not bonded in NVRAM", bdaddr);
#endif
        return DEV_INFO_NOT_FOUND;
    }
    return app_dev_cb.list[index].link_key.key_data.ble_addr_type;
}
