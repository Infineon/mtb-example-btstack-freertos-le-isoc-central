/*
 * Copyright 2016-2024, Cypress Semiconductor Corporation (an Infineon company) or
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
* File Name: app_dev.h
*
* Abstract: This file defines an interface for managing LE host lists, i.e.
* device address, link key, client configuration characteristic descriptor value
*******************************************************************************/
#ifndef APP_DEVICE_LIST_
#define APP_DEVICE_LIST_

#include "wiced_bt_dev.h"
#include "app_le.h"

#define DEV_INFO_NOT_FOUND     0xff

typedef struct
{
    /* This field must be the first of the structure) */ 
    wiced_bt_device_link_keys_t     link_key;

} app_dev_info_t;

#define APP_DEVICE_LIST_ELEMENT_SIZE sizeof(app_dev_info_t)

///////////////////////////////////////////////////////////////////////////////
/// Return true if host already in the list
///////////////////////////////////////////////////////////////////////////////
wiced_bool_t app_dev_exist(const wiced_bt_device_address_t device_bd_addr);

///////////////////////////////////////////////////////////////////////////////
/// Return true if host already in the list
///////////////////////////////////////////////////////////////////////////////
uint8_t app_dev_findAddr(const wiced_bt_device_address_t bdAddr);

///////////////////////////////////////////////////////////////////////////////
/// app_dev_remove_addr()
///////////////////////////////////////////////////////////////////////////////
wiced_bool_t app_dev_remove_addr(wiced_bt_device_address_t device_bd_addr);

///////////////////////////////////////////////////////////////////////////////
/// app_dev_remove_all()
/// Delete all bonded devices from NVRAM VS section and reset BleHostList to 0
///////////////////////////////////////////////////////////////////////////////
void app_dev_remove_all(void);

////////////////////////////////////////////////////////////////////////////////
/// Read bonded devices information from NVRAM VS section and initialize
/// hostlist_List
///
/// \param none
///
/// \return none
////////////////////////////////////////////////////////////////////////////////
void app_dev_init(void);

////////////////////////////////////////////////////////////////////////////////
/// device_set_link_key: activate host with link info
///   if host does not exist in database, it creates and add to new host.
///   if host already exist, replace current host and move to active host.
///
/// \param bdAddr host address to activate
/// \param link_key host link_key
/// \param transport host tranport type
///
////////////////////////////////////////////////////////////////////////////////
uint8_t app_dev_set_link_key(wiced_bt_device_link_keys_t * link_keys);

////////////////////////////////////////////////////////////////////////////////
/// device_get_link_key: get a copy of active link key data
///  return FALSE if bdaddr does not match
////////////////////////////////////////////////////////////////////////////////
wiced_bool_t app_dev_get_link_key(const wiced_bt_device_address_t bdAddr,
                                  wiced_bt_device_link_keys_t * link_key);

////////////////////////////////////////////////////////////////////////////////
/// Returns true if the nvram space is full
/// \param none
////////////////////////////////////////////////////////////////////////////////
wiced_bool_t app_dev_full();

////////////////////////////////////////////////////////////////////////////////
/// app_dev_delete_index
/// \param index
////////////////////////////////////////////////////////////////////////////////
wiced_bool_t app_dev_delete_index(uint8_t idx);

////////////////////////////////////////////////////////////////////////////////
/// app_dev_set_info
/// \param index
////////////////////////////////////////////////////////////////////////////////
wiced_bool_t app_dev_set_info(app_dev_info_t * p_data);

////////////////////////////////////////////////////////////////////////////////
/// app_dev_get_info
///
///   get paired host info.
///
/// param
///   buf  -- a pointer to a size of HOST_INFO_SIZE buffer to hold host info.
///
/// \return data length
////////////////////////////////////////////////////////////////////////////////
wiced_bool_t app_dev_get_info(uint8_t idx, uint8_t * buf);

////////////////////////////////////////////////////////////////////////////////
/// app_dev_get_address_type
////////////////////////////////////////////////////////////////////////////////
wiced_bt_ble_address_type_t app_dev_get_address_type(wiced_bt_device_address_t
                                bdaddr);

#endif // APP_DEVICE_LIST_
