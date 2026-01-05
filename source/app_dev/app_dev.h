/*
 * (c) 2016-2025, Infineon Technologies AG, or an affiliate of Infineon
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
