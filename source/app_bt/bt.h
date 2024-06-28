/*
 * $ Copyright YEAR Cypress Semiconductor $
 */

/** @file
*
* BT management header file
*/
#ifndef APP_BT_H_
#define APP_BT_H_

#include "wiced_bt_dev.h"
#include "wiced_bt_ble.h"

#ifndef ISOC_ACL_CONN_INTERVAL
 #define ISOC_ACL_CONN_INTERVAL      24
#endif
#define NON_ISOC_ACL_FAST_CONN_INTERVAL  6
#define NON_ISOC_ACL_CONN_INTERVAL  24
#define NON_ISOC_ACL_LINK_SUPERVISION_TIMEOUT  500     // 5 sec timeout
#define ISOC_ACL_LINK_SUPERVISION_TIMEOUT      (15 * ISOC_ACL_CONN_INTERVAL * 125 / 1000)     // 15 times of connection interval

extern const wiced_bt_cfg_settings_t * p_wiced_bt_cfg_settings;

/***********************************************************
 *  wiced_bt_dev_local_addr_ext_t * dev_info()
 ***********************************************************
 * Summary:
 *   Returns a pointer points to extended device info
 *
 * Parameters:
 *   none
 *
 * Return:
 *   Returns extended device info
 *
 ***********************************************************/
wiced_bt_dev_local_addr_ext_t * dev_info();

/***********************************************************
 *  void bt_init()
 ***********************************************************
 * Summary:
 *   Initialize BT Management stack
 *
 * Parameters:
 *   none
 *
 * Return:
 *   wiced_result_t
 *
 ***********************************************************/
wiced_result_t bt_init();

/********************************************************************
 * Function Name: bt_enter_pairing
 ********************************************************************
 * Summary:
 *
 * Parameters:
 *    none
 *
 * Return:
 *    none
 *
 ********************************************************************/
#define bt_enter_pairing()   \
        bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL)

/********************************************************************
 * Function Name: bt_stop_advertisement
 ********************************************************************
 * Summary:
 *  Stops advertisement.
 *
 * Parameters:
 *    none
 *
 * Return:
 *    none
 *
 ********************************************************************/
#define bt_stop_advertisement() \
        bt_start_advertisements(BTM_BLE_ADVERT_OFF, 0, NULL)

/********************************************************************
 * Function Name: bt_get_advertising_mode
 ********************************************************************
 * Summary:
 *  It returns the current advertizing mode.If returns 0 (BTM_BLE_ADVERT_OFF) if
 *  the device is not advertising.
 *
 * Parameters:
 *    none
 *
 * Return:
 *    wiced_bt_ble_advert_mode_t
 *
 ********************************************************************/
#define bt_get_advertising_mode()  wiced_bt_ble_get_current_advert_mode()

/********************************************************************
 * Function Name: bt_is_advertising
 ********************************************************************
 * Summary:
 *  Returns TRUE (none zero) if the device is advertising
 *
 * Parameters:
 *    none
 *
 * Return:
 *    Returns TRUE (none zero) or FALSE (0)
 *
 ********************************************************************/
#define bt_is_advertising()        bt_get_advertising_mode()

/********************************************************************
 * Function Name: bt_start_advertisements()
 ********************************************************************
 * Summary:
 *  Saves the current adv mode and calls wiced_bt_start_advertisements()
 *
 * Parameters:
 *  see wiced_bt_start_advertisements()
 *
 * Return:
 *  wiced_result_t
 *
 *******************************************************************/
wiced_result_t bt_start_advertisements(wiced_bt_ble_advert_mode_t advert_mode,
               wiced_bt_ble_address_type_t directed_advertisement_bdaddr_type,
               wiced_bt_device_address_ptr_t directed_advertisement_bdaddr_ptr);


#endif // APP_BT_H_
