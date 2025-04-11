/*
 * Copyright 2025, Cypress Semiconductor Corporation (an Infineon company) or
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

#include "app.h"
#include "wiced_bt_trace.h"


#include "app_terminal_trace.h"
/******************************************************
 *                     Constants
 ******************************************************/
#define HCI_CONTROL_LE_SCAN_RESULT_MAX  20

/******************************************************
 *                     Structures
 ******************************************************/

/******************************************************
 *                     Functions
 ******************************************************/

/*
 * app_le_scan_result_cback
 * Process advertisement packet received
 */
static void app_le_scan_result_cback(wiced_bt_ble_scan_results_t *p_scan_result,
        uint8_t *p_adv_data)
{
    wiced_result_t         result = WICED_BT_SUCCESS;
    uint8_t                length = 0u;
    uint8_t                *adv_name;
    uint8_t                *device_name = (uint8_t *) "IFX ISOC";
    wiced_bt_device_link_keys_t link_key;

    wiced_bool_t           initiate_connection = WICED_FALSE;

    if ( p_scan_result )
    {
        adv_name = wiced_bt_ble_check_advertising_data(p_adv_data,
                                BTM_BLE_ADVERT_TYPE_NAME_COMPLETE,
                                                         &length);
        if(NULL == adv_name)
        {
            if(p_scan_result->ble_evt_type
               == BTM_BLE_EVT_CONNECTABLE_DIRECTED_ADVERTISEMENT)
            {
                // Check if device is paired
                if (app_dev_get_link_key(p_scan_result->remote_bd_addr,
                    &link_key) && (memcmp(p_scan_result->remote_bd_addr,
                    dev_info()->local_addr, BD_ADDR_LEN) == 0))
                {
#ifdef ENABLE_BT_SPY_LOG
                    WICED_BT_TRACE("Found bonded peer device with directed advs"
                                   "BD Addr: %B",p_scan_result->remote_bd_addr);
#endif
                    initiate_connection = WICED_TRUE;
                }
            }

            if(initiate_connection == WICED_FALSE)
            {
#ifdef ENABLE_BT_SPY_LOG
                WICED_BT_TRACE("Found a peer device %B without a name",
                               p_scan_result->remote_bd_addr);
#endif
                return;
            }
        }

        /* Check if the peer device's name is "IFX ISOC" */
        if(initiate_connection || (0 == memcmp(adv_name, device_name,
           strlen((char *) device_name))))
        {
#ifdef ENABLE_BT_SPY_LOG
            WICED_BT_TRACE("Found the peer device \"%s\", BD Addr: %B",
                    (adv_name)?adv_name:(uint8_t *)"Using Directed Advs",
                    p_scan_result->remote_bd_addr);
#endif
            /* Device found. Stop scan. */
            if((result = wiced_bt_ble_scan(BTM_BLE_SCAN_TYPE_NONE, WICED_TRUE,
               app_le_scan_result_cback)) != WICED_BT_SUCCESS)
            {
                WICED_BT_TRACE("scan off status %d", result);
            }
            else
            {
                WICED_BT_TRACE("Scan completed");
            }

            WICED_BT_TRACE("Initiating connection");
            /* Initiate the connection */
            if(wiced_bt_gatt_le_connect(p_scan_result->remote_bd_addr,
                                        p_scan_result->ble_addr_type,
                                        BLE_CONN_MODE_HIGH_DUTY,
                                        WICED_TRUE) != WICED_TRUE)
            {
                WICED_BT_TRACE("wiced_bt_gatt_le_connect failed");
            }
        }
        else
        {
#ifdef ENABLE_BT_SPY_LOG
            WICED_BT_TRACE("Ignoring BD Addr: %B (%s)",
                           p_scan_result->remote_bd_addr, adv_name);
#endif
            return;    //Skip - This is not the device we are looking for.
        }
    }
}

/*
 * Stack runs the scan state machine s4ng between high duty, low
 * duty, no scan, based on the wiced_cfg.  All changes are notified
 * through this callback.
 */
void app_le_scan_state_changed( wiced_bt_ble_scan_type_t state )
{
    WICED_BT_TRACE( "Scan state changed to %d", state );

    switch ( state )
    {
    case BTM_BLE_SCAN_TYPE_NONE:
        led_blink_stop(LED_LINK);
        break;
    case BTM_BLE_SCAN_TYPE_HIGH_DUTY:
        led_blink(LED_LINK, 0, 200);
        break;
    case BTM_BLE_SCAN_TYPE_LOW_DUTY:
        led_blink(LED_LINK, 0, 500);
        break;
    default:
        break;
    }
}

/*
 * handle scan command from UART
 */
void app_le_handle_scan_cmd(wiced_bool_t enable,
                            wiced_bool_t filter_duplicates)
{
    wiced_result_t status;
    if ( enable )
    {
        status = wiced_bt_ble_scan(BTM_BLE_SCAN_TYPE_HIGH_DUTY,
                                   filter_duplicates, app_le_scan_result_cback);
    }
    else
    {
        status = wiced_bt_ble_scan(BTM_BLE_SCAN_TYPE_NONE, filter_duplicates,
                                   app_le_scan_result_cback );
    }
    WICED_BT_TRACE( "app_le_handle_scan_cmd:%d status:%x", enable, status );
}
