/*
 * $ Copyright YEAR Cypress Semiconductor $
 */

/** @file
*/

#include "wiced_bt_stack.h"
#include "wiced_memory.h"
#include "wiced_bt_trace.h"
#include "nvram_lib.h"
#include "app.h"
#include "app_bt_utils.h"
#include "cybt_result.h"
#ifdef ENABLE_BT_SPY_LOG
#include "cybt_debug_uart.h"
#include "cybt_platform_trace.h"
#endif

#include "app_terminal_trace.h"
/******************************************************************************
 * Defines
 ******************************************************************************/
#define APP_STACK_HEAP_SIZE 0x1000                      // stack size
#define VS_ID_LOCAL_IDENTITY "local_identity"
#define WICED_PIN_CODE_LEN                  4

/******************************************************************************
 * Structures
 ******************************************************************************/
static struct {
    wiced_bt_dev_local_addr_ext_t   dev;
    wiced_bt_ble_advert_mode_t      adv_mode, intended_adv_mode;
    uint8_t                         adv_bdAddr[BD_ADDR_LEN];
} bt = {0};

const uint8_t pincode[WICED_PIN_CODE_LEN] = { 0x30, 0x30, 0x30, 0x30 };

const wiced_bt_cfg_settings_t * p_wiced_bt_cfg_settings=&wiced_bt_cfg_settings;

static wiced_bt_cfg_isoc_t cfg_isoc = {
    .max_sdu_size = ISO_SDU_SIZE,
    .channel_count = 1,
    .max_cis_conn = MAX_CIS_PER_CIG,
    .max_cig_count = 1,
    .max_buffers_per_cis = 4,
    .max_big_count = 0
};

/* Custom Bluetooth stack configuration */
static wiced_bt_cfg_settings_t wiced_bt_cfg_settings_custom;

static void bt_get_cfg()
{
    //if bt-configurator does not assign isoc_cfg, we have to assign it manually
    if (!p_wiced_bt_cfg_settings->p_isoc_cfg)
    {
        memcpy(&wiced_bt_cfg_settings_custom, p_wiced_bt_cfg_settings,
               sizeof(wiced_bt_cfg_settings_t));
        wiced_bt_cfg_settings_custom.p_isoc_cfg = &cfg_isoc;
        p_wiced_bt_cfg_settings = &wiced_bt_cfg_settings_custom;
    }
}

#ifdef ENABLE_BT_SPY_LOG
static cybt_result_t debug_uart_send_hci_trace(uint8_t type, uint16_t data_size,
                                               uint8_t* p_data)
{
    // Don't show ISOC traces.
    if ((type == 4) || (type == 5))
    {
        return CYBT_SUCCESS;
    }
    if(type == 0 && (
      ((p_data[0] == 0x3e) && (p_data[2] == 0x0d)) ||
      (p_data[0] == 0x13)))
    {
        // skip
        return CYBT_SUCCESS;
    }
    return cybt_debug_uart_send_hci_trace(type, data_size, p_data);
}
#endif
/******************************************************************************
 *     Variables
 ******************************************************************************/
// forced to use addr 0x123456789000
wiced_bt_device_address_t dev_addr = {0x12, 0x34, 0x56, 0x78, 0x90, 0x00};

/******************************************************************************
 *     Private Functions
 ******************************************************************************/

/*
 *  Management callback receives various notifications from the stack
 */
static wiced_result_t app_bt_management( wiced_bt_management_evt_t event,
        wiced_bt_management_evt_data_t *p_event_data )
{
    wiced_result_t                     result = WICED_BT_SUCCESS;
    wiced_bt_dev_encryption_status_t  *p_encryption_status;
    wiced_bt_power_mgmt_notification_t *p_power_mgmt_notification;
    wiced_bt_dev_pairing_cplt_t        *p_pairing_cmpl;
    uint8_t                            pairing_result;
#ifdef SUPPORT_CONN_PARAM_UPDATE
    wiced_bt_ble_connection_param_update_t *p_ble_conn_param_update;
#endif
    uint8_t *p_keys;

    WICED_BT_TRACE( "BT event: %d, %s", event, get_btm_event_name(event) );

    switch( event )
    {
        /* Bluetooth  stack enabled */
        case BTM_ENABLED_EVT:
            if ( p_event_data->enabled.status == WICED_BT_SUCCESS )
            {
#ifdef ENABLE_BT_SPY_LOG
                /* Register HCI Trace callback */
                wiced_bt_dev_register_hci_trace( (wiced_bt_hci_trace_cback_t*)
                                                 debug_uart_send_hci_trace );
                cybt_platform_set_trace_level(CYBT_TRACE_ID_STACK, CYBT_TRACE_LEVEL_DEBUG);
#endif
                WICED_BT_TRACE("BTM initialized");

                wiced_bt_set_local_bdaddr (dev_addr, BLE_ADDR_PUBLIC);

                /* read extended device info */
                wiced_bt_dev_read_local_addr_ext(&bt.dev);
#ifdef ENABLE_BT_SPY_LOG
                WICED_BT_TRACE("Local Addr: %B", dev_info()->local_addr);
#endif
                app_init();
            }
            else
            {
                WICED_BT_TRACE("** BT Enable failed, status:%d",
                               p_event_data->enabled.status);
                CY_ASSERT(0);
            }
            break;

        case BTM_DISABLED_EVT:
            break;

        case BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT:
            p_event_data->pairing_io_capabilities_ble_request.local_io_cap 
                = BTM_IO_CAPABILITIES_NONE;
            p_event_data->pairing_io_capabilities_ble_request.oob_data
                = BTM_OOB_NONE;
            p_event_data->pairing_io_capabilities_ble_request.auth_req
                = BTM_LE_AUTH_REQ_BOND | BTM_LE_AUTH_REQ_MITM;
            p_event_data->pairing_io_capabilities_ble_request.max_key_size
                = 0x10;
            p_event_data->pairing_io_capabilities_ble_request.init_keys
                = BTM_LE_KEY_PENC | BTM_LE_KEY_PID;
            p_event_data->pairing_io_capabilities_ble_request.resp_keys
                = BTM_LE_KEY_PENC | BTM_LE_KEY_PID;
            break;

        case BTM_PAIRING_COMPLETE_EVT:
            p_pairing_cmpl = &p_event_data->pairing_complete;
            if(p_pairing_cmpl->transport == BT_TRANSPORT_BR_EDR)
            {
                pairing_result =
                    p_pairing_cmpl->pairing_complete_info.br_edr.status;
            }
            else
            {
                pairing_result=p_pairing_cmpl->pairing_complete_info.ble.reason;
            }
            WICED_BT_TRACE( "Pairing Result: %d", pairing_result );
            break;

        case BTM_ENCRYPTION_STATUS_EVT:
            p_encryption_status = &p_event_data->encryption_status;
#ifdef ENABLE_BT_SPY_LOG
            WICED_BT_TRACE("Encryption address:%B result:%d (%sencrypted)",
                   p_encryption_status->bd_addr,
                   p_event_data->encryption_status.result,
                   p_event_data->encryption_status.result==WICED_SUCCESS?
                           "": "not ");
#endif
            if (p_encryption_status->result == WICED_BT_SUCCESS)
            {
                // Set the preferred Phy to 2M
                app_set_phy(p_encryption_status->bd_addr,BTM_BLE_PREFER_2M_PHY);
            }
            else
            {
                WICED_BT_TRACE("Failed to encrypt. Something may be wrong with"
                                "the link keys.");
            }


            break;

        case BTM_SECURITY_REQUEST_EVT:
            wiced_bt_ble_security_grant( p_event_data->security_request.bd_addr,
                                         WICED_BT_SUCCESS );
            break;

        case BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT:
            WICED_BT_TRACE("BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT");
            app_dev_set_link_key(&p_event_data->paired_device_link_keys_update);
            break;

        case BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT:
#ifdef ENABLE_BT_SPY_LOG
            WICED_BT_TRACE("LINK_KEYS_REQUEST_EVT %B\n",
                   p_event_data->paired_device_link_keys_request.bd_addr);
#endif
            if (!app_dev_get_link_key(
                p_event_data->paired_device_link_keys_request.bd_addr,
                &p_event_data->paired_device_link_keys_request))
            {
#ifdef ENABLE_BT_SPY_LOG
                WICED_BT_TRACE("requsted %B link_key not available",
                    p_event_data->paired_device_link_keys_request.bd_addr);
#endif
                result = WICED_BT_ERROR;
            }
            break;

        case BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT:
            WICED_BT_TRACE("BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT");
            /* save keys to NVRAM */
            p_keys = (uint8_t *)&p_event_data->local_identity_keys_update;
            nvram_write (VS_ID_LOCAL_IDENTITY, p_keys,
                         sizeof( wiced_bt_local_identity_keys_t ));
            break;

        case BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT:
            WICED_BT_TRACE("BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT");
            /* read keys from NVRAM */
            p_keys = (uint8_t *)&p_event_data->local_identity_keys_request;
            if (!nvram_read( VS_ID_LOCAL_IDENTITY, p_keys,
                sizeof(wiced_bt_local_identity_keys_t)))
            {
                WICED_BT_TRACE("Local Identity Key not available");
            }
            break;

        case BTM_POWER_MANAGEMENT_STATUS_EVT:
#ifdef ENABLE_BT_SPY_LOG
            p_power_mgmt_notification = &p_event_data->power_mgmt_notification;
            WICED_BT_TRACE( "Power mgmt status event: bd ( %B ) status:%d"
                            "hci_status:%d",
                            p_power_mgmt_notification->bd_addr,
                            p_power_mgmt_notification->status,
                            p_power_mgmt_notification->hci_status);
#else
            (void)p_power_mgmt_notification;
#endif
            break;

        case BTM_BLE_SCAN_STATE_CHANGED_EVT:
            app_le_scan_state_changed( p_event_data->ble_scan_state_changed );
            break;

        case BTM_BLE_CONNECTION_PARAM_UPDATE:
#ifdef SUPPORT_CONN_PARAM_UPDATE
            p_ble_conn_param_update=&p_event_data->ble_connection_param_update;
            WICED_BT_TRACE ("status:%d interval:%d latency:%d lsto:%d",
                                p_ble_conn_param_update->status,
                                p_ble_conn_param_update->conn_interval,
                                p_ble_conn_param_update->conn_latency,
                                p_ble_conn_param_update->supervision_timeout);
            if(p_ble_conn_param_update->status == WICED_BT_SUCCESS)
            {
                app_update_acl_conn_interval(p_ble_conn_param_update->bd_addr,
                                      p_ble_conn_param_update->conn_interval);

                if(p_ble_conn_param_update->conn_interval
                   == ISOC_ACL_CONN_INTERVAL)
                {
                    isoc_start(0);
                }
                else if(p_ble_conn_param_update->conn_interval
                        == NON_ISOC_ACL_CONN_INTERVAL)
                {
                    app_change_isoc_acl_mode();
                }
            }
            else
            {
                // Retry setting the connection interval if perviously failed
                app_set_acl_conn_interval(wiced_bt_dev_get_acl_conn_handle(
                                          p_ble_conn_param_update->bd_addr,
                                          BT_TRANSPORT_LE),
                        ((p_ble_conn_param_update->conn_interval ==
                          ISOC_ACL_CONN_INTERVAL) ? NON_ISOC_ACL_CONN_INTERVAL
                          : ISOC_ACL_CONN_INTERVAL));
            }
#endif
            break;

        case BTM_BLE_PHY_UPDATE_EVT:
#ifdef ENABLE_BT_SPY_LOG
            WICED_BT_TRACE("PHY update: status:%d Tx:%d Rx:%d BDA %B",
                           p_event_data->ble_phy_update_event.status,
                p_event_data->ble_phy_update_event.tx_phy,
                p_event_data->ble_phy_update_event.rx_phy,
                p_event_data->ble_phy_update_event.bd_address);
#endif
            app_phy_update(p_event_data->ble_phy_update_event.bd_address,
                           p_event_data->ble_phy_update_event.tx_phy);
            break;

        default:
            WICED_BT_TRACE( "Unhandled management event" );
            result = WICED_BT_ERROR;
            break;
    }
    return result;
}

/******************************************************************************
 *     Public Functions
 ******************************************************************************/


/********************************************************************
 * Function Name: dev_info
 ********************************************************************
 * Summary:
 *  This function returns the device infomation data structure.
 *
 * Parameters:
 *    none
 *
 * Return:
 *    wiced_bt_dev_local_addr_ext_t
 *
 ********************************************************************/
wiced_bt_dev_local_addr_ext_t * dev_info()
{
    return &bt.dev;
}

/********************************************************************
 * Function Name: bt_init
 ********************************************************************
 * Summary:
 *  This is one of the first function to be called upon device reset.
 *  It initialize BT Stack. When BT stack is up, it will call application
 *  to continue system initialization.
 *
 * Parameters:
 *    none
 *
 * Return:
 *    wiced_result_t
 *
 ********************************************************************/
wiced_result_t bt_init()
{
    bt_get_cfg();

    wiced_result_t result = wiced_bt_stack_init(app_bt_management,
                                                p_wiced_bt_cfg_settings);

    if ((result == WICED_BT_SUCCESS)
        && wiced_bt_create_heap("app",NULL,APP_STACK_HEAP_SIZE,NULL,WICED_TRUE)
        == NULL)
    {
        /* Create default heap */
        {
            WICED_BT_TRACE("create default heap error: size %d",
                           APP_STACK_HEAP_SIZE);
            result = WICED_BT_NO_RESOURCES;
        }
    }

    return result;
}

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
               wiced_bt_device_address_ptr_t directed_advertisement_bdaddr_ptr)
{
    bt.intended_adv_mode = advert_mode;
    return wiced_bt_start_advertisements( advert_mode,
                                          directed_advertisement_bdaddr_type,
                                          directed_advertisement_bdaddr_ptr);
}

/* end of file */
