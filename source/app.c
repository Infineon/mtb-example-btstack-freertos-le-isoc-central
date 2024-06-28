/*
 * $ Copyright YEAR Cypress Semiconductor $
 */

/**
 * file app.c
 *
 * This is the LE Isochronous demo application for Central role.This application
 * should be used together with le-isoc-peripheral code example running on a
 * peer device.
 *
 * Related Document: See README.md
 *
 */

/*******************************************************************************
*        Header Files
*******************************************************************************/
#include "wiced_bt_stack.h"
#include "cy_retarget_io.h"
#include "cycfg_gatt_db.h"
#include "cycfg_gap.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_types.h"
#include "wiced_bt_sdp.h"
#include "wiced_bt_l2c.h"
#include "nvram_lib.h"
#include "app_bt_utils.h"
#include "app.h"
#include "string.h"

#include "app_terminal_trace.h"

typedef struct
{
    uint16_t id;          // connect id
    uint16_t acl_handle;  // acl connection handle
    uint16_t acl_conn_interval; // acl connection interval
    uint16_t cis_handle;  // ISOC cis handle
    wiced_bt_device_address_t       bdAddr;
    wiced_bt_device_address_t       resolved_bdAddr;
    wiced_bt_ble_host_phy_preferences_t phy;
} app_conn_info_t;

app_conn_info_t conn[MAX_CIS_PER_CIG] = {0};

/********************************************************************
 * Function Name: app_isoc_mode
 ********************************************************************
 * Summary:
 *  Bring up ISOC link.
 *  This function search for none-ISOC link and change that to ISOC link.
 *
 * Parameters:
 *  none
 *
 * Return:
 *  TRUE: found none-ISOC link and bringing up ISOC
 *  FALSE: all links are already in ISOC mode.
 *
 *******************************************************************/
static wiced_bool_t app_isoc_mode()
{
    int index;

    for (index = 0; index < MAX_CIS_PER_CIG; index++)
    {
        // if ACL link is up && ISOC is not up
        if (conn[index].acl_handle && !conn[index].cis_handle)
        {
            if (conn[index].phy == BTM_BLE_PREFER_2M_PHY)
            {
                WICED_BT_TRACE("Open ISOC for %4x", conn[index].acl_handle);
                isoc_open(conn[index].acl_handle);
            }
            else
            {
#ifdef ENABLE_BT_SPY_LOG
                WICED_BT_TRACE("Set %B phy to 2M", conn[index].bdAddr);
#endif
                app_set_phy(conn[index].bdAddr, BTM_BLE_PREFER_2M_PHY);
            }
            return TRUE;
        }
    }
    return FALSE;
}

/********************************************************************
 * Function Name: app_get_index_by_conn_id
 ********************************************************************
 * Summary:
 *  Find index by conn_id
 *
 * Parameters:
 *  [IN]  uint16_t conn_id;
 *  [OUT] uint8_t * index;
 *
 * Return:
 *  TRUE - conn_id is found, *index contains the index
 *  FALSE -- conn_id is not found, *index contains MAX_CIS_PER_CIG
 *
 *******************************************************************/
static wiced_bool_t app_get_index_by_conn_id(uint16_t conn_id, uint8_t * index)
{
    uint8_t i = 0;
    wiced_bool_t ret = FALSE;

    if (index == NULL) {
        WICED_BT_TRACE("Invalid pointer in %s\r\n", __FUNCTION__);
        goto exit;
    }

    for (i = 0; i < MAX_CIS_PER_CIG; i++)
    {
        if (conn[i].id == conn_id)
        {
            *index = i;
            ret = TRUE;
            break;
        }
    }

exit:
    return ret;
}

/********************************************************************
 * Function Name: app_get_index_by_acl_handle
 ********************************************************************
 * Summary:
 *  Find index by acl_handle
 *
 * Parameters:
 *  [IN]  uint16_t acl_handle;
 *  [OUT] uint8_t * index;
 *
 * Return:
 *  TRUE - acl_handle is found, *index contains the index
 *  FALSE -- acl_handle is not found, *index contains MAX_CIS_PER_CIG
 *
 *******************************************************************/
static wiced_bool_t app_get_index_by_acl_handle(uint16_t acl_handle,
                                                uint8_t * index)
{
    uint8_t i = 0;
    wiced_bool_t ret = FALSE;

    if (index == NULL) {
        WICED_BT_TRACE("Invalid pointer in %s\r\n", __FUNCTION__);
        goto exit;
    }

    for (i= 0; i < MAX_CIS_PER_CIG; i++)
    {
        if (conn[i].acl_handle == acl_handle)
        {
            ret = TRUE;
            break;
        }
    }

    *index = i;

exit:
    return ret;
}

/********************************************************************
 * Function Name: app_get_index_by_bd_addr
 ********************************************************************
 * Summary:
 *  Find index by bdAddr
 *
 * Parameters:
 *  [IN]  wiced_bt_device_address_t bdAddr;
 *  [OUT] uint8_t * index;
 *
 * Return:
 *  TRUE - acl_handle is found, *index contains the index
 *  FALSE -- acl_handle is not found, *index contains MAX_CIS_PER_CIG
 *
 *******************************************************************/
static wiced_bool_t app_get_index_by_bd_addr(
                        const wiced_bt_device_address_t bdAddr, uint8_t * index)
{
    wiced_bt_device_address_t null_addr = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    for (*index = 0; *index < MAX_CIS_PER_CIG; (*index)++)
    {
        if (memcmp(&conn[*index].bdAddr, bdAddr, BD_ADDR_LEN) == 0)
        {
            return TRUE;
        }

        // Check if resolved address is available
        if ((memcmp(&conn[*index].resolved_bdAddr, null_addr, BD_ADDR_LEN) != 0)
            && (memcmp(&conn[*index].resolved_bdAddr, bdAddr, BD_ADDR_LEN) == 0))
        {
            {
                return TRUE;
            }
        }
    }
    return FALSE;
}

/********************************************************************
 * Function Name: app_update_to_resolved_address
 ********************************************************************
 * Summary:
 *  Updates the RPA connected address to the resolved address
 * ********************************************************************/
void app_add_resolved_address(const wiced_bt_device_address_t resolved_bdAddr,
                              wiced_bt_device_address_t conn_bdAddr)
{
    for (uint8_t index = 0; index < MAX_CIS_PER_CIG; index++)
    {
        if (memcmp(conn[index].bdAddr, conn_bdAddr, BD_ADDR_LEN) == 0)
        {
            memcpy(conn[index].resolved_bdAddr, resolved_bdAddr, BD_ADDR_LEN);
#ifdef ENABLE_BT_SPY_LOG
            WICED_BT_TRACE("Updated conn_cb from %B to %B", conn_bdAddr,
                           conn[index].bdAddr);
#endif
        }
    }
}

/********************************************************************
 * Function Name: app_phy_update
 ********************************************************************
 * Summary:
 *  Change Set connection interval
 * ********************************************************************/
void app_phy_update(wiced_bt_device_address_t bdAddr,
                    wiced_bt_ble_host_phy_preferences_t phy)
{
    uint8_t index;

    if (app_get_index_by_bd_addr(bdAddr, &index))
    {
        conn[index].phy = phy;

        // Start isoc mode only if all peripherals are connected
        if (app_acl_connected_count() == MAX_CIS_PER_CIG)
        {
            if( conn[index].acl_conn_interval == NON_ISOC_ACL_CONN_INTERVAL)
            {
                app_change_isoc_acl_mode();
            }
            else
            {
                app_set_acl_conn_interval(conn[index].acl_handle,
                                          NON_ISOC_ACL_CONN_INTERVAL);
            }
        }
    }
}

/********************************************************************
 * Function Name: app_update_cis_handle
 ********************************************************************
 * Summary:
 *  Update ISOC cis_handle
 * ********************************************************************/
void app_update_cis_handle(uint16_t acl_handle, uint16_t cis_handle)
{
    uint8_t index;

    WICED_BT_TRACE("app_update_cis_handle %d %d", acl_handle, cis_handle);

    if (app_get_index_by_acl_handle(acl_handle, &index))
    {
        conn[index].cis_handle = cis_handle;

        if (cis_handle)
        {
            app_isoc_mode();
        }
    }
    else
        WICED_BT_TRACE("invalid index !!");
}

/********************************************************************
 * Function Name: app_set_acl_conn_interval
 ********************************************************************
 * Summary:
 *  Change Set connection interval
 * ********************************************************************/
wiced_bool_t app_set_acl_conn_interval(uint16_t acl_handle, uint16_t interval)
{
    uint8_t index;

    if (app_get_index_by_acl_handle(acl_handle, &index))
    {
        if (conn[index].acl_conn_interval != interval)
        {
            WICED_BT_TRACE("ACL %d: Set connection interval to %d",
                           conn[index].acl_handle, interval);
            wiced_bt_l2cap_update_ble_conn_params(conn[index].bdAddr, interval,
                           interval, 0, interval < 10 ?
                           NON_ISOC_ACL_LINK_SUPERVISION_TIMEOUT :
                           ISOC_ACL_LINK_SUPERVISION_TIMEOUT);
            return TRUE;
        }else
        {
            isoc_start(index);
        }
    }
    return FALSE;
}

/********************************************************************
 * Function Name: app_update_acl_conn_interval
 ********************************************************************
 * Summary:
 *  Updates connection interval
 * ********************************************************************/
void app_update_acl_conn_interval(wiced_bt_device_address_t bdAddr,
                                  uint16_t interval)
{
    uint8_t index;

    if (app_get_index_by_bd_addr(bdAddr, &index))
    {
        conn[index].acl_conn_interval = interval;
        WICED_BT_TRACE("UPDATED");
    }
}

/********************************************************************
 * Function Name: app_acl_connected_count
 ********************************************************************
 * Summary:
 *  return the connected device count.
 *
 * Parameters:
 *  none
 *
 * Return:
 *  return the number of connected devices
 *  (TRUE - has any device connected, FALSE -- no device is connected)
 *
 *******************************************************************/
uint8_t app_acl_connected_count()
{
    uint8_t count=0;

    for (uint8_t index = 0; index < MAX_CIS_PER_CIG; index++)
    {
        if (conn[index].acl_handle)
        {
            count++;
        }
    }
    return count;
}

/********************************************************************
 * Function Name: app_acl_disconnect_all
 ********************************************************************
 * Summary:
 *  Disconnects all acl connections
 *******************************************************************/
void app_acl_disconnect_all()
{
    for (uint8_t index = 0; index < MAX_CIS_PER_CIG; index++)
    {
        if (conn[index].id)
        {
            wiced_bt_gatt_disconnect(conn[index].id);
        }
    }
}

/********************************************************************
 * Function Name: app_change_isoc_acl_mode
 ********************************************************************
 * Summary:
 *  switch between ISOC or ACL mode
 * ********************************************************************/
void app_change_isoc_acl_mode()
{
    WICED_BT_TRACE("Establish ISOC");
    app_isoc_mode();
}

/********************************************************************
 * Function Name: app_set_phy
 ********************************************************************
 * Summary:
 *  Change Phy speed setting
 *  If bdaddr is NULL, it used previous set_phy bdaddr.
 * ********************************************************************/
wiced_result_t app_set_phy(wiced_bt_device_address_t bdAddr,
                           wiced_bt_ble_host_phy_preferences_t phy)
{
    wiced_bt_ble_phy_preferences_t phy_preferences = {0};

    memcpy(phy_preferences.remote_bd_addr, bdAddr, BD_ADDR_LEN);
    phy_preferences.rx_phys = phy;
    phy_preferences.tx_phys = phy;

    wiced_result_t status = wiced_bt_ble_set_phy(&phy_preferences);
#ifdef ENABLE_BT_SPY_LOG
    WICED_BT_TRACE("Set %B PHY to %dM status %d", bdAddr, phy, status);
#endif
    return status;
}

/********************************************************************
 * Function Name: app_link_up
 ********************************************************************
 * Summary:
 *  This function is called when link is up
 *
 * Parameters:
 *  wiced_bt_gatt_connection_status_t * p_status -- pointer to the connection
 *  status.
 *
 * Return:
 *  none
 *
 *******************************************************************/
void app_link_up(wiced_bt_gatt_connection_status_t * p_status)
{
    wiced_bt_ble_conn_params_t conn_parameters;
    uint8_t index;
#ifndef DISABLE_ENCRYPTION
    wiced_bt_device_link_keys_t link_key;
    wiced_result_t result;
    wiced_bt_ble_sec_action_type_t encryption_type = BTM_BLE_SEC_ENCRYPT;
#endif
#ifdef ENABLE_BT_SPY_LOG
    WICED_BT_TRACE("Link up, conn_id:%04x peer_addr:%B type:%d",
                   p_status->conn_id, p_status->bd_addr, p_status->addr_type);
#endif
    /*  link is already up or found an empty slot */
    if (app_get_index_by_conn_id(p_status->conn_id, &index) ||
        app_get_index_by_conn_id(0, &index))
    {
        wiced_bt_l2cap_enable_update_ble_conn_params(p_status->bd_addr, TRUE);

        wiced_bt_ble_get_connection_parameters(p_status->bd_addr,
                                               &conn_parameters);

        conn[index].id = p_status->conn_id;
        conn[index].acl_handle = wiced_bt_dev_get_acl_conn_handle(
                                     p_status->bd_addr, BT_TRANSPORT_LE);
        conn[index].acl_conn_interval = conn_parameters.conn_interval;
        memcpy(conn[index].bdAddr, p_status->bd_addr, BD_ADDR_LEN);
        led_on(LED_LINK);

#ifndef DISABLE_ENCRYPTION
        if (app_dev_get_link_key(p_status->bd_addr, &link_key))
        {
            /* Already Paired. Let's Start Encryption */
            result = wiced_bt_dev_set_encryption (p_status->bd_addr,
                         BT_TRANSPORT_LE, &encryption_type);
            WICED_BT_TRACE("wiced_bt_dev_set_encryption %d", result);
        }
        else
        {
            /* Start Pairing */
            result = wiced_bt_dev_sec_bond(p_status->bd_addr,
                                           p_status->addr_type,
                                           BT_TRANSPORT_LE, 0, NULL);
            WICED_BT_TRACE("wiced_bt_dev_sec_bond %d", result);
        }
#else

        // Set the preferred Phy to 2M
        app_set_phy(conn[index].bdAddr, BTM_BLE_PREFER_2M_PHY);
#endif
    }
    else
    {
        WICED_BT_TRACE("Exceed max supported link, disconnecting...");
        wiced_bt_gatt_disconnect(p_status->conn_id);
    }
}

/********************************************************************
 * Function Name: app_link_down
 ********************************************************************
 * Summary:
 *  This function is called when link is down
 *
 * Parameters:
 *  wiced_bt_gatt_connection_status_t * p_status -- pointer to the connection
 *  status.
 *
 * Return:
 *  none
 *
 *******************************************************************/
void app_link_down(const wiced_bt_gatt_connection_status_t * p_status)
{
    uint8_t index;
    uint8_t * p = NULL;

    WICED_BT_TRACE("Link down, id:0x%04x reason: %s",  p_status->conn_id,
                   get_bt_gatt_disconn_reason_name(p_status->reason));

    if (app_get_index_by_conn_id(p_status->conn_id, &index))
    {
        if(index < MAX_CIS_PER_CIG) {
            p = (uint8_t *)&conn[index];
            memset(p, 0, sizeof(app_conn_info_t));
        }
    }

    // if we still have have any link connected, don't turn off LED
    if(!app_is_any_device_connected())
    {
        led_off(LED_LINK);
        led_off(LED_RED);
        led_blink_stop(LED_RED);
    }
}

/********************************************************************
 * Function Name: app_adv_state_changed
 ********************************************************************
 * Summary:
 *  This function is called when advertisment state is changed
 *
 * Parameters:
 *  wiced_bt_ble_advert_mode_t adv  -- new advertisment mode.
 *
 * Return:
 *  none
 *
 *******************************************************************/
void app_adv_state_changed(wiced_bt_ble_advert_mode_t old_adv,
                           wiced_bt_ble_advert_mode_t adv)
{
    if (adv == BTM_BLE_ADVERT_OFF)
    {
        WICED_BT_TRACE("Advertisement Stopped");
        led_blink_stop(LED_LINK);
    }
    else
    {
        if (old_adv == BTM_BLE_ADVERT_OFF)
        {
            WICED_BT_TRACE("Advertisement %d started", adv);
            /* flash LED for adv use faster blink LINK line to
               indicate reconnecting*/
            led_blink(LED_LINK, 0, 500);
        }
        else
        {
            WICED_BT_TRACE("Advertisement State Change: %d -> %d",
                           old_adv, adv);
        }
    }
}

/********************************************************************
 * Function Name: app_init
 ********************************************************************
 * Summary:
 *  When BT Management Stack is initialized successfully,
 *  this function is called.
 *
 * Parameters:
 *  none
 *
 * Return:
 *  none
 *
 *******************************************************************/
wiced_result_t app_init(void)
{
    WICED_BT_TRACE("app_init");

    /* Initialize each submodule */
    app_dev_init();
#ifdef AUTO_PAIRING
    WICED_BT_TRACE("Pairing infomation is erased");
    app_dev_remove_all();
#endif
    button_init();
    gatt_initialize();
    isoc_init();
    led_init();

    /* Allow peer to pair */
    wiced_bt_set_pairable_mode(WICED_TRUE, FALSE);

#ifdef AUTO_PAIRING
    app_le_start_scan();
#endif

    return WICED_BT_SUCCESS;
}

/*******************************************************************************
 * Function Name: application_start()
********************************************************************************
 *  Entry point to the application. Set device configuration and start Bluetooth
 *  stack initialization.  The actual application initialization (app_init) will
 *  be called when stack reports that Bluetooth device is ready.
 *
 * Parameters:
 *  none
 *
 * Return:
 *  none
 *
 ******************************************************************************/
void application_start( void )
{
    /* WICED_BT_TRACE starts to work after bt_init()*/
    bt_init();
    /* use default default NVRAM read/write*/
    nvram_init(NULL);

    WICED_BT_TRACE("\n\n******** LE ISOC Central Application Start *********");
    WICED_BT_TRACE("Name: \"%s\"", app_gap_device_name);
    WICED_BT_TRACE("DEV=%d btstack:%d.%d.%d", CHIP,
                   WICED_BTSTACK_VERSION_MAJOR, WICED_BTSTACK_VERSION_MINOR,
                   WICED_BTSTACK_VERSION_PATCH);

#if defined(LED_SUPPORT) || defined(BUTTON_SUPPORT)
 #ifdef LED_SUPPORT
  #define LED_STRING " LED"
 #else
  #define LED_STRING ""
 #endif
 #ifdef BUTTON_SUPPORT
  #define BUTTON_STRING " BUTTON"
 #else
  #define BUTTON_STRING ""
 #endif
    WICED_BT_TRACE("Supported Component:" LED_STRING BUTTON_STRING);
#endif

#if GATT_TRACE || LED_TRACE || NVRAM_TRACE || DEVICE_TRACE
 #if GATT_TRACE
  #define GATT_TSTRING " GATT"
 #else
  #define GATT_TSTRING ""
 #endif
 #if LED_TRACE
  #define LED_TSTRING " LED"
 #else
  #define LED_TSTRING ""
 #endif
 #if NVRAM_TRACE
  #define NVRAM_TSTRING " NVRAM"
 #else
  #define NVRAM_TSTRING ""
 #endif
 #if DEVICE_TRACE
  #define DEVICE_TSTRING " DEVICE"
 #else
  #define DEVICE_TSTRING ""
 #endif
    WICED_BT_TRACE("Enabled Trace:" GATT_TSTRING LED_TSTRING
                   NVRAM_TSTRING DEVICE_TSTRING);
#endif
}

/* end of file */
