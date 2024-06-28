/******************************************************************************
* File Name: ctc_client.c
*
* Description: This file contains macros, structures, enumerations and function
*              prototypes used in hidh.c file.
*
* Related Document: See README.md
*
*******************************************************************************
 * $ Copyright YEAR Cypress Semiconductor $
*******************************************************************************/

/*******************************************************************************
*        Header Files
*******************************************************************************/
#include <FreeRTOS.h>
#include <task.h>
#include "wiced_bt_dev.h"
#include "cyabs_rtos_impl.h"
#include "cycfg_bt_settings.h"

#include "bt.h"
#include "gatt.h"
#include "led.h"
#include "button.h"
#include "app_dev/app_dev.h"
#include "isoc_central.h"

/*******************************************************************************
*        Macro Definitions
*******************************************************************************/
/* Macros for button interrupt and button task */
/* Interrupt priority for the GPIO connected to the user button */

/* Priority for GPIO Button Interrupt */
#define GPIO_INTERRUPT_PRIORITY     (7u)

#define MAX_SUPPORTED_DEV_COUNT     MAX_CIS_PER_CIG

#define ISO_SDU_SIZE                100

/********************************************************************
 * Function Name: app_update_cis_handle
 ********************************************************************
 * Summary:
 *  Update ISOC cis_handle
 * ********************************************************************/
void app_update_cis_handle(uint16_t acl_handle, uint16_t cis_handle);

/********************************************************************
 * Function Name: app_add_resolved_address
 ********************************************************************
 * Summary:
 *  Updates the RPA connected address to the resolved address
 * ********************************************************************/
void app_add_resolved_address(const wiced_bt_device_address_t resolved_bdAddr,
                              wiced_bt_device_address_t conn_bdAddr);

/********************************************************************
 * Function Name: app_acl_disconnect_all
 ********************************************************************
 * Summary:
 *  Disconnects all acl connections
 *******************************************************************/
void app_acl_disconnect_all();

/********************************************************************
 * Function Name: app_change_isoc_acl_mode
 ********************************************************************
 * Summary:
 *  switch between ISOC or ACL mode
 * ********************************************************************/
void app_change_isoc_acl_mode();

/********************************************************************
 * Function Name: app_set_phy
 ********************************************************************
 * Summary:
 *  Change Phy speed setting
 *  If bdaddr is NULL, it used previous set_phy bdaddr.
 * ********************************************************************/
wiced_result_t app_set_phy(wiced_bt_device_address_t bdAddr,
                           wiced_bt_ble_host_phy_preferences_t phy);

/********************************************************************
 * Function Name: app_phy_update
 ********************************************************************
 * Summary:
 *  Handles Phy update event
 * ********************************************************************/
void app_phy_update(wiced_bt_device_address_t bdAddr,
                    wiced_bt_ble_host_phy_preferences_t phy);

/********************************************************************
 * Function Name: app_set_acl_conn_interval
 ********************************************************************
 * Summary:
 *  Change Set connection interval
 * ********************************************************************/
wiced_bool_t app_set_acl_conn_interval(uint16_t acl_handle, uint16_t interval);

/********************************************************************
 * Function Name: app_update_acl_conn_interval
 ********************************************************************
 * Summary:
 *  Updates connection interval
 * ********************************************************************/
void app_update_acl_conn_interval(wiced_bt_device_address_t bdAddr,
                                  uint16_t interval);

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
 *
 * Alias:
 *  app_is_any_device_connected(): return TRUE - if has any device connected,
 *  FALSE -- if no device is connected
 *
 *******************************************************************/
uint8_t app_acl_connected_count();
#define app_is_any_device_connected() app_acl_connected_count()

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
                           wiced_bt_ble_advert_mode_t adv);

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
wiced_result_t app_init(void);

/********************************************************************
 * Function Name: app_link_up
 ********************************************************************
 * Summary:
 *  This function is called when link is up
 *
 * Parameters:
 *  wiced_bt_gatt_connection_status_t * p_status -- pointer to the
 *  connection status.
 *
 * Return:
 *  none
 *
 *******************************************************************/
void app_link_up(wiced_bt_gatt_connection_status_t * p_status);

/********************************************************************
 * Function Name: app_link_down
 ********************************************************************
 * Summary:
 *  This function is called when link is down
 *
 * Parameters:
 *  wiced_bt_gatt_connection_status_t * p_status -- pointer to
 *  the connection status.
 *
 * Return:
 *  none
 *
 *******************************************************************/
void app_link_down(const wiced_bt_gatt_connection_status_t * p_status);
