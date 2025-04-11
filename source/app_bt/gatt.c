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

/** @file
 *
 * GATT callback function and handlers
 *
 */

#include <app_le.h>
#include "wiced_memory.h"
#include "app.h"
#include "cycfg_gap.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_l2c.h"

#if GATT_TRACE
# define APP_GATT_TRACE        WICED_BT_TRACE
# if GATT_TRACE>1
#  define APP_GATT_TRACE2      WICED_BT_TRACE
# else
#  define APP_GATT_TRACE2(...)
# endif
#else
# define APP_GATT_TRACE(...)
# define APP_GATT_TRACE2(...)
#endif

#include "app_terminal_trace.h"
/*******************************************************************************
 * typedef
 ******************************************************************************/
typedef void (*pfn_free_buffer_t)(uint8_t *);

/*******************************************************************************
 * Private functions
 ******************************************************************************/

/*----------------------------------------------------------
 * Function Name: gatt_free_buffer
 *----------------------------------------------------------
 * Summary:
 *  Frees buffer.
 *
 * Parameters:
 *  uint8_t * p_data
 *
 * Return:
 *  none
 *
 *---------------------------------------------------------*/
static void gatt_free_buffer(uint8_t *p_data)
{
    wiced_bt_free_buffer(p_data);
    APP_GATT_TRACE2("[%s] free 0x%x", __FUNCTION__, p_data);
}

/*----------------------------------------------------------
 * Function Name: gatt_alloc_buffer
 *----------------------------------------------------------
 * Summary:
 *  allocate buffer function.
 *
 * Parameters:
 *  uint16_t len
 *
 * Return:
 *  buffer pointer
 *
 *---------------------------------------------------------*/
static uint8_t * gatt_alloc_buffer(uint16_t len)
{
    uint8_t *p = (uint8_t *)wiced_bt_get_buffer(len);
    APP_GATT_TRACE2("[%s] allo len %d, 0x%x", __FUNCTION__, len, p);
    return p;
}

/*----------------------------------------------------------
 * Function Name: gatt_conn_state_change
 *----------------------------------------------------------
 * Summary:
 *  Handles connection state change. This function is called when the link is 
 *  up or down.It calls link module for the link event.
 *
 * Parameters:
 *  wiced_bt_gatt_connection_status_t * p_status
 *
 * Return:
 *  wiced_bt_gatt_status_t
 *
 *---------------------------------------------------------*/
static wiced_bt_gatt_status_t gatt_conn_state_change(
                              wiced_bt_gatt_connection_status_t * p_status )
{
    if(p_status->connected)
    {
#ifndef DISABLE_ENCRYPTION
        // configure mtu size from bt configuration
        if (p_wiced_bt_cfg_settings->p_ble_cfg->ble_max_rx_pdu_size 
            > GATT_BLE_DEFAULT_MTU_SIZE )
        {
            APP_GATT_TRACE("configure mtu: %d", 
                p_wiced_bt_cfg_settings->p_ble_cfg->ble_max_rx_pdu_size);
            wiced_bt_gatt_client_configure_mtu( p_status->conn_id,
                p_wiced_bt_cfg_settings->p_ble_cfg->ble_max_rx_pdu_size );
        }
#endif
        app_link_up(p_status);
    }
    else
    {
        app_link_down(p_status);
    }
    return WICED_BT_SUCCESS;
}

/*----------------------------------------------------------
 * Function Name: gatt_callback
 *----------------------------------------------------------
 * Summary:
 *  This is a GATT event callback handler
 *
 * Parameters:
 *  wiced_bt_gatt_evt_t event
 *  wiced_bt_gatt_event_data_t * p_data
 *
 * Return:
 *  wiced_bt_gatt_status_t
 *
 *---------------------------------------------------------*/
static wiced_bt_gatt_status_t gatt_callback(wiced_bt_gatt_evt_t event,
                                            wiced_bt_gatt_event_data_t * p_data)
{
    wiced_bt_gatt_status_t result = WICED_BT_GATT_SUCCESS;

    APP_GATT_TRACE2("wiced_le_gatt_callback event:%d", event);

    switch(event)
    {
        case GATT_CONNECTION_STATUS_EVT:
            APP_GATT_TRACE2("GATT_CONNECTION_STATUS_EVT");
            result = gatt_conn_state_change(&p_data->connection_status);
            break;

        case GATT_OPERATION_CPLT_EVT:
            /* GATT Read/Write complete events */
            APP_GATT_TRACE2("GATT_OPERATION_CPLT_EVT conn_id:0x%x",
                            p_data->operation_complete.conn_id);
            break;

        case GATT_DISCOVERY_RESULT_EVT:
            /* GATT Discovery events */
            APP_GATT_TRACE2("GATT_DISCOVERY_RESULT_EVT conn_id:0x%x",
                            p_data->discovery_result.conn_id);
            break;

        case GATT_DISCOVERY_CPLT_EVT:
            /* GATT Discovery Complete events */
            APP_GATT_TRACE2("GATT_DISCOVERY_CPLT_EVT conn_id:0x%x",
                            p_data->discovery_complete.conn_id);
            break;

        case GATT_ATTRIBUTE_REQUEST_EVT:
            APP_GATT_TRACE2("GATT_ATTRIBUTE_REQUEST_EVT");
            /* GATT Request events (received from peer, Client, device */
            break;

        case GATT_CONGESTION_EVT:
            APP_GATT_TRACE2("GATT_CONGESTION_EVT:%d",
                            p_data->congestion.congested);
            break;

        case GATT_GET_RESPONSE_BUFFER_EVT:
            APP_GATT_TRACE2("GATT_GET_RESPONSE_BUFFER_EVT");
            p_data->buffer_request.buffer.p_app_rsp_buffer = gatt_alloc_buffer(
                p_data->buffer_request.len_requested);
            p_data->buffer_request.buffer.p_app_ctxt = (void *)gatt_free_buffer;
            result = WICED_BT_GATT_SUCCESS;
            break;

        case GATT_APP_BUFFER_TRANSMITTED_EVT:
            APP_GATT_TRACE2("GATT_APP_BUFFER_TRANSMITTED_EVT");
            {
                pfn_free_buffer_t pfn_free = (pfn_free_buffer_t)
                    p_data->buffer_xmitted.p_app_ctxt;

                /* If the buffer is dynamic, the context will point to a 
                   function to free it. */
                if (pfn_free)
                {
                    pfn_free(p_data->buffer_xmitted.p_app_data);
                }
                result = WICED_BT_GATT_SUCCESS;
            }
            break;

        default:
            APP_GATT_TRACE("gatts_callback: unhandled event!!!:0x%x", event);
            break;
    }

    return result;
}

/*******************************************************************************
 * PUBLIC FUNCTIONS
 ******************************************************************************/

/***********************************************************
 * Function Name: gatt_initialize
 ***********************************************************
 * Summary:
 *  Initialize gatt database.
 *  The advertisement data CY_BT_ADV_PACKET_DATA_SIZ are cy_bt_adv_packet_data 
 *  are generated by BT Configurator in cycfg_gap.h/c.
 *  The gatt database gatt_database, and gatt_database_len are generated by 
 *  BT Configurator in cycfg_gatt_db.h/c.
 *
 * Parameters:
 *  None
 *
 * Return:
 *  wiced_bt_gatt_status_t
 *
 ***********************************************************/
wiced_bt_gatt_status_t gatt_initialize()
{
    APP_GATT_TRACE("[%s]", __FUNCTION__);

    wiced_bt_ble_set_raw_advertisement_data(CY_BT_ADV_PACKET_DATA_SIZE,
                                            cy_bt_adv_packet_data);

    /* Register with stack to receive GATT callback */
    wiced_bt_gatt_register( gatt_callback );

    return wiced_bt_gatt_db_init( gatt_database, gatt_database_len, NULL );
}

/* end of file */
