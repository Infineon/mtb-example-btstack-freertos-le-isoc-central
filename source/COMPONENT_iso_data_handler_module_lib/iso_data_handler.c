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


#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "wiced_bt_isoc.h"
#include "wiced_bt_trace.h"
#include "wiced_memory.h"

#include "iso_data_handler.h"

#include "cybt_platform_interface.h"
#include "app_terminal_trace.h"

#define ISO_DATA_HEADER_SIZE 4

#define ISO_LOAD_HEADER_SIZE_WITH_TS 8
#define ISO_LOAD_HEADER_SIZE_WITHOUT_TS 4

#define ISO_PKT_PB_FLAG_MASK 3
#define ISO_PKT_PB_FLAG_OFFSET 12

#define ISO_PKT_PB_FLAG_FIRST_FRAGMENT 0
#define ISO_PKT_PB_FLAG_CONTINUATION_FRAGMENT 1
#define ISO_PKT_PB_FLAG_COMPLETE 2
#define ISO_PKT_PB_FLAG_LAST_FRAGMENT 3

#define ISO_PKT_TS_FLAG_MASK 1
#define ISO_PKT_TS_FLAG_OFFSET 14

#define ISO_PKT_RESERVED_FLAG_MASK 1
#define ISO_PKT_RESERVED_FLAG_OFFSET 15

#define ISO_PKT_DATA_LOAD_LENGTH_MASK 0x3FFF
#define ISO_PKT_SDU_LENGTH_MASK 0x0FFF

wiced_bt_buffer_t *g_cis_iso_pool = NULL;
static iso_dhm_num_complete_evt_cb_t g_num_complete_cb;
static iso_dhm_rx_evt_cb_t g_rx_data_cb;


void iso_dhm_process_rx_data(uint8_t *p_data, uint32_t length)
{
    uint16_t handle_and_flags = 0;
    uint16_t data_load_length = 0;
    uint16_t ts_flag = 0;
    uint16_t pb_flag = 0;
    uint16_t psn = 0;
    uint16_t sdu_len = 0;
    uint32_t ts = 0;

    if (!length) { WICED_BT_TRACE("dhm rx data len = 0 "); return; }

    STREAM_TO_UINT16(handle_and_flags, p_data);
    STREAM_TO_UINT16(data_load_length, p_data);

    pb_flag = (handle_and_flags & (ISO_PKT_PB_FLAG_MASK << ISO_PKT_PB_FLAG_OFFSET)) >> ISO_PKT_PB_FLAG_OFFSET;
    ts_flag = (handle_and_flags & (ISO_PKT_TS_FLAG_MASK << ISO_PKT_TS_FLAG_OFFSET)) >> ISO_PKT_TS_FLAG_OFFSET;

    handle_and_flags &= ~(ISO_PKT_PB_FLAG_MASK << ISO_PKT_PB_FLAG_OFFSET);
    handle_and_flags &= ~(ISO_PKT_TS_FLAG_MASK << ISO_PKT_TS_FLAG_OFFSET);
    handle_and_flags &= ~(ISO_PKT_RESERVED_FLAG_MASK << ISO_PKT_RESERVED_FLAG_OFFSET);

    if (ts_flag) { STREAM_TO_UINT32(ts, p_data); }

    STREAM_TO_UINT16(psn, p_data);
    STREAM_TO_UINT16(sdu_len, p_data);

    data_load_length &= ISO_PKT_DATA_LOAD_LENGTH_MASK;
    sdu_len &= ISO_PKT_SDU_LENGTH_MASK;

     //WICED_BT_TRACE("Recv isoc data size %d ", sdu_len);
     //WICED_BT_TRACE_ARRAY(p_data, sdu_len, "ISO Data");
     //WICED_BT_TRACE("TS %d PB flag %d psn %d ", ts, pb_flag, psn);
    if(sdu_len)
    {
    //TRACE_ISOC_DATA(1);
    //WICED_BT_TRACE("%d %d", sdu_len, psn);
    //TRACE_ISOC_DATA(0);
    }

    (void)ts;
    (void)pb_flag;
    (void)psn;

    if (!sdu_len) { return; }

    if (g_rx_data_cb) { g_rx_data_cb(handle_and_flags, p_data, sdu_len); }
}

CY_SECTION_RAMFUNC_BEGIN
wiced_bool_t iso_dhm_process_num_completed_pkts(uint8_t *p_buf)
{
    uint8_t num_handles, xx;
    uint16_t handle;
    uint16_t num_sent;
    wiced_bool_t complete = WICED_TRUE;


    STREAM_TO_UINT8(num_handles, p_buf);

    for (xx = 0; xx < num_handles; xx++)
    {
        STREAM_TO_UINT16(handle, p_buf);
        STREAM_TO_UINT16(num_sent, p_buf);

        // WICED_BT_TRACE("[%s] handle 0x%x num_sent %d", __FUNCTION__, handle, num_sent);

        //validate handle
        if (wiced_ble_isoc_is_cis_connected_with_conn_hdl(handle) || wiced_ble_isoc_is_bis_created(handle))
        {
            //callback to app to send more packets
            if (g_num_complete_cb) {
                    g_num_complete_cb(handle, num_sent);
            }
        }
        else {
            complete = WICED_FALSE;
        }
    }
    return complete;
}
CY_SECTION_RAMFUNC_END

void iso_dhm_init(const wiced_bt_cfg_isoc_t *p_isoc_cfg,
                  iso_dhm_num_complete_evt_cb_t num_complete_cb,
                  iso_dhm_rx_evt_cb_t rx_data_cb)
{
    wiced_ble_isoc_register_data_cb(iso_dhm_process_rx_data, iso_dhm_process_num_completed_pkts);

    int buff_size =
        (p_isoc_cfg->max_sdu_size * p_isoc_cfg->channel_count) + ISO_LOAD_HEADER_SIZE_WITH_TS + ISO_DATA_HEADER_SIZE;

    // Allocate only once, allowing multiple calls to update callbacks
    if (!g_cis_iso_pool)
        g_cis_iso_pool = wiced_bt_create_pool("ISO SDU", buff_size, p_isoc_cfg->max_buffers_per_cis, NULL);

    WICED_BT_TRACE("[%s] g_cis_iso_pool 0x%x size %d count %d",
                   __FUNCTION__,
                   (int)g_cis_iso_pool,
                   buff_size,
                   p_isoc_cfg->max_buffers_per_cis);

    g_num_complete_cb = num_complete_cb;
    g_rx_data_cb = rx_data_cb;
}

CY_SECTION_RAMFUNC_BEGIN
uint8_t *iso_dhm_get_data_buffer(void)
{
    uint8_t *p_buf = wiced_bt_get_buffer_from_pool(g_cis_iso_pool);

    if (p_buf)
        return p_buf + ISO_LOAD_HEADER_SIZE_WITH_TS + ISO_DATA_HEADER_SIZE;
    else
        return NULL;
}
CY_SECTION_RAMFUNC_END

CY_SECTION_RAMFUNC_BEGIN
void iso_dhm_free_data_buffer(uint8_t *p_buf)
{
    wiced_bt_free_buffer(p_buf - (ISO_LOAD_HEADER_SIZE_WITH_TS + ISO_DATA_HEADER_SIZE));
}
CY_SECTION_RAMFUNC_END

CY_SECTION_RAMFUNC_BEGIN
wiced_bool_t iso_dhm_send_packet(uint16_t psn,
                         uint16_t conn_handle,
                         uint8_t ts_flag,
                         uint8_t *p_data_buf,
                         uint32_t data_buf_len)
{
    uint8_t *p = NULL;
    uint8_t *p_iso_sdu = NULL;
    uint16_t handle_and_flags = conn_handle;
    uint16_t data_load_length = 0;
//  uint16_t psn = 0xFFFF;

    wiced_bool_t result = WICED_FALSE;

    if (data_buf_len > 550)
    {
    WICED_BT_TRACE_CRIT("Received packet larger than the ISO SDU len supported");
        return WICED_FALSE;
    }

    //TRACE_SEND_PKT(1);
    //TRACE_RX_ISR(1);

    handle_and_flags |= (ISO_PKT_PB_FLAG_COMPLETE << ISO_PKT_PB_FLAG_OFFSET);
    handle_and_flags |= (ts_flag << ISO_PKT_TS_FLAG_OFFSET);

    if (ts_flag)
    {
        //timestamp supported, header size is 4 + 8
        p_iso_sdu = p = p_data_buf - (ISO_LOAD_HEADER_SIZE_WITH_TS + ISO_DATA_HEADER_SIZE);
        data_load_length = data_buf_len + ISO_LOAD_HEADER_SIZE_WITH_TS;
    }
    else
    {
        //timestamp not supported, header size is 4 + 4
        p_iso_sdu = p = p_data_buf - (ISO_LOAD_HEADER_SIZE_WITHOUT_TS + ISO_DATA_HEADER_SIZE);
        data_load_length = data_buf_len + ISO_LOAD_HEADER_SIZE_WITHOUT_TS;
    }

    data_load_length &= ISO_PKT_DATA_LOAD_LENGTH_MASK;
    data_buf_len &= ISO_PKT_SDU_LENGTH_MASK;

    UINT16_TO_STREAM(p, handle_and_flags);
    UINT16_TO_STREAM(p, data_load_length);
    UINT16_TO_STREAM(p, psn);
    UINT16_TO_STREAM(p, data_buf_len);

    //result = btu_write_iso_to_lower(BT_TRANSPORT_LE, p_iso_sdu, data_load_length + ISO_DATA_HEADER_SIZE);
    result = wiced_ble_isoc_write_data_to_lower(p_iso_sdu, data_load_length + ISO_DATA_HEADER_SIZE);

    //TRACE_RES_5(1);
    iso_dhm_free_data_buffer(p_data_buf);
    //TRACE_RES_5(0);

    //TRACE_SEND_PKT(0);
    //TRACE_RX_ISR(0);
    return result;
}
CY_SECTION_RAMFUNC_END

uint32_t iso_dhm_get_header_size()
{
    return ISO_LOAD_HEADER_SIZE_WITH_TS + ISO_DATA_HEADER_SIZE;
}
