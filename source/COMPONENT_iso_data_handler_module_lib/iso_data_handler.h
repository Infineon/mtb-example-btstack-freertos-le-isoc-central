/*
 * (c) 2025, Infineon Technologies AG, or an affiliate of Infineon
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
#ifndef ISO_DATA_HANDLER_H_
#define ISO_DATA_HANDLER_H_

#include "wiced_bt_cfg.h"

typedef void (*iso_dhm_num_complete_evt_cb_t)(uint16_t cis_handle, uint16_t num_sent);
typedef void (*iso_dhm_rx_evt_cb_t)(uint16_t cis_handle, uint8_t *p_data, uint32_t length);

void iso_dhm_init(const wiced_bt_cfg_isoc_t *p_isoc_cfg, iso_dhm_num_complete_evt_cb_t num_complete_cb, iso_dhm_rx_evt_cb_t rx_data_cb);

uint8_t *iso_dhm_get_data_buffer(void);
void iso_dhm_free_data_buffer(uint8_t *p_buf);

//void iso_dhm_send_packet(wiced_bool_t is_cis, uint16_t conn_handle, uint8_t ts_flag, uint8_t *p_data_buf, uint32_t data_buf_len);
wiced_bool_t iso_dhm_send_packet(uint16_t psn, uint16_t conn_handle, uint8_t ts_flag, uint8_t *p_data_buf, uint32_t data_buf_len);

wiced_bool_t iso_dhm_process_num_completed_pkts(uint8_t *p_buf);
void iso_dhm_process_rx_data(uint8_t *p_data, uint32_t length);

uint32_t iso_dhm_get_header_size();
#endif /* ISO_DATA_HANDLER_H_ */
