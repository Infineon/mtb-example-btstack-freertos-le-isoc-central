/*
 * $ Copyright YEAR Cypress Semiconductor $
 */
/*
 * isoc_central.c
 */

#include "app_le.h"
#include "isoc_central.h"
#include "iso_data_handler.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_types.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_cfg.h"
#include "wiced_timer.h"
#include "wiced_memory.h"
#include "app.h"

#include "app_terminal_trace.h"
#include "cyabs_rtos.h"
/******************************************************************************
 *  defines
 ******************************************************************************/
#if ISOC_TRACE
# define APP_ISOC_TRACE                        WICED_BT_TRACE
# define APP_ISOC_TRACE_ARRAY(ptr, len)        WICED_BT_TRACE("%A", ptr, len)
# define APP_ISOC_TRACE_S_ARRAY(str, ptr, len)  \
         WICED_BT_TRACE("%s %A", str, ptr, len)
#else
# define APP_ISOC_TRACE(...)
# define APP_ISOC_TRACE_ARRAY(ptr, len)
# define APP_ISOC_TRACE_S_ARRAY(str, ptr, len)
#endif

#define ISO_MAX_CIG                         1
#define ISOC_MAX_BURST_COUNT                1      // Must be >= 1
#define CIS_QUEUE_LEN                       (10)
#define ISO_SDU_INTERVAL                    10000 //sdu interval in micro-second
//4 minute keep alive timer to ensure app and controller psn stays synchronized
#define ISOC_KEEP_ALIVE_TIMEOUT_IN_SECONDS  120

#define ISOC_STATS    // ISOC statistics periodically printed with this flag
#ifdef ISOC_STATS
#define ISOC_STATS_TIMEOUT                  5
#endif

// ISOC statistics periodically printed with this flag
#define ISOC_MONITOR_FOR_DROPPED_SDUs   
#ifdef ISOC_MONITOR_FOR_DROPPED_SDUs
#define ISOC_ERROR_DROPPED_SDU_VSE_OPCODE   0x008b

#pragma pack(1)
typedef struct
{
    uint16_t  connHandle;
    uint16_t  psn;
    uint32_t  timestamp;
    uint16_t  expected_psn;
    uint32_t  expected_timestamp;
} isoc_error_dropped_sdu_t;
#pragma pack()
#endif

/******************************************************************************
 *  local variables
 ******************************************************************************/
enum
{
    CIS_CLOSED,
    CIS_CREATE,
    CIS_ESTABLISHED,
    CIS_REMOVE,
    CIS_OPEN,
};

typedef struct
{
    uint8_t  status;
    uint8_t  id;            // CIS ID
    uint16_t handle;        // CIS Handle
    uint16_t acl_handle;    // ACL Connection Handle for this CIS
    uint16_t psn;
    uint8_t credits;
    QueueHandle_t   m_q;
    SemaphoreHandle_t m_mtx;
} isoc_cis_info_t;

typedef struct
{
    uint8_t  id;            // CIG ID
    uint8_t  cis_count;     // CIS count
    isoc_cis_info_t cis[MAX_CIS_PER_CIG];    
} isoc_cig_info_t;

typedef struct
{
    uint16_t interval;      // ISO_interval
    uint8_t  cig_count;     // CIG count
    wiced_timer_t isoc_keep_alive_timer[MAX_CIS_PER_CIG];
    isoc_cig_info_t cig[ISO_MAX_CIG];
} isoc_info_t;

isoc_info_t iso = {ISO_SDU_INTERVAL, ISO_MAX_CIG};

#pragma pack(1)
typedef struct
{
    uint16_t    cis_id;
    uint16_t    sequence_num;
    uint8_t     button_state;
} peripheral_button_state_type_t;
#pragma pack()

typedef enum {
    BTN_PRESSED,      // btn is pressed
    BTN_RELEASED,     // btn is released
    DUMMY             // need to send an empty to maintain PSN in BTSS
}tISOC_DATA_TYPE;
// after the button is pressed, ISOC_MAX_BURST_COUNT packets
// will be sent
static uint16_t iso_sdu_count[MAX_CIS_PER_CIG] = {0};
uint8_t isoc_teardown_pending = 0;

const uint16_t ISOC_START_HDL_VALUE = 0x60;
const int CONTROLLER_ISO_DATA_PACKET_BUFS = 6;
#define CREDITS_PER_CIS (CONTROLLER_ISO_DATA_PACKET_BUFS / MAX_CIS_PER_CIG)

// ISOC stats counters
static uint32_t isoc_rx_count[MAX_CIS_PER_CIG] = {0};
static uint32_t isoc_tx_count[MAX_CIS_PER_CIG] = {0};
static uint32_t isoc_tx_fail_count[MAX_CIS_PER_CIG] = {0};
static uint32_t isoc_tx_dropped_count[MAX_CIS_PER_CIG] = {0};
static wiced_timer_t iso_stats_timer;
wiced_ble_isoc_data_path_direction_t dp_dir;
static void isoc_send_data_payload(bool from_tx_cmplt, const uint8_t index, uint8_t pressed );
void start_read_psn_using_vsc(uint16_t hdl);
void app_send_dummy(uint16_t handle);
bool is_tx_in_progress(const int idx);

/*******************************************************************************
 * Macros
 ******************************************************************************/
#define VALID_INDEX(i)  (i < iso.CIG.cis_count)
#define INVALID_INDEX(i) (i >= iso.CIG.cis_count)
#define CIG cig[0]          // we only support one cig for this app

/*******************************************************************************
 * private functions
 ******************************************************************************/

void append_button_status(int idx, uint8_t data)
{
   xSemaphoreTake(iso.CIG.cis[idx].m_mtx, 3);
   if(uxQueueSpacesAvailable(iso.CIG.cis[idx].m_q))
   {
       xQueueSend( iso.CIG.cis[idx].m_q, &data, 0);
   }
   xSemaphoreGive(iso.CIG.cis[idx].m_mtx);
}

// caller should make sure there is data in queue
uint8_t get_button_status(int idx, bool pop)
{
   uint8_t data = DUMMY;
   xSemaphoreTake(iso.CIG.cis[idx].m_mtx, 3);
   if(pop)
   {
        xQueueReceive( iso.CIG.cis[idx].m_q, &data, 3);
   }else{
        xQueuePeek( iso.CIG.cis[idx].m_q, &data, 0);
   }
   
   xSemaphoreGive(iso.CIG.cis[idx].m_mtx);

   return data;
}

bool is_queue_empty(int idx)
{
   bool v;

   xSemaphoreTake(iso.CIG.cis[idx].m_mtx, 3);
  
   v = (CIS_QUEUE_LEN == uxQueueSpacesAvailable(iso.CIG.cis[idx].m_q));
   
   xSemaphoreGive(iso.CIG.cis[idx].m_mtx);

   //printf("[%d] Q available %d\n", idx, ret& 0x0FF);
   return v;
}

static int find_index_by_cis_handle(uint16_t cis_handle);

/***************************************************************************
 * Function Name: find_index_by_cis_handle
 ***************************************************************************
 * Summary:
 *  Returns index into the cis control block based on the provided handle
 ***************************************************************************/
CY_SECTION_RAMFUNC_BEGIN
static int find_index_by_cis_handle(uint16_t cis_handle)
{
    uint8_t i;

    for (i=0; VALID_INDEX(i); i++)
    {
        if (iso.CIG.cis[i].handle == cis_handle)
        {
            break;
        }
    }
    return i;
}
CY_SECTION_RAMFUNC_END

void app_send_dummy(uint16_t handle)
{
    int i = handle - ISOC_START_HDL_VALUE;
 
   if(!is_tx_in_progress(i))
    {
        printf("CIS[%d] skip app_send_dummy\n", i);
        return;
    }
    uint8_t* p_buf = iso_dhm_get_data_buffer();
    iso.CIG.cis[i].credits--;
    printf("CIS[%d] app_send_dummy credits %d\n", i, iso.CIG.cis[i].credits);
    iso_dhm_send_packet(iso.CIG.cis[i].psn,
                        iso.CIG.cis[i].handle, WICED_FALSE, p_buf, 0);
}

// is packet in controller?
bool is_tx_in_progress(const int idx)
{
    return iso.CIG.cis[idx].credits == CREDITS_PER_CIS;
}
/********************************************************************
 * Function Name: isoc_send_data_payload
 ********************************************************************
 * Summary:
 *  Helper function for isoc_send_data_payload
 ********************************************************************/
CY_SECTION_RAMFUNC_BEGIN
static void isoc_send_data_payload(bool from_tx_cmplt, const uint8_t index, uint8_t pressed)
{
    uint8_t result;
    uint32_t data_length;
    uint8_t* p_buf = NULL;
    uint8_t* p_data = NULL;    

    // increase the packet count to send
    // if it is call from tx complete call back. there is no need to 
    // increase pending number.
    if(!from_tx_cmplt)
    {
        append_button_status(index, pressed);
        if(!is_tx_in_progress(index))
        {
            printf("CIS[%d] increase pending only\n", index);        
            return;
        }
    }
    if((p_buf = iso_dhm_get_data_buffer()) != NULL)
    {
        p_data = p_buf;

        UINT16_TO_STREAM(p_data, iso.CIG.cis[index].handle);
        UINT16_TO_STREAM(p_data, iso.CIG.cis[index].psn);
        UINT8_TO_STREAM(p_data, pressed);

        data_length = p_wiced_bt_cfg_settings->p_isoc_cfg->max_sdu_size;        
        iso.CIG.cis[index].credits--;
        printf("CIS[%d] isoc_send_data_payload credits %d\n", index, iso.CIG.cis[index].credits);
       
        result=iso_dhm_send_packet(iso.CIG.cis[index].psn,
                                   iso.CIG.cis[index].handle, WICED_FALSE,
                                   p_buf, data_length);

        if(result)
        {
            isoc_tx_count[index]++;            
        }
        else
        {
            isoc_tx_fail_count[index]++;
        }

        APP_ISOC_TRACE("[isoc_send_data_payload] handle:0x%x SN:%d data_length:"
                       "%d sdu_count:%d result:%d\n", iso.CIG.cis[index].handle,
                       iso.CIG.cis[index].psn, (int)data_length,
                       (int)iso_sdu_count[index], result);
    }
}
CY_SECTION_RAMFUNC_END

/********************************************************************
 * Function Name: rx_handler
 ********************************************************************
 * Summary:
 *  Handles received ISOC data
 *******************************************************************/
CY_SECTION_RAMFUNC_BEGIN
static void rx_handler(uint16_t cis_handle, uint8_t *p_data, uint32_t length)
{
    int index = find_index_by_cis_handle(cis_handle);
    peripheral_button_state_type_t* p_rx_data =
        (peripheral_button_state_type_t*) p_data;

    //APP_ISOC_TRACE("[%s] length:%d", __FUNCTION__, length);

    if ((index < MAX_CIS_PER_CIG) &&
        (length >= sizeof(peripheral_button_state_type_t)))
    {
            set_gpio_high(C_ACTION);

            APP_ISOC_TRACE("[rx_data][%x] cis_id:0x%x SN:%d button_state:%d", cis_handle,
                           p_rx_data->cis_id, p_rx_data->sequence_num,
                           p_rx_data->button_state);
            isoc_rx_count[index]++;

            set_gpio_low(C_ACTION);

            led_blink2(LED_RED, 1, 250, 250);
    }
}
CY_SECTION_RAMFUNC_END

/********************************************************************
 * Function Name: isoc_set_cig
 ********************************************************************
 * Summary:
 *  Configures the CIG and sends the HCI_LE_SET_CIG_Parameters_command
 *  HCI command to the controller
 *******************************************************************/
#define  USE_CIS_TEST_PARAM
#ifdef  USE_CIS_TEST_PARAM
static wiced_result_t isoc_set_cig()
{
    wiced_result_t                 result;
    wiced_bt_ble_cis_config_test_t cis_config_list_test[MAX_CIS_PER_CIG];
    wiced_ble_isoc_cig_param_test_t  cig_param_test;
    uint8_t                        index;

        printf("[%s] cis_per_cig count %d \r\n", __FUNCTION__, MAX_CIS_PER_CIG);

        cig_param_test.cig_id    = 0;
        cig_param_test.cis_count = MAX_CIS_PER_CIG;

        cig_param_test.sdu_interval_c_to_p       = ISO_SDU_INTERVAL;
        cig_param_test.sdu_interval_p_to_c       = ISO_SDU_INTERVAL;
        cig_param_test.worst_case_sca = 0;

        cig_param_test.packing = WICED_BLE_ISOC_SEQUENTIAL_PACKING;
        cig_param_test.framing = WICED_BLE_ISOC_UNFRAMED;

        cig_param_test.ft_c_to_p    = 1;
        cig_param_test.ft_p_to_c    = 1;
        cig_param_test.iso_interval = 8;

        cig_param_test.p_cis_config_list = cis_config_list_test;

        for (index = 0; index < cig_param_test.cis_count; index++)
        {
            cig_param_test.p_cis_config_list[index].cis_id = index;
            cig_param_test.p_cis_config_list[index].nse    = 2;

            cig_param_test.p_cis_config_list[index].max_sdu_c_to_p = p_wiced_bt_cfg_settings->p_isoc_cfg->max_sdu_size;;
            cig_param_test.p_cis_config_list[index].max_pdu_c_to_p = p_wiced_bt_cfg_settings->p_isoc_cfg->max_sdu_size;
            cig_param_test.p_cis_config_list[index].bn_c_to_p      = 1;
            cig_param_test.p_cis_config_list[index].phy_c_to_p     = WICED_BLE_ISOC_LE_2M_PHY;

            cig_param_test.p_cis_config_list[index].max_sdu_p_to_c = p_wiced_bt_cfg_settings->p_isoc_cfg->max_sdu_size;;
            cig_param_test.p_cis_config_list[index].max_pdu_p_to_c = p_wiced_bt_cfg_settings->p_isoc_cfg->max_sdu_size;;

            cig_param_test.p_cis_config_list[index].phy_p_to_c     = WICED_BLE_ISOC_LE_2M_PHY;
            cig_param_test.p_cis_config_list[index].bn_p_to_c      = 1;
        }
        result = wiced_ble_isoc_central_set_cig_param_test(&cig_param_test);

        printf("[%s] exit %d\n", __FUNCTION__, result);

        return result;
}
#else
static wiced_result_t isoc_set_cig()
{
    wiced_result_t result;

    APP_ISOC_TRACE("[%s] cis_per_cig count %d ", __FUNCTION__, MAX_CIS_PER_CIG);

    wiced_ble_isoc_cis_config_t cis_config_list[MAX_CIS_PER_CIG];
    wiced_ble_isoc_cig_param_t cig_param;

    uint8_t index;

    cig_param.cig_id = 0;
    cig_param.cis_count = MAX_CIS_PER_CIG;

    /* 48_2 ISO params: SDU_Interval=10ms, Framing=UNframed, Max_sdu_size=100,
       RTN=5, Max_transport_latency=20 */
    /**< Time interval in microseconds between the start of consecutive SDUs
         from the Central for all the CISes in the CIG */
    cig_param.sdu_interval_c_to_p = ISO_SDU_INTERVAL;
    /**< Time interval in microseconds between the start of consecutive SDUs
         from the Peripheral for all the CISes in the CIG. */
    cig_param.sdu_interval_p_to_c = ISO_SDU_INTERVAL;
    /**< Peripheral Clock Accuracy */
    cig_param.worst_case_sca = 0;
    /**< Maximum time, in microseconds, for an SDU to be transported
         from the Central Controller to Peripheral Controller */
    cig_param.max_trans_latency_c_to_p = 20;
    /**< Maximum time, in microseconds, for an SDU to be transported from the
         Peripheral Controller to Peripheral Controller */
    cig_param.max_trans_latency_p_to_c = 20;

#if (MAX_CIS_PER_CIG == 1)
     /**< Sequential Packing method, see wiced_bt_isoc_packing_t */
    cig_param.packing = WICED_BLE_ISOC_SEQUENTIAL_PACKING;
#else
    /**< Interleaved Packing method, see wiced_bt_isoc_packing_t */
    cig_param.packing = WICED_BLE_ISOC_INTERLEAVED_PACKING;
#endif
    /**< Framing parameter, see wiced_bt_isoc_framing_t */
    cig_param.framing = WICED_BLE_ISOC_UNFRAMED;

    cig_param.p_cis_config_list = cis_config_list;
    for (index = 0; index < cig_param.cis_count; index++)
    {
        cig_param.p_cis_config_list[index].cis_id = index;
        /**< Maximum size, in octets, of an SDU from the central,
             Valid Range 0x000 to 0xFFF*/
        cig_param.p_cis_config_list[index].max_sdu_c_to_p =
            p_wiced_bt_cfg_settings->p_isoc_cfg->max_sdu_size;
        /**< Maximum number of times every CIS Data PDU should be retransmitted
             from the central to peripheral */
        cig_param.p_cis_config_list[index].rtn_c_to_p = 5/MAX_CIS_PER_CIG;
        /**< The transmitter PHY of packets from the central */
        cig_param.p_cis_config_list[index].phy_c_to_p=WICED_BLE_ISOC_LE_2M_PHY;
        /**< Maximum size, in octets, of an SDU from the peripheral,
             Valid Range 0x000 to 0xFFF*/
        cig_param.p_cis_config_list[index].max_sdu_p_to_c =
            p_wiced_bt_cfg_settings->p_isoc_cfg->max_sdu_size;
        /**< The transmitter PHY of packets from the peripheral */
        cig_param.p_cis_config_list[index].phy_p_to_c=WICED_BLE_ISOC_LE_2M_PHY;
        /**< Maximum number of times every CIS Data PDU should be retransmitted
             from the peripheral to central */
        cig_param.p_cis_config_list[index].rtn_p_to_c = 5/MAX_CIS_PER_CIG;
    }
    result = wiced_ble_isoc_central_set_cig_param(&cig_param);

    APP_ISOC_TRACE("[%s] exit %d", __FUNCTION__, result);

    return result;
}
#endif
/********************************************************************
 * Function Name: isoc_stop
 ********************************************************************
 * Summary:
 *  Re-enables HCI traces once the ISOC channel is closed
 ********************************************************************/
static void isoc_stop()
{
    wiced_bt_dev_update_hci_trace_mode(TRUE);
    APP_ISOC_TRACE("[%s] re-enable HCI trace", __FUNCTION__);
    led_blink_stop(LED_RED);
    isoc_cis_connected() ? led_on(LED_RED) : led_off(LED_RED);

    for(int i = 0; i< MAX_CIS_PER_CIG; i++)
    {
        wiced_stop_timer(&iso.isoc_keep_alive_timer[i]);
    }
#ifdef ISOC_STATS
    wiced_stop_timer(&iso_stats_timer);
#endif
}

#ifdef ISOC_STATS
/********************************************************************
 * Function Name: isoc_stats_init
 ********************************************************************
 * Summary:
 *
 ********************************************************************/
static void isoc_stats_init( int index )
{
    isoc_rx_count[index] = 0;
    isoc_tx_count[index] = 0;
    isoc_tx_fail_count[index] = 0;
    isoc_tx_dropped_count[index] = 0;
}

/********************************************************************
 * Function Name: isoc_stats_timeout
 ********************************************************************
 * Summary:
 *  Prints isoc stats upon timeout
 ********************************************************************/
static void isoc_stats_timeout( WICED_TIMER_PARAM_TYPE param )
{
    for(int i = 0; i < MAX_CIS_PER_CIG; i++)
    {
        APP_ISOC_TRACE("[ISOC STATS][%d] isoc_rx_count:%d isoc_tx_count:%d "
                       "isoc_tx_fail_count:%d isoc_tx_dropped_count:%d", i,
                       (int)isoc_rx_count[i], (int)isoc_tx_count[i],
                       (int)isoc_tx_fail_count[i],
                       (int)isoc_tx_dropped_count[i]);
    }
}
#endif

/********************************************************************
 * Function Name: isoc_close
 ********************************************************************
 * Summary:
 *  Called when CIS or data path is closed
 ********************************************************************/
static void isoc_close(uint16_t cis_handle)
{
    int index = find_index_by_cis_handle(cis_handle);

    if (INVALID_INDEX(index))
    {
        APP_ISOC_TRACE("[%s] Invalid cis_handle %d", __FUNCTION__, cis_handle);
        return;
    }

    if (index < MAX_CIS_PER_CIG)
    {
        if (iso.CIG.cis[index].status == CIS_OPEN)
        {
            iso.CIG.cis[index].status = CIS_REMOVE;

            if (!wiced_ble_isoc_remove_data_path(cis_handle,
                                                WICED_BLE_ISOC_DPD_OUTPUT_BIT, NULL))
            {
                WICED_BT_TRACE("[%s] failed sending remove output data path"
                               "command", __FUNCTION__);
                return;
            }

            if (!wiced_ble_isoc_remove_data_path(cis_handle,
                                                WICED_BLE_ISOC_DPD_INPUT_BIT, NULL))
            {
                WICED_BT_TRACE("[%s] failed sending remove input data path"
                               "command", __FUNCTION__);
                return;
            }

            APP_ISOC_TRACE("[%s] CIS Handle %d, removing data path",
                           __FUNCTION__, cis_handle);
        }
        else
        {
            iso.CIG.cis[index].status = CIS_CLOSED;
            APP_ISOC_TRACE("[%s] CIS Handle %d closed",__FUNCTION__,cis_handle);
            if (iso.CIG.cis[index].acl_handle)
            {
                app_set_acl_conn_interval(iso.CIG.cis[index].acl_handle,
                                          NON_ISOC_ACL_CONN_INTERVAL);
                iso.CIG.cis[index].acl_handle = 0;

                 xQueueReset(iso.CIG.cis[index].m_q);
                if(isoc_teardown_pending)
                    isoc_teardown_pending--;
            }
            isoc_stop();
        }
    }
}

/********************************************************************
 * Function Name: isoc_management_cback
 ********************************************************************
 * Summary:
 *  This is the callback function for ISOC Management.
 ********************************************************************/
static void isoc_management_cback(wiced_ble_isoc_event_t event,
                                    wiced_ble_isoc_event_data_t *p_event_data)
{
    uint8_t index = 0;
    wiced_result_t result;
    wiced_ble_isoc_setup_data_path_info_t data_path_info =
                {.isoc_conn_hdl = iso.CIG.cis[index].handle,
                .data_path_dir = WICED_BLE_ISOC_DPD_OUTPUT,
                .data_path_id = WICED_BLE_ISOC_DPID_HCI,
                .controller_delay = 0,
                .codec_id = {0,0,0,0,0},
                .csc_length = 0,
                .p_csc = NULL,
                .p_app_ctx = &dp_dir,
                };
    wiced_ble_isoc_data_path_direction_t *p_dir = (wiced_ble_isoc_data_path_direction_t *)p_event_data->datapath.p_app_ctx;
    APP_ISOC_TRACE("[%s] %d", __FUNCTION__, event);

    switch (event)
    {
        case WICED_BLE_ISOC_SET_CIG_CMD_COMPLETE_EVT:
        {
            APP_ISOC_TRACE("WICED_BLE_ISOC_SET_CIG_CMD_COMPLETE");
            if (WICED_BT_SUCCESS != p_event_data->cig_status_data.status)
            {
                WICED_BT_TRACE("[%s] Set CIG Status failure ", __FUNCTION__);
                return;
            }

            // get CIS Connection Handle List
            iso.CIG.id = p_event_data->cig_status_data.cig_id;
            iso.CIG.cis_count = p_event_data->cig_status_data.cis_count;
            APP_ISOC_TRACE("[%s] cig_id=%d, cis_count=%d",__FUNCTION__,
                           iso.CIG.id, iso.CIG.cis_count);

            for (index = 0; index < iso.CIG.cis_count; index++)
            {
                iso.CIG.cis[index].handle =
                p_event_data->cig_status_data.cis_connection_handle_list[index];
                iso.CIG.cis[index].id = index;
                iso.CIG.cis[index].status = CIS_CLOSED;

                APP_ISOC_TRACE("[%s] cig_id:%d, cis_id:%d, cis_handle:%d ",
                               __FUNCTION__, iso.CIG.id, iso.CIG.cis[index].id,
                               iso.CIG.cis[index].handle);
            }
        }
        break;

        case WICED_BLE_ISOC_CIS_ESTABLISHED_EVT:
        {
            APP_ISOC_TRACE("WICED_BLE_ISOC_CIS_ESTABLISHED");
            index = find_index_by_cis_handle(
                    p_event_data->cis_established_data.cis.cis_conn_handle);
            if (WICED_BT_SUCCESS != p_event_data->cis_established_data.status
                || (index >= MAX_CIS_PER_CIG))
            {
                WICED_BT_TRACE("[%s] CIS establishment Status failure %d"
                       "cis_handle:%d", __FUNCTION__,
                       p_event_data->cis_established_data.status,
                       p_event_data->cis_established_data.cis.cis_conn_handle);
                isoc_close(p_event_data->cis_established_data.cis.cis_conn_handle);
                return;
            }

            iso.CIG.cis[index].status = CIS_ESTABLISHED;
            APP_ISOC_TRACE("[%s] index %d, CIS %d handle %d established",
                           __FUNCTION__,index, iso.CIG.cis[index].id,
                           iso.CIG.cis[index].handle);

            dp_dir = WICED_BLE_ISOC_DPD_OUTPUT;
            data_path_info.data_path_dir = WICED_BLE_ISOC_DPD_OUTPUT;
            data_path_info.isoc_conn_hdl = p_event_data->cis_established_data.cis.cis_conn_handle;
#if defined(CYW55572) || WICED_BTSTACK_VERSION_MINOR > 8
            result = (wiced_result_t) wiced_ble_isoc_setup_data_path(&data_path_info);
#else
            result = (wiced_result_t) wiced_ble_isoc_setup_data_path(&data_path_info);
#endif
            APP_ISOC_TRACE("[%s] setup_data_path [OUT] result=%d, hdl %d", __FUNCTION__,
                           result, data_path_info.isoc_conn_hdl);
        }
        break;

        case WICED_BLE_ISOC_CIS_DISCONNECTED_EVT:
            APP_ISOC_TRACE("** CIS Disconnected %d, reason %d",
                           p_event_data->cis_disconnect.cis.cis_conn_handle,
                           p_event_data->cis_disconnect.reason);
            isoc_close(p_event_data->cis_disconnect.cis.cis_conn_handle);
            break;

        case WICED_BLE_ISOC_DATA_PATH_SETUP_EVT:
            APP_ISOC_TRACE("WICED_BLE_ISOC_DATA_PATH_SETUP");
            index = find_index_by_cis_handle(p_event_data->datapath.conn_hdl);
            if (WICED_BT_SUCCESS != p_event_data->datapath.status
                || (index >= MAX_CIS_PER_CIG))
            {
                WICED_BT_TRACE("[%s] Data path setup failure %d ",
                               __FUNCTION__, p_event_data->datapath.conn_hdl);
                isoc_close(p_event_data->datapath.conn_hdl);
                return;
            }            
            APP_ISOC_TRACE("[%s] %d data_path_dir = %d ", __FUNCTION__,
                           p_event_data->datapath.conn_hdl,
                            *p_dir);


            if(*p_dir==WICED_BLE_ISOC_DPD_OUTPUT)
            {
                dp_dir = WICED_BLE_ISOC_DPD_INPUT;
                data_path_info.data_path_dir = WICED_BLE_ISOC_DPD_INPUT;
                data_path_info.isoc_conn_hdl = p_event_data->datapath.conn_hdl;

#if defined(CYW55572) || WICED_BTSTACK_VERSION_MINOR > 8
                result = (wiced_result_t) wiced_ble_isoc_setup_data_path(&data_path_info);
#else
                result = (wiced_result_t) wiced_ble_isoc_setup_data_path(&data_path_info);
#endif
                APP_ISOC_TRACE("[%s] setup_data_path [IN] result=%d, hdl %d", __FUNCTION__,
                               result, data_path_info.isoc_conn_hdl);
            }
            else
            {
                iso.CIG.cis[index].status = CIS_OPEN;
                app_set_acl_conn_interval(iso.CIG.cis[index].acl_handle,
                                          ISOC_ACL_CONN_INTERVAL);
                app_update_cis_handle(iso.CIG.cis[index].acl_handle,
                                      iso.CIG.cis[index].handle);
                app_send_dummy(p_event_data->datapath.conn_hdl);
            }
            break;

        case WICED_BLE_ISOC_DATA_PATH_REMOVED_EVT:
            APP_ISOC_TRACE("WICED_BLE_ISOC_DATA_PATH_REMOVED");
            index = find_index_by_cis_handle(p_event_data->datapath.conn_hdl);
            if (WICED_BT_SUCCESS != p_event_data->datapath.status
                || (index >= MAX_CIS_PER_CIG))
            {
                WICED_BT_TRACE("[%s] Datapath remove failure ", __FUNCTION__);
                return;
            }

            APP_ISOC_TRACE("[%s] Datapath removed ", __FUNCTION__);
            result = wiced_ble_isoc_disconnect_cis(
                    p_event_data->datapath.conn_hdl);
            APP_ISOC_TRACE("[%s] disconnect cis on DP removed %d", __FUNCTION__,
                           result);
            app_update_cis_handle(iso.CIG.cis[index].acl_handle, 0);
            isoc_close(p_event_data->datapath.conn_hdl);
            break;

       default:
            APP_ISOC_TRACE("[%s]unknown event %d", __FUNCTION__, event);
            break;
    }

    CY_UNUSED_PARAMETER(result);
}

/********************************************************************
 * Function Name: isoc_send_data_num_complete_packets_evt
 ********************************************************************
 * Summary:
 *  Handle Number of Complete Packets event from controller
 *******************************************************************/
CY_SECTION_RAMFUNC_BEGIN
static void isoc_send_data_num_complete_packets_evt(uint16_t cis_handle,
                                                    uint16_t num_sent)
{
    int index = cis_handle - ISOC_START_HDL_VALUE;
    iso.CIG.cis[index].credits += num_sent;
    APP_ISOC_TRACE("[%d] cmplt, credist %d/%d, num %d\n", index, iso.CIG.cis[index].credits, CREDITS_PER_CIS, num_sent);

    if(!is_queue_empty(index))
    {
         uint8_t data = get_button_status(index, true);
         if(data == DUMMY)
         {
            APP_ISOC_TRACE("[%d] send dummy for IDLE\n", index);
            app_send_dummy(cis_handle);
         }else
         {
            isoc_send_data_payload(true, index, data);
         }
    }else {
         APP_ISOC_TRACE("[%d] start idle timer\n", index);
         wiced_start_timer(&iso.isoc_keep_alive_timer[index], ISOC_KEEP_ALIVE_TIMEOUT_IN_SECONDS);
    }
}
CY_SECTION_RAMFUNC_END

#ifdef ISOC_MONITOR_FOR_DROPPED_SDUs
/********************************************************************
 * Function Name: isoc_vse_cback
 ********************************************************************
 * Summary:
 *  VSE callback used to monitor for dropped sdu error events from
 *  the controller
 *******************************************************************/
static void isoc_vse_cback(uint8_t len, uint8_t *p)
{
    uint16_t opcode;
    isoc_error_dropped_sdu_t* p_isoc_error_dropped_sdu_vse;
    int index;

    STREAM_TO_UINT16(opcode, p);

    if(opcode == ISOC_ERROR_DROPPED_SDU_VSE_OPCODE)
    {
        p_isoc_error_dropped_sdu_vse = (isoc_error_dropped_sdu_t*) p;

        index = find_index_by_cis_handle(
               p_isoc_error_dropped_sdu_vse->connHandle);

        APP_ISOC_TRACE("[ISOC_ERROR_DROPPED_SDU %02x] PSN: %d  Expected_PSN: %d"
                "timestamp: %d  expected_ts: %d\n",
                p_isoc_error_dropped_sdu_vse->connHandle,
                (int)p_isoc_error_dropped_sdu_vse->psn,
                (int)p_isoc_error_dropped_sdu_vse->expected_psn,
                (int)p_isoc_error_dropped_sdu_vse->timestamp,
                (int)p_isoc_error_dropped_sdu_vse->expected_timestamp);

        if ((index >= MAX_CIS_PER_CIG) || INVALID_INDEX(index))
        {
            APP_ISOC_TRACE("[%s] Invalid cis_handle %d", __FUNCTION__,
                           p_isoc_error_dropped_sdu_vse->connHandle);
            return;
        }

        // Set sequence number to next expected PSN
        iso.CIG.cis[index].psn =
                p_isoc_error_dropped_sdu_vse->expected_psn + 1;
    }
}
#endif // ISOC_MONITOR_FOR_DROPPED_SDUs

/*******************************************************************************
 * public functions
 ******************************************************************************/
/********************************************************************
 * Function Name: isoc_start
 ********************************************************************
 * Summary:
 *  Called once the ISOC data patch has been established.
 *******************************************************************/
void isoc_start(int index)
{
    //APP_ISOC_TRACE("[%s] disable HCI trace", __FUNCTION__);
    //wiced_bt_dev_update_hci_trace_mode(FALSE);
    led_blink_stop(LED_RED);
    led_on(LED_RED);

#ifdef ISOC_STATS
    isoc_stats_init(index);
    wiced_start_timer(&iso_stats_timer, ISOC_STATS_TIMEOUT);
#endif
}

/********************************************************************
 * Function Name: isoc_open
 ********************************************************************
 * Summary:
 *  Creates an ISOC connection to the given ACL handle
 *******************************************************************/
wiced_result_t isoc_open(uint16_t acl_handle)
{
    wiced_result_t result = WICED_ALREADY_INITIALIZED;
    wiced_ble_isoc_cis_acl_t cis_param;
    uint8_t index;

    APP_ISOC_TRACE("[%s] ACL Handle %d, isoc cis_count=%d",
                   __FUNCTION__, acl_handle, iso.CIG.cis_count);

    for (index = 0; VALID_INDEX(index); index++)
    {
        if (iso.CIG.cis[index].acl_handle == acl_handle)
        {
            APP_ISOC_TRACE("[%s] ISOC already established for this ACL handle %d",
                           __FUNCTION__, acl_handle & 0x0ffff);
            return result;
        }
    }

    for (index = 0; VALID_INDEX(index); index++)
    {
        if (iso.CIG.cis[index].status == CIS_CLOSED)
        {
            iso.CIG.cis[index].acl_handle = acl_handle;
            iso.CIG.cis[index].status = CIS_CREATE;
            cis_param.acl_conn_handle = acl_handle;
            cis_param.cis_conn_handle = iso.CIG.cis[index].handle;
            led_blink(LED_RED, 0, 500);

            result = wiced_ble_isoc_central_create_cis(1,&cis_param);
            APP_ISOC_TRACE("[%s] index %d acl_handle %d cis_handle %d,"
                           "result=%d",__FUNCTION__, index, acl_handle,
                           iso.CIG.cis[index].handle, result);
            break;
        }
    }

    APP_ISOC_TRACE("[%s] exit result %d ", __FUNCTION__, result);
    return result;
}

/********************************************************************
 * Function Name: isoc_send_data
 ********************************************************************
 * Summary:
 *  Called when configured for burst tx mode when button is pressed.
 *  Number of packets sent defined by ISOC_MAX_BURST_COUNT.
 *******************************************************************/
CY_SECTION_RAMFUNC_BEGIN
void isoc_send_data(wiced_bool_t c)
{
    // start to burst out data
    for(int i = 0; i < MAX_CIS_PER_CIG; i++)
    {
        APP_ISOC_TRACE("start sending %d for button\n", i);
        iso_sdu_count[i] += ISOC_MAX_BURST_COUNT;
        append_button_status(i, c);

        // stop keep alive timer if it is running
        if (wiced_is_timer_in_use(&iso.isoc_keep_alive_timer[i]))
        {
            wiced_stop_timer(&iso.isoc_keep_alive_timer[i]);
            while(wiced_is_timer_in_use(&iso.isoc_keep_alive_timer[i]))
            {
                cy_rtos_delay_milliseconds(10);
            }
            app_send_dummy(iso.CIG.cis[i].handle);
        }
    }

}
CY_SECTION_RAMFUNC_END

static void isoc_idle_timeout( WICED_TIMER_PARAM_TYPE param )
{
    int i = (int)param;
    APP_ISOC_TRACE("[%d] idle tout\n", i);
    append_button_status(i, DUMMY);
    app_send_dummy(iso.CIG.cis[i].handle);;
}
/********************************************************************
 * Function Name: isoc_init
 ********************************************************************
 * Summary:
 *  Registers ISOC callbacks.
 *  Sets Phy preferences to ISOC.
 *******************************************************************/
void isoc_init(void)
{
    APP_ISOC_TRACE("[%s] ", __FUNCTION__);

    wiced_ble_isoc_cfg_t isoc_config = {
        .max_bis =0,
        .max_cis =MAX_CIS_PER_CIG,
    };

    // Register ISOC management callback
    wiced_ble_isoc_init(&isoc_config, &isoc_management_cback);

    // Init data handler module and register ISOC receive data handler
    iso_dhm_init(p_wiced_bt_cfg_settings->p_isoc_cfg,
                 isoc_send_data_num_complete_packets_evt, rx_handler);

    // Set to 2M phy
    wiced_bt_ble_phy_preferences_t phy_preferences = {0};
    phy_preferences.rx_phys = WICED_BLE_ISOC_LE_2M_PHY;
    phy_preferences.tx_phys = WICED_BLE_ISOC_LE_2M_PHY;
    wiced_result_t status = wiced_bt_ble_set_default_phy(&phy_preferences);
    APP_ISOC_TRACE("[%s] Set default 2M phy status %d", __FUNCTION__, status);
    CY_UNUSED_PARAMETER( status );

    /* Set CIG and update cig_present */
    if (WICED_SUCCESS != isoc_set_cig())
    {
        WICED_BT_TRACE("set CIG param failed!!");
    }

#ifdef ISOC_STATS
    // Init stats timer
    wiced_init_timer(&iso_stats_timer, isoc_stats_timeout, 0,
                     WICED_SECONDS_PERIODIC_TIMER);
#endif

#ifdef ISOC_MONITOR_FOR_DROPPED_SDUs
    wiced_bt_dev_register_vse_callback(isoc_vse_cback);
#endif
    for(int i = 0; i < MAX_CIS_PER_CIG; i++)
    {
       iso.CIG.cis[i].credits = CREDITS_PER_CIS;
       iso.CIG.cis[i].m_q = xQueueCreate( CIS_QUEUE_LEN, sizeof(uint8_t) );
       iso.CIG.cis[i].m_mtx = xSemaphoreCreateMutex();
           // Init keep alive timer
       wiced_init_timer(&iso.isoc_keep_alive_timer[i], isoc_idle_timeout,
                     (void*)i, WICED_SECONDS_PERIODIC_TIMER);
    }
}

/********************************************************************
 * Function Name: isoc_cis_connected
 ********************************************************************
 * Summary:
 *  Returns TRUE if any CIS connections open
 *******************************************************************/
wiced_bool_t isoc_cis_connected()
{
    for (int index=0; VALID_INDEX(index); index++)
    {
        if( iso.CIG.cis[index].status == CIS_OPEN )
            return TRUE;
    }

    return FALSE;
}

/********************************************************************
 * Function Name: isoc_cis_connected_count
 ********************************************************************
 * Summary:
 *  Returns number of CIS connections currently opened
 *******************************************************************/
uint8_t isoc_cis_connected_count()
{
    uint8_t count=0;

    for (int index=0; VALID_INDEX(index); index++)
    {
        if(iso.CIG.cis[index].status == CIS_OPEN)
        {
            count++;
        }
    }
    return count;
}

/********************************************************************
 * Function Name: isoc_close_all
 ********************************************************************
 * Summary:
 *  Closes all CIS connections
 *******************************************************************/
void isoc_close_all()
{
    int i;

    WICED_BT_TRACE("Closing all ISOC channels");

    for (i=0; i < MAX_CIS_PER_CIG; i++)
    {
        if (iso.CIG.cis[i].acl_handle)
        {
            isoc_teardown_pending++;
            isoc_close(iso.CIG.cis[i].handle);
        }
    }

    app_acl_disconnect_all();
}
