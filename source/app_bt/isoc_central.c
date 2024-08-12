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
#define ISOC_MAX_BURST_COUNT                4      // Must be >= 1

#define ISO_SDU_INTERVAL                    10000 //sdu interval in micro-second
#define ISOC_TIMEOUT_IN_MSECONDS            ISO_SDU_INTERVAL / 1000
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


typedef enum
{
    SN_IDLE,
    SN_PENDING,
    SN_VALID,
} sequence_number_state_e;

typedef struct
{
    sequence_number_state_e state;
    uint16_t sequence_number;
} isoc_psn_info_t;

typedef struct
{
    uint8_t  status;
    uint8_t  id;            // CIS ID
    uint16_t handle;        // CIS Handle
    uint16_t acl_handle;    // ACL Connection Handle for this CIS
    isoc_psn_info_t psn_info;
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
    wiced_timer_t isoc_send_data_timer;
    wiced_timer_t isoc_keep_alive_timer;
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

#pragma pack(1)
typedef struct
{
    uint16_t    cis_conn_handle;
    uint16_t    iso_payload_len;
    uint8_t     iso_payload[ISO_SDU_SIZE];
} iso_tx_packet_t;
#pragma pack()

iso_tx_packet_t isoc_tx_buffer[MAX_CIS_PER_CIG];

static uint16_t iso_sdu_count = 0;
static wiced_bool_t pressed_saved;
uint8_t isoc_teardown_pending = 0;

#define CONTROLLER_ISO_DATA_PACKET_BUFS   8
static uint8_t number_of_iso_data_packet_bufs = CONTROLLER_ISO_DATA_PACKET_BUFS;

// ISOC stats counters
static uint32_t isoc_rx_count[MAX_CIS_PER_CIG] = {0};
static uint32_t isoc_tx_count[MAX_CIS_PER_CIG] = {0};
static uint32_t isoc_tx_fail_count[MAX_CIS_PER_CIG] = {0};
static uint32_t isoc_tx_dropped_count[MAX_CIS_PER_CIG] = {0};
static wiced_timer_t iso_stats_timer;


/*******************************************************************************
 * Macros
 ******************************************************************************/
#define VALID_INDEX(i)  (i < iso.CIG.cis_count)
#define INVALID_INDEX(i) (i >= iso.CIG.cis_count)
#define CIG cig[0]          // we only support one cig for this app
#define isoc_acl_handle(i) iso.CIG.cis[i].acl_handle
#define cis_ status iso.CIG.cis[index].status


/*******************************************************************************
 * private functions
 ******************************************************************************/
static void isoc_send_null_payload(void);
static void isoc_get_psn_start( WICED_TIMER_PARAM_TYPE param );
static void isoc_set_psn_idle(void);
static wiced_bool_t isoc_is_psn_valid(void);

static int find_index_by_cis_handle(uint16_t cis_handle);

#define  VSC_0XFDFA
#ifdef VSC_0XFDFA
#pragma pack( push, 1 )
typedef struct {
    uint8_t status;
    uint16_t connHandle;
    uint16_t packetSeqNum;
    uint32_t timeStamp;
    uint8_t  timeOffset[3];
}tREAD_PSN_EVT;
#pragma pack( pop )

static void read_psn_cb(wiced_bt_dev_vendor_specific_command_complete_params_t
                        *p_command_complete_params)
{
    tREAD_PSN_EVT * evt =
            (tREAD_PSN_EVT *)p_command_complete_params->p_param_buf;
    uint8_t index = find_index_by_cis_handle(evt->connHandle);
    int toffset = evt->timeOffset[0];

    toffset |= (evt->timeOffset[1] & 0x0ff)<<8;
    toffset |= (evt->timeOffset[2] & 0x0ff)<< 16;

    APP_ISOC_TRACE("[%s] status:%d handle:0x%x psn=%d timestamp:%d"
                   "time_offset:%d\n",
                   __FUNCTION__,evt->status & 0x0FF,
                   evt->connHandle& 0x0FFFF, evt->packetSeqNum& 0x0FFFF,
                   (int)evt->timeStamp, toffset);

    if ((index >= MAX_CIS_PER_CIG) || INVALID_INDEX(index))
    {
        APP_ISOC_TRACE("[%s] Invalid cis_handle %d", __FUNCTION__,
                       evt->connHandle);
        return;
    }

    if(evt->status != 0)
    {
        APP_ISOC_TRACE("[%s] status %d", __FUNCTION__, evt->status);
        iso.CIG.cis[index].psn_info.state = SN_IDLE;
        return;
    }

    if( iso.CIG.cis[index].psn_info.state != SN_PENDING )
        return;

    // If initial transmission, no need to increment
    if( evt->packetSeqNum == 0 )
        iso.CIG.cis[index].psn_info.sequence_number = evt->packetSeqNum;
    else
        iso.CIG.cis[index].psn_info.sequence_number = evt->packetSeqNum + 2;

    iso.CIG.cis[index].psn_info.state = SN_VALID;

    // Send NULL payload if the idle timer is running
    if( wiced_is_timer_in_use(&iso.isoc_keep_alive_timer) )
    {
        isoc_send_null_payload();
    }
}

#define READ_PSN_VSC_OPCODE   (0xFDFA)
void start_read_psn_using_vsc(uint16_t hdl)
{
    wiced_bt_dev_vendor_specific_command (READ_PSN_VSC_OPCODE, 2,
                                          (uint8_t *)&hdl,read_psn_cb);
}

#else
/***************************************************************************
 * Function Name: isoc_read_tx_sync_complete_cback
 ***************************************************************************
 * Summary:
 *
 ***************************************************************************/
CY_SECTION_RAMFUNC_BEGIN
static void isoc_read_tx_sync_complete_cback(
                    wiced_bt_isoc_read_tx_sync_complete_t
                    *p_event_data)
{
    uint8_t index = find_index_by_cis_handle(p_event_data->conn_hdl);

    APP_ISOC_TRACE("[%s] status:%d handle:0x%x psn=%d timestamp:%d"
                   "time_offset:%d",__FUNCTION__,p_event_data->status,
                   p_event_data->conn_hdl,p_event_data->psn,
                   p_event_data->tx_timestamp,p_event_data->time_offset);

    if(p_event_data->status != 0)
    {
        APP_ISOC_TRACE("[%s] status %d", __FUNCTION__, p_event_data->status);
        iso.CIG.cis[0].psn_info.state = SN_IDLE;
        return;
    }

    if ((index >= MAX_CIS_PER_CIG) || INVALID_INDEX(index))
    {
        APP_ISOC_TRACE("[%s] Invalid cis_handle %d", __FUNCTION__,
                       p_event_data->conn_hdl);
        return;
    }

    if( iso.CIG.cis[index].psn_info.state != SN_PENDING )
        return;

    // If initial transmission, no need to increment
    if( p_event_data->psn == 0 )
        iso.CIG.cis[index].psn_info.sequence_number = p_event_data->psn;
    else
        iso.CIG.cis[index].psn_info.sequence_number = p_event_data->psn + 2;

    iso.CIG.cis[index].psn_info.state = SN_VALID;

    // Send NULL payload if the idle timer is running
    if( wiced_is_timer_in_use(&iso.isoc_keep_alive_timer) )
    {
        isoc_send_null_payload();
    }
}
CY_SECTION_RAMFUNC_END
#endif
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

/********************************************************************
 * Function Name: isoc_send_null_payload
 ********************************************************************
 * Summary:
 *
 ********************************************************************/
CY_SECTION_RAMFUNC_BEGIN
static void isoc_send_null_payload(void)
{
    wiced_bool_t result;
    uint8_t* p_buf = NULL;

    for(uint8_t i=0; i<MAX_CIS_PER_CIG; i++)
    {
        if( iso.CIG.cis[i].psn_info.state == SN_VALID )
        {
            // Allocate buffer for ISOC header
            if((p_buf = iso_dhm_get_data_buffer()) != NULL)
            {
                result = iso_dhm_send_packet(
                        iso.CIG.cis[i].psn_info.sequence_number,
                        iso.CIG.cis[i].handle, WICED_FALSE, p_buf, 0);

                APP_ISOC_TRACE("[%s] sent null payload handle %02x result %d",
                               __FUNCTION__, iso.CIG.cis[i].handle, result);

                // Set PSN state back to idle
                iso.CIG.cis[i].psn_info.state = SN_IDLE;
            }
        }
    }
    wiced_start_timer(&iso.isoc_keep_alive_timer,
                          ISOC_KEEP_ALIVE_TIMEOUT_IN_SECONDS);
}
CY_SECTION_RAMFUNC_END

void app_send_dummy(uint16_t handle)
{
    uint8_t* p_buf = iso_dhm_get_data_buffer();
    int i = handle - 0x60;
    iso_dhm_send_packet(iso.CIG.cis[i].psn_info.sequence_number,
                        iso.CIG.cis[i].handle, WICED_FALSE, p_buf, 0);
}

/********************************************************************
 * Function Name: isoc_send_data_payload
 ********************************************************************
 * Summary:
 *  Helper function for isoc_send_data_handler
 ********************************************************************/
CY_SECTION_RAMFUNC_BEGIN
static void isoc_send_data_payload( uint8_t index )
{
    uint8_t result;
    uint32_t data_length;
    uint8_t* p_buf = NULL;
    uint8_t* p_data = NULL;
    uint8_t pressed = pressed_saved;

    if((p_buf = iso_dhm_get_data_buffer()) != NULL)
    {
        p_data = p_buf;

        UINT16_TO_STREAM(p_data, iso.CIG.cis[index].handle);
        UINT16_TO_STREAM(p_data, iso.CIG.cis[index].psn_info.sequence_number);
        UINT8_TO_STREAM(p_data, pressed);

#if 0  // Normally you would only send the required payload but here we want to
       // exercise the max_sdu_size to stress the system more
        data_length = p_data - p_buf;
#else
        data_length = p_wiced_bt_cfg_settings->p_isoc_cfg->max_sdu_size;
#endif

        result=iso_dhm_send_packet(iso.CIG.cis[index].psn_info.sequence_number,
                                   iso.CIG.cis[index].handle, WICED_FALSE,
                                   p_buf, data_length);

        if(result)
        {
            number_of_iso_data_packet_bufs--;
            isoc_tx_count[index]++;
        }
        else
        {
            isoc_tx_fail_count[index]++;
        }

        APP_ISOC_TRACE("[isoc_send_data_handler] handle:0x%x SN:%d data_length:"
                       "%d sdu_count:%d result:%d", iso.CIG.cis[index].handle,
                       iso.CIG.cis[index].psn_info.sequence_number, (int)data_length,
                       (int)iso_sdu_count, result);
    }
}
CY_SECTION_RAMFUNC_END

/********************************************************************
 * Function Name: isoc_send_data_handler
 ********************************************************************
 * Summary:
 *  Updates the send buffer and submits data to the controller
 ********************************************************************/
CY_SECTION_RAMFUNC_BEGIN
static void isoc_send_data_handler( WICED_TIMER_PARAM_TYPE param )
{
    if( !isoc_is_psn_valid() )
        return;

    for(uint8_t i=0; i<MAX_CIS_PER_CIG; i++)
    {
        if(number_of_iso_data_packet_bufs)
        {
            isoc_send_data_payload(i);
        }
        else
        {
            // No buffer to send data
            isoc_tx_dropped_count[i]++;
        }

        // Increment the sequence number
        iso.CIG.cis[i].psn_info.sequence_number++;
    }

    iso_sdu_count--;

    if(iso_sdu_count == 0)
    {
        // Stop the send data timer if no more data to send
        wiced_stop_timer(&iso.isoc_send_data_timer);

        isoc_set_psn_idle();

        // Start keep alive timer
        wiced_start_timer(&iso.isoc_keep_alive_timer,
                          ISOC_KEEP_ALIVE_TIMEOUT_IN_SECONDS);

        APP_ISOC_TRACE("Started keep alive timer");
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

            APP_ISOC_TRACE("[rx_data] cis_id:0x%x SN:%d button_state:%d",
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
        wiced_bt_ble_cig_param_test_t  cig_param_test;
        uint8_t                        index;

        printf("[%s] cis_per_cig count %d \r\n", __FUNCTION__, MAX_CIS_PER_CIG);

        cig_param_test.cig_id    = 0;
        cig_param_test.cis_count = MAX_CIS_PER_CIG;

        cig_param_test.sdu_interval_c_to_p       = ISO_SDU_INTERVAL;
        cig_param_test.sdu_interval_p_to_c       = ISO_SDU_INTERVAL;
        cig_param_test.peripheral_clock_accuracy = 0;

        cig_param_test.packing = WICED_BLE_ISOC_SEQUENTIAL_PACKING;
        cig_param_test.framing = WICED_BLE_ISOC_UNFRAMED;

        cig_param_test.ft_c_to_p    = 2;
        cig_param_test.ft_p_to_c    = 2;
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
        result = wiced_bt_isoc_central_set_cig_param_test(&cig_param_test, NULL);

        printf("[%s] exit %d", __FUNCTION__, result);

        return result;
}
#else
static wiced_result_t isoc_set_cig()
{
    wiced_result_t result;

    APP_ISOC_TRACE("[%s] cis_per_cig count %d ", __FUNCTION__, MAX_CIS_PER_CIG);

    wiced_bt_ble_cis_config_t cis_config_list[MAX_CIS_PER_CIG];
    wiced_bt_ble_cig_param_t cig_param;

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
    cig_param.peripheral_clock_accuracy = 0;
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
    result = wiced_bt_isoc_central_set_cig_param(&cig_param, NULL);

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

    wiced_stop_timer(&iso.isoc_send_data_timer);

       wiced_stop_timer(&iso.isoc_keep_alive_timer);

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

            if (!wiced_bt_isoc_remove_data_path(cis_handle, TRUE,
                                                WICED_BLE_ISOC_DPD_OUTPUT_BIT))
            {
                WICED_BT_TRACE("[%s] failed sending remove output data path"
                               "command", __FUNCTION__);
                return;
            }

            if (!wiced_bt_isoc_remove_data_path(cis_handle, TRUE,
                                                WICED_BLE_ISOC_DPD_INPUT_BIT))
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
static void isoc_management_cback(wiced_bt_isoc_event_t event,
                                  wiced_bt_isoc_event_data_t *p_event_data)
{
    uint8_t index;
    wiced_result_t result;

    APP_ISOC_TRACE("[%s] %d", __FUNCTION__, event);

    switch (event)
    {
        case WICED_BLE_ISOC_SET_CIG_CMD_COMPLETE:
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
                wiced_bt_isoc_update_cis_conn_handle(iso.CIG.id,
                                                     iso.CIG.cis[index].id,
                                                     iso.CIG.cis[index].handle);

                APP_ISOC_TRACE("[%s] cig_id:%d, cis_id:%d, cis_handle:%d ",
                               __FUNCTION__, iso.CIG.id, iso.CIG.cis[index].id,
                               iso.CIG.cis[index].handle);
            }
        }
        break;

        case WICED_BLE_ISOC_CIS_ESTABLISHED:
        {
            APP_ISOC_TRACE("WICED_BLE_ISOC_CIS_ESTABLISHED");
            index = find_index_by_cis_handle(
                    p_event_data->cis_established_data.cis_conn_handle);
            if (WICED_BT_SUCCESS != p_event_data->cis_established_data.status
                || (index >= MAX_CIS_PER_CIG))
            {
                WICED_BT_TRACE("[%s] CIS establishment Status failure %d"
                       "cis_handle:%d", __FUNCTION__,
                       p_event_data->cis_established_data.status,
                       p_event_data->cis_established_data.cis_conn_handle);
                isoc_close(p_event_data->cis_established_data.cis_conn_handle);
                return;
            }

            iso.CIG.cis[index].status = CIS_ESTABLISHED;
            APP_ISOC_TRACE("[%s] index %d, CIS %d handle %d established",
                           __FUNCTION__,index, iso.CIG.cis[index].id,
                           iso.CIG.cis[index].handle);
#if defined(CYW55572) || WICED_BTSTACK_VERSION_MINOR > 8
            result = (wiced_result_t) wiced_bt_isoc_setup_data_path(
                                                      iso.CIG.cis[index].handle,
                                                      TRUE,
                                                      WICED_BLE_ISOC_DPD_OUTPUT,
                                                      WICED_BLE_ISOC_DPID_HCI,
                                                      0,0,0);
#else
            result = (wiced_result_t) wiced_bt_isoc_setup_data_path(
                                          iso.CIG.cis[index].handle,
                                          TRUE,
                                          WICED_BLE_ISOC_DPD_OUTPUT,
                                          WICED_BLE_ISOC_DPID_HCI,
                                          0);
#endif
            APP_ISOC_TRACE("[%s] setup_data_path result=%d", __FUNCTION__,
                           result);
        }
        break;

        case WICED_BLE_ISOC_CIS_DISCONNECTED:
            APP_ISOC_TRACE("** CIS Disconnected %d, reason %d",
                           p_event_data->cis_disconnect.cis_conn_handle,
                           p_event_data->cis_disconnect.reason);
            isoc_close(p_event_data->cis_disconnect.cis_conn_handle);
            break;

        case WICED_BLE_ISOC_DATA_PATH_SETUP:
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
                           p_event_data->datapath.data_path_dir);


            if(p_event_data->datapath.data_path_dir==WICED_BLE_ISOC_DPD_OUTPUT)
            {
                result = (wiced_result_t) wiced_bt_isoc_setup_data_path(
                                          iso.CIG.cis[index].handle,
                                          TRUE,
                                          WICED_BLE_ISOC_DPD_INPUT,
                                          WICED_BLE_ISOC_DPID_HCI,
#if defined(CYW55572) || WICED_BTSTACK_VERSION_MINOR > 8
                                          0,0,0);
#else
                                          0);
#endif
                APP_ISOC_TRACE("[%s] setup_data_path result=%d", __FUNCTION__,
                               result);
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

        case WICED_BLE_ISOC_DATA_PATH_REMOVED:
            APP_ISOC_TRACE("WICED_BLE_ISOC_DATA_PATH_REMOVED");
            index = find_index_by_cis_handle(p_event_data->datapath.conn_hdl);
            if (WICED_BT_SUCCESS != p_event_data->datapath.status
                || (index >= MAX_CIS_PER_CIG))
            {
                WICED_BT_TRACE("[%s] Datapath remove failure ", __FUNCTION__);
                return;
            }

            APP_ISOC_TRACE("[%s] Datapath removed ", __FUNCTION__);
            result = wiced_bt_isoc_disconnect_cis(
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
 * Function Name: isoc_is_psn_valid
 ********************************************************************
 * Summary:
 *
 *******************************************************************/
CY_SECTION_RAMFUNC_BEGIN
static wiced_bool_t isoc_is_psn_valid(void)
{
    for(uint8_t i=0; i<MAX_CIS_PER_CIG; i++)
    {
        if( iso.CIG.cis[i].psn_info.state != SN_VALID )
        {
            return WICED_FALSE;
        }
    }
    return WICED_TRUE;
}
CY_SECTION_RAMFUNC_END

/********************************************************************
 * Function Name: isoc_set_psn_idle
 ********************************************************************
 * Summary:
 *
 *******************************************************************/
CY_SECTION_RAMFUNC_BEGIN
static void isoc_set_psn_idle(void)
{
    for(uint8_t i=0; i<MAX_CIS_PER_CIG; i++)
    {
        iso.CIG.cis[i].psn_info.state = SN_IDLE;
    }
}
CY_SECTION_RAMFUNC_END

/********************************************************************
 * Function Name: isoc_get_psn_start
 ********************************************************************
 * Summary:
 * Returns the PSN start value for the current transmission packet.
 *******************************************************************/
CY_SECTION_RAMFUNC_BEGIN
static void isoc_get_psn_start(WICED_TIMER_PARAM_TYPE param)
{
    for(uint8_t i=0; i<MAX_CIS_PER_CIG; i++)
    {
        if( iso.CIG.cis[i].psn_info.state == SN_IDLE && iso.CIG.cis[i].handle)
        {
            APP_ISOC_TRACE("[%s] sending HCI_BLE_ISOC_READ_TX_SYNC for handle"
                           "%02x", __FUNCTION__, iso.CIG.cis[i].handle);
#ifndef VSC_0XFDFA
            wiced_bt_isoc_read_tx_sync(iso.CIG.cis[i].handle, WICED_TRUE,
                                       isoc_read_tx_sync_complete_cback);
#else
            start_read_psn_using_vsc(iso.CIG.cis[i].handle);
#endif
            iso.CIG.cis[i].psn_info.state = SN_PENDING;
        }
    }
}
CY_SECTION_RAMFUNC_END


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
    number_of_iso_data_packet_bufs += num_sent;
    wiced_start_timer(&iso.isoc_keep_alive_timer,
                      ISOC_KEEP_ALIVE_TIMEOUT_IN_SECONDS);
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
                "timestamp: %d  expected_ts: %d",
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
        iso.CIG.cis[index].psn_info.sequence_number =
                p_isoc_error_dropped_sdu_vse->expected_psn + 1;

        if (wiced_is_timer_in_use(&iso.isoc_keep_alive_timer))
        {
            // idle packet is failed. so send it again
            app_send_dummy(p_isoc_error_dropped_sdu_vse->connHandle);
        }else
        {
            isoc_tx_count[index]--;
        }
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
    wiced_bt_isoc_create_cis_param_t cis_param;
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

    cis_param.cis_count = 1;
    for (index = 0; VALID_INDEX(index); index++)
    {
        if (iso.CIG.cis[index].status == CIS_CLOSED)
        {
            iso.CIG.cis[index].acl_handle = acl_handle;
            iso.CIG.cis[index].status = CIS_CREATE;
            cis_param.acl_handle_list = &acl_handle;
            cis_param.cis_handle_list = &iso.CIG.cis[index].handle;
            led_blink(LED_RED, 0, 500);

            result = wiced_bt_isoc_central_create_cis(&cis_param);
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
    // save button state
    pressed_saved = c;

    // start to burst out data
    iso_sdu_count += ISOC_MAX_BURST_COUNT;

    // stop keep alive timer if it is running
    if (wiced_is_timer_in_use(&iso.isoc_keep_alive_timer))
    {
        wiced_stop_timer(&iso.isoc_keep_alive_timer);
    }

    // get the PSN info from the controller
    isoc_get_psn_start(0);

    // start to burst out data
    if (!wiced_is_timer_in_use(&iso.isoc_send_data_timer))
    {
        //APP_ISOC_TRACE("[%s] start %dms timer", __FUNCTION__);
        wiced_start_timer(&iso.isoc_send_data_timer, ISOC_TIMEOUT_IN_MSECONDS);
    }
}
CY_SECTION_RAMFUNC_END

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

    // Init data handler module and register ISOC receive data handler
    iso_dhm_init(p_wiced_bt_cfg_settings->p_isoc_cfg,
                 isoc_send_data_num_complete_packets_evt, rx_handler);

    // Register ISOC management callback
    wiced_bt_isoc_register_cb(&isoc_management_cback);

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

    isoc_set_psn_idle();

    // Init send data timer
    wiced_init_timer(&iso.isoc_send_data_timer, isoc_send_data_handler,
                     0, WICED_MILLI_SECONDS_PERIODIC_TIMER);

    // Init keep alive timer
    wiced_init_timer(&iso.isoc_keep_alive_timer, isoc_get_psn_start,
                     0, WICED_SECONDS_PERIODIC_TIMER);

#ifdef ISOC_STATS
    // Init stats timer
    wiced_init_timer(&iso_stats_timer, isoc_stats_timeout, 0,
                     WICED_SECONDS_PERIODIC_TIMER);
#endif

#ifdef ISOC_MONITOR_FOR_DROPPED_SDUs
    wiced_bt_dev_register_vse_callback(isoc_vse_cback);
#endif
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
