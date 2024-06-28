/*
 * $ Copyright YEAR Cypress Semiconductor $
 */

/** @file
*
* main.c
*
*/
#include "FreeRTOS.h"
#include "task.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#ifdef ENABLE_BT_SPY_LOG
#include "cybt_debug_uart.h"
#endif
#include "cybt_platform_config.h"
#include "cybt_platform_trace.h"
#include "cybsp_bt_config.h"
#include "cyhal.h"

/******************************************************************************/
/* extern                                                                     */
/******************************************************************************/
extern void application_start( void );

/*******************************************************************************
 * Function Name : main
 * *****************************************************************************
 * Summary :
*   Entry point to the application. Set device configuration and start BT
 *  stack initialization.  The actual application initialization will happen
 *  when stack reports that BT device is ready.
 *
 * Parameters:
 *    None
 *
 * Return:
 *    None
 ******************************************************************************/
int main()
{
    /* Initialise the BSP and Verify the BSP initialization */
    CY_ASSERT(CY_RSLT_SUCCESS == cybsp_init());

    /* Enable global interrupts */
    __enable_irq();

#if defined(ENABLE_BT_SPY_LOG)
    cybt_debug_uart_config_t config = {
        .uart_tx_pin = CYBSP_DEBUG_UART_TX,
        .uart_rx_pin = CYBSP_DEBUG_UART_RX,
        .uart_cts_pin = CYBSP_DEBUG_UART_CTS,
        .uart_rts_pin = CYBSP_DEBUG_UART_RTS,
        .baud_rate = DEBUG_UART_BAUDRATE,
        .flow_control = TRUE};
    cybt_debug_uart_init(&config, NULL);
#else
    /* Initialize retarget-io to use the debug UART port */
    cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX,
                        CY_RETARGET_IO_BAUDRATE);
#endif //ENABLE_BT_SPY_LOG

    cybt_platform_set_trace_level(CYBT_TRACE_ID_STACK, CYBT_TRACE_ID_MAX);

    cybt_platform_config_init(&cybsp_bt_platform_cfg);

    application_start();

    /* Start the FreeRTOS scheduler */
    vTaskStartScheduler();

    /* Should never get here */
    CY_ASSERT(0);
}

/* end of file */
