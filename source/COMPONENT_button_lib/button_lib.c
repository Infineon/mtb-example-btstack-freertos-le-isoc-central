/*
 *  Copyright 2025, Cypress Semiconductor Corporation (an Infineon company) or
 *  an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 * 
 *  This software, including source code, documentation and related
 *  materials ("Software") is owned by Cypress Semiconductor Corporation
 *  or one of its affiliates ("Cypress") and is protected by and subject to
 *  worldwide patent protection (United States and foreign),
 *  United States copyright laws and international treaty provisions.
 *  Therefore, you may use this Software only as provided in the license
 *  agreement accompanying the software package from which you
 *  obtained this Software ("EULA").
 *  If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 *  non-transferable license to copy, modify, and compile the Software
 *  source code solely for use in connection with Cypress's
 *  integrated circuit products.  Any reproduction, modification, translation,
 *  compilation, or representation of this Software except as specified
 *  above is prohibited without the express written permission of Cypress.
 * 
 *  Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 *  EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 *  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 *  reserves the right to make changes to the Software without notice. Cypress
 *  does not assume any liability arising out of the application or use of the
 *  Software or any product or circuit described in the Software. Cypress does
 *  not authorize its products for use in any products where a malfunction or
 *  failure of the Cypress product may reasonably be expected to result in
 *  significant property damage, injury or death ("High Risk Product"). By
 *  including Cypress's product in a High Risk Product, the manufacturer
 *  of such system or application assumes all risk of such use and in doing
 *  so agrees to indemnify Cypress against all liability.
 */

/**
 * file button_lib.c
 *
 * This file consists of the functions that are necessary for
 * push button use cases
 */

#include "FreeRTOS.h"
#include "task.h"
#include "cycfg_pins.h"
#include "cyabs_rtos.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_types.h"
#include "button_lib.h"
#include "app_terminal_trace.h"
/*******************************************************************************
 *                              CONSTANTS
 ******************************************************************************/

#define MAX_READ_RETRY 10
#define MAX_SUPPORTED_BUTTON 32
#define BUTTON_INTERRUPT_PRIORITY 7

#define BUTTON_TASK_PRIORITY 2
#define BUTTON_TASK_STACK_SIZE    (256u)

/*******************************************************************************
 *   Global Variables
 ******************************************************************************/
static TaskHandle_t button_task_handle;

static struct {
    const button_cfg_t *   cfg;
    uint8_t                count;
    button_state_change_t  cb;
} button_cfg = {0};

static cyhal_gpio_callback_data_t isr_data[MAX_SUPPORTED_BUTTON];

/*******************************************************************************
 *   Private functions
 ******************************************************************************/

/**
 * Task that takes care of button activities
 */
static void button_task(void *args)
{
    static uint32_t old_button_state=0;
    uint32_t ulNotifiedValue, btn_state;

    while(1)
    {
        xTaskNotifyWait(0, 0xffffffff, &ulNotifiedValue, portMAX_DELAY);
        btn_state = button_state();
        if (button_cfg.cb && (old_button_state != btn_state))
        {
            old_button_state = btn_state;
            button_cfg.cb(ulNotifiedValue);
        }
    }
}

/**
 *  button state change interrupt handler
 */
static void button_interrupt_handler(void *handler_arg, cyhal_gpio_event_t event)
{
    BaseType_t xHigherPriorityTaskWoken;
    uint32_t arg = 1 << (uint32_t) handler_arg;

    xTaskNotifyFromISR(button_task_handle, arg, eSetBits, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/*******************************************************************************
 *   Public functions
 ********************************************************************************/

/**
 * Initialize button functions. This is the first function to be called before other
 * button functions can be used.
 */
wiced_bool_t button_lib_init(uint8_t count, const button_cfg_t * cfg, button_state_change_t cb)
{
    if (count && cfg && (count <= MAX_SUPPORTED_BUTTON))
    {
        button_cfg.count = count;
        button_cfg.cfg = cfg;
        button_cfg.cb = cb;

        for (int index = 0; index < count; index++, cfg++)
        {
            cy_rslt_t status;
            cyhal_gpio_drive_mode_t drive_mode = cfg->released_value ? CYHAL_GPIO_DRIVE_OPENDRAINDRIVESHIGH : CYHAL_GPIO_DRIVE_OPENDRAINDRIVESLOW;
            /* Initialize the user button */
            status = cyhal_gpio_init(cfg->pin, CYHAL_GPIO_DIR_INPUT, drive_mode, cfg->released_value);

            if((CY_RSLT_SUCCESS == status) && (index == 0))
            {
                isr_data[index].callback = button_interrupt_handler;
                isr_data[index].callback_arg = (void *) index;

                /* Configure GPIO interrupt for User Button */
                cyhal_gpio_register_callback(cfg->pin, &isr_data[index]);

                /* Enable GPIO Interrupt on both edge for User Button */
                cyhal_gpio_enable_event(cfg->pin, CYHAL_GPIO_IRQ_BOTH, BUTTON_INTERRUPT_PRIORITY, TRUE);
            }
            else
            {
                WICED_BT_TRACE("button GPIO P%d_%d init failed",cfg->pin >> 3, cfg->pin & 0x7);
            }
        }

        /* Initialize button task */
        xTaskCreate(button_task,
                                  "Button Task",
                                  BUTTON_TASK_STACK_SIZE,
                                  NULL,
                                  BUTTON_TASK_PRIORITY,
                                  &button_task_handle);
        return TRUE;
    }
    return FALSE;
}

/**
 * return true when button is pressed.
 */
wiced_bool_t button_is_pressed(uint8_t index)
{
    bool  value;

    // we read twice with same value to be considered valid (for debouncing), max read MAX_READ_RETRY times
    for (int retry=0; retry<MAX_READ_RETRY; retry++)
    {
        value = cyhal_gpio_read(button_cfg.cfg[index].pin);
        cy_rtos_delay_milliseconds(1);

        // if we read twice with same value, return
        if (cyhal_gpio_read(button_cfg.cfg[index].pin) == value)
        {
            break;
        }
    }
    return  value != button_cfg.cfg[index].released_value;
}

/**
 * Returns all button states. Each bit represents each button's state, etc bit 0 for index 0, bit 1 for index 1...
 * The button is pressed when the bit value is 1.
 */
uint32_t button_state()
{
    uint32_t value;

    value = 0;
    for (uint32 i=0, mask=1; i<button_cfg.count; i++, mask<<=1)
    {
        if (button_is_pressed(i))
        {
            value |= mask;
        }
    }
    return value;
}

/* end of file */
