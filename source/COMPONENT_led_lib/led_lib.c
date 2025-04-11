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

/**
 * file led_lib.c
 *
 * This file consists of the LED functions
 */

#include "wiced_bt_types.h"
#include "wiced_bt_trace.h"
#include "wiced_timer.h"
#include "led_lib.h"
#include "cyhal_system.h"
#include "app_terminal_trace.h"
/*******************************************************************************
* Defines
*******************************************************************************/
#if LED_TRACE
# define LED_LIB_TRACE    WICED_BT_TRACE
#else
# define LED_LIB_TRACE(...)
#endif

#define BLINK_CODE_SPEED            500
#define ERROR_CODE_BLINK_BREAK      4000

#define LED_LIB_STARTUP_INDEX       0
#define LED_LIB_MAX                 8

#define LED_LIB_START_UP_BLINK_COUNT 5
#define LED_LIB_START_UP_BLINK_SPEED 100

#define led_initialized()  (led_cfg.count)
#define VALID_LED_IDX(idx) (led_initialized() && idx < led_cfg.count)

/*******************************************************************************
* Data
*******************************************************************************/
static struct {
    wiced_timer_t blinking_timer;
    uint16_t      blinking_duration_on;
    uint16_t      blinking_duration_off;
    uint8_t       blinking_count;
    uint8_t       blinking_repeat_code;

    uint8_t       curr_state:1;
    uint8_t       state:1;
    uint8_t       blinking:1;

} led[LED_LIB_MAX] = {0};

static struct {
    const led_config_t * platform;
    uint8_t       count;
} led_cfg = {0};

/*******************************************************************************
* Private Functions
*******************************************************************************/

/**
 * Set the LED state givin by index
 */
static void led_set_state(uint32_t idx, wiced_bool_t newState)
{
    if (VALID_LED_IDX(idx))
    {
        uint8_t logic = led_cfg.platform[idx].default_state;  // logic = LED off
        if (newState == LED_ON)
        {
           logic = !logic;   // logic = LED on
        }
        LED_LIB_TRACE("LED%d port=%d.%d, set value %d", idx, CYHAL_GET_PORT(led_cfg.platform[idx].gpio), CYHAL_GET_PIN(led_cfg.platform[idx].gpio), logic);
        cyhal_gpio_write(led_cfg.platform[idx].gpio, logic);
        led[idx].curr_state = newState;
    }
}

/**
 * The LED blinking handler. When timer expires, this function is called.
 * Based on the blinking configuration, it determine what to do next.
 */
static void led_blink_handler(TIMER_PARAM_TYPE arg)
{
    uint32 idx = (uint32) arg;

    if (VALID_LED_IDX(idx) && led[idx].blinking)
    {
        led_set_state(idx, !led[idx].curr_state);  // invert LED current state

        // if LED state is back to original state and we are counting, we want to check if we should stop
        if (led[idx].curr_state == led[idx].state && led[idx].blinking_count)
        {
            // if counted to 0, we should either stop or if it is repeating code, we restart the code
            if (!--led[idx].blinking_count)
            {
                // Are we repeating a code?
                if (led[idx].blinking_repeat_code)
                {
                    // blinking the code, reload counter
                    led[idx].blinking_count = led[idx].blinking_repeat_code;
                    // give a long break before start another error code blinking
                    wiced_start_timer(&led[idx].blinking_timer, ERROR_CODE_BLINK_BREAK);
                }
                else // blinking stopped, set LED to original state
                {
                    led_set_state(idx, led[idx].state);
                }
                return;
            }
        }
        wiced_start_timer(&led[idx].blinking_timer,
                          led[idx].curr_state == led_cfg.platform[idx].default_state ?
                          led[idx].blinking_duration_off : led[idx].blinking_duration_on);
    }
}

/*******************************************************************************
* Public Functions
*******************************************************************************/

/**
 * Return true if the LED is blinking.
 */
wiced_bool_t led_is_blinking(uint32_t idx)
{
    return VALID_LED_IDX(idx) ? wiced_is_timer_in_use(&led[idx].blinking_timer) : FALSE;
}

/**
 * Stops LED blinking.
 */
void led_blink_stop(uint32_t idx)
{
    LED_LIB_TRACE("\nstop blink led idx %d", idx);
    if (VALID_LED_IDX(idx))
    {
        if (led_is_blinking(idx))
        {
            led[idx].blinking_repeat_code = 0;
            wiced_stop_timer(&led[idx].blinking_timer);
        }
        led[idx].blinking = 0;
        led_set_state(idx, led[idx].state);
    }
}

/**
 * Turn on or off LED.
 */
void led_set(uint32_t idx, wiced_bool_t on_off)
{
    if (VALID_LED_IDX(idx))
    {
        led[idx].state = on_off;
        if (!led_is_blinking(idx))
        {
            led_set_state(idx, on_off);
        }
    }
}

/**
 * To get the LED status.
 */
wiced_bool_t led_is_on(uint32_t idx)
{
    if (VALID_LED_IDX(idx))
    {
        return led[idx].state;
    }
    return FALSE;
}

/**
 * To start LED blinking. The LED on and LED off has the same duration.
 * This function is implemented in macro using led_blink2().
 */
void led_blink2(uint32_t idx, uint8_t count, uint16_t duration_on, uint16_t duration_off)
{
    LED_LIB_TRACE("\nSTART blink led idx %d", idx);
    if (VALID_LED_IDX(idx))
    {
        led_blink_stop(idx);
        led[idx].blinking = 1;
        led[idx].blinking_duration_on = duration_on;
        led[idx].blinking_duration_off = duration_off;
        led[idx].blinking_count = count;
        led_blink_handler((TIMER_PARAM_TYPE) idx);
    }
}

/**
 * To start to blink LED code.
 *
 * The code to blink followed by a long break.
 * The LED on and off duration defined by BLINK_CODE_SPEED and
 * the long break duration is defined by ERROR_CODE_BLINK_BREAK.
 */
void led_blink_code(uint32_t idx, uint8_t code)
{
    if (VALID_LED_IDX(idx))
    {
        led[idx].blinking_repeat_code = code;
        led_blink(idx, code, BLINK_CODE_SPEED);
    }
}

/**
 * Initialize LED module. This function must be called once first before using any of
 * other LED functions. It uses LED configuration from Platform and initialize hardware
 * to default state (LED off state)
 */
wiced_bool_t led_lib_init(uint8_t count, const led_config_t * cfg, wiced_bool_t feedback)
{
    LED_LIB_TRACE("[%s] count = %d", __FUNCTION__, count);
    // if LED is defined in platform
    if (count && cfg && (count <= LED_LIB_MAX))
    {
        led_cfg.count = count;
        led_cfg.platform = cfg;

        // initialize LED based on platform defines
        for (uint32_t idx=0;idx<count;idx++)
        {
            wiced_init_timer( &led[idx].blinking_timer, led_blink_handler, (TIMER_PARAM_TYPE) idx, WICED_MILLI_SECONDS_TIMER );

            LED_LIB_TRACE("LED=%d, port=%d.%d cfg:%04x, default:%d", idx, CYHAL_GET_PORT(led_cfg.platform[idx].gpio), CYHAL_GET_PIN(led_cfg.platform[idx].gpio), cfg[idx].config, cfg[idx].default_state);
            cyhal_gpio_init(led_cfg.platform[idx].gpio, led_cfg.platform[idx].direction, led_cfg.platform[idx].config, led_cfg.platform[idx].default_state);
            led_set(idx, LED_OFF); // default to turn LED off
        }

        if ((cyhal_system_get_reset_reason()==CYHAL_SYSTEM_RESET_NONE) && feedback)
        {
            LED_LIB_TRACE("Blink LED startup %d times", LED_LIB_STARTUP_INDEX);
            led_blink(LED_LIB_STARTUP_INDEX, LED_LIB_START_UP_BLINK_COUNT, LED_LIB_START_UP_BLINK_SPEED);    // start up indicator
        }
        return TRUE;
    }
    return FALSE;
}

/* End of File */
