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
 * @file    led_lib.h
 *
 * @brief   This file consists of the function prototypes that are
 *          necessary for developing LED use cases
 */
#ifndef LED_LIB_H_
#define LED_LIB_H_

#include "wiced_bt_types.h"
#include "cybsp_types.h"
#include "cyhal_gpio.h"

/*******************************************************************************
* Defines
*******************************************************************************/

/*******************************************************************************
* struct/enums
*******************************************************************************/

/**
 * @brief   LED configuration
 */
typedef struct
{
    cyhal_gpio_t            gpio;           /**< GPIO pin */
    cyhal_gpio_direction_t  direction;      /**< CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DIR_OUTPUT, or CYHAL_GPIO_DIR_BIDIRECTIONAL */
    cyhal_gpio_drive_mode_t config;         /**< CYHAL_GPIO_DRIVE_XXXX (see cyhal_gpio_drive_mode_t) */
    uint8_t                 default_state;  /**< logic to turn LED off */

} led_config_t;

/**
 * @brief   enum for turning LED on/off
 */
enum {
    LED_OFF,           /**< turn LED off */
    LED_ON,            /**< turn LED on */
};

/*******************************************************************************
* Functions
*******************************************************************************/

/**
 * @brief     Initialize LED libraray.
 *
 * @detail    This function must be called once first before using any of
 *            other LED functions. It uses LED configuration from Platform
 *            and initialize hardware to default state (LED off state)
 *
 * @param[in] count    : Number of LED
 * @param[in] cfg      : Array of LED configuration
 * @param[in] feedback : When TRUE, blinks LED 5 times to indicate LED initialization is successful
 *
 * @return    TRUE when intialization is successful
 */
wiced_bool_t led_lib_init(uint8_t count, const led_config_t * cfg, wiced_bool_t feedback);

/**
 * @brief     Turn on or off LED.
 *
 * @param[in] idx    : The LED index
 * @param[in] on_off : Use LED_ON to turn on LED, LED_OFF to turn off LED
 *
 * @return    None
 */
void led_set(uint32_t idx, wiced_bool_t on_off);

/**
 * @brief     Turn on LED.
 *
 * @param[in] idx : The LED index
 *
 * @return    None
 */
#define led_on( idx) led_set(idx, LED_ON)

/**
 * @brief     Turn off LED.
 *
 * @param[in] idx : The LED index
 *
 * @return    None
 */
#define led_off(idx) led_set(idx, LED_OFF)

/**
 * @brief     Check if LED is on.
 *
 * @param[in] idx    : The LED index
 *
 * @return    Turn TRUE when LED is on
 */
wiced_bool_t led_is_on(uint32_t idx);

/**
 * @brief     Start to blink LED
 *
 * @param[in] idx          : The LED index to blink
 * @param[in] count        : How many times to blink. Use 0 to blink LED forever.
 * @param[in] duration_on  : The during for LED is on in milliseconds
 * @param[in] duration_off : The during for LED is off in milliseconds
 *
 * @return    none
 */
void led_blink2(uint32_t idx, uint8_t count, uint16_t duration_on, uint16_t duration_off);

/**
 * @brief     Start to blink LED
 *
 * @param[in] idx      : The LED index to blink
 * @param[in] count    : How many times to blink. Use 0 to blink LED forever.
 * @param[in] duration : The during for LED is on and off in milliseconds
 *
 * @return    none
 */
#define led_blink(idx, count, duration) led_blink2(idx, count, duration, duration)

/**
 * @brief     Stop LED blinking
 *
 * @param[in] idx      : The LED index to blink
 *
 * @return    none
 */
void led_blink_stop(uint32_t idx);

/**
 * @brief     Check if LED is blinking
 *
 * @param[in] idx : The LED index to blink
 *
 * @return    TRUE when LED is blinking
 */
wiced_bool_t led_is_blinking(uint32_t idx);

/**
 * @brief     To start to blink LED code
 *
 * @detail    This function is to be used for blinking error code.
 *            It blinks code followed by a long break and repeats
 *            until led_blink_stop() is called.
 *
 * @param[in] idx  : the LED index
 * @param[in] code : the code to blink followed by a long break
 *
 * @return    None
 */
void led_blink_code(uint32_t idx, uint8_t code);

#endif // LED_LIB_H_

/* End of File */
