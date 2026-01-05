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
