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
 * @file button_lib.h
 *
 * @brief  This file consists of the function prototypes that are
 *         necessary for developing push button use cases
 */
#ifndef BUTTON_LIB_H_
#define BUTTON_LIB_H_

#include "cyhal_gpio.h"

/*******************************************************************************
 * typedefs
 ******************************************************************************/

/**
 * @brief   Button state change callback function defines
 *
 * @param[in] state_changed : Each bit represents each button state change.
 *                            A value 1 indicate the button state is changed.
 *                            For example, if the parameter state_changed
 *                            contains value 6, it means button 1 and button 2
 *                            the button state has been changed.
 */
typedef void (*button_state_change_t)(uint32_t state_changed);

/**
 * @brief   button configuration
 */
typedef struct {
    cyhal_gpio_t pin;               /**< GPIO pin used for the button */
    bool         released_value;    /**< Polarity */
} button_cfg_t;

/*******************************************************************************
 * functions
 ******************************************************************************/

/**
 * @brief     Initialize button functions. This is the first function to be called
 *            before any other button library functions can be used.
 *
 * @param[in] count : button count to be handled by this library.
 * @param[in] cfg   : button configuration data array. The application must pass
 *                    the number of configuration element specified by parameter
 *                    count.
 * @param[in] cb    : The callback function when any button state is changed
 *
 * @return    TRUE when button functions are initialized correctly
 */
wiced_bool_t button_lib_init(uint8_t count, const button_cfg_t * cfg, button_state_change_t cb);


/**
 * @brief   check if the button is pressed.
 *
 * @param   index : The button index
 *
 * @return  TRUE when button is pressed for the givin index.
 */
wiced_bool_t button_is_pressed(uint8_t index);

/**
 *   @brief   Returns all button states. Each bit represents each button's state, etc bit 0 for index 0, bit 1 for index 1...
 *            The button is pressed when the bit value is 1.
 *
 *   @param   none
 *
 *   @return  The button states for all buttons.
 *
 */
uint32_t     button_state();

#endif // BUTTON_LIB_H_

/* [] END OF FILE */
