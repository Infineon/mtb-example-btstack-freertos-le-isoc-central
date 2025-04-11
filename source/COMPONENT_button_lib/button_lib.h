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
