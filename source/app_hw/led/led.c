/******************************************************************************
* File Name:   led.c
*
* Description: This file consists of the function prototypes that are
*              necessary for developing push button use cases.
*
*
*******************************************************************************
 * $ Copyright YEAR Cypress Semiconductor $
*******************************************************************************/
#ifdef LED_SUPPORT
#include "led.h"

/*******************************************************************************
 * led config table
 ******************************************************************************/
led_config_t led_cfg[APP_LED_MAX] =
{
    [LED_YELLOW]= {CYBSP_USER_LED1, CYHAL_GPIO_DIR_OUTPUT, 
                   CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF},
    [LED_RED]   = {CYBSP_USER_LED2,   CYHAL_GPIO_DIR_OUTPUT,
                   CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF},
    [C_RX]      = {C_APP_RX_PIN,  CYHAL_GPIO_DIR_OUTPUT,
                   CYHAL_GPIO_DRIVE_STRONG, OUTPUT_LOW},
    [C_ACTION]  = {C_ACTION_PIN,  CYHAL_GPIO_DIR_OUTPUT, 
                   CYHAL_GPIO_DRIVE_STRONG, OUTPUT_LOW},
    [C_DBG2]    = {C_DBG2_PIN,    CYHAL_GPIO_DIR_OUTPUT, 
                   CYHAL_GPIO_DRIVE_STRONG, OUTPUT_LOW},
};


void led_init()
{
    led_lib_init(APP_LED_MAX, led_cfg, TRUE);
}

#endif

/* End of File */
