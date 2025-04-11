/*******************************************************************************
* File Name: button.c
*
* Description: This file consists of the function prototypes that are
*              necessary for developing push button use cases.
*
*******************************************************************************
* Copyright 2021-2025, Cypress Semiconductor Corporation (an Infineon company) or
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
*******************************************************************************/

/*******************************************************************************
 *                              INCLUDES
 ******************************************************************************/
#ifdef BUTTON_SUPPORT

#include "wiced_bt_types.h"
#include "wiced_bt_trace.h"
#include "wiced_timer.h"
#include "button.h"
#include "app.h"
#include "app_terminal_trace.h"
//#ifndef CYBSP_BTN_OFF
// #define CYBSP_BTN_OFF  (1U)
//#endif
#define BUTTON_MAX 2        // We only have one button
#define CONNECT_BUTTON 0    // index 0
#define LE_FAST_CI_BUTTON 1    // index 0

static button_cfg_t button_cfg[BUTTON_MAX] = {
  { CYBSP_USER_BTN, CYBSP_BTN_OFF },
  { CYBSP_USER_BTN2, CYBSP_BTN_OFF },
};

#define BUTTON_HOLD_TIME_IN_SEC      3   // 3 second
static wiced_timer_t buttonTimer;

////////////////////////////////////////////////////////////////////////////////
/// This function is the timeout handler for commit_timer
////////////////////////////////////////////////////////////////////////////////
static void button_hold_timer_cb( TIMER_PARAM_TYPE arg )
{
    app_le_stop_scan();

    if (isoc_cis_connected())
    {
        isoc_close_all();
    }
    else
    {
        //delete all pairing info
        WICED_BT_TRACE("Button is held for %d sec, removing all paired devices",
                       BUTTON_HOLD_TIME_IN_SEC);
        app_dev_remove_all();
    }
}

////////////////////////////////////////////////////////////////////////////////

/*
 *
 */
static void button_state_changed(uint32_t changed)
{
    if (changed & (1<<CONNECT_BUTTON)) // is connect button changed?
    {
        wiced_bool_t pressed = button_is_pressed(USER_BUTTON);

        if(pressed)
        {
            wiced_start_timer(&buttonTimer, BUTTON_HOLD_TIME_IN_SEC);
        }
        else
        {
            if(wiced_is_timer_in_use(&buttonTimer))
                wiced_stop_timer(&buttonTimer);
        }

        // If all isoc channels connected, send out data
        if(isoc_cis_connected()
           && (isoc_cis_connected_count() == MAX_CIS_PER_CIG))
        {
            //WICED_BT_TRACE("Sending isoc data");
            isoc_send_data( pressed );
            return;
        }
        else if((app_acl_connected_count() < MAX_CIS_PER_CIG) && pressed)
        {
            WICED_BT_TRACE("Scanning for device 'IFX ISOC' to connect");
            app_le_start_scan();
        }
    }
    else
    {
        WICED_BT_TRACE("something is wrong: unknown button changed");
    }
}

/*******************************************************************************
*   PUBLIC FUNCTIONS
*******************************************************************************/

void button_init(void)
{
    wiced_result_t result;

    result = wiced_init_timer(&buttonTimer, button_hold_timer_cb, 0,
                              WICED_SECONDS_TIMER);

    WICED_BT_TRACE("wiced_init_timer %d", result);

    button_lib_init(BUTTON_MAX, button_cfg, button_state_changed);
}

#endif /* BUTTON_SUPPORT */
/* end of file */
