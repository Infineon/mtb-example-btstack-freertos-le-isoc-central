/******************************************************************************
* File Name:   led.h
*
* Description: This file consists of the function prototypes that are
*              necessary for developing push button use cases.
*
*
*******************************************************************************
 * $ Copyright YEAR Cypress Semiconductor $
*******************************************************************************/

#ifndef LED_H_
#define LED_H_

#ifdef LED_SUPPORT
 #include "led_lib.h"
 #define APP_LED_MAX     5   // This BSP has 2 LEDs, 3 GPIO debug
 #define LED_GREEN       0   // The first index is Green/yellow LED
 #define LED_RED         1   // The second index is Red LED
 #define C_RX            2   // Borrow LED functions for GPIO debug
 #define C_ACTION        3   // Borrow LED functions for GPIO debug
 #define C_DBG2          4   // Borrow LED functions for GPIO debug

 #define LED_LINK        LED_GREEN
 #define LED_YELLOW      LED_GREEN  // Alias

 #define C_APP_RX_PIN   (P1_5)  // D7 on M2 board
 #define C_ACTION_PIN   (P3_7)  // D2 on M2 board
 #define C_DBG2_PIN     (P1_6)  // D8 on M2 board

 #ifdef CYBSP_USER_LED1
  #define LED_GREEN_PIN      CYBSP_USER_LED1   // EVAL board, should be P0_0 (YELLOW)
 #else
  #define LED_GREEN_PIN      (P1_1)            // Should be actual remote (LED2), use P1.1
 #endif
 #ifdef CYBSP_USER_LED2
  #define LED_RED_PIN        CYBSP_USER_LED2   // EVAL board, should be P0_1 (RED)
 #else
  #define LED_RED_PIN        (P0_2)            // Should be actual remote (LED1), use P0.2
 #endif
 #define OUTPUT_HIGH 1
 #define OUTPUT_LOW  0
 #define set_gpio_high(i) cyhal_gpio_write(led_cfg[i].gpio, OUTPUT_HIGH)
 #define set_gpio_low(i) cyhal_gpio_write(led_cfg[i].gpio, OUTPUT_LOW)
 #define gpio_toggle(i, n) for (int cnt=n;cnt--;){set_gpio_high(i); set_gpio_low(i);}
 void led_init();
 extern led_config_t led_cfg[];
#else
 #define led_blink(l,n,s)
 #define led_blink_stop(l)
 #define led_on(l)
 #define led_off(l)
 #define led_init(c, a, b)
 #define led_init()
 #define set_gpio_high(i)
 #define set_gpio_low(i)
 #define gpio_toggle(i,n)
#endif

#endif //LED_H_

/* [] END OF FILE */
