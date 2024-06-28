/******************************************************************************
* File Name:   button.h
*
* Description: This file consists of the function prototypes that are
*              necessary for developing push button use cases.
*
*
*******************************************************************************
 * $ Copyright YEAR Cypress Semiconductor $
*******************************************************************************/
#ifndef BUTTON_H_
#define BUTTON_H_

#ifdef BUTTON_SUPPORT

#include "button_lib.h"

#define USER_BUTTON 0    // index 0

/*******************************************************************************
 * Function prototype
 ******************************************************************************/
void button_init(void);

#else
 #define button_init()
#endif     // BUTTON_SUPPORT

#endif      /* BUTTON_H_ */

/* [] END OF FILE */
