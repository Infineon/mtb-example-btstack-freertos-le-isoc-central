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
 * @file nvram_lib.h
 *
 * @brief This is the API defines for nvram access
 */
#ifndef NVRAM_LIB_H_
#define NVRAM_LIB_H_

#include "mtb_kvstore.h"

/*******************************************************************************
* Functions
*******************************************************************************/

/**
 * @brief     This function performs a write to NVRAM
 *
 * @param[in] key  : identifying key string
 * @param[in] ptr  : points to data to write
 * @param[in] size : size of data to write
 *
 * @return    CY_RSLT_SUCCESS when operation is successful.
 *            See cy_rslt_t or mtb_kvstore_write() for possible return codes.
 */
cy_rslt_t nvram_write(const char* key, uint8_t * ptr, uint32_t size);

/**
 * @brief  This function performs a read from NVRAM
 *
 * @param[in] key  : identifying key string
 * @param[in] ptr  : points to data buffer to read
 * @param[in] size : size of data to read
 *
 * @return CY_RSLT_SUCCESS when operation is successful.
 *         See cy_rslt_t or See mtb_kvstore_read() for possible error return code.
 */
cy_rslt_t nvram_read(const char* key, uint8_t * ptr, uint32_t len);

/**
* @brief     This function initializes the SMIF and kv-store library
*
* @param[in] cfg : kvstore configuration
*
* @return    TRUE when initialization successful.
*/
wiced_bool_t nvram_init(mtb_kvstore_bd_t * cfg);

#endif // NVRAM_LIB_H_

/* [] END OF FILE */
