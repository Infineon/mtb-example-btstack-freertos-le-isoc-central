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
