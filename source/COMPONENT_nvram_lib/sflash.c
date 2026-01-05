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
 * file sflash.c
 *
 * Description: This file contains block device function implementations
 *              required by kv-store library for external serial flash
 *
 * Related Document: See README.md
 */
#ifndef USE_INTERNAL_FLASH

#include "wiced_bt_trace.h"
#include "cybsp.h"
#include "cycfg_qspi_memslot.h"
#include "cy_serial_flash_qspi.h"
#include "mtb_kvstore.h"
#include "app_terminal_trace.h"
#if NVRAM_TRACE
# define NVRAM_LIB_TRACE     WICED_BT_TRACE
#else
# define NVRAM_LIB_TRACE(...)
#endif

/*******************************************************************************
 * Defines
 ******************************************************************************/
#define  QSPI_BUS_FREQ                       (50000000l)

#ifndef CYBSP_QSPI_SS
 #define CYBSP_QSPI_SS ioss_0_port_2_pin_0
#endif
#ifndef CYBSP_QSPI_D3
 #define CYBSP_QSPI_D3 ioss_0_port_2_pin_1
#endif
#ifndef CYBSP_QSPI_D2
 #define CYBSP_QSPI_D2 ioss_0_port_2_pin_2
#endif
#ifndef CYBSP_QSPI_D1
 #define CYBSP_QSPI_D1 ioss_0_port_2_pin_3
#endif
#ifndef CYBSP_QSPI_D0
 #define CYBSP_QSPI_D0 ioss_0_port_2_pin_4
#endif
#ifndef CYBSP_QSPI_SCK
 #define CYBSP_QSPI_SCK ioss_0_port_2_pin_5
#endif

/*******************************************************************************
 * Data
 ******************************************************************************/
static cy_stc_smif_context_t SMIFContext;

/*******************************************************************************
 * Functions
 ******************************************************************************/

/**
 * This function initializes the underlying block device (in this case external flash).
 */
void nvram_device_init(void)
{
    cy_rslt_t rslt;

    /* Initialize the QSPI*/
    rslt = cy_serial_flash_qspi_init(smifMemConfigs[0], CYBSP_QSPI_D0,
                                     CYBSP_QSPI_D1, CYBSP_QSPI_D2, CYBSP_QSPI_D3,
                                     NC,NC, NC, NC, CYBSP_QSPI_SCK,
                                     CYBSP_QSPI_SS, QSPI_BUS_FREQ);

    /*Check if the QSPI initialization was successful */
    if (CY_RSLT_SUCCESS == rslt)
    {
        NVRAM_LIB_TRACE("successfully initialized QSPI");
    }
    else
    {
        WICED_BT_TRACE("*** failed to initialize QSPI");
    }

}

/**
 * This function is used to define the bond data storage (in this case external flash).
 */
void nvram_get_init_params(uint32_t *length, uint32_t *start_addr)
{
    uint32_t sector_size = 0;

    /* If the device is not a hybrid memory, use last sector to erase since
     * first sector has some configuration data used during boot from
     * flash operation.
     */
    if (0u == smifMemConfigs[0]->deviceCfg->hybridRegionCount)
    {
        *start_addr = (smifMemConfigs[0]->deviceCfg->memSize/2 -
                       smifMemConfigs[0]->deviceCfg->eraseSize *2);
    }

    /* Define the space to be used for Bond Data Storage */
    sector_size = cy_serial_flash_qspi_get_erase_size(*start_addr);
    *length = (sector_size * 2);
}

/**
 * Function to get the read size of the block device for a specific address.
 */
static uint32_t bd_read_size(void* context, uint32_t addr)
{
    (void)context;
    (void)addr;
    return 1;
}

/**
 * Function to get the program size of the block device for a specific address.
 */
static uint32_t bd_program_size(void* context, uint32_t addr)
{
    (void)context;
    CY_UNUSED_PARAMETER(addr);
    return (size_t)smifBlockConfig.memConfig[0]->deviceCfg->programSize;
}

/**
 * Function prototype to get the erase size of the block device for a specific address.
 */
static uint32_t bd_erase_size(void* context, uint32_t addr)
{
    (void)context;
    size_t                            erase_sector_size;
    cy_stc_smif_hybrid_region_info_t* hybrid_info = NULL;

    cy_en_smif_status_t smif_status =
        Cy_SMIF_MemLocateHybridRegion(smifBlockConfig.memConfig[0], &hybrid_info, addr);

    if (CY_SMIF_SUCCESS != smif_status)
    {
        erase_sector_size = (size_t)smifBlockConfig.memConfig[0]->deviceCfg->eraseSize;
    }
    else
    {
        erase_sector_size = (size_t)hybrid_info->eraseSize;
    }

    return erase_sector_size;
}

/**
 * Function for reading data from the block device.
 */
static cy_rslt_t bd_read(void* context, uint32_t addr, uint32_t length, uint8_t* buf)
{
    (void)context;
    cy_rslt_t result = 0;
    // Cy_SMIF_MemRead() returns error if (addr + length) > total flash size.
    result = (cy_rslt_t)Cy_SMIF_MemRead(SMIF0, smifBlockConfig.memConfig[0],
            addr,
            buf, length, &SMIFContext);

    return result;
}

/**
 * Function for programming of a block device for a specific address
 */
static cy_rslt_t bd_program(void* context, uint32_t addr, uint32_t length, const uint8_t* buf)
{
    (void)context;
    cy_rslt_t result = 0;
    // Cy_SMIF_MemWrite() returns error if (addr + length) > total flash size.
    result = (cy_rslt_t)Cy_SMIF_MemWrite(SMIF0, smifBlockConfig.memConfig[0],
            addr,
            (uint8_t*)buf, length, &SMIFContext);

    return result;
}

/**
 * Function for erasing of a block device for a specific address
 */
static cy_rslt_t bd_erase(void* context, uint32_t addr, uint32_t length)
{
    (void)context;
    cy_rslt_t result = 0;
    // If the erase is for the entire chip, use chip erase command
    if ((addr == 0u) && (length == (size_t)smifBlockConfig.memConfig[0]->deviceCfg->memSize))
    {
        result =
                (cy_rslt_t)Cy_SMIF_MemEraseChip(SMIF0,
                        smifBlockConfig.memConfig[0],
                        &SMIFContext);
    }
    else
    {
        // Cy_SMIF_MemEraseSector() returns error if (addr + length) > total flash size or if
        // addr is not aligned to erase sector size or if (addr + length) is not aligned to
        // erase sector size.
        result =
                (cy_rslt_t)Cy_SMIF_MemEraseSector(SMIF0,
                        smifBlockConfig.memConfig[0],
                        addr, length, &SMIFContext);
    }

    return result;
}

/**
 * default mtb_kvstore_bd_t
 */
mtb_kvstore_bd_t nvram_cb =
{
    .read         = bd_read,
    .program      = bd_program,
    .erase        = bd_erase,
    .read_size    = bd_read_size,
    .program_size = bd_program_size,
    .erase_size   = bd_erase_size,
    .context      = NULL,
};

#endif // !USE_INTERNAL_FLASH

/* end of file */
