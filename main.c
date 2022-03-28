/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the PMG1 Flash Write Example
*              for ModusToolbox.
*
* Related Document: See README.md
*
*
*******************************************************************************
* Copyright 2021-2022, Cypress Semiconductor Corporation (an Infineon company) or
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
#include "cybsp.h"
#include "cy_pdl.h"


/*******************************************************************************
* Macros
*******************************************************************************/
#define CY_ASSERT_FAILED          (0u)

/*This array reserves space in the flash for one row of size
 * CY_FLASH_SIZEOF_ROW. Explicit initialization is required so that memory is
 * allocated in flash instead of RAM. CY_ALIGN ensures that the array address
 * is an integer multiple of the row size so that the array occupies one
 * complete row.*/
const uint32_t row[CY_FLASH_SIZEOF_ROW]CY_ALIGN(CY_FLASH_SIZEOF_ROW) = {0};

/* Array to hold the data to be written into flash */

uint32_t Data[CY_FLASH_SIZEOF_ROW];

/*******************************************************************************
* Global Variable
*******************************************************************************/
cy_stc_scb_uart_context_t CYBSP_UART_context;

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
*  System entrance point. This function performs
*  - initial setup of device
*  - configure the SCB block as UART interface
*  - prints out status of Flash operation via UART interface
*  
*
* Parameters:
*  none
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    uint16_t index;
    cy_en_flashdrv_status_t flashWriteStatus;
    cy_rslt_t result;

    /* Initialize the device and board peripherals */
    result = cybsp_init();

    /* Board init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(CY_ASSERT_FAILED);
    }

    /* Configure and enable the UART peripheral */
    Cy_SCB_UART_Init(CYBSP_UART_HW, &CYBSP_UART_config, &CYBSP_UART_context);
    Cy_SCB_UART_Enable(CYBSP_UART_HW);

    /* Enable global interrupts */
    __enable_irq();

    /* Initialize the data in RAM that will be written into flash */
    for(index = 0; index < CY_FLASH_SIZEOF_ROW; index++)
    {
        Data[index] = index;
    }

    /* Cy_Flash_WriteRow() is a blocking API that does not return until the
     * flash write is complete
     */
    flashWriteStatus = Cy_Flash_WriteRow((uint32_t)row, (const uint32_t *)Data);

    if((flashWriteStatus == CY_FLASH_DRV_SUCCESS) && (memcmp(Data,row, CY_FLASH_SIZEOF_ROW) == 0u))
    {
        /* Send a string over serial terminal */
        Cy_SCB_UART_PutString(CYBSP_UART_HW, "Flash write successful \r\n");
    }

    else
    {
        /* Send a string over serial terminal */
        Cy_SCB_UART_PutString(CYBSP_UART_HW, "Flash write failed \r\n");
    }

    for(;;)
    {
        //Infinite Loop
    }
}

/* [] END OF FILE */

