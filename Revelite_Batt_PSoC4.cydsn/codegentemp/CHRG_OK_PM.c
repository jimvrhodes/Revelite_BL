/*******************************************************************************
* File Name: CHRG_OK.c  
* Version 2.20
*
* Description:
*  This file contains APIs to set up the Pins component for low power modes.
*
* Note:
*
********************************************************************************
* Copyright 2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#include "cytypes.h"
#include "CHRG_OK.h"

static CHRG_OK_BACKUP_STRUCT  CHRG_OK_backup = {0u, 0u, 0u};


/*******************************************************************************
* Function Name: CHRG_OK_Sleep
****************************************************************************//**
*
* \brief Stores the pin configuration and prepares the pin for entering chip 
*  deep-sleep/hibernate modes. This function applies only to SIO and USBIO pins.
*  It should not be called for GPIO or GPIO_OVT pins.
*
* <b>Note</b> This function is available in PSoC 4 only.
*
* \return 
*  None 
*  
* \sideeffect
*  For SIO pins, this function configures the pin input threshold to CMOS and
*  drive level to Vddio. This is needed for SIO pins when in device 
*  deep-sleep/hibernate modes.
*
* \funcusage
*  \snippet CHRG_OK_SUT.c usage_CHRG_OK_Sleep_Wakeup
*******************************************************************************/
void CHRG_OK_Sleep(void)
{
    #if defined(CHRG_OK__PC)
        CHRG_OK_backup.pcState = CHRG_OK_PC;
    #else
        #if (CY_PSOC4_4200L)
            /* Save the regulator state and put the PHY into suspend mode */
            CHRG_OK_backup.usbState = CHRG_OK_CR1_REG;
            CHRG_OK_USB_POWER_REG |= CHRG_OK_USBIO_ENTER_SLEEP;
            CHRG_OK_CR1_REG &= CHRG_OK_USBIO_CR1_OFF;
        #endif
    #endif
    #if defined(CYIPBLOCK_m0s8ioss_VERSION) && defined(CHRG_OK__SIO)
        CHRG_OK_backup.sioState = CHRG_OK_SIO_REG;
        /* SIO requires unregulated output buffer and single ended input buffer */
        CHRG_OK_SIO_REG &= (uint32)(~CHRG_OK_SIO_LPM_MASK);
    #endif  
}


/*******************************************************************************
* Function Name: CHRG_OK_Wakeup
****************************************************************************//**
*
* \brief Restores the pin configuration that was saved during Pin_Sleep(). This 
* function applies only to SIO and USBIO pins. It should not be called for
* GPIO or GPIO_OVT pins.
*
* For USBIO pins, the wakeup is only triggered for falling edge interrupts.
*
* <b>Note</b> This function is available in PSoC 4 only.
*
* \return 
*  None
*  
* \funcusage
*  Refer to CHRG_OK_Sleep() for an example usage.
*******************************************************************************/
void CHRG_OK_Wakeup(void)
{
    #if defined(CHRG_OK__PC)
        CHRG_OK_PC = CHRG_OK_backup.pcState;
    #else
        #if (CY_PSOC4_4200L)
            /* Restore the regulator state and come out of suspend mode */
            CHRG_OK_USB_POWER_REG &= CHRG_OK_USBIO_EXIT_SLEEP_PH1;
            CHRG_OK_CR1_REG = CHRG_OK_backup.usbState;
            CHRG_OK_USB_POWER_REG &= CHRG_OK_USBIO_EXIT_SLEEP_PH2;
        #endif
    #endif
    #if defined(CYIPBLOCK_m0s8ioss_VERSION) && defined(CHRG_OK__SIO)
        CHRG_OK_SIO_REG = CHRG_OK_backup.sioState;
    #endif
}


/* [] END OF FILE */
