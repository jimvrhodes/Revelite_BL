/*******************************************************************************
* File Name: VBUS_SENSE.c  
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
#include "VBUS_SENSE.h"

static VBUS_SENSE_BACKUP_STRUCT  VBUS_SENSE_backup = {0u, 0u, 0u};


/*******************************************************************************
* Function Name: VBUS_SENSE_Sleep
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
*  \snippet VBUS_SENSE_SUT.c usage_VBUS_SENSE_Sleep_Wakeup
*******************************************************************************/
void VBUS_SENSE_Sleep(void)
{
    #if defined(VBUS_SENSE__PC)
        VBUS_SENSE_backup.pcState = VBUS_SENSE_PC;
    #else
        #if (CY_PSOC4_4200L)
            /* Save the regulator state and put the PHY into suspend mode */
            VBUS_SENSE_backup.usbState = VBUS_SENSE_CR1_REG;
            VBUS_SENSE_USB_POWER_REG |= VBUS_SENSE_USBIO_ENTER_SLEEP;
            VBUS_SENSE_CR1_REG &= VBUS_SENSE_USBIO_CR1_OFF;
        #endif
    #endif
    #if defined(CYIPBLOCK_m0s8ioss_VERSION) && defined(VBUS_SENSE__SIO)
        VBUS_SENSE_backup.sioState = VBUS_SENSE_SIO_REG;
        /* SIO requires unregulated output buffer and single ended input buffer */
        VBUS_SENSE_SIO_REG &= (uint32)(~VBUS_SENSE_SIO_LPM_MASK);
    #endif  
}


/*******************************************************************************
* Function Name: VBUS_SENSE_Wakeup
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
*  Refer to VBUS_SENSE_Sleep() for an example usage.
*******************************************************************************/
void VBUS_SENSE_Wakeup(void)
{
    #if defined(VBUS_SENSE__PC)
        VBUS_SENSE_PC = VBUS_SENSE_backup.pcState;
    #else
        #if (CY_PSOC4_4200L)
            /* Restore the regulator state and come out of suspend mode */
            VBUS_SENSE_USB_POWER_REG &= VBUS_SENSE_USBIO_EXIT_SLEEP_PH1;
            VBUS_SENSE_CR1_REG = VBUS_SENSE_backup.usbState;
            VBUS_SENSE_USB_POWER_REG &= VBUS_SENSE_USBIO_EXIT_SLEEP_PH2;
        #endif
    #endif
    #if defined(CYIPBLOCK_m0s8ioss_VERSION) && defined(VBUS_SENSE__SIO)
        VBUS_SENSE_SIO_REG = VBUS_SENSE_backup.sioState;
    #endif
}


/* [] END OF FILE */
