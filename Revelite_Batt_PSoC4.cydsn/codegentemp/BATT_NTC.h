/*******************************************************************************
* File Name: BATT_NTC.h  
* Version 2.20
*
* Description:
*  This file contains Pin function prototypes and register defines
*
********************************************************************************
* Copyright 2008-2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_PINS_BATT_NTC_H) /* Pins BATT_NTC_H */
#define CY_PINS_BATT_NTC_H

#include "cytypes.h"
#include "cyfitter.h"
#include "BATT_NTC_aliases.h"


/***************************************
*     Data Struct Definitions
***************************************/

/**
* \addtogroup group_structures
* @{
*/
    
/* Structure for sleep mode support */
typedef struct
{
    uint32 pcState; /**< State of the port control register */
    uint32 sioState; /**< State of the SIO configuration */
    uint32 usbState; /**< State of the USBIO regulator */
} BATT_NTC_BACKUP_STRUCT;

/** @} structures */


/***************************************
*        Function Prototypes             
***************************************/
/**
* \addtogroup group_general
* @{
*/
uint8   BATT_NTC_Read(void);
void    BATT_NTC_Write(uint8 value);
uint8   BATT_NTC_ReadDataReg(void);
#if defined(BATT_NTC__PC) || (CY_PSOC4_4200L) 
    void    BATT_NTC_SetDriveMode(uint8 mode);
#endif
void    BATT_NTC_SetInterruptMode(uint16 position, uint16 mode);
uint8   BATT_NTC_ClearInterrupt(void);
/** @} general */

/**
* \addtogroup group_power
* @{
*/
void BATT_NTC_Sleep(void); 
void BATT_NTC_Wakeup(void);
/** @} power */


/***************************************
*           API Constants        
***************************************/
#if defined(BATT_NTC__PC) || (CY_PSOC4_4200L) 
    /* Drive Modes */
    #define BATT_NTC_DRIVE_MODE_BITS        (3)
    #define BATT_NTC_DRIVE_MODE_IND_MASK    (0xFFFFFFFFu >> (32 - BATT_NTC_DRIVE_MODE_BITS))

    /**
    * \addtogroup group_constants
    * @{
    */
        /** \addtogroup driveMode Drive mode constants
         * \brief Constants to be passed as "mode" parameter in the BATT_NTC_SetDriveMode() function.
         *  @{
         */
        #define BATT_NTC_DM_ALG_HIZ         (0x00u) /**< \brief High Impedance Analog   */
        #define BATT_NTC_DM_DIG_HIZ         (0x01u) /**< \brief High Impedance Digital  */
        #define BATT_NTC_DM_RES_UP          (0x02u) /**< \brief Resistive Pull Up       */
        #define BATT_NTC_DM_RES_DWN         (0x03u) /**< \brief Resistive Pull Down     */
        #define BATT_NTC_DM_OD_LO           (0x04u) /**< \brief Open Drain, Drives Low  */
        #define BATT_NTC_DM_OD_HI           (0x05u) /**< \brief Open Drain, Drives High */
        #define BATT_NTC_DM_STRONG          (0x06u) /**< \brief Strong Drive            */
        #define BATT_NTC_DM_RES_UPDWN       (0x07u) /**< \brief Resistive Pull Up/Down  */
        /** @} driveMode */
    /** @} group_constants */
#endif

/* Digital Port Constants */
#define BATT_NTC_MASK               BATT_NTC__MASK
#define BATT_NTC_SHIFT              BATT_NTC__SHIFT
#define BATT_NTC_WIDTH              1u

/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup intrMode Interrupt constants
     * \brief Constants to be passed as "mode" parameter in BATT_NTC_SetInterruptMode() function.
     *  @{
     */
        #define BATT_NTC_INTR_NONE      ((uint16)(0x0000u)) /**< \brief Disabled             */
        #define BATT_NTC_INTR_RISING    ((uint16)(0x5555u)) /**< \brief Rising edge trigger  */
        #define BATT_NTC_INTR_FALLING   ((uint16)(0xaaaau)) /**< \brief Falling edge trigger */
        #define BATT_NTC_INTR_BOTH      ((uint16)(0xffffu)) /**< \brief Both edge trigger    */
    /** @} intrMode */
/** @} group_constants */

/* SIO LPM definition */
#if defined(BATT_NTC__SIO)
    #define BATT_NTC_SIO_LPM_MASK       (0x03u)
#endif

/* USBIO definitions */
#if !defined(BATT_NTC__PC) && (CY_PSOC4_4200L)
    #define BATT_NTC_USBIO_ENABLE               ((uint32)0x80000000u)
    #define BATT_NTC_USBIO_DISABLE              ((uint32)(~BATT_NTC_USBIO_ENABLE))
    #define BATT_NTC_USBIO_SUSPEND_SHIFT        CYFLD_USBDEVv2_USB_SUSPEND__OFFSET
    #define BATT_NTC_USBIO_SUSPEND_DEL_SHIFT    CYFLD_USBDEVv2_USB_SUSPEND_DEL__OFFSET
    #define BATT_NTC_USBIO_ENTER_SLEEP          ((uint32)((1u << BATT_NTC_USBIO_SUSPEND_SHIFT) \
                                                        | (1u << BATT_NTC_USBIO_SUSPEND_DEL_SHIFT)))
    #define BATT_NTC_USBIO_EXIT_SLEEP_PH1       ((uint32)~((uint32)(1u << BATT_NTC_USBIO_SUSPEND_SHIFT)))
    #define BATT_NTC_USBIO_EXIT_SLEEP_PH2       ((uint32)~((uint32)(1u << BATT_NTC_USBIO_SUSPEND_DEL_SHIFT)))
    #define BATT_NTC_USBIO_CR1_OFF              ((uint32)0xfffffffeu)
#endif


/***************************************
*             Registers        
***************************************/
/* Main Port Registers */
#if defined(BATT_NTC__PC)
    /* Port Configuration */
    #define BATT_NTC_PC                 (* (reg32 *) BATT_NTC__PC)
#endif
/* Pin State */
#define BATT_NTC_PS                     (* (reg32 *) BATT_NTC__PS)
/* Data Register */
#define BATT_NTC_DR                     (* (reg32 *) BATT_NTC__DR)
/* Input Buffer Disable Override */
#define BATT_NTC_INP_DIS                (* (reg32 *) BATT_NTC__PC2)

/* Interrupt configuration Registers */
#define BATT_NTC_INTCFG                 (* (reg32 *) BATT_NTC__INTCFG)
#define BATT_NTC_INTSTAT                (* (reg32 *) BATT_NTC__INTSTAT)

/* "Interrupt cause" register for Combined Port Interrupt (AllPortInt) in GSRef component */
#if defined (CYREG_GPIO_INTR_CAUSE)
    #define BATT_NTC_INTR_CAUSE         (* (reg32 *) CYREG_GPIO_INTR_CAUSE)
#endif

/* SIO register */
#if defined(BATT_NTC__SIO)
    #define BATT_NTC_SIO_REG            (* (reg32 *) BATT_NTC__SIO)
#endif /* (BATT_NTC__SIO_CFG) */

/* USBIO registers */
#if !defined(BATT_NTC__PC) && (CY_PSOC4_4200L)
    #define BATT_NTC_USB_POWER_REG       (* (reg32 *) CYREG_USBDEVv2_USB_POWER_CTRL)
    #define BATT_NTC_CR1_REG             (* (reg32 *) CYREG_USBDEVv2_CR1)
    #define BATT_NTC_USBIO_CTRL_REG      (* (reg32 *) CYREG_USBDEVv2_USB_USBIO_CTRL)
#endif    
    
    
/***************************************
* The following code is DEPRECATED and 
* must not be used in new designs.
***************************************/
/**
* \addtogroup group_deprecated
* @{
*/
#define BATT_NTC_DRIVE_MODE_SHIFT       (0x00u)
#define BATT_NTC_DRIVE_MODE_MASK        (0x07u << BATT_NTC_DRIVE_MODE_SHIFT)
/** @} deprecated */

#endif /* End Pins BATT_NTC_H */


/* [] END OF FILE */
