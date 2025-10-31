/*******************************************************************************
* File Name: VBUS_SENSE.h  
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

#if !defined(CY_PINS_VBUS_SENSE_H) /* Pins VBUS_SENSE_H */
#define CY_PINS_VBUS_SENSE_H

#include "cytypes.h"
#include "cyfitter.h"
#include "VBUS_SENSE_aliases.h"


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
} VBUS_SENSE_BACKUP_STRUCT;

/** @} structures */


/***************************************
*        Function Prototypes             
***************************************/
/**
* \addtogroup group_general
* @{
*/
uint8   VBUS_SENSE_Read(void);
void    VBUS_SENSE_Write(uint8 value);
uint8   VBUS_SENSE_ReadDataReg(void);
#if defined(VBUS_SENSE__PC) || (CY_PSOC4_4200L) 
    void    VBUS_SENSE_SetDriveMode(uint8 mode);
#endif
void    VBUS_SENSE_SetInterruptMode(uint16 position, uint16 mode);
uint8   VBUS_SENSE_ClearInterrupt(void);
/** @} general */

/**
* \addtogroup group_power
* @{
*/
void VBUS_SENSE_Sleep(void); 
void VBUS_SENSE_Wakeup(void);
/** @} power */


/***************************************
*           API Constants        
***************************************/
#if defined(VBUS_SENSE__PC) || (CY_PSOC4_4200L) 
    /* Drive Modes */
    #define VBUS_SENSE_DRIVE_MODE_BITS        (3)
    #define VBUS_SENSE_DRIVE_MODE_IND_MASK    (0xFFFFFFFFu >> (32 - VBUS_SENSE_DRIVE_MODE_BITS))

    /**
    * \addtogroup group_constants
    * @{
    */
        /** \addtogroup driveMode Drive mode constants
         * \brief Constants to be passed as "mode" parameter in the VBUS_SENSE_SetDriveMode() function.
         *  @{
         */
        #define VBUS_SENSE_DM_ALG_HIZ         (0x00u) /**< \brief High Impedance Analog   */
        #define VBUS_SENSE_DM_DIG_HIZ         (0x01u) /**< \brief High Impedance Digital  */
        #define VBUS_SENSE_DM_RES_UP          (0x02u) /**< \brief Resistive Pull Up       */
        #define VBUS_SENSE_DM_RES_DWN         (0x03u) /**< \brief Resistive Pull Down     */
        #define VBUS_SENSE_DM_OD_LO           (0x04u) /**< \brief Open Drain, Drives Low  */
        #define VBUS_SENSE_DM_OD_HI           (0x05u) /**< \brief Open Drain, Drives High */
        #define VBUS_SENSE_DM_STRONG          (0x06u) /**< \brief Strong Drive            */
        #define VBUS_SENSE_DM_RES_UPDWN       (0x07u) /**< \brief Resistive Pull Up/Down  */
        /** @} driveMode */
    /** @} group_constants */
#endif

/* Digital Port Constants */
#define VBUS_SENSE_MASK               VBUS_SENSE__MASK
#define VBUS_SENSE_SHIFT              VBUS_SENSE__SHIFT
#define VBUS_SENSE_WIDTH              1u

/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup intrMode Interrupt constants
     * \brief Constants to be passed as "mode" parameter in VBUS_SENSE_SetInterruptMode() function.
     *  @{
     */
        #define VBUS_SENSE_INTR_NONE      ((uint16)(0x0000u)) /**< \brief Disabled             */
        #define VBUS_SENSE_INTR_RISING    ((uint16)(0x5555u)) /**< \brief Rising edge trigger  */
        #define VBUS_SENSE_INTR_FALLING   ((uint16)(0xaaaau)) /**< \brief Falling edge trigger */
        #define VBUS_SENSE_INTR_BOTH      ((uint16)(0xffffu)) /**< \brief Both edge trigger    */
    /** @} intrMode */
/** @} group_constants */

/* SIO LPM definition */
#if defined(VBUS_SENSE__SIO)
    #define VBUS_SENSE_SIO_LPM_MASK       (0x03u)
#endif

/* USBIO definitions */
#if !defined(VBUS_SENSE__PC) && (CY_PSOC4_4200L)
    #define VBUS_SENSE_USBIO_ENABLE               ((uint32)0x80000000u)
    #define VBUS_SENSE_USBIO_DISABLE              ((uint32)(~VBUS_SENSE_USBIO_ENABLE))
    #define VBUS_SENSE_USBIO_SUSPEND_SHIFT        CYFLD_USBDEVv2_USB_SUSPEND__OFFSET
    #define VBUS_SENSE_USBIO_SUSPEND_DEL_SHIFT    CYFLD_USBDEVv2_USB_SUSPEND_DEL__OFFSET
    #define VBUS_SENSE_USBIO_ENTER_SLEEP          ((uint32)((1u << VBUS_SENSE_USBIO_SUSPEND_SHIFT) \
                                                        | (1u << VBUS_SENSE_USBIO_SUSPEND_DEL_SHIFT)))
    #define VBUS_SENSE_USBIO_EXIT_SLEEP_PH1       ((uint32)~((uint32)(1u << VBUS_SENSE_USBIO_SUSPEND_SHIFT)))
    #define VBUS_SENSE_USBIO_EXIT_SLEEP_PH2       ((uint32)~((uint32)(1u << VBUS_SENSE_USBIO_SUSPEND_DEL_SHIFT)))
    #define VBUS_SENSE_USBIO_CR1_OFF              ((uint32)0xfffffffeu)
#endif


/***************************************
*             Registers        
***************************************/
/* Main Port Registers */
#if defined(VBUS_SENSE__PC)
    /* Port Configuration */
    #define VBUS_SENSE_PC                 (* (reg32 *) VBUS_SENSE__PC)
#endif
/* Pin State */
#define VBUS_SENSE_PS                     (* (reg32 *) VBUS_SENSE__PS)
/* Data Register */
#define VBUS_SENSE_DR                     (* (reg32 *) VBUS_SENSE__DR)
/* Input Buffer Disable Override */
#define VBUS_SENSE_INP_DIS                (* (reg32 *) VBUS_SENSE__PC2)

/* Interrupt configuration Registers */
#define VBUS_SENSE_INTCFG                 (* (reg32 *) VBUS_SENSE__INTCFG)
#define VBUS_SENSE_INTSTAT                (* (reg32 *) VBUS_SENSE__INTSTAT)

/* "Interrupt cause" register for Combined Port Interrupt (AllPortInt) in GSRef component */
#if defined (CYREG_GPIO_INTR_CAUSE)
    #define VBUS_SENSE_INTR_CAUSE         (* (reg32 *) CYREG_GPIO_INTR_CAUSE)
#endif

/* SIO register */
#if defined(VBUS_SENSE__SIO)
    #define VBUS_SENSE_SIO_REG            (* (reg32 *) VBUS_SENSE__SIO)
#endif /* (VBUS_SENSE__SIO_CFG) */

/* USBIO registers */
#if !defined(VBUS_SENSE__PC) && (CY_PSOC4_4200L)
    #define VBUS_SENSE_USB_POWER_REG       (* (reg32 *) CYREG_USBDEVv2_USB_POWER_CTRL)
    #define VBUS_SENSE_CR1_REG             (* (reg32 *) CYREG_USBDEVv2_CR1)
    #define VBUS_SENSE_USBIO_CTRL_REG      (* (reg32 *) CYREG_USBDEVv2_USB_USBIO_CTRL)
#endif    
    
    
/***************************************
* The following code is DEPRECATED and 
* must not be used in new designs.
***************************************/
/**
* \addtogroup group_deprecated
* @{
*/
#define VBUS_SENSE_DRIVE_MODE_SHIFT       (0x00u)
#define VBUS_SENSE_DRIVE_MODE_MASK        (0x07u << VBUS_SENSE_DRIVE_MODE_SHIFT)
/** @} deprecated */

#endif /* End Pins VBUS_SENSE_H */


/* [] END OF FILE */
