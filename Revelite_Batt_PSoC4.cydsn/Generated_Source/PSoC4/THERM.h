/*******************************************************************************
* File Name: THERM.h  
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

#if !defined(CY_PINS_THERM_H) /* Pins THERM_H */
#define CY_PINS_THERM_H

#include "cytypes.h"
#include "cyfitter.h"
#include "THERM_aliases.h"


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
} THERM_BACKUP_STRUCT;

/** @} structures */


/***************************************
*        Function Prototypes             
***************************************/
/**
* \addtogroup group_general
* @{
*/
uint8   THERM_Read(void);
void    THERM_Write(uint8 value);
uint8   THERM_ReadDataReg(void);
#if defined(THERM__PC) || (CY_PSOC4_4200L) 
    void    THERM_SetDriveMode(uint8 mode);
#endif
void    THERM_SetInterruptMode(uint16 position, uint16 mode);
uint8   THERM_ClearInterrupt(void);
/** @} general */

/**
* \addtogroup group_power
* @{
*/
void THERM_Sleep(void); 
void THERM_Wakeup(void);
/** @} power */


/***************************************
*           API Constants        
***************************************/
#if defined(THERM__PC) || (CY_PSOC4_4200L) 
    /* Drive Modes */
    #define THERM_DRIVE_MODE_BITS        (3)
    #define THERM_DRIVE_MODE_IND_MASK    (0xFFFFFFFFu >> (32 - THERM_DRIVE_MODE_BITS))

    /**
    * \addtogroup group_constants
    * @{
    */
        /** \addtogroup driveMode Drive mode constants
         * \brief Constants to be passed as "mode" parameter in the THERM_SetDriveMode() function.
         *  @{
         */
        #define THERM_DM_ALG_HIZ         (0x00u) /**< \brief High Impedance Analog   */
        #define THERM_DM_DIG_HIZ         (0x01u) /**< \brief High Impedance Digital  */
        #define THERM_DM_RES_UP          (0x02u) /**< \brief Resistive Pull Up       */
        #define THERM_DM_RES_DWN         (0x03u) /**< \brief Resistive Pull Down     */
        #define THERM_DM_OD_LO           (0x04u) /**< \brief Open Drain, Drives Low  */
        #define THERM_DM_OD_HI           (0x05u) /**< \brief Open Drain, Drives High */
        #define THERM_DM_STRONG          (0x06u) /**< \brief Strong Drive            */
        #define THERM_DM_RES_UPDWN       (0x07u) /**< \brief Resistive Pull Up/Down  */
        /** @} driveMode */
    /** @} group_constants */
#endif

/* Digital Port Constants */
#define THERM_MASK               THERM__MASK
#define THERM_SHIFT              THERM__SHIFT
#define THERM_WIDTH              1u

/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup intrMode Interrupt constants
     * \brief Constants to be passed as "mode" parameter in THERM_SetInterruptMode() function.
     *  @{
     */
        #define THERM_INTR_NONE      ((uint16)(0x0000u)) /**< \brief Disabled             */
        #define THERM_INTR_RISING    ((uint16)(0x5555u)) /**< \brief Rising edge trigger  */
        #define THERM_INTR_FALLING   ((uint16)(0xaaaau)) /**< \brief Falling edge trigger */
        #define THERM_INTR_BOTH      ((uint16)(0xffffu)) /**< \brief Both edge trigger    */
    /** @} intrMode */
/** @} group_constants */

/* SIO LPM definition */
#if defined(THERM__SIO)
    #define THERM_SIO_LPM_MASK       (0x03u)
#endif

/* USBIO definitions */
#if !defined(THERM__PC) && (CY_PSOC4_4200L)
    #define THERM_USBIO_ENABLE               ((uint32)0x80000000u)
    #define THERM_USBIO_DISABLE              ((uint32)(~THERM_USBIO_ENABLE))
    #define THERM_USBIO_SUSPEND_SHIFT        CYFLD_USBDEVv2_USB_SUSPEND__OFFSET
    #define THERM_USBIO_SUSPEND_DEL_SHIFT    CYFLD_USBDEVv2_USB_SUSPEND_DEL__OFFSET
    #define THERM_USBIO_ENTER_SLEEP          ((uint32)((1u << THERM_USBIO_SUSPEND_SHIFT) \
                                                        | (1u << THERM_USBIO_SUSPEND_DEL_SHIFT)))
    #define THERM_USBIO_EXIT_SLEEP_PH1       ((uint32)~((uint32)(1u << THERM_USBIO_SUSPEND_SHIFT)))
    #define THERM_USBIO_EXIT_SLEEP_PH2       ((uint32)~((uint32)(1u << THERM_USBIO_SUSPEND_DEL_SHIFT)))
    #define THERM_USBIO_CR1_OFF              ((uint32)0xfffffffeu)
#endif


/***************************************
*             Registers        
***************************************/
/* Main Port Registers */
#if defined(THERM__PC)
    /* Port Configuration */
    #define THERM_PC                 (* (reg32 *) THERM__PC)
#endif
/* Pin State */
#define THERM_PS                     (* (reg32 *) THERM__PS)
/* Data Register */
#define THERM_DR                     (* (reg32 *) THERM__DR)
/* Input Buffer Disable Override */
#define THERM_INP_DIS                (* (reg32 *) THERM__PC2)

/* Interrupt configuration Registers */
#define THERM_INTCFG                 (* (reg32 *) THERM__INTCFG)
#define THERM_INTSTAT                (* (reg32 *) THERM__INTSTAT)

/* "Interrupt cause" register for Combined Port Interrupt (AllPortInt) in GSRef component */
#if defined (CYREG_GPIO_INTR_CAUSE)
    #define THERM_INTR_CAUSE         (* (reg32 *) CYREG_GPIO_INTR_CAUSE)
#endif

/* SIO register */
#if defined(THERM__SIO)
    #define THERM_SIO_REG            (* (reg32 *) THERM__SIO)
#endif /* (THERM__SIO_CFG) */

/* USBIO registers */
#if !defined(THERM__PC) && (CY_PSOC4_4200L)
    #define THERM_USB_POWER_REG       (* (reg32 *) CYREG_USBDEVv2_USB_POWER_CTRL)
    #define THERM_CR1_REG             (* (reg32 *) CYREG_USBDEVv2_CR1)
    #define THERM_USBIO_CTRL_REG      (* (reg32 *) CYREG_USBDEVv2_USB_USBIO_CTRL)
#endif    
    
    
/***************************************
* The following code is DEPRECATED and 
* must not be used in new designs.
***************************************/
/**
* \addtogroup group_deprecated
* @{
*/
#define THERM_DRIVE_MODE_SHIFT       (0x00u)
#define THERM_DRIVE_MODE_MASK        (0x07u << THERM_DRIVE_MODE_SHIFT)
/** @} deprecated */

#endif /* End Pins THERM_H */


/* [] END OF FILE */
