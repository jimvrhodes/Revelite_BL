/*******************************************************************************
* File Name: OTG_VAP.h  
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

#if !defined(CY_PINS_OTG_VAP_H) /* Pins OTG_VAP_H */
#define CY_PINS_OTG_VAP_H

#include "cytypes.h"
#include "cyfitter.h"
#include "OTG_VAP_aliases.h"


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
} OTG_VAP_BACKUP_STRUCT;

/** @} structures */


/***************************************
*        Function Prototypes             
***************************************/
/**
* \addtogroup group_general
* @{
*/
uint8   OTG_VAP_Read(void);
void    OTG_VAP_Write(uint8 value);
uint8   OTG_VAP_ReadDataReg(void);
#if defined(OTG_VAP__PC) || (CY_PSOC4_4200L) 
    void    OTG_VAP_SetDriveMode(uint8 mode);
#endif
void    OTG_VAP_SetInterruptMode(uint16 position, uint16 mode);
uint8   OTG_VAP_ClearInterrupt(void);
/** @} general */

/**
* \addtogroup group_power
* @{
*/
void OTG_VAP_Sleep(void); 
void OTG_VAP_Wakeup(void);
/** @} power */


/***************************************
*           API Constants        
***************************************/
#if defined(OTG_VAP__PC) || (CY_PSOC4_4200L) 
    /* Drive Modes */
    #define OTG_VAP_DRIVE_MODE_BITS        (3)
    #define OTG_VAP_DRIVE_MODE_IND_MASK    (0xFFFFFFFFu >> (32 - OTG_VAP_DRIVE_MODE_BITS))

    /**
    * \addtogroup group_constants
    * @{
    */
        /** \addtogroup driveMode Drive mode constants
         * \brief Constants to be passed as "mode" parameter in the OTG_VAP_SetDriveMode() function.
         *  @{
         */
        #define OTG_VAP_DM_ALG_HIZ         (0x00u) /**< \brief High Impedance Analog   */
        #define OTG_VAP_DM_DIG_HIZ         (0x01u) /**< \brief High Impedance Digital  */
        #define OTG_VAP_DM_RES_UP          (0x02u) /**< \brief Resistive Pull Up       */
        #define OTG_VAP_DM_RES_DWN         (0x03u) /**< \brief Resistive Pull Down     */
        #define OTG_VAP_DM_OD_LO           (0x04u) /**< \brief Open Drain, Drives Low  */
        #define OTG_VAP_DM_OD_HI           (0x05u) /**< \brief Open Drain, Drives High */
        #define OTG_VAP_DM_STRONG          (0x06u) /**< \brief Strong Drive            */
        #define OTG_VAP_DM_RES_UPDWN       (0x07u) /**< \brief Resistive Pull Up/Down  */
        /** @} driveMode */
    /** @} group_constants */
#endif

/* Digital Port Constants */
#define OTG_VAP_MASK               OTG_VAP__MASK
#define OTG_VAP_SHIFT              OTG_VAP__SHIFT
#define OTG_VAP_WIDTH              1u

/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup intrMode Interrupt constants
     * \brief Constants to be passed as "mode" parameter in OTG_VAP_SetInterruptMode() function.
     *  @{
     */
        #define OTG_VAP_INTR_NONE      ((uint16)(0x0000u)) /**< \brief Disabled             */
        #define OTG_VAP_INTR_RISING    ((uint16)(0x5555u)) /**< \brief Rising edge trigger  */
        #define OTG_VAP_INTR_FALLING   ((uint16)(0xaaaau)) /**< \brief Falling edge trigger */
        #define OTG_VAP_INTR_BOTH      ((uint16)(0xffffu)) /**< \brief Both edge trigger    */
    /** @} intrMode */
/** @} group_constants */

/* SIO LPM definition */
#if defined(OTG_VAP__SIO)
    #define OTG_VAP_SIO_LPM_MASK       (0x03u)
#endif

/* USBIO definitions */
#if !defined(OTG_VAP__PC) && (CY_PSOC4_4200L)
    #define OTG_VAP_USBIO_ENABLE               ((uint32)0x80000000u)
    #define OTG_VAP_USBIO_DISABLE              ((uint32)(~OTG_VAP_USBIO_ENABLE))
    #define OTG_VAP_USBIO_SUSPEND_SHIFT        CYFLD_USBDEVv2_USB_SUSPEND__OFFSET
    #define OTG_VAP_USBIO_SUSPEND_DEL_SHIFT    CYFLD_USBDEVv2_USB_SUSPEND_DEL__OFFSET
    #define OTG_VAP_USBIO_ENTER_SLEEP          ((uint32)((1u << OTG_VAP_USBIO_SUSPEND_SHIFT) \
                                                        | (1u << OTG_VAP_USBIO_SUSPEND_DEL_SHIFT)))
    #define OTG_VAP_USBIO_EXIT_SLEEP_PH1       ((uint32)~((uint32)(1u << OTG_VAP_USBIO_SUSPEND_SHIFT)))
    #define OTG_VAP_USBIO_EXIT_SLEEP_PH2       ((uint32)~((uint32)(1u << OTG_VAP_USBIO_SUSPEND_DEL_SHIFT)))
    #define OTG_VAP_USBIO_CR1_OFF              ((uint32)0xfffffffeu)
#endif


/***************************************
*             Registers        
***************************************/
/* Main Port Registers */
#if defined(OTG_VAP__PC)
    /* Port Configuration */
    #define OTG_VAP_PC                 (* (reg32 *) OTG_VAP__PC)
#endif
/* Pin State */
#define OTG_VAP_PS                     (* (reg32 *) OTG_VAP__PS)
/* Data Register */
#define OTG_VAP_DR                     (* (reg32 *) OTG_VAP__DR)
/* Input Buffer Disable Override */
#define OTG_VAP_INP_DIS                (* (reg32 *) OTG_VAP__PC2)

/* Interrupt configuration Registers */
#define OTG_VAP_INTCFG                 (* (reg32 *) OTG_VAP__INTCFG)
#define OTG_VAP_INTSTAT                (* (reg32 *) OTG_VAP__INTSTAT)

/* "Interrupt cause" register for Combined Port Interrupt (AllPortInt) in GSRef component */
#if defined (CYREG_GPIO_INTR_CAUSE)
    #define OTG_VAP_INTR_CAUSE         (* (reg32 *) CYREG_GPIO_INTR_CAUSE)
#endif

/* SIO register */
#if defined(OTG_VAP__SIO)
    #define OTG_VAP_SIO_REG            (* (reg32 *) OTG_VAP__SIO)
#endif /* (OTG_VAP__SIO_CFG) */

/* USBIO registers */
#if !defined(OTG_VAP__PC) && (CY_PSOC4_4200L)
    #define OTG_VAP_USB_POWER_REG       (* (reg32 *) CYREG_USBDEVv2_USB_POWER_CTRL)
    #define OTG_VAP_CR1_REG             (* (reg32 *) CYREG_USBDEVv2_CR1)
    #define OTG_VAP_USBIO_CTRL_REG      (* (reg32 *) CYREG_USBDEVv2_USB_USBIO_CTRL)
#endif    
    
    
/***************************************
* The following code is DEPRECATED and 
* must not be used in new designs.
***************************************/
/**
* \addtogroup group_deprecated
* @{
*/
#define OTG_VAP_DRIVE_MODE_SHIFT       (0x00u)
#define OTG_VAP_DRIVE_MODE_MASK        (0x07u << OTG_VAP_DRIVE_MODE_SHIFT)
/** @} deprecated */

#endif /* End Pins OTG_VAP_H */


/* [] END OF FILE */
