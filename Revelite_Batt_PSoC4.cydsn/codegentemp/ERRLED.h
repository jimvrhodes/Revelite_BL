/*******************************************************************************
* File Name: ERRLED.h  
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

#if !defined(CY_PINS_ERRLED_H) /* Pins ERRLED_H */
#define CY_PINS_ERRLED_H

#include "cytypes.h"
#include "cyfitter.h"
#include "ERRLED_aliases.h"


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
} ERRLED_BACKUP_STRUCT;

/** @} structures */


/***************************************
*        Function Prototypes             
***************************************/
/**
* \addtogroup group_general
* @{
*/
uint8   ERRLED_Read(void);
void    ERRLED_Write(uint8 value);
uint8   ERRLED_ReadDataReg(void);
#if defined(ERRLED__PC) || (CY_PSOC4_4200L) 
    void    ERRLED_SetDriveMode(uint8 mode);
#endif
void    ERRLED_SetInterruptMode(uint16 position, uint16 mode);
uint8   ERRLED_ClearInterrupt(void);
/** @} general */

/**
* \addtogroup group_power
* @{
*/
void ERRLED_Sleep(void); 
void ERRLED_Wakeup(void);
/** @} power */


/***************************************
*           API Constants        
***************************************/
#if defined(ERRLED__PC) || (CY_PSOC4_4200L) 
    /* Drive Modes */
    #define ERRLED_DRIVE_MODE_BITS        (3)
    #define ERRLED_DRIVE_MODE_IND_MASK    (0xFFFFFFFFu >> (32 - ERRLED_DRIVE_MODE_BITS))

    /**
    * \addtogroup group_constants
    * @{
    */
        /** \addtogroup driveMode Drive mode constants
         * \brief Constants to be passed as "mode" parameter in the ERRLED_SetDriveMode() function.
         *  @{
         */
        #define ERRLED_DM_ALG_HIZ         (0x00u) /**< \brief High Impedance Analog   */
        #define ERRLED_DM_DIG_HIZ         (0x01u) /**< \brief High Impedance Digital  */
        #define ERRLED_DM_RES_UP          (0x02u) /**< \brief Resistive Pull Up       */
        #define ERRLED_DM_RES_DWN         (0x03u) /**< \brief Resistive Pull Down     */
        #define ERRLED_DM_OD_LO           (0x04u) /**< \brief Open Drain, Drives Low  */
        #define ERRLED_DM_OD_HI           (0x05u) /**< \brief Open Drain, Drives High */
        #define ERRLED_DM_STRONG          (0x06u) /**< \brief Strong Drive            */
        #define ERRLED_DM_RES_UPDWN       (0x07u) /**< \brief Resistive Pull Up/Down  */
        /** @} driveMode */
    /** @} group_constants */
#endif

/* Digital Port Constants */
#define ERRLED_MASK               ERRLED__MASK
#define ERRLED_SHIFT              ERRLED__SHIFT
#define ERRLED_WIDTH              1u

/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup intrMode Interrupt constants
     * \brief Constants to be passed as "mode" parameter in ERRLED_SetInterruptMode() function.
     *  @{
     */
        #define ERRLED_INTR_NONE      ((uint16)(0x0000u)) /**< \brief Disabled             */
        #define ERRLED_INTR_RISING    ((uint16)(0x5555u)) /**< \brief Rising edge trigger  */
        #define ERRLED_INTR_FALLING   ((uint16)(0xaaaau)) /**< \brief Falling edge trigger */
        #define ERRLED_INTR_BOTH      ((uint16)(0xffffu)) /**< \brief Both edge trigger    */
    /** @} intrMode */
/** @} group_constants */

/* SIO LPM definition */
#if defined(ERRLED__SIO)
    #define ERRLED_SIO_LPM_MASK       (0x03u)
#endif

/* USBIO definitions */
#if !defined(ERRLED__PC) && (CY_PSOC4_4200L)
    #define ERRLED_USBIO_ENABLE               ((uint32)0x80000000u)
    #define ERRLED_USBIO_DISABLE              ((uint32)(~ERRLED_USBIO_ENABLE))
    #define ERRLED_USBIO_SUSPEND_SHIFT        CYFLD_USBDEVv2_USB_SUSPEND__OFFSET
    #define ERRLED_USBIO_SUSPEND_DEL_SHIFT    CYFLD_USBDEVv2_USB_SUSPEND_DEL__OFFSET
    #define ERRLED_USBIO_ENTER_SLEEP          ((uint32)((1u << ERRLED_USBIO_SUSPEND_SHIFT) \
                                                        | (1u << ERRLED_USBIO_SUSPEND_DEL_SHIFT)))
    #define ERRLED_USBIO_EXIT_SLEEP_PH1       ((uint32)~((uint32)(1u << ERRLED_USBIO_SUSPEND_SHIFT)))
    #define ERRLED_USBIO_EXIT_SLEEP_PH2       ((uint32)~((uint32)(1u << ERRLED_USBIO_SUSPEND_DEL_SHIFT)))
    #define ERRLED_USBIO_CR1_OFF              ((uint32)0xfffffffeu)
#endif


/***************************************
*             Registers        
***************************************/
/* Main Port Registers */
#if defined(ERRLED__PC)
    /* Port Configuration */
    #define ERRLED_PC                 (* (reg32 *) ERRLED__PC)
#endif
/* Pin State */
#define ERRLED_PS                     (* (reg32 *) ERRLED__PS)
/* Data Register */
#define ERRLED_DR                     (* (reg32 *) ERRLED__DR)
/* Input Buffer Disable Override */
#define ERRLED_INP_DIS                (* (reg32 *) ERRLED__PC2)

/* Interrupt configuration Registers */
#define ERRLED_INTCFG                 (* (reg32 *) ERRLED__INTCFG)
#define ERRLED_INTSTAT                (* (reg32 *) ERRLED__INTSTAT)

/* "Interrupt cause" register for Combined Port Interrupt (AllPortInt) in GSRef component */
#if defined (CYREG_GPIO_INTR_CAUSE)
    #define ERRLED_INTR_CAUSE         (* (reg32 *) CYREG_GPIO_INTR_CAUSE)
#endif

/* SIO register */
#if defined(ERRLED__SIO)
    #define ERRLED_SIO_REG            (* (reg32 *) ERRLED__SIO)
#endif /* (ERRLED__SIO_CFG) */

/* USBIO registers */
#if !defined(ERRLED__PC) && (CY_PSOC4_4200L)
    #define ERRLED_USB_POWER_REG       (* (reg32 *) CYREG_USBDEVv2_USB_POWER_CTRL)
    #define ERRLED_CR1_REG             (* (reg32 *) CYREG_USBDEVv2_CR1)
    #define ERRLED_USBIO_CTRL_REG      (* (reg32 *) CYREG_USBDEVv2_USB_USBIO_CTRL)
#endif    
    
    
/***************************************
* The following code is DEPRECATED and 
* must not be used in new designs.
***************************************/
/**
* \addtogroup group_deprecated
* @{
*/
#define ERRLED_DRIVE_MODE_SHIFT       (0x00u)
#define ERRLED_DRIVE_MODE_MASK        (0x07u << ERRLED_DRIVE_MODE_SHIFT)
/** @} deprecated */

#endif /* End Pins ERRLED_H */


/* [] END OF FILE */
