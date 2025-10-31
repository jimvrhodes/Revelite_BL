/*******************************************************************************
* File Name: VBUS_SENSE.h  
* Version 2.20
*
* Description:
*  This file contains the Alias definitions for Per-Pin APIs in cypins.h. 
*  Information on using these APIs can be found in the System Reference Guide.
*
* Note:
*
********************************************************************************
* Copyright 2008-2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_PINS_VBUS_SENSE_ALIASES_H) /* Pins VBUS_SENSE_ALIASES_H */
#define CY_PINS_VBUS_SENSE_ALIASES_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"


/***************************************
*              Constants        
***************************************/
#define VBUS_SENSE_0			(VBUS_SENSE__0__PC)
#define VBUS_SENSE_0_PS		(VBUS_SENSE__0__PS)
#define VBUS_SENSE_0_PC		(VBUS_SENSE__0__PC)
#define VBUS_SENSE_0_DR		(VBUS_SENSE__0__DR)
#define VBUS_SENSE_0_SHIFT	(VBUS_SENSE__0__SHIFT)
#define VBUS_SENSE_0_INTR	((uint16)((uint16)0x0003u << (VBUS_SENSE__0__SHIFT*2u)))

#define VBUS_SENSE_INTR_ALL	 ((uint16)(VBUS_SENSE_0_INTR))


#endif /* End Pins VBUS_SENSE_ALIASES_H */


/* [] END OF FILE */
