/*******************************************************************************
* File Name: THERM.h  
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

#if !defined(CY_PINS_THERM_ALIASES_H) /* Pins THERM_ALIASES_H */
#define CY_PINS_THERM_ALIASES_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"


/***************************************
*              Constants        
***************************************/
#define THERM_0			(THERM__0__PC)
#define THERM_0_PS		(THERM__0__PS)
#define THERM_0_PC		(THERM__0__PC)
#define THERM_0_DR		(THERM__0__DR)
#define THERM_0_SHIFT	(THERM__0__SHIFT)
#define THERM_0_INTR	((uint16)((uint16)0x0003u << (THERM__0__SHIFT*2u)))

#define THERM_INTR_ALL	 ((uint16)(THERM_0_INTR))


#endif /* End Pins THERM_ALIASES_H */


/* [] END OF FILE */
