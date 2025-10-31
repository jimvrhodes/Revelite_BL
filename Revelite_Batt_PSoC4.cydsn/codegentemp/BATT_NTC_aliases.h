/*******************************************************************************
* File Name: BATT_NTC.h  
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

#if !defined(CY_PINS_BATT_NTC_ALIASES_H) /* Pins BATT_NTC_ALIASES_H */
#define CY_PINS_BATT_NTC_ALIASES_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"


/***************************************
*              Constants        
***************************************/
#define BATT_NTC_0			(BATT_NTC__0__PC)
#define BATT_NTC_0_PS		(BATT_NTC__0__PS)
#define BATT_NTC_0_PC		(BATT_NTC__0__PC)
#define BATT_NTC_0_DR		(BATT_NTC__0__DR)
#define BATT_NTC_0_SHIFT	(BATT_NTC__0__SHIFT)
#define BATT_NTC_0_INTR	((uint16)((uint16)0x0003u << (BATT_NTC__0__SHIFT*2u)))

#define BATT_NTC_INTR_ALL	 ((uint16)(BATT_NTC_0_INTR))


#endif /* End Pins BATT_NTC_ALIASES_H */


/* [] END OF FILE */
