/*******************************************************************************
* File Name: ACTLED.h  
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

#if !defined(CY_PINS_ACTLED_ALIASES_H) /* Pins ACTLED_ALIASES_H */
#define CY_PINS_ACTLED_ALIASES_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"


/***************************************
*              Constants        
***************************************/
#define ACTLED_0			(ACTLED__0__PC)
#define ACTLED_0_PS		(ACTLED__0__PS)
#define ACTLED_0_PC		(ACTLED__0__PC)
#define ACTLED_0_DR		(ACTLED__0__DR)
#define ACTLED_0_SHIFT	(ACTLED__0__SHIFT)
#define ACTLED_0_INTR	((uint16)((uint16)0x0003u << (ACTLED__0__SHIFT*2u)))

#define ACTLED_INTR_ALL	 ((uint16)(ACTLED_0_INTR))


#endif /* End Pins ACTLED_ALIASES_H */


/* [] END OF FILE */
