/*******************************************************************************
* File Name: ERRLED.h  
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

#if !defined(CY_PINS_ERRLED_ALIASES_H) /* Pins ERRLED_ALIASES_H */
#define CY_PINS_ERRLED_ALIASES_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"


/***************************************
*              Constants        
***************************************/
#define ERRLED_0			(ERRLED__0__PC)
#define ERRLED_0_PS		(ERRLED__0__PS)
#define ERRLED_0_PC		(ERRLED__0__PC)
#define ERRLED_0_DR		(ERRLED__0__DR)
#define ERRLED_0_SHIFT	(ERRLED__0__SHIFT)
#define ERRLED_0_INTR	((uint16)((uint16)0x0003u << (ERRLED__0__SHIFT*2u)))

#define ERRLED_INTR_ALL	 ((uint16)(ERRLED_0_INTR))


#endif /* End Pins ERRLED_ALIASES_H */


/* [] END OF FILE */
