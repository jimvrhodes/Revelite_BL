/*******************************************************************************
* File Name: LATCH_INT.h  
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

#if !defined(CY_PINS_LATCH_INT_ALIASES_H) /* Pins LATCH_INT_ALIASES_H */
#define CY_PINS_LATCH_INT_ALIASES_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"


/***************************************
*              Constants        
***************************************/
#define LATCH_INT_0			(LATCH_INT__0__PC)
#define LATCH_INT_0_PS		(LATCH_INT__0__PS)
#define LATCH_INT_0_PC		(LATCH_INT__0__PC)
#define LATCH_INT_0_DR		(LATCH_INT__0__DR)
#define LATCH_INT_0_SHIFT	(LATCH_INT__0__SHIFT)
#define LATCH_INT_0_INTR	((uint16)((uint16)0x0003u << (LATCH_INT__0__SHIFT*2u)))

#define LATCH_INT_INTR_ALL	 ((uint16)(LATCH_INT_0_INTR))


#endif /* End Pins LATCH_INT_ALIASES_H */


/* [] END OF FILE */
