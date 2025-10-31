/*******************************************************************************
* File Name: OC_FLAG.h  
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

#if !defined(CY_PINS_OC_FLAG_ALIASES_H) /* Pins OC_FLAG_ALIASES_H */
#define CY_PINS_OC_FLAG_ALIASES_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"


/***************************************
*              Constants        
***************************************/
#define OC_FLAG_0			(OC_FLAG__0__PC)
#define OC_FLAG_0_PS		(OC_FLAG__0__PS)
#define OC_FLAG_0_PC		(OC_FLAG__0__PC)
#define OC_FLAG_0_DR		(OC_FLAG__0__DR)
#define OC_FLAG_0_SHIFT	(OC_FLAG__0__SHIFT)
#define OC_FLAG_0_INTR	((uint16)((uint16)0x0003u << (OC_FLAG__0__SHIFT*2u)))

#define OC_FLAG_INTR_ALL	 ((uint16)(OC_FLAG_0_INTR))


#endif /* End Pins OC_FLAG_ALIASES_H */


/* [] END OF FILE */
