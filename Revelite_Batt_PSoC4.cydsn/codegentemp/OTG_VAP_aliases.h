/*******************************************************************************
* File Name: OTG_VAP.h  
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

#if !defined(CY_PINS_OTG_VAP_ALIASES_H) /* Pins OTG_VAP_ALIASES_H */
#define CY_PINS_OTG_VAP_ALIASES_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"


/***************************************
*              Constants        
***************************************/
#define OTG_VAP_0			(OTG_VAP__0__PC)
#define OTG_VAP_0_PS		(OTG_VAP__0__PS)
#define OTG_VAP_0_PC		(OTG_VAP__0__PC)
#define OTG_VAP_0_DR		(OTG_VAP__0__DR)
#define OTG_VAP_0_SHIFT	(OTG_VAP__0__SHIFT)
#define OTG_VAP_0_INTR	((uint16)((uint16)0x0003u << (OTG_VAP__0__SHIFT*2u)))

#define OTG_VAP_INTR_ALL	 ((uint16)(OTG_VAP_0_INTR))


#endif /* End Pins OTG_VAP_ALIASES_H */


/* [] END OF FILE */
