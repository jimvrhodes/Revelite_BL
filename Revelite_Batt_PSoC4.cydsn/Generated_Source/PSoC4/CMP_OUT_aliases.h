/*******************************************************************************
* File Name: CMP_OUT.h  
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

#if !defined(CY_PINS_CMP_OUT_ALIASES_H) /* Pins CMP_OUT_ALIASES_H */
#define CY_PINS_CMP_OUT_ALIASES_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"


/***************************************
*              Constants        
***************************************/
#define CMP_OUT_0			(CMP_OUT__0__PC)
#define CMP_OUT_0_PS		(CMP_OUT__0__PS)
#define CMP_OUT_0_PC		(CMP_OUT__0__PC)
#define CMP_OUT_0_DR		(CMP_OUT__0__DR)
#define CMP_OUT_0_SHIFT	(CMP_OUT__0__SHIFT)
#define CMP_OUT_0_INTR	((uint16)((uint16)0x0003u << (CMP_OUT__0__SHIFT*2u)))

#define CMP_OUT_INTR_ALL	 ((uint16)(CMP_OUT_0_INTR))


#endif /* End Pins CMP_OUT_ALIASES_H */


/* [] END OF FILE */
