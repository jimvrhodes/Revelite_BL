/*******************************************************************************
* File Name: CHRG_OK.h  
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

#if !defined(CY_PINS_CHRG_OK_ALIASES_H) /* Pins CHRG_OK_ALIASES_H */
#define CY_PINS_CHRG_OK_ALIASES_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"


/***************************************
*              Constants        
***************************************/
#define CHRG_OK_0			(CHRG_OK__0__PC)
#define CHRG_OK_0_PS		(CHRG_OK__0__PS)
#define CHRG_OK_0_PC		(CHRG_OK__0__PC)
#define CHRG_OK_0_DR		(CHRG_OK__0__DR)
#define CHRG_OK_0_SHIFT	(CHRG_OK__0__SHIFT)
#define CHRG_OK_0_INTR	((uint16)((uint16)0x0003u << (CHRG_OK__0__SHIFT*2u)))

#define CHRG_OK_INTR_ALL	 ((uint16)(CHRG_OK_0_INTR))


#endif /* End Pins CHRG_OK_ALIASES_H */


/* [] END OF FILE */
