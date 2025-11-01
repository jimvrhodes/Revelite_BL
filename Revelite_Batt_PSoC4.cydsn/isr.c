// File: isr.c
// Project: Revelite battery
//
//
#include "project.h"
#include "main.h"
#include "buttons.h"
#include <math.h>
#include <CyFlash.h>
#include <stdio.h>

// EEPROM FLASH BLOCKs
#define INFO_ROW       	(CY_FLASH_NUMBER_ROWS - 5u)
#define INFO_ADDR      	(INFO_ROW * CY_FLASH_SIZEOF_ROW)
//
extern structInfo Info;
//
//    
#define BLINKTIME 500
uint16 uiBlinkTimer = BLINKTIME;
volatile uint16 uiWriteEEPROMTimer;
volatile uint32 ulUARTTempOutputTimer;
volatile uint16 uiWDResetTimer = WDRESETTIME;
volatile uint16 uiButtonTimer[NUMBUTTONS];
volatile uint16 uiButton_Poll_Timer;
volatile uint32 uiAuto_Off_Timer;  // Auto-off timer in ms
volatile uint16 uiLED_Display_Timer;  // LED indicator display timer in ms
//
// timer ISR at 1ms
CY_ISR(fnTIMER_ISR) {
            
    if(uiBlinkTimer)
        uiBlinkTimer--;
    
    if(uiWriteEEPROMTimer)
        uiWriteEEPROMTimer--;
        
    if(ulUARTTempOutputTimer)
        ulUARTTempOutputTimer--;
    
    if(uiButton_Poll_Timer)
        uiButton_Poll_Timer--;
    
    if(uiAuto_Off_Timer)
        uiAuto_Off_Timer--;
    
    if(uiLED_Display_Timer)
        uiLED_Display_Timer--;
    
    for(uint8 n=0;n<NUMBUTTONS;n++) { // button timers
        if(uiButtonTimer[n])
            uiButtonTimer[n]--;
    }
        
    if(uiWDResetTimer) // reset here if I2C goes down
        uiWDResetTimer--;
    else {
#ifdef WDRESETON               
        CySoftwareReset();   
#endif 
    }
}
//
volatile uint16 uiPWMTarget1 = PWMOFF;
volatile uint16 uiPWMTarget2 = PWMOFF;
extern float flSmoothFilter;
//
CY_ISR(DIM_ISR_Handler) {
    static uint16 uiPWMval1 = 0;
    static float flPWMval1 = 0;
    static uint16 uiPWMval2 = 0;
    static float flPWMval2 = 0;
    
//    if(flPWMval1 != 0)
        flPWMval1 = ((flPWMval1 / flSmoothFilter) + ((float)uiPWMTarget1 - ((float)uiPWMTarget1 / flSmoothFilter)));
    
	if(isnan(flPWMval1))
	    flPWMval1 = uiPWMval1;
    
    uiPWMval1 = (uint16)flPWMval1;
    
    if(uiPWMval1 >= PWMPERIOD)
        uiPWMval1 = PWMPERIOD-1;

	PWM_1_WriteCompareBuf((uint16)(float)uiPWMval1);  // channel 1 and master and current

    
//    if(flPWMval2 != 0)
        flPWMval2 = ((flPWMval2 / flSmoothFilter) + ((float)uiPWMTarget2 - ((float)uiPWMTarget2 / flSmoothFilter)));
    
	if(isnan(flPWMval2))
	    flPWMval2 = uiPWMval2;
    
    uiPWMval2 = (uint16)flPWMval2;
    
    if(uiPWMval2 >= PWMPERIOD)
        uiPWMval2 = PWMPERIOD-1;

    PWM_2_WriteCompareBuf((uint16)(float)uiPWMval2); // channel 2 and master and current

	// Clear the interrupt flag so that the interrupt does not keep retriggering until the PWM is ready
    PWM_1_ClearInterrupt(PWM_1_INTR_MASK_TC);
}


// used by the ADC   
extern volatile uint16 result[ADC_TOTAL_CHANNELS_NUM];
extern volatile bool bDataReady;
CY_ISR(ADC_ISR_LOC) {
    uint32 range_status;

    /* Check for End of Scan interrupt */
    if(ADC_SAR_INTR_MASKED_REG & ADC_EOS_MASK) {
        /* Read range detect status */
        range_status = ADC_SAR_RANGE_INTR_MASKED_REG;
        /* Verify that the conversion result met the condition Low_Limit <= Result < High_Limit  */
// why?       if((range_status & (uint32)(1ul << 0)) != 0u) {
            // Read conversion results
            result[e_VCHARGER] = ADC_GetResult16(e_VCHARGER);       // input charger volts
            result[e_VBUS] = ADC_GetResult16(e_VBUS);               // USB volts            
            result[e_TEMP] = ADC_GetResult16(e_TEMP);             // local thermal
            result[e_BATTTEMP] = ADC_GetResult16(e_BATTTEMP);       // battery thermal through mux system                            
            result[e_BATTVSENSE] = ADC_GetResult16(e_BATTVSENSE);   // battery voltage
                            
            bDataReady = true;
//        }    
        /* Clear range detect status */
        ADC_SAR_RANGE_INTR_REG = range_status;
//        dataReady |= ADC_EOS_MASK;
    }
	
    /* Clear handled interrupt */
    ADC_SAR_INTR_REG &= ADC_EOS_MASK;
}

