// File: main.c
// Project: Cerno Revelite BL (nattery) and charger 
// Current Version 1.00
//
// TODO
//
// Revision History
// 1.00B0		09-29-2025	First compile
//
//                          
//
// TODO
// 1) 
// 2) 
//
//
/*****************************************************************************
* Copyright (2019-2025), Newton Engineering and Design Group LLC (The NED Group)
******************************************************************************
* This software is owned by The NED Group (NED) and is
* protected by and subject to worldwide patent protection (United States and
* foreign), United States copyright laws and international treaty provisions.
* NED hereby grants to licensee a personal, non-exclusive, non-transferable
* license to copy, use, modify, create derivative works of, and compile the
* NED Source Code and derivative works for the sole purpose of creating
* custom software in support of licensee product to be used only in conjunction
* with a NED product or circuit as specified in the applicable agreement.
* Any reproduction, modification, translation, compilation, or representation of
* this software except as specified above is prohibited without the express
* written permission of NED.

* This file contains Source Code. You may not use this file without written
* authorization from NED.
*
* Please see the License and Usage Agreement for the specific language governing rights
* and limitations under the Agreement.
*
*
* Disclaimer: NED MAKES NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, WITH
* REGARD TO THIS MATERIAL, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
* NED reserves the right to make changes without further notice to the
* materials described herein. NED does not assume any liability arising out
* of the application or use of any product or circuit described herein. NED
* does not authorize its products for use as critical components in life-support
* systems where a malfunction or failure may reasonably be expected to result in
* significant injury to the user. The inclusion of NEDs product in a life-
* support systems application implies that the manufacturer assumes all risk of
* such use and in doing so indemnifies NED against all charges. Use may be
* limited by and subject to the applicable NED software license agreement.
*****************************************************************************/
//
//#include "project.h"
#include "main.h"
#include <math.h>
#include <buttons.h>
#include <battery.h>
#include <LTC2944.h>
#include <CLI.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
//
//
volatile structInfo Info;
volatile structButtonState ButtonState; // state of the guttons

//
extern volatile bool bWeHaveBooted;
extern uint16 uiBlinkTimer;
extern volatile uint16 uiWriteEEPROMTimer;
extern volatile uint32 ulUARTTempOutputTimer;
extern volatile uint16 uiWDResetTimer;
//
// ADC stuff
volatile bool bDataReady;
volatile uint16 result[ADC_TOTAL_CHANNELS_NUM];
extern bool bWeHaveSampledTEMP;

//
extern volatile uint16 uiPWMTarget1;  // channel 1
extern volatile uint16 uiPWMTarget2;  // channel 2
volatile float flSmoothFilter = FILTERSTART;
volatile float flSample;
volatile float flTempScalar = DEFAULTTEMPSCALAR; // this is the master temp scalar
volatile uint16 uiTemp = 0;
//
// UART stuff
volatile char byUARTBuffer[100];

// latch stuff
volatile uint8 byLatch1;
volatile uint8 byLatch2;

volatile uint16 uiChargerStatus;

volatile uint8_t byButtons = 0;

int main(void) {
        
    bool bWriteInfo = false;
    float fMasterScalar = 1.0f;
    bool bACTLEDIsOn = true;
    bool bWeNeedToStoreOffSaved = false;
    uint8 byLastButton = 0;

    
    PWM1_Write(0);
    PWM2_Write(0);
    
    // ms timer
    CyIntSetSysVector((SysTick_IRQn + 16), fnTIMER_ISR);
	SysTick_Config(48000000/1000);
    
    CyGlobalIntEnable; /* Enable global interrupts. */
    
    PWM_1_Start();
    PWM_1_WritePeriod(PWMPERIOD);
    PWM_1_WriteCompare(0);  // Force off
    PWM_2_Start();
    PWM_2_WritePeriod(PWMPERIOD);
    PWM_2_WriteCompare(0);  // Force off
    DIM_isr_StartEx(DIM_ISR_Handler);

  	RW_EEPROMData(e_EEPROMREAD); // 1st time is to set eeprom to default if first time boot

    uiPWMTarget1 = 0;  // Start at zero
    uiPWMTarget2 = 0;  // Start at zero
    flSmoothFilter = FILTERSTART;
    
    I2CM_Start();
    LatchWrite(LATCH1, PCLA9538_CONFIG, LATCH1CONFIG);
    LatchWrite(LATCH1, PCLA9538_PUPEN, LATCH1CONFIG);
    LatchWrite(LATCH2, PCLA9538_CONFIG, LATCH2CONFIG);
    byLatch1 = DEFAULTLATCH1;
    LatchWrite(LATCH1, PCLA9538_OUTREG, byLatch1);
    byLatch2 = DEFAULTLATCH2;
    LatchWrite(LATCH2, PCLA9538_OUTREG, byLatch2);
    
    // Initialize quadrature decoder  
    InitQuadDec();
    CyDelay(10);  // Give it time to settle
    QuadDec_WriteCounter(0);  // Force to zero again
    CyDelay(10);
    
    // turn on the boost
    byLatch2 |= BOOSTEN;
    LatchWrite(LATCH2, PCLA9538_OUTREG, byLatch2);
    
    // Initialize battery charger (BQ25730) for 4S LiPo
    if (BQ25730_Init()) {
        // Charger initialized successfully
        // Already configured for 16.8V max, 1A charge current
        uiChargerStatus = BQ25730_Read(BQ25730_CHARGER_STATUS);
    }
    
    // Initialize fuel gauge (LTC2944)
    // Use M_1024 prescaler for ~3500mAh battery
    LTC2944_Init((uint16_t)BATTERY_CAPACITY_MAH, LTC2944_PRESCALER_M_1024);
    
#ifdef DEBUGOUT
    UART_Start();
    CLI_Init();  // Initialize command line interface
#endif
    
    ADC_Start();
	ADC_StartConvert();
    ADC_IRQ_StartEx(ADC_ISR_LOC);
    
    ACTLED_Write(1);
    
#ifdef WDRESETON    
    CySysWdtWriteMode(CY_SYS_WDT_COUNTER0, CY_SYS_WDT_MODE_RESET); // watchdog
    uint32 ulLoad = (((uint32)((uint32)30000000 * ILO_FREQ))/1000);
    CySysWdtWriteMatch(CY_SYS_WDT_COUNTER0, ulLoad);
    CySysWdtWriteClearOnMatch(CY_SYS_WDT_COUNTER0, 1u);
    CySysWdtEnable(CY_SYS_WDT_COUNTER0_MASK);
    while(!CySysWdtReadEnabledStatus(CY_SYS_WDT_COUNTER0));
#endif
    
    for(;;) {
        
#ifdef WDRESETON        
        CySysWdtResetCounters(CY_SYS_WDT_COUNTER0_RESET); // reset watchdog, 1 second intervals
        uiWDResetTimer = WDRESETTIME; // in case we get stuck here
#endif       

        if(!uiBlinkTimer) {
            ACTLED_Write(~ACTLED_Read());
            uiBlinkTimer = BLINKTIME;
            
            if(ACTLED_Read()) {
                byLatch1 |= LED1;
            } else {
                byLatch1 &= ~LED1;
            }
            LatchWrite(LATCH1, PCLA9538_OUTREG, byLatch1);
        }
        
        byButtons = ReadButtonsOnChange(); // get button status
        if(byButtons != 0x00)  // generic indicator
            ERRLED_Write(1);
        else
            ERRLED_Write(0);

        if(byButtons & BUTTON1)
            byLatch1 |= LED2;
        else
            byLatch1 &= ~LED2;
        
        if(byButtons & BUTTON2)
            byLatch1 |= LED3;
        else
            byLatch1 &= ~LED3;
            
        if(byButtons & BUTTON3)
            byLatch1 |= LED4;
        else
            byLatch1 &= ~LED4;
            
            
#ifdef DEBUGOUT
        // Process CLI input
        if (UART_SpiUartGetRxBufferSize() > 0) {
            uint8_t byte = UART_SpiUartReadRxData();
            CLI_ProcessByte((char)byte);
        }
#endif

        // Get brightness from quadrature decoder (0.0 to 1.0)
        float fBrightness = QuadDec_GetBrightnessScalar();
        
        // Clamp brightness to valid range
        if (fBrightness < 0.0f) fBrightness = 0.0f;
        if (fBrightness > 1.0f) fBrightness = 1.0f;
        
        uiPWMTarget1 = (uint16_t)(PWMMAX * fBrightness);
        uiPWMTarget2 = (uint16_t)(PWMMAX * fBrightness);

#ifdef DEBUGOUT
        // Heartbeat every 2 seconds
        static uint16_t heartbeat = 0;
        if (!heartbeat) {
            heartbeat = 2000;
            int16_t pos = QuadDec_GetPosition();
            uint16_t count = sprintf((char*)byUARTBuffer,"Pos:%d\n\r", pos);
            UART_SpiUartPutArray((uint8*)&byUARTBuffer, count);
            // Don't wait for TX to complete
        }
        if (heartbeat) heartbeat--;
#endif

        uiTemp = GetAverageTEMP(); // DRIVER temperature rollback?
        if(bWeHaveSampledTEMP) {
            if(uiTemp <= MINTEMPADC)
                uiTemp = MINTEMPADC;
            if(uiTemp >= MAXTEMPADC)
                uiTemp = MAXTEMPADC;
            flTempScalar = INTERPOLATE_FLOAT ((float)uiTemp, MINTEMPADC, MAXTEMPADC, MINTEMPSCALAR, MAXTEMPSCALAR);
        }
            

        
        
        
        
        if(bWriteInfo && !uiWriteEEPROMTimer) {
            bWriteInfo = false;
            RW_EEPROMData(e_EEPROMWRITE);
        }
    }
}

/* [] END OF FILE */
