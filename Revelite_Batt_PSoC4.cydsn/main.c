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

// Battery monitoring
static uint16_t battery_update_timer = 0;
static float battery_soc = 100.0f;  // State of charge percentage
static uint16_t battery_voltage_mv = 0;
static bool battery_low_warning = false;

// Timer states for auto-off feature
typedef enum {
    TIMER_OFF = 0,      // No timer, run until battery exhausted
    TIMER_1HR,          // Auto off at 1 hour
    TIMER_3HR,          // Auto off at 3 hours
    TIMER_5HR,          // Auto off at 5 hours
    TIMER_MAX
} TimerState_t;

static TimerState_t timer_state = TIMER_OFF;
static uint32_t auto_off_timer = 0;  // Countdown in milliseconds

int main(void) {
        
    bool bWriteInfo = false;
    float fMasterScalar = 1.0f;
    bool bACTLEDIsOn = true;
    bool bWeNeedToStoreOffSaved = false;
    uint8 byLastButton = 0;
    bool bLEDsOn = true;  // Track LED on/off state
    float fLastBrightness = 0.0f;  // Store last brightness when turning off

    
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
#ifdef DEBUGOUT
    UART_SpiUartPutString("Init BQ25730...\r\n");
    // Try reading device ID first
    uint16_t dev_id = BQ25730_Read(BQ25730_DEVICE_ID);
    uint16_t mfg_id = BQ25730_Read(BQ25730_MANUFACTURER_ID);
    uint16_t cnt = sprintf((char*)byUARTBuffer,"BQ ID:0x%04X MFG:0x%04X\r\n", dev_id, mfg_id);
    UART_SpiUartPutArray((uint8*)&byUARTBuffer, cnt);
#endif
    bool chg_ok = BQ25730_Init();
#ifdef DEBUGOUT
    if(chg_ok) {
        UART_SpiUartPutString("BQ25730 OK\r\n");
    } else {
        UART_SpiUartPutString("BQ25730 FAIL\r\n");
    }
#endif
    
    // Initialize fuel gauge (LTC2944)
    // Use M_1024 prescaler for ~3500mAh battery
#ifdef DEBUGOUT
    UART_SpiUartPutString("Init LTC2944...\r\n");
#endif
    bool fg_ok = LTC2944_Init((uint16_t)BATTERY_CAPACITY_MAH, LTC2944_PRESCALER_M_1024);
#ifdef DEBUGOUT
    if(fg_ok) {
        UART_SpiUartPutString("LTC2944 OK\r\n");
    } else {
        UART_SpiUartPutString("LTC2944 FAIL\r\n");
    }
    uint8_t fg_ctrl = LTC2944_Read(LTC2944_REG_CONTROL);
    cnt = sprintf((char*)byUARTBuffer,"LTC2944 Ctrl: 0x%02X (expect 0xEA)\r\n", fg_ctrl);
    UART_SpiUartPutArray((uint8*)&byUARTBuffer, cnt);
#endif
    
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
        }
        
        byButtons = ReadButtonsOnChange(); // get button status
        if(byButtons != 0x00)  // generic indicator
            ERRLED_Write(1);
        else
            ERRLED_Write(0);

        // TODO: Battery monitoring disabled until charger/fuel gauge verified
        // Update battery status every 500ms
        //if(!battery_update_timer) {
        //    battery_update_timer = 500;
        //    
        //    // Read fuel gauge
        //    FuelGauge_t fg_status = LTC2944_GetStatus();
        //    battery_soc = fg_status.state_of_charge;
        //    battery_voltage_mv = (uint16_t)fg_status.voltage_mv;
        //    
        //    // Check for low battery condition
        //    if(battery_voltage_mv < BATTERY_LOW_CUTOFF_MV) {
        //        battery_low_warning = true;
        //    } else if(battery_voltage_mv > (BATTERY_LOW_CUTOFF_MV + 200)) {  // 200mV hysteresis
        //        battery_low_warning = false;
        //    }
        //    
        //    // Read charger status
        //    uiChargerStatus = BQ25730_Read(BQ25730_CHARGER_STATUS);
        //}
        //if(battery_update_timer) battery_update_timer--;

        // TODO: Battery charge indicator - disabled until battery monitoring working
        // Battery charge indicator using four LEDs on LATCH1
        // Display when BUTTON1 is pressed
        //if(byButtons & BUTTON1) {
        //    // Clear all battery LEDs first
        //    byLatch1 &= ~(LED1 | LED2 | LED3 | LED4);
        //    
        //    if(battery_low_warning) {
        //        // Flash single LED for critically low battery
        //        if(uiBlinkTimer < (BLINKTIME/2)) {
        //            byLatch1 |= LED1;
        //        }
        //    } else if(battery_soc < 25.0f) {
        //        // One LED - less than 25%
        //        byLatch1 |= LED1;
        //    } else if(battery_soc < 50.0f) {
        //        // Two LEDs - 25-50%
        //        byLatch1 |= (LED1 | LED2);
        //    } else if(battery_soc < 75.0f) {
        //        // Three LEDs - 50-75%
        //        byLatch1 |= (LED1 | LED2 | LED3);
        //    } else {
        //        // Four LEDs - 75-100%
        //        byLatch1 |= (LED1 | LED2 | LED3 | LED4);
        //    }
        //}
        
        // Timer indicator - cycle through timer states with BUTTON2
        // LED1 - no timer, just run until battery is exhausted
        // LED2 - automatic off at 1 hour
        // LED3 - automatic off at 3 hours
        // LED4 - automatic off at 5 hours
        if((byButtons & BUTTON2) && !(byLastButton & BUTTON2)) {
            // Button just pressed - cycle to next timer state
            timer_state++;
            if(timer_state >= TIMER_MAX) {
                timer_state = TIMER_OFF;
            }
            
            // Set timer based on state
            switch(timer_state) {
                case TIMER_OFF:
                    auto_off_timer = 0;
                    break;
                case TIMER_1HR:
                    auto_off_timer = 60UL * 60UL * 1000UL;  // 1 hour in ms
                    break;
                case TIMER_3HR:
                    auto_off_timer = 3UL * 60UL * 60UL * 1000UL;  // 3 hours in ms
                    break;
                case TIMER_5HR:
                    auto_off_timer = 5UL * 60UL * 60UL * 1000UL;  // 5 hours in ms
                    break;
                default:
                    timer_state = TIMER_OFF;
                    auto_off_timer = 0;
                    break;
            }
        }
        
        // Display timer state on LEDs when not showing battery
        if(!(byButtons & BUTTON1)) {
            // Clear timer LEDs
            byLatch1 &= ~(LED2 | LED3 | LED4);
            
            switch(timer_state) {
                case TIMER_1HR:
                    byLatch1 |= LED2;
                    break;
                case TIMER_3HR:
                    byLatch1 |= LED3;
                    break;
                case TIMER_5HR:
                    byLatch1 |= LED4;
                    break;
                case TIMER_OFF:
                default:
                    // No LED for timer off
                    break;
            }
        }
        
        // Write the latch with battery/timer LED updates
        LatchWrite(LATCH1, PCLA9538_OUTREG, byLatch1);
            
        // this is our ON/OFF button - toggle LEDs
        if(bLEDsOn == true) {
            if(ButtonState.byButtonLong & BUTTON3)
                bLEDsOn = false;
        } else if( (byButtons & BUTTON3) && (byButtons != byLastButton)) {
            bLEDsOn = true;
        }        
        
        byLastButton = byButtons;
            
            
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
        
        // Auto-off timer is decremented by the 1ms timer ISR, not here
        // Check if timer expired
        if(bLEDsOn && timer_state != TIMER_OFF && auto_off_timer == 0) {
            bLEDsOn = false;  // Auto turn off
        }
        
        // TODO: Re-enable when battery monitoring is working correctly
        // Check for low battery - force off if too low (only on transition)
        //static bool was_low_warning = false;
        //if(battery_low_warning && !was_low_warning && bLEDsOn) {
        //    bLEDsOn = false;
        //}
        //was_low_warning = battery_low_warning;
        
        // Apply ON/OFF control
        if (bLEDsOn) {
            // LEDs are ON - use the brightness from encoder
            uiPWMTarget1 = (uint16_t)(PWMMAX * fBrightness);
            uiPWMTarget2 = (uint16_t)(PWMMAX * fBrightness);
            fLastBrightness = fBrightness;  // Save for when we turn back on
        } else {
            // LEDs are OFF - set brightness to 0
            uiPWMTarget1 = 0;
            uiPWMTarget2 = 0;
        }

#ifdef DEBUGOUT
        // Heartbeat every 2 seconds - test charger and fuel gauge
        static uint16_t heartbeat = 0;
        static bool did_i2c_scan = false;
        
        // Do I2C scan once on startup
        if(!did_i2c_scan) {
            did_i2c_scan = true;
            uint16_t count = sprintf((char*)byUARTBuffer,"I2C Scan: ");
            UART_SpiUartPutArray((uint8*)&byUARTBuffer, count);
            
            for(uint8_t addr = 0x08; addr < 0x78; addr++) {
                I2CM_I2CMasterClearStatus();
                uint32_t status = I2CM_I2CMasterSendStart(addr, I2CM_I2C_WRITE_XFER_MODE, 100);
                I2CM_I2CMasterSendStop(25);
                if(status == I2CM_I2C_MSTR_NO_ERROR) {
                    count = sprintf((char*)byUARTBuffer,"0x%02X ", addr);
                    UART_SpiUartPutArray((uint8*)&byUARTBuffer, count);
                }
            }
            count = sprintf((char*)byUARTBuffer,"\n\r");
            UART_SpiUartPutArray((uint8*)&byUARTBuffer, count);
        }
        
        if (!heartbeat) {
            heartbeat = 2000;
            
            // Test BQ25730 charger
            uint16_t chg_status = BQ25730_Read(BQ25730_CHARGER_STATUS);
            uint16_t chg_voltage = BQ25730_GetBatteryVoltage_mV();
            
            // Test LTC2944 fuel gauge - calculate voltage properly
            uint16_t fg_voltage_raw = LTC2944_Read16(LTC2944_REG_VOLTAGE_MSB);
            uint16_t fg_voltage_mv = (uint16_t)((float)fg_voltage_raw * 1.44f);  // 1.44mV per LSB
            uint16_t fg_charge_raw = LTC2944_Read16(LTC2944_REG_ACC_CHARGE_MSB);
            uint8_t fg_control = LTC2944_Read(LTC2944_REG_CONTROL);
            
            int16_t pos = QuadDec_GetPosition();
            uint16_t count = sprintf((char*)byUARTBuffer,"Pos:%d Chg:0x%04X ChgV:%umV FgV:%umV(0x%04X) FgQ:0x%04X FgCtrl:0x%02X\n\r", 
                                    pos, chg_status, chg_voltage, fg_voltage_mv, fg_voltage_raw, fg_charge_raw, fg_control);
            UART_SpiUartPutArray((uint8*)&byUARTBuffer, count);
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
