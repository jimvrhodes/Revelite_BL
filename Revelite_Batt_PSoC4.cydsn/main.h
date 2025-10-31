//
// main.h
// Revelite battery device
//
#ifndef _main_h_
#define _main_h_

#include "project.h"
#include "cytypes.h"
#include <stdbool.h>
#include <math.h>

#define MAJORVERSION 1
#define MINORVERSION 0
#define BUILDVERSION 0x00
//
//#define WDRESETON
    #define WDRESETTIME (60*1000) // one min reset timer
    #define ILO_FREQ 32000 // for watchdog
#define DEBUGOUT
//    
typedef uint16 UINT;
typedef uint8 BYTE;
//
// prototypes
uint16 GetAverageTEMP(void);
void LatchWrite(uint8 byAddress, uint8 byReg, uint8 byData);
bool I2CM_SyncWrite(uint8 bySlaveAddr, uint8 *buffer, uint8 length);
void LatchRead(uint8 byAddress, uint8 byReg, uint8* byData, uint8 byLength);
bool I2CM_SyncRead(uint8 bySlaveAddr, uint8 *buffer, uint8 length);

// Quadrature decoder functions
void InitQuadDec(void);
float QuadDec_GetBrightnessScalar(void);
int16_t QuadDec_GetPosition(void);
void QuadDec_SetPosition(int16_t position);

//
void RW_EEPROMData(uint8);
//
CY_ISR_PROTO(fnTIMER_ISR);
CY_ISR(ADC_ISR_LOC);
CY_ISR(DIM_ISR_Handler);
//
//    
// PCLA9538 registers
//
#define PCLA9538_INREG      0x00
#define PCLA9538_OUTREG     0x01
#define PCLA9538_PLOINV     0x02
#define PCLA9538_CONFIG     0x03
#define PCLA9538_OUTDRIVE0  0x40
#define PCLA9538_OUTDRIVE1  0x41
#define PCLA9538_INLATCH    0x42
#define PCLA9538_PUPEN      0x43
#define PCLA9538_PUPSEL     0x44
#define PCLA9538_INTMASK    0x45
#define PCLA9538_INTSTAT    0x46
#define PCLA9538_OUTCONFIG  0x4F    
//
//
// 0x70 PCLA9538 bit defines
#define LATCH1      0x70  // I2C address
#define LED4		(0x01 << 0)         // out
#define LED3    	(0x01 << 1)         // out
#define LED2		(0x01 << 2)         // out
#define LED1		(0x01 << 3)         // out
#define PUSH1		(0x01 << 4)         // in
#define PUSH2		(0x01 << 5)         // in
#define ROTPUSH	    (0x01 << 6)         // in
#define VBUSON		(0x01 << 7)         // out
#define DEFAULTLATCH1 0x00
#define LATCH1CONFIG 0x70
//
// 0x71 PCLA9538 bit defines
#define LATCH2      0x71  // I2C address
#define NTCSEL1		(0x01 << 0)         // out
#define NTCSEL2		(0x01 << 1)         // out
#define NTCSEL3		(0x01 << 2)         // out
#define ALCC		(0x01 << 3)         // in alert from gas gauge
#define BOOSTEN		(0x01 << 4)         // out
#define SPARE1		(0x01 << 5)         // out
#define SPARE2	    (0x01 << 6)         // out
#define SPARE3		(0x01 << 7)//       // out
#define DEFAULTLATCH2 0x00
#define LATCH2CONFIG 0x08
//
//
#define BLINKTIME 500
#define WRITEEEPROMTIME 5000
//
#define INTERPOLATE_FLOAT(value, old_min, old_max, new_min, new_max) ( (float)value - (float)old_min ) / ( (float)old_max - (float)old_min) * ( (float)new_max - (float)new_min) + (float)new_min
//
//
//#define PWMPERIOD 65535/32 // output PWM frequency
//#define PWMPERIOD 65535/16 // output PWM frequency
#define PWMPERIOD 65535 // output PWM frequency
#define PWMOFF 0
#define PWMMAX PWMPERIOD // duty cycle
//
#define PWMJUSTOFF 1 // to get the low end dimming
#define PWMJUSTOFFBOT 1
#define PWMMAXOUT (PWMPERIOD *0.60f)  // was 0.50 for 12V
//
#define ONDELAYTIME 100
//
//#define FILTER1 1.150f
#define FILTER1 1.0125f
#define FILTERSTART FILTER1 // first timed on filter
//
enum { // ADC stuff
    e_VCHARGER,
    e_VBUS,
    e_TEMP,
    e_BATTTEMP,
    e_BATTVSENSE,
};

#define MINADC 0.0f
#define MAXADC 2047.0f
//
// for the NTCs
// 1158 is 62C
#define MINTEMPADC 345      // TODO this needs to be 80C
#define MAXTEMPADC 410     // TODO this needs to be 75C
#define MINTEMPSCALAR 0.250f
#define MAXTEMPSCALAR 1.00f
#define DEFAULTTEMPSCALAR 1.0f
//
// EEPROM stuff
enum {
  e_EEPROMWRITE = 0,
  e_EEPROMREAD = 1,
  e_EEPROMFLUSHRESTORE = 2
};
//
// this is the EEPROM save structure
//
typedef struct __attribute__ ((__packed__)) {

    bool bWeHaveLearned;
    
    uint16 uiCalCh1;
    uint16 uiCalCh2;
        
    BYTE byEEPROMSave1;
    BYTE byEEPROMSave2;
} structInfo;

// UART debug
#define UARTTEMPOUTPUTTIME 2500


#endif // End of _main_h_

