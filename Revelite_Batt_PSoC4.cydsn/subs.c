// File: subs.c
// Project: Revelite
//
//

#include "project.h"
#include "main.h"
#include <math.h>
#include <stdio.h>

extern structInfo Info;
extern bool blSaveToInfo;

//
// ADC stuff
extern volatile uint32 dataReady;
extern volatile int16 result[ADC_TOTAL_CHANNELS_NUM];
uint16 GetAverageTemp(void);
uint16 GetAverageVsense(void);
//
//
//
#define SAMPLES 64 // temps and ZTEN
bool bWeHaveSampledTEMP = false;
uint16 GetAverageTEMP(void) {
    static uint16 uiBuffer[SAMPLES] = {0};
    static uint16 byPointer = 0;
    uint32 uiAverage = 0;
    
    // get the next sample into buffer, make 10-bit value for stability
    uiBuffer[byPointer] = result[e_TEMP];
    if(uiBuffer[byPointer] > MAXADC)
        uiBuffer[byPointer] = 0;
    else if(++byPointer > SAMPLES) {
        byPointer = 0;
        bWeHaveSampledTEMP = true;
    }
    // average by adding them all together, then dividing
    for(uint8 n = 0;n<SAMPLES;n++)
        uiAverage += uiBuffer[n];
    // return the average    
    return((uint16)(uiAverage/SAMPLES));
}
//

// this is the write to the connector board PCLA9538 latch
void LatchWrite(uint8 byAddress, uint8 byReg, uint8 byData) {
    
    uint8 buffer[2];
    buffer[0] = byReg;
    buffer[1] = byData;
        
    uint8 error = I2CM_SyncWrite(byAddress, (uint8*)buffer, 2);
}

//
bool I2CM_SyncWrite(uint8 bySlaveAddr, uint8 *buffer, uint8 length) {
    
    bool error = false;
    
    I2CM_I2CMasterClearStatus();
    uint32 status = I2CM_I2CMasterSendStart(bySlaveAddr, I2CM_I2C_WRITE_XFER_MODE, 100);
    if(status == I2CM_I2C_MSTR_NO_ERROR)	{	
		for(uint8 i = 0; i<length; i++) {
			status = I2CM_I2CMasterWriteByte(buffer[i], 25);
			if(status != I2CM_I2C_MSTR_NO_ERROR)
				break;
		}
    }
    
	I2CM_I2CMasterSendStop(25);    
    return error;
}

// this is the write to the connector board PCLA9538 latch
void LatchRead(uint8 byAddress, uint8 byReg, uint8* byData, uint8 byLength) {
    
    uint8 buffer[2];
    buffer[0] = byReg;
    I2CM_SyncWrite(byAddress, buffer, 1);    
    
    uint8 error = I2CM_SyncRead(byAddress, (uint8*)buffer, 1);
    
    memcpy(byData, buffer, byLength);  // return value
}

//
// I2C using Start, Data, Stop methods, these are called by the next set of routines
//
bool I2CM_SyncRead(uint8 bySlaveAddr, uint8 *buffer, uint8 length) {
    bool error = false;
    
  	I2CM_I2CMasterClearStatus();
    uint32 status = I2CM_I2CMasterSendStart(bySlaveAddr, I2CM_I2C_READ_XFER_MODE, 100);
	if(status == I2CM_I2C_MSTR_NO_ERROR) {
    	for(uint8 i=0; i<length; i++) {
    		if(i<(length-1))
    			I2CM_I2CMasterReadByte(I2CM_I2C_ACK_DATA, (buffer+i), 10);
    		else
    			I2CM_I2CMasterReadByte(I2CM_I2C_NAK_DATA, (buffer+i), 10);
        }
    }
    
	I2CM_I2CMasterSendStop(25);
    return(error);
}


// EEPROM stuff
const structInfo defaultInfo = {

    .bWeHaveLearned = false,
    .uiCalCh1 = 0,
    .uiCalCh2 = 0,
	
	.byEEPROMSave1 = 0x55,
	.byEEPROMSave2 = 0xAA,
};

#define INFO_ROW       	(CY_FLASH_NUMBER_ROWS - 2u)
#define INFO_ADDR      	(INFO_ROW * CY_FLASH_SIZEOF_ROW)
uint8 Info_Mem[CY_FLASH_SIZEOF_ROW];
void RW_EEPROMData(uint8 RW) {
    
	if (RW == e_EEPROMWRITE) {
		Info.byEEPROMSave1 = 0x55;
		Info.byEEPROMSave2 = 0xAA;
		memcpy(Info_Mem,(uint8*)&Info, sizeof(structInfo));
	    CySysFlashWriteRow(INFO_ROW, Info_Mem);
	} else {
		memcpy((uint8*)&Info, (uint8*)INFO_ADDR, sizeof(structInfo));
		if (Info.byEEPROMSave1 != 0x55 || Info.byEEPROMSave2 != 0xAA || RW == e_EEPROMFLUSHRESTORE){
            Info = defaultInfo; // we are brand new or are told to be brand new, make it happen
            memcpy(Info_Mem,(uint8*)&Info, sizeof(structInfo));
			CySysFlashWriteRow(INFO_ROW, Info_Mem);
		}
	}
}


