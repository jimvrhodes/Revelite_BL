// File: RT9478M.c
// Project: Revelite - RT9478M USB PD Controller/Charger Driver
//

#include "project.h"
#include "main.h"
#include "RT9478M.h"
#include <math.h>

// Write 8-bit value to RT9478M register
void RT9478M_Write(uint8_t reg, uint8_t value) {
    I2CM_I2CMasterClearStatus();
    
    if (I2CM_I2CMasterSendStart(RT9478M_ADDR, I2CM_I2C_WRITE_XFER_MODE, 100) == I2CM_I2C_MSTR_NO_ERROR) {
        I2CM_I2CMasterWriteByte(reg, 25);
        I2CM_I2CMasterWriteByte(value, 25);
    }
    
    I2CM_I2CMasterSendStop(25);
}

// Read 8-bit value from RT9478M register
uint8_t RT9478M_Read(uint8_t reg) {
    uint8_t value = 0;
    
    I2CM_I2CMasterClearStatus();
    
    // Write register address
    if (I2CM_I2CMasterSendStart(RT9478M_ADDR, I2CM_I2C_WRITE_XFER_MODE, 100) == I2CM_I2C_MSTR_NO_ERROR) {
        I2CM_I2CMasterWriteByte(reg, 25);
    }
    
    // Repeated start for read
    if (I2CM_I2CMasterSendRestart(RT9478M_ADDR, I2CM_I2C_READ_XFER_MODE, 100) == I2CM_I2C_MSTR_NO_ERROR) {
        I2CM_I2CMasterReadByte(I2CM_I2C_NAK_DATA, (uint8*)&value, 25);
    }
    
    I2CM_I2CMasterSendStop(25);
    
    return value;
}

// Initialize RT9478M
bool RT9478M_Init(void) {
    CyDelay(100);  // Give chip time to be ready
    
    // Read device ID to verify communication
    uint8_t dev_id = RT9478M_GetDeviceID();
    
    // Try to enable charging (register addresses are guesses without datasheet)
    // This will need to be updated with actual register map
    RT9478M_Write(RT9478M_REG_CONTROL1, RT9478M_CTRL_CHARGE_ENABLE);
    CyDelay(10);
    
    return (dev_id != 0x00 && dev_id != 0xFF);  // Valid if not all 0s or all 1s
}

// Get device ID
uint8_t RT9478M_GetDeviceID(void) {
    return RT9478M_Read(RT9478M_REG_DEVICE_ID);
}

// Get status register
uint8_t RT9478M_GetStatus(void) {
    return RT9478M_Read(RT9478M_REG_STATUS);
}

// Check if charging
bool RT9478M_IsCharging(void) {
    uint8_t status = RT9478M_GetStatus();
    return (status & RT9478M_STATUS_CHARGING) ? true : false;
}

// Enable/disable charging
void RT9478M_EnableCharging(bool enable) {
    uint8_t ctrl = RT9478M_Read(RT9478M_REG_CONTROL1);
    
    if (enable) {
        ctrl |= RT9478M_CTRL_CHARGE_ENABLE;
    } else {
        ctrl &= ~RT9478M_CTRL_CHARGE_ENABLE;
    }
    
    RT9478M_Write(RT9478M_REG_CONTROL1, ctrl);
}

// Set charge current (placeholder - needs actual register format)
void RT9478M_SetChargeCurrent_mA(uint16_t current_ma) {
    // This needs actual datasheet to implement correctly
    // For now, just write a scaled value
    uint8_t ichg_reg = (uint8_t)(current_ma / 64);  // Guess at 64mA per LSB
    RT9478M_Write(RT9478M_REG_ICHG, ichg_reg);
}

// Set charge voltage (placeholder - needs actual register format)
void RT9478M_SetChargeVoltage_mV(uint16_t voltage_mv) {
    // This needs actual datasheet to implement correctly
    // For now, just write a scaled value
    uint8_t vchg_reg = (uint8_t)(voltage_mv / 64);  // Guess at 64mV per LSB
    RT9478M_Write(RT9478M_REG_VCHG, vchg_reg);
}
