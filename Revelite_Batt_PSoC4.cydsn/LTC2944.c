// File: LTC2944.c
// Project: Revelite - LTC2944 Battery Fuel Gauge Driver
//
//

#include "project.h"
#include "main.h"
#include "battery.h"
#include "LTC2944.h"
#include <math.h>
#include <stdio.h>

extern structInfo Info;
extern bool blSaveToInfo;

//=============================================================================
// Low-level I2C Functions - Using unified I2C functions from subs.c
//=============================================================================

// Write 8-bit value to LTC2944 register
void LTC2944_Write(uint8_t reg, uint8_t value) {
    I2C_WriteByte(LTC2944_ADDR, reg, value);
}

// Read 8-bit value from LTC2944 register
uint8_t LTC2944_Read(uint8_t reg) {
    return I2C_ReadByte(LTC2944_ADDR, reg);
}

// Read 16-bit value (MSB first) starting at specified register
// LTC2944 uses big-endian (MSB first) format
uint16_t LTC2944_Read16(uint8_t reg_msb) {
    return I2C_ReadWord_MSB(LTC2944_ADDR, reg_msb);
}

//=============================================================================
// LTC2944 Initialization and Configuration
//=============================================================================

// Initialize the LTC2944
bool LTC2944_Init(uint16_t battery_capacity_mAh, uint8_t prescaler) {
    // Configure control register: prescaler, ALCC mode, ADC mode automatic
    // Control register bits:
    // [7:6] ADC_MODE: 11 = Automatic, 10 = Scan, 01 = Manual, 00 = Sleep
    // [5:3] PRESCALER: M value (000=1, 001=4, 010=16, 011=64, 100=256, 101=1024, 110=4096)
    // [2:1] ALCC_CONFIG: 00=Disabled, 01=Alert, 10=Charge complete
    // [0]   SHUTDOWN: 0=Normal, 1=Shutdown
    
    uint8_t control = 0;
    
    // Set ADC to automatic mode (bits 7:6 = 11)
    control |= (LTC2944_ADC_MODE_AUTOMATIC << 6);  // 0xC0
    
    // Set prescaler (bits 5:3)
    control |= prescaler;  // Should be 0x28 for M=1024
    
    // Set ALCC pin to alert mode (bits 2:1 = 01)
    control |= LTC2944_ALCC_MODE_ALERT;  // 0x04 >> 1 = 0x02? Check this!
    
    // Expected value with M=1024: 0xC0 | 0x28 | 0x02 = 0xEA
    // But we need to check the actual bit positions
    
    // Write control register
    LTC2944_Write(LTC2944_REG_CONTROL, control);
    CyDelay(10);
    
    // Read back to verify
    uint8_t readback = LTC2944_Read(LTC2944_REG_CONTROL);
    
    // Clear any existing alerts
    LTC2944_ClearAlerts();
    
    // Set accumulated charge to half of full-scale for better headroom
    LTC2944_SetAccumulatedCharge(battery_capacity_mAh / 2);
    
    return (readback == control);  // Return true if write succeeded
}

// Set accumulated charge value based on mAh
void LTC2944_SetAccumulatedCharge(uint16_t charge_mAh) {
    // Convert mAh to raw counts based on prescaler and RSENSE
    // This is a simplified conversion and may need calibration
    // For more accuracy, you should use the exact formula from datasheet
    uint16_t counts = (uint16_t)(charge_mAh / LTC2944_CHARGE_LSB);
    
    // Clamp to 16-bit
    if (counts > 0xFFFF) counts = 0xFFFF;

    // Write to accumulated charge registers
    uint8_t msb = (counts >> 8) & 0xFF;
    uint8_t lsb = counts & 0xFF;

    LTC2944_Write(LTC2944_REG_ACC_CHARGE_MSB, msb);
    LTC2944_Write(LTC2944_REG_ACC_CHARGE_LSB, lsb);
}

//=============================================================================
// LTC2944 Monitoring Functions
//=============================================================================

// Get comprehensive fuel gauge status
FuelGauge_t LTC2944_GetStatus(void) {
    FuelGauge_t status = {0};

    // Read status register
    status.status_reg = LTC2944_Read(LTC2944_REG_STATUS);
    status.alert_present = (status.status_reg & (LTC2944_STATUS_CHARGE_ALERT_HIGH |
                                                 LTC2944_STATUS_CHARGE_ALERT_LOW |
                                                 LTC2944_STATUS_VOLTAGE_ALERT |
                                                 LTC2944_STATUS_CURRENT_ALERT |
                                                 LTC2944_STATUS_TEMP_ALERT)) ? true : false;

    // Read accumulated charge
    status.accumulated_charge_raw = LTC2944_Read16(LTC2944_REG_ACC_CHARGE_MSB);
    status.accumulated_charge_mAh = (float)status.accumulated_charge_raw * LTC2944_CHARGE_LSB;

    // Read voltage
    status.voltage_raw = LTC2944_Read16(LTC2944_REG_VOLTAGE_MSB);
    status.voltage_mv = (float)status.voltage_raw * LTC2944_VOLTAGE_LSB;

    // Read current (two's complement)
    status.current_raw = LTC2944_Read16(LTC2944_REG_CURRENT_MSB);
    int16_t current_signed = (int16_t)status.current_raw;
    status.current_ma = (float)current_signed * (LTC2944_CURRENT_LSB_60mV / RSENSE);

    // Read temperature
    status.temperature_raw = LTC2944_Read16(LTC2944_REG_TEMPERATURE_MSB);
    status.temperature_c = ((float)status.temperature_raw * LTC2944_TEMP_LSB) - 273.15f;

    // Estimate state of charge (basic)
    float capacity_mAh = (float)BATTERY_CAPACITY_MAH;
    status.state_of_charge = (status.accumulated_charge_mAh / capacity_mAh) * 100.0f;
    if (status.state_of_charge < 0.0f) status.state_of_charge = 0.0f;
    if (status.state_of_charge > 100.0f) status.state_of_charge = 100.0f;

    return status;
}

float LTC2944_GetVoltage_mV(void) {
    uint16_t raw = LTC2944_Read16(LTC2944_REG_VOLTAGE_MSB);
    return (float)raw * LTC2944_VOLTAGE_LSB;
}

float LTC2944_GetCurrent_MA_Internal(void) {
    uint16_t raw = LTC2944_Read16(LTC2944_REG_CURRENT_MSB);
    int16_t signed_current = (int16_t)raw;
    return (float)signed_current * (LTC2944_CURRENT_LSB_60mV / RSENSE);
}

float LTC2944_GetCurrent_mA(void) {
    return LTC2944_GetCurrent_MA_Internal();
}

float LTC2944_GetTemperature_C(void) {
    uint16_t raw = LTC2944_Read16(LTC2944_REG_TEMPERATURE_MSB);
    return ((float)raw * LTC2944_TEMP_LSB) - 273.15f;
}

float LTC2944_GetAccumulatedCharge_mAh(void) {
    uint16_t raw = LTC2944_Read16(LTC2944_REG_ACC_CHARGE_MSB);
    return (float)raw * LTC2944_CHARGE_LSB;
}

float LTC2944_GetStateOfCharge(void) {
    float acc_mAh = LTC2944_GetAccumulatedCharge_mAh();
    float capacity_mAh = (float)BATTERY_CAPACITY_MAH;
    float soc = (acc_mAh / capacity_mAh) * 100.0f;
    if (soc < 0.0f) soc = 0.0f;
    if (soc > 100.0f) soc = 100.0f;
    return soc;
}

uint8_t LTC2944_GetAlerts(void) {
    return LTC2944_Read(LTC2944_REG_STATUS);
}

void LTC2944_ClearAlerts(void) {
    // Reading the status register clears the alert bits
    (void)LTC2944_Read(LTC2944_REG_STATUS);
}

void LTC2944_Shutdown(bool shutdown) {
    uint8_t control = LTC2944_Read(LTC2944_REG_CONTROL);
    if (shutdown) {
        control |= LTC2944_SHUTDOWN_BIT;
    } else {
        control &= ~LTC2944_SHUTDOWN_BIT;
    }
    LTC2944_Write(LTC2944_REG_CONTROL, control);
}


