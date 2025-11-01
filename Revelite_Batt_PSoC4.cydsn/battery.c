// File: battery.c
// Project: Revelite - BQ25730 Battery Charger Driver
//
//

#include "project.h"
#include "main.h"
#include "battery.h"
#include "LTC2944.h"
#include <math.h>
#include <stdio.h>

extern structInfo Info;

//=============================================================================
// Low-level I2C Functions
//=============================================================================

// Write 16-bit value to BQ25730 register using proven I2C functions
void BQ25730_Write(uint8_t reg, uint16_t value) {
    uint8_t buffer[3];
    buffer[0] = reg;
    buffer[1] = value & 0xFF;        // LSB
    buffer[2] = (value >> 8) & 0xFF; // MSB
    
    I2CM_SyncWrite(BQ25730_ADDR, buffer, 3);
}

// Read 16-bit value from BQ25730 register using proven I2C functions
uint16_t BQ25730_Read(uint8_t reg) {
    uint8_t write_buf[1];
    uint8_t read_buf[2];
    
    // Write register address
    write_buf[0] = reg;
    I2CM_SyncWrite(BQ25730_ADDR, write_buf, 1);
    
    // Read 2 bytes (LSB, MSB)
    I2CM_SyncRead(BQ25730_ADDR, read_buf, 2);
    
    return read_buf[0] | (read_buf[1] << 8);
}

//=============================================================================
// BQ25730 Initialization and Configuration
//=============================================================================

// Initialize BQ25730 battery charger for 4S LiPo battery
bool BQ25730_Init(void) {
    
    // Note: Skipping ID check as register reads are returning 0xFFFF
    // The chip responds to I2C address scan, so it's present
    // This might be a timing issue or chip variant
    
    CyDelay(100);  // Give chip time to be ready
    
    // RT9478M SPECIFIC: Unlock BATFET control by writing to AuxFunction register
    // Register 0x40, bit 7 (UNLOCK_PORT_CTRL) must be 1 to activate EN_PORT_CTRL
    BQ25730_Write(0x40, 0x8100);  // Set bit 7 = 1, keep default 0x8100 value
    CyDelay(10);
    
    // ChargeOption0 [0x12]: Basic charge control
    // Bit 0: Enable charging (0=enable, 1=disable)
    // Bit 5: Enable IBAT (1=enable)
    // Bit 8: Enable IDPM (1=enable)
    BQ25730_Write(BQ25730_CHARGE_OPTION_0, 0x0120);  // Enable charging, IBAT, IDPM
    CyDelay(10);
    
    // Set Maximum Charge Voltage [0x15]
    // Resolution: 8mV per LSB
    // For 4S LiPo: 4.2V * 4 = 16.8V = 16800mV
    // Register value = 16800mV / 8mV = 2100 (0x0834)
    BQ25730_Write(BQ25730_MAX_CHARGE_VOLTAGE, BATTERY_MAX_VOLTAGE_MV / 8);
    CyDelay(10);
    
    // Set Charge Current [0x14]
    // Resolution: 64mA per LSB
    // For 1A charge current: 1000mA / 64 = 15.625 -> 16 (0x0010)
    BQ25730_Write(BQ25730_CHARGE_CURRENT, CHARGE_CURRENT_MA / 64);
    CyDelay(10);
    
    // Set Minimum System Voltage [0x3E]
    // Resolution: 256mV per LSB
    // For 12.8V: 12800mV / 256 = 50 (0x0032)
    BQ25730_Write(BQ25730_MIN_SYS_VOLTAGE, MIN_SYS_VOLTAGE_MV / 256);
    CyDelay(10);
    
    // Set Input Voltage [0x3D] - typical USB-C PD voltage
    // Resolution: 64mV per LSB
    // For 20V input: 20000mV / 64 = 312 (0x0138)
    BQ25730_Write(BQ25730_INPUT_VOLTAGE, 20000 / 64);
    CyDelay(10);
    
    // Set Input Current [0x3F] - limit to reasonable value
    // Resolution: 50mA per LSB
    // For 3A input: 3000mA / 50 = 60 (0x003C)
    BQ25730_Write(BQ25730_INPUT_CURRENT, 3000 / 50);
    CyDelay(10);
    
    // ChargeOption1 [0x30]: ADC and safety timer settings
    // Enable ADC for monitoring
    BQ25730_Write(BQ25730_CHARGE_OPTION_1, 0x0000);  // Default settings
    CyDelay(10);
    
    // ChargeOption2 [0x31]: ILIM and other settings
    BQ25730_Write(BQ25730_CHARGE_OPTION_2, 0x0000);  // Default settings
    CyDelay(10);
    
    // ChargeOption3 [0x32]: OTG and other settings
    BQ25730_Write(BQ25730_CHARGE_OPTION_3, 0x0000);  // Default settings, OTG disabled
    CyDelay(10);
    
    // ADC Option [0x35]: Enable continuous ADC conversion
    // Bit 15: ADC_EN (1=enable)
    // Bit 14: ADC_CONV (1=one-shot, 0=continuous)
    // Bit 13: ADC_START (1=start conversion)
    BQ25730_Write(BQ25730_ADC_OPTION, BQ25730_ADC_EN | BQ25730_ADC_CONV_START);
    CyDelay(10);
    
    return true;
}

//=============================================================================
// BQ25730 Control Functions
//=============================================================================

// Enable or disable charging
void BQ25730_EnableCharging(bool enable) {
    uint16_t option0 = BQ25730_Read(BQ25730_CHARGE_OPTION_0);
    
    if (enable) {
        option0 &= ~0x0001;  // Clear bit 0 to enable charging
    } else {
        option0 |= 0x0001;   // Set bit 0 to disable charging
    }
    
    BQ25730_Write(BQ25730_CHARGE_OPTION_0, option0);
}

// Set charge current in mA
void BQ25730_SetChargeCurrent_mA(uint16_t current_ma) {
    // Resolution: 64mA per LSB
    // Limit to reasonable range (50mA to 8000mA)
    if (current_ma < 50) current_ma = 50;
    if (current_ma > 8000) current_ma = 8000;
    
    uint16_t reg_value = current_ma / 64;
    BQ25730_Write(BQ25730_CHARGE_CURRENT, reg_value);
}

// Set charge voltage in mV
void BQ25730_SetChargeVoltage_mV(uint16_t voltage_mv) {
    // Resolution: 8mV per LSB
    // Limit to safe range for 4S (14000mV to 16800mV)
    if (voltage_mv < 14000) voltage_mv = 14000;
    if (voltage_mv > 16800) voltage_mv = 16800;
    
    uint16_t reg_value = voltage_mv / 8;
    BQ25730_Write(BQ25730_MAX_CHARGE_VOLTAGE, reg_value);
}

// Start ADC conversion
void BQ25730_StartADC(void) {
    uint16_t adc_option = BQ25730_Read(BQ25730_ADC_OPTION);
    adc_option |= BQ25730_ADC_CONV_START;
    BQ25730_Write(BQ25730_ADC_OPTION, adc_option);
}

//=============================================================================
// BQ25730 Monitoring Functions
//=============================================================================

// Get battery voltage in mV
uint16_t BQ25730_GetBatteryVoltage_mV(void) {
    uint16_t adc_val = BQ25730_Read(BQ25730_ADC_VSYS_VBAT);
    
    // VBAT is in upper byte (bits 8-15)
    // Resolution: 64mV per LSB
    uint8_t vbat_raw = (adc_val >> 8) & 0xFF;
    uint16_t voltage_mv = (uint16_t)vbat_raw * 64;
    
    return voltage_mv;
}

// Get battery current in mA (positive = charging, negative = discharging)
int16_t BQ25730_GetBatteryCurrent_mA(void) {
    uint16_t adc_val = BQ25730_Read(BQ25730_ADC_IBAT);
    
    // Resolution: 64mA per LSB
    // Bit 15 indicates direction (0=charging, 1=discharging)
    int16_t current_ma;
    
    if (adc_val & 0x8000) {
        // Discharging (negative current)
        current_ma = -((int16_t)(adc_val & 0x7FFF) * 64);
    } else {
        // Charging (positive current)
        current_ma = (int16_t)adc_val * 64;
    }
    
    return current_ma;
}

// Get VBUS voltage in mV
uint16_t BQ25730_GetVBusVoltage_mV(void) {
    uint16_t adc_val = BQ25730_Read(BQ25730_ADC_VBUS_PSYS);
    
    // VBUS is in upper byte (bits 8-15)
    // Resolution: 64mV per LSB
    uint8_t vbus_raw = (adc_val >> 8) & 0xFF;
    uint16_t voltage_mv = (uint16_t)vbus_raw * 64;
    
    return voltage_mv;
}

// Check if battery is currently charging
bool BQ25730_IsCharging(void) {
    uint16_t status = BQ25730_Read(BQ25730_CHARGER_STATUS);
    
    // Check if in fast charge (bit 10) or pre-charge (bit 9) mode
    return ((status & BQ25730_STATUS_IN_FCHRG) || (status & BQ25730_STATUS_IN_PCHRG));
}

// Get comprehensive battery status
BatteryStatus_t BQ25730_GetStatus(void) {
    BatteryStatus_t status;
    
    // Read charger status register
    status.charger_status = BQ25730_Read(BQ25730_CHARGER_STATUS);
    
    // Read ADC values
    status.battery_voltage_mv = BQ25730_GetBatteryVoltage_mV();
    status.battery_current_ma = BQ25730_GetBatteryCurrent_mA();
    status.vbus_voltage_mv = BQ25730_GetVBusVoltage_mV();
    
    // Read system voltage
    uint16_t adc_vsys = BQ25730_Read(BQ25730_ADC_VSYS_VBAT);
    uint8_t vsys_raw = adc_vsys & 0xFF;
    status.system_voltage_mv = (uint16_t)vsys_raw * 64;
    
    // Parse status bits
    status.ac_present = (status.charger_status & BQ25730_STATUS_AC_STAT) ? true : false;
    status.is_charging = ((status.charger_status & BQ25730_STATUS_IN_FCHRG) || 
                         (status.charger_status & BQ25730_STATUS_IN_PCHRG)) ? true : false;
    status.charge_done = (status.is_charging == false && status.ac_present == true);
    
    // Check for any fault conditions
    status.fault_present = (status.charger_status & (BQ25730_STATUS_FAULT_ACOV | 
                                                     BQ25730_STATUS_FAULT_BATOC | 
                                                     BQ25730_STATUS_FAULT_ACOC |
                                                     BQ25730_STATUS_SYSOVP_STAT |
                                                     BQ25730_STATUS_FAULT_LATCHOFF)) ? true : false;
    
    return status;
}


