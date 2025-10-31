//
// LTC2944.h
// Revelite battery device - LTC2944 Battery Gas Gauge
//
#ifndef _LTC2944_h_
#define _LTC2944_h_

#include "project.h"
#include "cytypes.h"
#include <stdbool.h>
#include <stdint.h>

// LTC2944 I2C Address
#define LTC2944_ADDR 0x64  // 7-bit I2C address

// LTC2944 Register Addresses
#define LTC2944_REG_STATUS              0x00
#define LTC2944_REG_CONTROL             0x01
#define LTC2944_REG_ACC_CHARGE_MSB      0x02
#define LTC2944_REG_ACC_CHARGE_LSB      0x03
#define LTC2944_REG_CHARGE_THRESH_H_MSB 0x04
#define LTC2944_REG_CHARGE_THRESH_H_LSB 0x05
#define LTC2944_REG_CHARGE_THRESH_L_MSB 0x06
#define LTC2944_REG_CHARGE_THRESH_L_LSB 0x07
#define LTC2944_REG_VOLTAGE_MSB         0x08
#define LTC2944_REG_VOLTAGE_LSB         0x09
#define LTC2944_REG_VOLTAGE_THRESH_H    0x0A
#define LTC2944_REG_VOLTAGE_THRESH_L    0x0B
#define LTC2944_REG_CURRENT_MSB         0x0E
#define LTC2944_REG_CURRENT_LSB         0x0F
#define LTC2944_REG_CURRENT_THRESH_H    0x10
#define LTC2944_REG_CURRENT_THRESH_L    0x11
#define LTC2944_REG_TEMPERATURE_MSB     0x14
#define LTC2944_REG_TEMPERATURE_LSB     0x15
#define LTC2944_REG_TEMPERATURE_THRESH_H 0x16
#define LTC2944_REG_TEMPERATURE_THRESH_L 0x17

// Control Register Bits (0x01)
#define LTC2944_PRESCALER_M_1           0x00
#define LTC2944_PRESCALER_M_4           0x08
#define LTC2944_PRESCALER_M_16          0x10
#define LTC2944_PRESCALER_M_64          0x18
#define LTC2944_PRESCALER_M_256         0x20
#define LTC2944_PRESCALER_M_1024        0x28
#define LTC2944_PRESCALER_M_4096        0x30

#define LTC2944_ALCC_MODE_DISABLED      0x00
#define LTC2944_ALCC_MODE_ALERT         0x04
#define LTC2944_ALCC_MODE_CHARGE_COMPLETE 0x02

#define LTC2944_ADC_MODE_AUTOMATIC      0x03
#define LTC2944_ADC_MODE_SCAN           0x02
#define LTC2944_ADC_MODE_MANUAL         0x01
#define LTC2944_ADC_MODE_SLEEP          0x00

#define LTC2944_SHUTDOWN_BIT            0x01

// Status Register Bits (0x00)
#define LTC2944_STATUS_CHARGE_ALERT_HIGH   0x20
#define LTC2944_STATUS_CHARGE_ALERT_LOW    0x10
#define LTC2944_STATUS_VOLTAGE_ALERT       0x08
#define LTC2944_STATUS_CURRENT_ALERT       0x04
#define LTC2944_STATUS_TEMP_ALERT          0x02
#define LTC2944_STATUS_CHARGE_OVERFLOW     0x01

// Conversion Constants
#define LTC2944_CHARGE_LSB              0.34  // mAh per LSB with 50mOhm sense resistor
#define LTC2944_VOLTAGE_LSB             1.44  // mV per LSB
#define LTC2944_CURRENT_LSB_60mV        0.0293  // mA per LSB for 60mV full scale (typical)
#define LTC2944_TEMP_LSB                0.25  // °C per LSB
#define LTC2944_TEMP_OFFSET             273.15  // Kelvin offset

// Sense Resistor Value (Ohms) - adjust based on your hardware
#define RSENSE                          0.050  // 50 milliohms

// Battery capacity from battery.h will be used
// For 50.4Wh at 14.4V nominal = ~3500mAh

// Fuel Gauge Status Structure
typedef struct {
    uint16_t accumulated_charge_raw;  // Raw ADC value
    float    accumulated_charge_mAh;  // Accumulated charge in mAh
    uint16_t voltage_raw;             // Raw ADC value
    float    voltage_mv;              // Battery voltage in mV
    uint16_t current_raw;             // Raw ADC value
    float    current_ma;              // Battery current in mA
    uint16_t temperature_raw;         // Raw ADC value
    float    temperature_c;           // Temperature in Celsius
    uint8_t  status_reg;              // Status register value
    float    state_of_charge;         // State of charge (0-100%)
    bool     alert_present;           // Any alert active
} FuelGauge_t;

// Function Prototypes
void LTC2944_Write(uint8_t reg, uint8_t value);
uint8_t LTC2944_Read(uint8_t reg);
uint16_t LTC2944_Read16(uint8_t reg_msb);
bool LTC2944_Init(uint16_t battery_capacity_mAh, uint8_t prescaler);
void LTC2944_SetAccumulatedCharge(uint16_t charge_mAh);
FuelGauge_t LTC2944_GetStatus(void);
float LTC2944_GetVoltage_mV(void);
float LTC2944_GetCurrent_mA(void);
float LTC2944_GetTemperature_C(void);
float LTC2944_GetAccumulatedCharge_mAh(void);
float LTC2944_GetStateOfCharge(void);
uint8_t LTC2944_GetAlerts(void);
void LTC2944_ClearAlerts(void);
void LTC2944_Shutdown(bool shutdown);

#endif // _LTC2944_h_
