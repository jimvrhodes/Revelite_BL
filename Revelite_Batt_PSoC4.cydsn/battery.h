//
// battery.h
// Revelite battery device - BQ25730 Battery Charger
//
#ifndef _battery_h_
#define _battery_h_

#include "project.h"
#include "cytypes.h"
#include <stdbool.h>
#include <math.h>

// BQ25730 I2C Address
#define BQ25730_ADDR 0x6B // 7-bit I2C address (hardware strapped to 0x6B)

// BQ25730 Register Addresses
#define BQ25730_CHARGE_OPTION_0     0x12
#define BQ25730_CHARGE_CURRENT      0x14
#define BQ25730_MAX_CHARGE_VOLTAGE  0x15
#define BQ25730_CHARGE_OPTION_1     0x30
#define BQ25730_CHARGE_OPTION_2     0x31
#define BQ25730_CHARGE_OPTION_3     0x32
#define BQ25730_PROCHOT_OPTION_0    0x33
#define BQ25730_PROCHOT_OPTION_1    0x34
#define BQ25730_ADC_OPTION          0x35
#define BQ25730_CHARGER_STATUS      0x20
#define BQ25730_PROCHOT_STATUS      0x21
#define BQ25730_IIN_DPM             0x22
#define BQ25730_ADC_VBUS_PSYS       0x23
#define BQ25730_ADC_IBAT            0x24
#define BQ25730_ADC_CMPIN_IIN       0x25
#define BQ25730_ADC_VSYS_VBAT       0x26
#define BQ25730_OTG_VOLTAGE         0x3B
#define BQ25730_OTG_CURRENT         0x3C
#define BQ25730_INPUT_VOLTAGE       0x3D
#define BQ25730_MIN_SYS_VOLTAGE     0x3E
#define BQ25730_INPUT_CURRENT       0x3F
#define BQ25730_MANUFACTURER_ID     0xFE
#define BQ25730_DEVICE_ID           0xFF

// Battery Configuration for 4S LiPo (14.4V nominal, 16V max charge)
#define BATTERY_CELLS               4
#define CELL_MAX_VOLTAGE_MV         4000    // 4.0V per cell max (conservative)
#define BATTERY_MAX_VOLTAGE_MV      (CELL_MAX_VOLTAGE_MV * BATTERY_CELLS)  // 16000 mV
#define BATTERY_CAPACITY_WH         50.4    // 50.4 Watt-hours
#define BATTERY_NOMINAL_VOLTAGE     14.4    // Volts
#define BATTERY_CAPACITY_MAH        ((BATTERY_CAPACITY_WH / BATTERY_NOMINAL_VOLTAGE) * 1000)  // ~3500 mAh
#define BATTERY_LOW_CUTOFF_MV       12500   // Low voltage cutoff (3.125V per cell)

// Charging Parameters
#define CHARGE_CURRENT_MA           1000    // 1A charge current
#define MIN_SYS_VOLTAGE_MV          12500   // Minimum system voltage for safe operation

// Charger Status Bits
#define BQ25730_STATUS_AC_STAT          (1 << 15)
#define BQ25730_STATUS_ICO_DONE         (1 << 14)
#define BQ25730_STATUS_IN_VINDPM        (1 << 12)
#define BQ25730_STATUS_IN_IINDPM        (1 << 11)
#define BQ25730_STATUS_IN_FCHRG         (1 << 10)
#define BQ25730_STATUS_IN_PCHRG         (1 << 9)
#define BQ25730_STATUS_IN_OTG           (1 << 8)
#define BQ25730_STATUS_FAULT_ACOV       (1 << 7)
#define BQ25730_STATUS_FAULT_BATOC      (1 << 6)
#define BQ25730_STATUS_FAULT_ACOC       (1 << 5)
#define BQ25730_STATUS_SYSOVP_STAT      (1 << 4)
#define BQ25730_STATUS_FAULT_LATCHOFF   (1 << 2)
#define BQ25730_STATUS_FAULT_OTG_OVP    (1 << 1)
#define BQ25730_STATUS_FAULT_OTG_UVP    (1 << 0)

// ADC Control Bits
#define BQ25730_ADC_EN                  (1 << 15)
#define BQ25730_ADC_CONV                (1 << 14)
#define BQ25730_ADC_CONV_START          (1 << 13)

// Battery Status Structure
typedef struct {
    uint16_t charger_status;
    uint16_t battery_voltage_mv;
    int16_t  battery_current_ma;
    uint16_t vbus_voltage_mv;
    uint16_t system_voltage_mv;
    bool     is_charging;
    bool     charge_done;
    bool     ac_present;
    bool     fault_present;
} BatteryStatus_t;

// Function Prototypes
void BQ25730_Write(uint8_t reg, uint16_t value);
uint16_t BQ25730_Read(uint8_t reg);
bool BQ25730_Init(void);
void BQ25730_EnableCharging(bool enable);
BatteryStatus_t BQ25730_GetStatus(void);
uint16_t BQ25730_GetBatteryVoltage_mV(void);
int16_t BQ25730_GetBatteryCurrent_mA(void);
uint16_t BQ25730_GetVBusVoltage_mV(void);
bool BQ25730_IsCharging(void);
void BQ25730_SetChargeCurrent_mA(uint16_t current_ma);
void BQ25730_SetChargeVoltage_mV(uint16_t voltage_mv);
void BQ25730_StartADC(void);

// Legacy compatibility (BQ25720 was the old part number)
#define BQ25720_ADDR    BQ25730_ADDR
#define BQ25720_Write   BQ25730_Write
#define BQ25720_Read    BQ25730_Read
#define BQ25720_ReadStatus() BQ25730_Read(BQ25730_CHARGER_STATUS)

#endif // End of _battery_h_

