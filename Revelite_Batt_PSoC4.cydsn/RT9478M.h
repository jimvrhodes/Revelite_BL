//
// RT9478M.h
// Revelite battery device - RT9478M USB PD Controller/Charger Driver
//
#ifndef _RT9478M_h_
#define _RT9478M_h_

#include "project.h"
#include "cytypes.h"
#include <stdbool.h>

// RT9478M I2C Address
#define RT9478M_ADDR 0x22  // 7-bit I2C address

// RT9478M Register Addresses (based on typical Richtek charger ICs)
// Note: Need actual datasheet to confirm these
#define RT9478M_REG_DEVICE_ID       0x00
#define RT9478M_REG_STATUS          0x01
#define RT9478M_REG_CONTROL1        0x02
#define RT9478M_REG_CONTROL2        0x03
#define RT9478M_REG_ICHG            0x04  // Charge current
#define RT9478M_REG_VCHG            0x05  // Charge voltage
#define RT9478M_REG_EOC             0x06  // End of charge
#define RT9478M_REG_VBUS            0x07  // VBUS voltage
#define RT9478M_REG_VBAT            0x08  // Battery voltage
#define RT9478M_REG_FAULT           0x09
#define RT9478M_REG_TIMER           0x0A

// Status bits (register 0x01)
#define RT9478M_STATUS_VBUS_PRESENT (1 << 7)
#define RT9478M_STATUS_CHARGING     (1 << 6)
#define RT9478M_STATUS_CHARGE_DONE  (1 << 5)
#define RT9478M_STATUS_FAULT        (1 << 4)

// Control bits
#define RT9478M_CTRL_CHARGE_ENABLE  (1 << 0)
#define RT9478M_CTRL_HIZ_MODE       (1 << 7)

// Battery configuration
#define RT9478M_VCHG_4S_LIPO        16000  // 16V for 4S LiPo (mV)
#define RT9478M_ICHG_1A             1000   // 1A charge current (mA)

// Function prototypes
void RT9478M_Write(uint8_t reg, uint8_t value);
uint8_t RT9478M_Read(uint8_t reg);
bool RT9478M_Init(void);
uint8_t RT9478M_GetDeviceID(void);
uint8_t RT9478M_GetStatus(void);
bool RT9478M_IsCharging(void);
void RT9478M_EnableCharging(bool enable);
void RT9478M_SetChargeCurrent_mA(uint16_t current_ma);
void RT9478M_SetChargeVoltage_mV(uint16_t voltage_mv);

#endif // _RT9478M_h_
