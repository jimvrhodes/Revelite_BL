//
// fusb302.h
// Revelite USB-PD Controller - FUSB302BMPX Driver
//
#ifndef _fusb302_h_
#define _fusb302_h_

#include "project.h"
#include "cytypes.h"
#include <stdbool.h>

// FUSB302BMPX I2C Address
#define FUSB302_ADDR 0x22 // 7-bit I2C address

// FUSB302 Register Addresses
#define FUSB302_DEVICE_ID       0x01
#define FUSB302_SWITCHES0       0x02
#define FUSB302_SWITCHES1       0x03
#define FUSB302_MEASURE         0x04
#define FUSB302_SLICE           0x05
#define FUSB302_CONTROL0        0x06
#define FUSB302_CONTROL1        0x07
#define FUSB302_CONTROL2        0x08
#define FUSB302_CONTROL3        0x09
#define FUSB302_MASK            0x0A
#define FUSB302_POWER           0x0B
#define FUSB302_RESET           0x0C
#define FUSB302_OCPREG          0x0D
#define FUSB302_MASKA           0x0E
#define FUSB302_MASKB           0x0F
#define FUSB302_CONTROL4        0x10
#define FUSB302_STATUS0A        0x3C
#define FUSB302_STATUS1A        0x3D
#define FUSB302_INTERRUPTA      0x3E
#define FUSB302_INTERRUPTB      0x3F
#define FUSB302_STATUS0         0x40
#define FUSB302_STATUS1         0x41
#define FUSB302_INTERRUPT       0x42
#define FUSB302_FIFOS           0x43

// Device ID bits
#define FUSB302_VERSION_ID_MASK 0xF0
#define FUSB302_REVISION_ID_MASK 0x0F
#define FUSB302_VERSION_ID_302B 0x80

// SWITCHES0 register bits
#define FUSB302_SWITCHES0_PU_EN2    (1 << 7)  // Enable 3A pull-up current on CC2
#define FUSB302_SWITCHES0_PU_EN1    (1 << 6)  // Enable 3A pull-up current on CC1
#define FUSB302_SWITCHES0_VCONN_CC2 (1 << 5)  // Enable VCONN on CC2
#define FUSB302_SWITCHES0_VCONN_CC1 (1 << 4)  // Enable VCONN on CC1
#define FUSB302_SWITCHES0_MEAS_CC2  (1 << 3)  // Measure CC2
#define FUSB302_SWITCHES0_MEAS_CC1  (1 << 2)  // Measure CC1
#define FUSB302_SWITCHES0_PDWN2     (1 << 1)  // Pull-down on CC2
#define FUSB302_SWITCHES0_PDWN1     (1 << 0)  // Pull-down on CC1

// SWITCHES1 register bits
#define FUSB302_SWITCHES1_POWERROLE (1 << 7)  // 0=Sink, 1=Source
#define FUSB302_SWITCHES1_SPECREV_MASK 0x60   // PD Spec revision
#define FUSB302_SWITCHES1_DATAROLE  (1 << 4)  // 0=UFP, 1=DFP
#define FUSB302_SWITCHES1_AUTO_CRC  (1 << 2)  // Automatic CRC
#define FUSB302_SWITCHES1_TXCC2     (1 << 1)  // Transmit on CC2
#define FUSB302_SWITCHES1_TXCC1     (1 << 0)  // Transmit on CC1

// POWER register bits
#define FUSB302_POWER_PWR_MASK      0x0F
#define FUSB302_POWER_PWR_ALL       0x0F      // Power up everything

// CONTROL0 register bits
#define FUSB302_CONTROL0_TX_FLUSH   (1 << 6)  // Flush TX FIFO
#define FUSB302_CONTROL0_INT_MASK   (1 << 5)  // Interrupt mask
#define FUSB302_CONTROL0_HOST_CUR_MASK 0x0C   // Host current
#define FUSB302_CONTROL0_HOST_CUR_3A   0x0C   // 3A USB current
#define FUSB302_CONTROL0_HOST_CUR_1_5A 0x08   // 1.5A USB current
#define FUSB302_CONTROL0_HOST_CUR_USB  0x04   // USB default (500mA/900mA)

// CONTROL1 register bits
#define FUSB302_CONTROL1_ENSOP2DB   (1 << 6)  // Enable SOP'' Debug
#define FUSB302_CONTROL1_ENSOP1DB   (1 << 5)  // Enable SOP' Debug
#define FUSB302_CONTROL1_BIST_MODE2 (1 << 4)  // BIST Mode 2
#define FUSB302_CONTROL1_RX_FLUSH   (1 << 2)  // Flush RX FIFO
#define FUSB302_CONTROL1_ENSOP2     (1 << 1)  // Enable SOP''
#define FUSB302_CONTROL1_ENSOP1     (1 << 0)  // Enable SOP'

// CONTROL2 register bits
#define FUSB302_CONTROL2_TOG_SAVE_PWR  (3 << 6)  // Toggle state machine power save
#define FUSB302_CONTROL2_TOG_RD_ONLY   (1 << 5)  // Toggle Rd only
#define FUSB302_CONTROL2_WAKE_EN       (1 << 3)  // Wake interrupt enable
#define FUSB302_CONTROL2_MODE_MASK     0x06
#define FUSB302_CONTROL2_MODE_DRP      0x06      // DRP toggle
#define FUSB302_CONTROL2_MODE_SNK      0x04      // Sink only
#define FUSB302_CONTROL2_MODE_SRC      0x02      // Source only
#define FUSB302_CONTROL2_TOGGLE        (1 << 0)  // Toggle enable

// STATUS0A register bits
#define FUSB302_STATUS0A_SOFTFAIL   (1 << 5)  // Soft Reset received
#define FUSB302_STATUS0A_RETRYFAIL  (1 << 4)  // Retry counter exceeded
#define FUSB302_STATUS0A_POWER_MASK 0x0C      // Power role
#define FUSB302_STATUS0A_SOFTRST    (1 << 1)  // Soft Reset sent
#define FUSB302_STATUS0A_HARDRST    (1 << 0)  // Hard Reset sent

// STATUS0 register bits
#define FUSB302_STATUS0_VBUSOK      (1 << 7)  // VBUS OK
#define FUSB302_STATUS0_ACTIVITY    (1 << 6)  // PD activity
#define FUSB302_STATUS0_COMP        (1 << 5)  // Comparator output
#define FUSB302_STATUS0_CRC_CHK     (1 << 4)  // CRC check
#define FUSB302_STATUS0_ALERT       (1 << 3)  // Alert
#define FUSB302_STATUS0_WAKE        (1 << 2)  // Wake
#define FUSB302_STATUS0_BC_LVL_MASK 0x03      // BC level (CC termination)

// STATUS1 register bits
#define FUSB302_STATUS1_RXSOP2DB    (1 << 7)  // SOP'' Debug received
#define FUSB302_STATUS1_RXSOP1DB    (1 << 6)  // SOP' Debug received
#define FUSB302_STATUS1_RXSOP2      (1 << 5)  // SOP'' received
#define FUSB302_STATUS1_RXSOP1      (1 << 4)  // SOP' received
#define FUSB302_STATUS1_RX_EMPTY    (1 << 3)  // RX FIFO empty
#define FUSB302_STATUS1_RX_FULL     (1 << 2)  // RX FIFO full
#define FUSB302_STATUS1_TX_EMPTY    (1 << 1)  // TX FIFO empty
#define FUSB302_STATUS1_TX_FULL     (1 << 0)  // TX FIFO full

// BC_LVL values (CC termination detection)
#define FUSB302_BC_LVL_RA           0  // < 200mV (not connected)
#define FUSB302_BC_LVL_USB          1  // > 200mV, < 660mV (USB default)
#define FUSB302_BC_LVL_1500         2  // > 660mV, < 1230mV (1.5A)
#define FUSB302_BC_LVL_3000         3  // > 1230mV (3.0A)

// USB-PD Message Header
typedef union {
    struct {
        uint16_t message_type : 5;       // Message type
        uint16_t port_data_role : 1;     // 0=UFP, 1=DFP
        uint16_t spec_revision : 2;      // PD spec revision
        uint16_t port_power_role : 1;    // 0=Sink, 1=Source
        uint16_t message_id : 3;         // Message ID
        uint16_t num_data_objects : 3;   // Number of data objects
        uint16_t extended : 1;           // Extended message
    } fields;
    uint16_t raw;
} FUSB302_PDHeader_t;

// USB-PD Power Data Object (PDO) for Source Capabilities
typedef union {
    struct {
        uint32_t max_current : 10;       // Maximum current in 10mA units
        uint32_t voltage : 10;           // Voltage in 50mV units
        uint32_t peak_current : 2;       // Peak current capability
        uint32_t reserved : 3;
        uint32_t data_role_swap : 1;     // DR_Swap supported
        uint32_t usb_comm_capable : 1;   // USB communications capable
        uint32_t ext_powered : 1;        // Externally powered
        uint32_t usb_suspend : 1;        // USB suspend supported
        uint32_t dual_role_power : 1;    // Dual role power
        uint32_t supply_type : 2;        // 00=Fixed supply
    } fixed;
    uint32_t raw;
} FUSB302_PDO_t;

// FUSB302 Status Structure
typedef struct {
    uint8_t device_id;
    uint8_t status0;
    uint8_t status1;
    bool vbus_ok;
    bool cc_connected;
    uint8_t cc_termination;  // BC_LVL value
    bool pd_active;
    uint16_t negotiated_voltage_mv;  // Negotiated voltage in mV
    uint16_t negotiated_current_ma;  // Negotiated current in mA
} FUSB302_Status_t;

// Function Prototypes
void FUSB302_Write(uint8_t reg, uint8_t value);
uint8_t FUSB302_Read(uint8_t reg);
bool FUSB302_Init(void);
FUSB302_Status_t FUSB302_GetStatus(void);
bool FUSB302_IsVBusPresent(void);
uint8_t FUSB302_GetCCTermination(void);
bool FUSB302_RequestPower(uint16_t voltage_mv, uint16_t current_ma);
void FUSB302_ResetPD(void);

#endif // End of _fusb302_h_
