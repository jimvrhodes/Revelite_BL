// File: fusb302.c
// Project: Revelite - FUSB302BMPX USB-PD Controller Driver
//
//

#include "project.h"
#include "main.h"
#include "fusb302.h"
#include <stdio.h>

extern structInfo Info;

//=============================================================================
// Low-level I2C Functions
//=============================================================================

// Write 8-bit value to FUSB302 register
void FUSB302_Write(uint8_t reg, uint8_t value) {
    uint8_t buffer[2];
    buffer[0] = reg;
    buffer[1] = value;
    
    I2CM_SyncWrite(FUSB302_ADDR, buffer, 2);
}

// Read 8-bit value from FUSB302 register
uint8_t FUSB302_Read(uint8_t reg) {
    uint8_t write_buf[1];
    uint8_t read_buf[1];
    
    // Write register address
    write_buf[0] = reg;
    I2CM_SyncWrite(FUSB302_ADDR, write_buf, 1);
    
    // Read 1 byte
    I2CM_SyncRead(FUSB302_ADDR, read_buf, 1);
    
    return read_buf[0];
}

//=============================================================================
// FUSB302 Initialization and Configuration
//=============================================================================

// Initialize FUSB302 USB-PD controller as a sink device
bool FUSB302_Init(void) {
    
    // Read Device ID to verify communication
    uint8_t device_id = FUSB302_Read(FUSB302_DEVICE_ID);
    
#ifdef DEBUGOUT
    char buffer[50];
    uint16_t cnt = sprintf(buffer, "FUSB302: Device ID=0x%02X\r\n", device_id);
    UART_SpiUartPutArray((uint8*)buffer, cnt);
#endif
    
    // Check if this is a FUSB302B device
    if ((device_id & FUSB302_VERSION_ID_MASK) != FUSB302_VERSION_ID_302B) {
#ifdef DEBUGOUT
        UART_SpiUartPutArray((uint8*)"FUSB302: Warning - unexpected device ID\r\n", 42);
#endif
        // Continue anyway - might still work
    }
    
    CyDelay(10);
    
    // Perform a soft reset
    FUSB302_Write(FUSB302_RESET, 0x01);  // SW_RES bit
    CyDelay(10);
    
    // Power up all blocks
    FUSB302_Write(FUSB302_POWER, FUSB302_POWER_PWR_ALL);
    CyDelay(10);
    
    // Configure as UFP (device/sink) with pull-downs on both CC lines
    // This allows detection of which CC line the cable is using
    FUSB302_Write(FUSB302_SWITCHES0, FUSB302_SWITCHES0_PDWN1 | FUSB302_SWITCHES0_PDWN2);
    CyDelay(10);
    
    // Configure SWITCHES1 for sink mode, auto CRC
    // POWERROLE=0 (Sink), DATAROLE=0 (UFP), AUTO_CRC=1
    // Set PD spec revision to 2.0 (bits 5-6 = 01)
    FUSB302_Write(FUSB302_SWITCHES1, FUSB302_SWITCHES1_AUTO_CRC | 0x20);
    CyDelay(10);
    
    // Configure CONTROL0 - no interrupts masked, USB default current
    FUSB302_Write(FUSB302_CONTROL0, FUSB302_CONTROL0_HOST_CUR_USB);
    CyDelay(10);
    
    // Configure CONTROL1 - Enable only SOP messages (standard PD communication)
    FUSB302_Write(FUSB302_CONTROL1, 0x00);
    CyDelay(10);
    
    // Configure CONTROL2 - Sink mode with toggle for CC detection
    // MODE=SNK (Sink only), TOGGLE enabled for automatic CC detection
    FUSB302_Write(FUSB302_CONTROL2, FUSB302_CONTROL2_MODE_SNK | FUSB302_CONTROL2_TOGGLE);
    CyDelay(10);
    
    // Unmask interrupts we care about
    FUSB302_Write(FUSB302_MASK, 0x00);   // Unmask all for now
    FUSB302_Write(FUSB302_MASKA, 0x00);
    FUSB302_Write(FUSB302_MASKB, 0x00);
    CyDelay(10);
    
#ifdef DEBUGOUT
    UART_SpiUartPutArray((uint8*)"FUSB302: Init complete (Sink mode)\r\n", 37);
#endif
    
    return true;
}

//=============================================================================
// FUSB302 Status and Monitoring Functions
//=============================================================================

// Check if VBUS is present
bool FUSB302_IsVBusPresent(void) {
    uint8_t status0 = FUSB302_Read(FUSB302_STATUS0);
    return (status0 & FUSB302_STATUS0_VBUSOK) ? true : false;
}

// Get CC termination level (indicates current capability)
uint8_t FUSB302_GetCCTermination(void) {
    uint8_t status0 = FUSB302_Read(FUSB302_STATUS0);
    return (status0 & FUSB302_STATUS0_BC_LVL_MASK);
}

// Get comprehensive FUSB302 status
FUSB302_Status_t FUSB302_GetStatus(void) {
    FUSB302_Status_t status;
    
    // Read device ID
    status.device_id = FUSB302_Read(FUSB302_DEVICE_ID);
    
    // Read status registers
    status.status0 = FUSB302_Read(FUSB302_STATUS0);
    status.status1 = FUSB302_Read(FUSB302_STATUS1);
    
    // Parse status bits
    status.vbus_ok = (status.status0 & FUSB302_STATUS0_VBUSOK) ? true : false;
    status.pd_active = (status.status0 & FUSB302_STATUS0_ACTIVITY) ? true : false;
    status.cc_termination = status.status0 & FUSB302_STATUS0_BC_LVL_MASK;
    status.cc_connected = (status.cc_termination != FUSB302_BC_LVL_RA);
    
    // Estimate voltage/current based on BC_LVL if no PD negotiation
    // Note: Full PD negotiation requires state machine and message parsing
    switch (status.cc_termination) {
        case FUSB302_BC_LVL_USB:
            status.negotiated_voltage_mv = 5000;   // 5V USB default
            status.negotiated_current_ma = 500;    // 500mA default
            break;
        case FUSB302_BC_LVL_1500:
            status.negotiated_voltage_mv = 5000;   // 5V
            status.negotiated_current_ma = 1500;   // 1.5A
            break;
        case FUSB302_BC_LVL_3000:
            status.negotiated_voltage_mv = 5000;   // 5V
            status.negotiated_current_ma = 3000;   // 3A
            break;
        default:
            status.negotiated_voltage_mv = 0;
            status.negotiated_current_ma = 0;
            break;
    }
    
    return status;
}

//=============================================================================
// FUSB302 Control Functions
//=============================================================================

// Reset USB-PD protocol (send hard reset)
void FUSB302_ResetPD(void) {
    // Trigger a hard reset
    uint8_t control3 = FUSB302_Read(FUSB302_CONTROL3);
    control3 |= 0x40;  // SEND_HARD_RESET bit
    FUSB302_Write(FUSB302_CONTROL3, control3);
    
    CyDelay(100);
}

// Request specific power from USB-PD source
// Note: This is a simplified placeholder - full PD negotiation requires
// a state machine to handle GoodCRC, Accept/Reject messages, etc.
bool FUSB302_RequestPower(uint16_t voltage_mv, uint16_t current_ma) {
    // Full USB-PD power negotiation is complex and requires:
    // 1. Waiting for Source_Capabilities message
    // 2. Parsing available PDOs
    // 3. Sending Request message with desired PDO
    // 4. Waiting for Accept message
    // 5. Waiting for PS_RDY message
    // 6. Transitioning to new power level
    
    // For now, return false to indicate this is not yet implemented
#ifdef DEBUGOUT
    UART_SpiUartPutArray((uint8*)"FUSB302: Full PD negotiation not yet implemented\r\n", 51);
    UART_SpiUartPutArray((uint8*)"FUSB302: Using default USB Type-C current advertisement\r\n", 58);
#endif
    
    return false;
}
