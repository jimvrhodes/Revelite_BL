// File: CLI.c
// Project: Revelite - Simple Command Line Interface
// Minimal flash footprint CLI for debugging battery charger and fuel gauge

#include "project.h"
#include "main.h"
#include "CLI.h"
#include "battery.h"
#include "LTC2944.h"
#include <stdio.h>
#include <string.h>

// CLI state
static char cli_buffer[CLI_BUFFER_SIZE];
static uint8_t cli_index = 0;

// Helper function to send string via UART
static void CLI_Print(const char* str) {
    UART_SpiUartPutArray((const uint8_t*)str, strlen(str));
    while(UART_SpiUartGetTxBufferSize() != 0);
}

// Initialize CLI
void CLI_Init(void) {
    cli_index = 0;
    memset(cli_buffer, 0, CLI_BUFFER_SIZE);
    CLI_Print("\r\n*** Revelite Battery System CLI ***\r\n");
    CLI_Print("Type 'h' for help\r\n> ");
}

// Process incoming byte
void CLI_ProcessByte(char byte) {
    // Echo character
    char echo[2] = {byte, '\0'};
    if (byte >= 32 && byte < 127) {
        UART_SpiUartPutArray((const uint8_t*)echo, 1);
    }
    
    // Handle backspace
    if (byte == 8 || byte == 127) {
        if (cli_index > 0) {
            cli_index--;
            cli_buffer[cli_index] = '\0';
            CLI_Print("\b \b");  // Backspace, space, backspace
        }
        return;
    }
    
    // Handle newline/carriage return
    if (byte == '\r' || byte == '\n') {
        CLI_Print("\r\n");
        
        if (cli_index > 0) {
            // Process command
            char cmd = cli_buffer[0];
            
            switch(cmd) {
                case 'h':  // Help
                case 'H':
                case '?':
                    CLI_PrintHelp();
                    break;
                    
                case 'b':  // Battery charger status
                case 'B':
                    CLI_PrintBatteryStatus();
                    break;
                    
                case 'f':  // Fuel gauge status
                case 'F':
                    CLI_PrintFuelGaugeStatus();
                    break;
                    
                case 'i':  // Initialize both
                case 'I':
                    CLI_Print("Initializing charger...\r\n");
                    if (BQ25730_Init()) {
                        CLI_Print("  Charger OK\r\n");
                    } else {
                        CLI_Print("  Charger FAILED\r\n");
                    }
                    
                    CLI_Print("Initializing fuel gauge...\r\n");
                    if (LTC2944_Init((uint16_t)BATTERY_CAPACITY_MAH, LTC2944_PRESCALER_M_1024)) {
                        CLI_Print("  Fuel gauge OK\r\n");
                    } else {
                        CLI_Print("  Fuel gauge FAILED\r\n");
                    }
                    break;
                    
                case 'e':  // Enable charging
                case 'E':
                    BQ25730_EnableCharging(true);
                    CLI_Print("Charging ENABLED\r\n");
                    break;
                    
                case 'd':  // Disable charging
                case 'D':
                    BQ25730_EnableCharging(false);
                    CLI_Print("Charging DISABLED\r\n");
                    break;
                    
                case 'a':  // All status
                case 'A':
                    CLI_PrintBatteryStatus();
                    CLI_Print("\r\n");
                    CLI_PrintFuelGaugeStatus();
                    break;
                    
                case 'r':  // Read charger register
                case 'R':
                    CLI_Print("Reading BQ25730 registers...\r\n");
                    CLI_ReadChargerRegisters();
                    break;
                    
                case 's':  // I2C scan
                case 'S':
                    CLI_Print("Scanning I2C bus...\r\n");
                    CLI_I2CScan();
                    break;
                    
                case 'w':  // Write test
                case 'W':
                    CLI_Print("Writing ChargeOption0 to disable VSYS_UVP...\r\n");
                    BQ25730_Write(BQ25730_CHARGE_OPTION_0, 0xE34E);
                    CyDelay(100);
                    uint16_t readback = BQ25730_Read(BQ25730_CHARGE_OPTION_0);
                    char buf[60];
                    sprintf(buf, "  Wrote 0xE34E, readback: 0x%04X\r\n", readback);
                    CLI_Print(buf);
                    break;
                    
                default:
                    CLI_Print("Unknown command. Type 'h' for help\r\n");
                    break;
            }
        }
        
        // Reset buffer
        cli_index = 0;
        memset(cli_buffer, 0, CLI_BUFFER_SIZE);
        CLI_Print("> ");
        return;
    }
    
    // Add to buffer if printable and space available
    if (byte >= 32 && byte < 127) {
        if (cli_index < CLI_BUFFER_SIZE - 1) {
            cli_buffer[cli_index++] = byte;
            cli_buffer[cli_index] = '\0';
        }
    }
}

// Print help menu
void CLI_PrintHelp(void) {
    CLI_Print("Commands:\r\n");
    CLI_Print("  h - Help (this menu)\r\n");
    CLI_Print("  i - Initialize charger & fuel gauge\r\n");
    CLI_Print("  b - Battery charger status\r\n");
    CLI_Print("  f - Fuel gauge status\r\n");
    CLI_Print("  a - All status (charger + fuel gauge)\r\n");
    CLI_Print("  e - Enable charging\r\n");
    CLI_Print("  d - Disable charging\r\n");
    CLI_Print("  r - Read charger registers (debug)\r\n");
    CLI_Print("  s - Scan I2C bus (debug)\r\n");
    CLI_Print("  w - Write ChargeOption0 test (disable UVP)\r\n");
}

// Print battery charger status
void CLI_PrintBatteryStatus(void) {
    char buf[100];
    BatteryStatus_t status = BQ25730_GetStatus();
    
    CLI_Print("=== Battery Charger (BQ25730) ===\r\n");
    
    sprintf(buf, "  Status Reg: 0x%04X\r\n", status.charger_status);
    CLI_Print(buf);
    
    sprintf(buf, "  Battery:    %u mV, %d mA\r\n", 
            status.battery_voltage_mv, status.battery_current_ma);
    CLI_Print(buf);
    
    sprintf(buf, "  VBUS:       %u mV\r\n", status.vbus_voltage_mv);
    CLI_Print(buf);
    
    sprintf(buf, "  System:     %u mV\r\n", status.system_voltage_mv);
    CLI_Print(buf);
    
    CLI_Print("  AC Present: ");
    CLI_Print(status.ac_present ? "YES\r\n" : "NO\r\n");
    
    CLI_Print("  Charging:   ");
    CLI_Print(status.is_charging ? "YES\r\n" : "NO\r\n");
    
    CLI_Print("  Faults:     ");
    CLI_Print(status.fault_present ? "YES\r\n" : "NONE\r\n");
}

// Print fuel gauge status
void CLI_PrintFuelGaugeStatus(void) {
    char buf[100];
    FuelGauge_t status = LTC2944_GetStatus();
    
    CLI_Print("=== Fuel Gauge (LTC2944) ===\r\n");
    
    sprintf(buf, "  Status Reg: 0x%02X\r\n", status.status_reg);
    CLI_Print(buf);
    
    sprintf(buf, "  Voltage:    %u mV (raw:0x%04X)\r\n", 
            (uint16_t)status.voltage_mv, status.voltage_raw);
    CLI_Print(buf);
    
    sprintf(buf, "  Current:    %d mA (raw:0x%04X)\r\n", 
            (int16_t)status.current_ma, status.current_raw);
    CLI_Print(buf);
    
    sprintf(buf, "  Temp:       %d C (raw:0x%04X)\r\n", 
            (int16_t)status.temperature_c, status.temperature_raw);
    CLI_Print(buf);
    
    sprintf(buf, "  Acc Charge: %u mAh (raw:0x%04X)\r\n", 
            (uint16_t)status.accumulated_charge_mAh, status.accumulated_charge_raw);
    CLI_Print(buf);
    
    sprintf(buf, "  SOC:        %u %%\r\n", (uint16_t)status.state_of_charge);
    CLI_Print(buf);
    
    CLI_Print("  Alerts:     ");
    CLI_Print(status.alert_present ? "YES\r\n" : "NONE\r\n");
}

// Read and display key charger registers for debugging
void CLI_ReadChargerRegisters(void) {
    char buf[80];
    
    // Try to read manufacturer ID and device ID
    uint16_t mfr_id = BQ25730_Read(BQ25730_MANUFACTURER_ID);
    uint16_t dev_id = BQ25730_Read(BQ25730_DEVICE_ID);
    
    sprintf(buf, "  Mfr ID:     0x%04X (expect 0x001E for RT9478M)\r\n", mfr_id);
    CLI_Print(buf);
    
    sprintf(buf, "  Dev ID:     0x%04X (expect 0x001C for RT9478M)\r\n", dev_id);
    CLI_Print(buf);
    
    // Read key configuration registers
    uint16_t charge_opt0 = BQ25730_Read(BQ25730_CHARGE_OPTION_0);
    uint16_t charge_curr = BQ25730_Read(BQ25730_CHARGE_CURRENT);
    uint16_t charge_volt = BQ25730_Read(BQ25730_MAX_CHARGE_VOLTAGE);
    uint16_t status = BQ25730_Read(BQ25730_CHARGER_STATUS);
    
    sprintf(buf, "  ChgOpt0:    0x%04X\r\n", charge_opt0);
    CLI_Print(buf);
    
    sprintf(buf, "  ChgCurr:    0x%04X (%u mA)\r\n", charge_curr, charge_curr * 64);
    CLI_Print(buf);
    
    sprintf(buf, "  ChgVolt:    0x%04X (%u mV)\r\n", charge_volt, charge_volt * 8);
    CLI_Print(buf);
    
    sprintf(buf, "  Status:     0x%04X\r\n", status);
    CLI_Print(buf);
}

// Scan I2C bus for devices
void CLI_I2CScan(void) {
    char buf[60];
    uint8_t found = 0;
    
    CLI_Print("  Scanning 0x00-0x7F...\r\n");
    
    for (uint8_t addr = 0x08; addr < 0x78; addr++) {
        uint8_t dummy = 0;
        
        // Try to write 0 bytes (just address) to check for ACK
        I2CM_I2CMasterClearStatus();
        uint32_t status = I2CM_I2CMasterWriteBuf(addr, &dummy, 0, I2CM_I2C_MODE_COMPLETE_XFER);
        
        // Wait for operation to complete with timeout
        uint16_t timeout = 100;
        while ((I2CM_I2CMasterStatus() & I2CM_I2C_MSTAT_XFER_INP) && timeout > 0) {
            CyDelayUs(10);
            timeout--;
        }
        
        // Check if device ACK'd
        uint32_t i2c_status = I2CM_I2CMasterStatus();
        if ((status == I2CM_I2C_MSTR_NO_ERROR) && 
            !(i2c_status & I2CM_I2C_MSTAT_ERR_XFER)) {
            found++;
            sprintf(buf, "    Found device at 0x%02X", addr);
            CLI_Print(buf);
            
            // Identify known devices
            if (addr == BQ25730_ADDR) {
                CLI_Print(" (BQ25730 Charger)\r\n");
            } else if (addr == LTC2944_ADDR) {
                CLI_Print(" (LTC2944 Fuel Gauge)\r\n");
            } else if (addr == 0x70 || addr == 0x71) {
                CLI_Print(" (PCLA9538 Latch)\r\n");
            } else {
                CLI_Print("\r\n");
            }
        }
        
        I2CM_I2CMasterClearStatus();
        CyDelay(1);
    }
    
    sprintf(buf, "  Scan complete. Found %u device(s)\r\n", found);
    CLI_Print(buf);
}
