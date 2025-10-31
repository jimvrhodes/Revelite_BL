//
// CLI.h
// Revelite - Simple Command Line Interface for debugging
//
#ifndef _CLI_h_
#define _CLI_h_

#include "project.h"
#include "cytypes.h"
#include <stdbool.h>

// CLI buffer size
#define CLI_BUFFER_SIZE 64

// Function prototypes
void CLI_Init(void);
void CLI_ProcessByte(char byte);
void CLI_PrintHelp(void);
void CLI_PrintBatteryStatus(void);
void CLI_PrintFuelGaugeStatus(void);
void CLI_ReadChargerRegisters(void);
void CLI_I2CScan(void);

#endif // _CLI_h_
