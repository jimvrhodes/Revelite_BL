# I2C Devices in Revelite Battery System

This document lists all I2C devices on the system bus and their addresses.

## Device List

| Address | Device              | Description                                      |
|---------|---------------------|--------------------------------------------------|
| 0x22    | FUSB302BMPX         | USB-PD Controller (USB Type-C Power Delivery)    |
| 0x64    | LTC2944             | Fuel Gauge (Battery State of Charge monitoring)  |
| 0x6B    | BQ25730 / RT9478M   | Battery Charger (4S LiPo, USB-C PD input)        |
| 0x70    | LATCH1 (PCA9538)    | I/O Expander - LED indicators                    |
| 0x71    | LATCH2 (PCA9538)    | I/O Expander - Boost enable and other controls   |

## Device Details

### FUSB302BMPX (0x22) - USB-PD Controller
- **Driver**: `fusb302.c` / `fusb302.h`
- **Function**: Manages USB Type-C connection and Power Delivery negotiation
- **Mode**: Configured as sink (UFP) to receive power from USB-C charger
- **Features**:
  - Automatic CC line detection
  - VBUS presence detection
  - USB Type-C current advertisement detection (500mA, 1.5A, 3A)
  - PD message handling (basic implementation)
- **Status**: Basic initialization and monitoring implemented. Full USB-PD negotiation for higher voltages/currents is a future enhancement.

### LTC2944 (0x64) - Fuel Gauge
- **Driver**: `LTC2944.c` / `LTC2944.h`
- **Function**: Measures battery state of charge using coulomb counting
- **Configuration**: 
  - Battery capacity: ~3500mAh (50.4Wh @ 14.4V)
  - Sense resistor: 15mΩ
  - Prescaler: M_1024

### BQ25730 / RT9478M (0x6B) - Battery Charger
- **Driver**: `battery.c` / `battery.h`
- **Function**: Manages 4S LiPo battery charging
- **Note**: Hardware uses Richtek RT9478M, which is register-compatible with TI BQ25730
- **Configuration**:
  - Max charge voltage: 16.0V (4.0V per cell)
  - Charge current: 1A
  - Low cutoff: 12.5V (3.125V per cell)
  - Input: USB-C PD (configurable up to 20V)
  
### LATCH1 & LATCH2 (0x70, 0x71) - I/O Expanders
- **IC**: PCA9538 8-bit I/O expanders
- **Function**: Drive LEDs and control enables
- **LATCH1 (0x70)**: Battery charge status LEDs (LED1-LED4)
- **LATCH2 (0x71)**: Boost converter enable, other system controls

## Historical Notes

Previously, there was confusion about the I2C address 0x22. It was initially thought to be an alternate address for the BQ25730 charger, but investigation revealed it is actually the FUSB302BMPX USB-PD controller. The BQ25730-compatible charger (RT9478M) is correctly at address 0x6B.

## Software Integration

All devices are initialized in `main.c` during system startup:
1. I2C bus started
2. Latches configured
3. Quadrature decoder initialized
4. BQ25730/RT9478M charger initialized
5. LTC2944 fuel gauge initialized
6. FUSB302 USB-PD controller initialized

Debug output is available when `DEBUGOUT` is defined.
