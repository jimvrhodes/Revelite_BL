# BQ25730 Device ID Read Fix

## Problem
The BQ25730/RT9478M charger chip was returning incorrect values when reading the Manufacturer ID and Device ID registers:
- **Manufacturer ID**: Reading `0xFFFF` instead of expected `0x001E`
- **Device ID**: Reading `0x0EFF` instead of expected `0x001C`

## Root Cause
The RT9478M (hardware-compatible replacement for BQ25730) requires a specific unlock sequence before registers can be read properly. Additionally, SMBus timing requirements needed proper delays between write and read operations.

## Solution

### 1. Register Unlock Sequence
The RT9478M requires writing to register `0x40` (AuxFunction) with bit 7 set to `1` **before** any other register access:

```c
// Must be first operation after power-up
BQ25730_Write(0x40, 0x8100);  // Set bit 7 = 1 to unlock register access
CyDelay(50);  // Allow unlock to take effect
```

This unlock must happen:
- **Before** reading any registers (including device IDs)
- **Before** configuring any charger settings
- After each power cycle or reset

### 2. SMBus Timing Requirements
Added a 100µs delay between the register address write and the read operation in `BQ25730_Read()`:

```c
uint16_t BQ25730_Read(uint8_t reg) {
    // Write register address
    write_buf[0] = reg;
    I2CM_SyncWrite(BQ25730_ADDR, write_buf, 1);
    
    // SMBus timing delay
    CyDelayUs(100);  // 100µs delay for register settling
    
    // Read 2 bytes
    I2CM_SyncRead(BQ25730_ADDR, read_buf, 2);
    
    return read_buf[0] | (read_buf[1] << 8);
}
```

### 3. Enhanced Initialization with ID Verification
The `BQ25730_Init()` function now:
1. Performs the unlock sequence first
2. Reads and verifies the device IDs
3. Provides debug output showing the actual IDs read
4. Identifies whether it's a genuine TI BQ25730 or compatible device

## Expected Output
With DEBUGOUT enabled, you should now see:
```
Init BQ25730...
BQ25730: MfgID=0x001E DevID=0x001C
BQ25730: Verified TI BQ25730
BQ25730 Init OK
```

Or for RT9478M (compatible device):
```
Init BQ25730...
BQ25730: MfgID=0xXXXX DevID=0xXXXX
BQ25730: Compatible device detected
BQ25730 Init OK
```

## Files Modified
- `battery.c` - Updated `BQ25730_Read()` with timing delay and `BQ25730_Init()` with unlock sequence
- `main.c` - Removed duplicate ID reading (now handled in init function)

## Technical Notes
- The RT9478M is designed as a drop-in replacement for the BQ25730 but has this additional unlock requirement
- The unlock register (0x40) is specific to the Richtek RT9478M and may not be needed for genuine TI BQ25730 chips
- However, writing to this register on a genuine BQ25730 should be harmless
- The 100µs read delay is a conservative value that ensures proper SMBus timing for both chip variants

## Testing
After this fix, verify:
1. Device IDs read correctly (not 0xFFFF)
2. Charger status register reads return valid data
3. ADC readings (VBUS, VSYS, VBAT) are reasonable values
4. Charger responds to configuration register writes
