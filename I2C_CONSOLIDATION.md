# I2C Function Consolidation

## Summary
Consolidated all device-specific I2C read/write functions into a unified set of generic functions in `subs.c`. All devices now use the same I2C implementation with proper repeated start for register reads.

## Unified Functions (in subs.c)

### Buffer Operations (existing, kept as-is)
```c
bool I2CM_SyncWrite(uint8 bySlaveAddr, uint8 *buffer, uint8 length);
bool I2CM_SyncRead(uint8 bySlaveAddr, uint8 *buffer, uint8 length);
```

### New Single Byte Operations
```c
uint8_t I2C_ReadByte(uint8_t addr, uint8_t reg);          // Read 8-bit with repeated start
void I2C_WriteByte(uint8_t addr, uint8_t reg, uint8_t value);
```

### New Word (16-bit) Operations
```c
uint16_t I2C_ReadWord(uint8_t addr, uint8_t reg);         // Read 16-bit LSB-first with repeated start
uint16_t I2C_ReadWord_MSB(uint8_t addr, uint8_t reg);     // Read 16-bit MSB-first with repeated start
void I2C_WriteWord(uint8_t addr, uint8_t reg, uint16_t value);
```

## Device Updates

### battery.c (BQ25730/RT9478M Charger)
**Before:**
- Custom `BQ25730_Write()` using buffer API
- Complex `BQ25730_Read()` with WriteBuf/ReadBuf and NO_STOP/REPEAT_START flags

**After:**
```c
void BQ25730_Write(uint8_t reg, uint16_t value) {
    I2C_WriteWord(BQ25730_ADDR, reg, value);
}

uint16_t BQ25730_Read(uint8_t reg) {
    return I2C_ReadWord(BQ25730_ADDR, reg);
}
```
- Uses LSB-first format (standard SMBus)
- Proper repeated start
- **Reduced from ~60 lines to 4 lines**

### LTC2944.c (Fuel Gauge)
**Before:**
- Custom low-level I2C API calls with SendStart/SendRestart/WriteByte/ReadByte
- Separate implementations for 8-bit and 16-bit reads

**After:**
```c
void LTC2944_Write(uint8_t reg, uint8_t value) {
    I2C_WriteByte(LTC2944_ADDR, reg, value);
}

uint8_t LTC2944_Read(uint8_t reg) {
    return I2C_ReadByte(LTC2944_ADDR, reg);
}

uint16_t LTC2944_Read16(uint8_t reg_msb) {
    return I2C_ReadWord_MSB(LTC2944_ADDR, reg_msb);
}
```
- Uses MSB-first format (LTC2944 is big-endian)
- Proper repeated start
- **Reduced from ~60 lines to 7 lines**

### fusb302.c (USB-PD Controller)
**Before:**
- `FUSB302_Write()` using I2CM_SyncWrite with buffer
- `FUSB302_Read()` using separate SyncWrite + SyncRead (NO repeated start!)

**After:**
```c
void FUSB302_Write(uint8_t reg, uint8_t value) {
    I2C_WriteByte(FUSB302_ADDR, reg, value);
}

uint8_t FUSB302_Read(uint8_t reg) {
    return I2C_ReadByte(FUSB302_ADDR, reg);
}
```
- **Now uses repeated start** (was missing before!)
- **Reduced from ~27 lines to 4 lines**

## Benefits

1. **Consistency** - All devices use the same I2C implementation
2. **Repeated Start** - All reads now properly use repeated start (FUSB302 was missing this)
3. **Maintainability** - Single place to fix I2C timing issues
4. **Code Reduction** - Eliminated ~150 lines of duplicated code
5. **Endianness Support** - Separate functions for LSB-first and MSB-first devices

## Byte Order Reference

| Device | Byte Order | Function Used |
|--------|-----------|---------------|
| BQ25730 (Charger) | LSB-first (little-endian) | `I2C_ReadWord()` |
| LTC2944 (Fuel Gauge) | MSB-first (big-endian) | `I2C_ReadWord_MSB()` |
| FUSB302 (USB-PD) | 8-bit only | `I2C_ReadByte()` |

## Testing

After compilation:
1. Verify all devices still communicate correctly
2. Check that repeated start is present on logic analyzer for all reads
3. Confirm no regression in functionality

## Future Work

- Consider migrating `LatchWrite/LatchRead` to use unified functions
- Add error handling/return codes to unified functions if needed
- Consider adding I2C timeout error detection
