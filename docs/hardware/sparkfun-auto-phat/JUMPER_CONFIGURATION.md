# SparkFun Auto pHAT Jumper Configuration Guide

Complete reference for all configuration jumpers on the SparkFun Auto pHAT, including I2C address configuration and hardware options.

## Jumper Locations

All configuration jumpers are located on the **back (bottom) of the board**. Refer to the photo below:

![Board Back with Jumpers](photos/board_back.png)

**Jumper Groups Visible**:
- **SRV ADD** (Servo Address) - 6 pads, top right area
- **IMU ADD** (IMU Address) - Single jumper pad
- **MTR ADD** (Motor Address) - Multiple pads
- **ENC ADD** (Encoder Address) - Multiple pads

## Jumper Configuration Methods

### Open vs. Closed

- **Open** (default): No solder bridge between pads
- **Closed**: Solder bridge connecting the two pads

**To close a jumper**:
1. Use soldering iron with fine tip
2. Apply small amount of solder to bridge the gap
3. Verify connection with multimeter (should show ~0Ω resistance)

**To open a jumper**:
1. Use solder wick or desoldering pump
2. Remove all solder between pads
3. Verify isolation (should show infinite resistance)

---

## SRV ADD - Servo Controller (PCA9685) Address

### Location

Top right area of board back, labeled `SRV ADD` with 6 jumper pads: A0, A1, A2, A3, A4, A5

### Address Configuration

The PCA9685 I2C address is calculated as:

```
Address = 0x40 + (A5<<5 | A4<<4 | A3<<3 | A2<<2 | A1<<1 | A0<<0)
```

Each closed jumper adds its binary value to the base address of 0x40.

### Address Table

| A5 | A4 | A3 | A2 | A1 | A0 | Hex Offset | Final Address | Binary |
|----|----|----|----|----|----|------------|---------------|--------|
| 0  | 0  | 0  | 0  | 0  | 0  | +0x00      | **0x40** (default) | 1000000 |
| 0  | 0  | 0  | 0  | 0  | 1  | +0x01      | 0x41 | 1000001 |
| 0  | 0  | 0  | 0  | 1  | 0  | +0x02      | 0x42 | 1000010 |
| 0  | 0  | 0  | 0  | 1  | 1  | +0x03      | 0x43 | 1000011 |
| 0  | 0  | 0  | 1  | 0  | 0  | +0x04      | 0x44 | 1000100 |
| 0  | 0  | 0  | 1  | 0  | 1  | +0x05      | 0x45 | 1000101 |
| 0  | 0  | 0  | 1  | 1  | 0  | +0x06      | 0x46 | 1000110 |
| 0  | 0  | 0  | 1  | 1  | 1  | +0x07      | 0x47 | 1000111 |

**Note**: Only the first 8 addresses (0x40-0x47) are typically used. Closing A3, A4, or A5 allows for additional address space but is rarely needed.

### Common Configurations

#### Default (All Open)

```
A5 [ ] A4 [ ] A3 [ ] A2 [ ] A1 [ ] A0 [ ]
```
**Address**: 0x40

#### Example: 0x41

```
A5 [ ] A4 [ ] A3 [ ] A2 [ ] A1 [ ] A0 [X]  (A0 closed)
```
**Address**: 0x41

#### Example: 0x42

```
A5 [ ] A4 [ ] A3 [ ] A2 [ ] A1 [X] A0 [ ]  (A1 closed)
```
**Address**: 0x42

#### Example: 0x43

```
A5 [ ] A4 [ ] A3 [ ] A2 [ ] A1 [X] A0 [X]  (A1 and A0 closed)
```
**Address**: 0x43

### Multiple PCA9685 Boards

If you stack multiple Auto pHATs or add external PCA9685 boards via Qwiic, configure each with a unique address to avoid conflicts.

**Example with 3 boards**:
- Board 1: All open → 0x40
- Board 2: A0 closed → 0x41
- Board 3: A1 closed → 0x42

### Verification

After changing jumpers:

```bash
# Power cycle the board
sudo reboot

# Scan I2C bus
sudo i2cdetect -y 7

# Look for device at new address
```

---

## IMU ADD - IMU (ICM-20948) Address

### Location

Single jumper pad labeled `IMU ADD` or `AD0`

### Address Configuration

| Jumper State | AD0 Pin | I2C Address |
|--------------|---------|-------------|
| **Closed** (default) | HIGH | **0x69** |
| Open | LOW | 0x68 |

### Factory Default

**The IMU ADD jumper is CLOSED by default**, setting the address to **0x69**.

This is done to avoid conflict with other common I2C devices that default to 0x68 (like the MPU6050).

### When to Change

Open the jumper (change to 0x68) if:
- You have another device at 0x69 on the same I2C bus
- You want to use two ICM-20948 IMUs (one at 0x68, one at 0x69)

### Verification

```bash
# Check current IMU address
sudo i2cdetect -y 7

# Should show:
# - 69 if jumper is closed (default)
# - 68 if jumper is open
```

---

## MTR ADD - Motor Driver (PSoC 4245) Address

### Location

Multiple jumper pads labeled `MTR ADD`

### Address Configuration

The motor driver (PSoC 4245) supports 10 different I2C addresses in the range **0x58 to 0x61**.

**Default address**: **0x5D**

### Address Selection

The specific jumper configuration depends on the board revision. Consult the schematic for the exact mapping:

```bash
# View schematic
open docs/hardware/sparkfun-auto-phat/datasheets/SparkFun_Auto_pHAT_Schematic.pdf
# Look for MTR ADD jumper connections
```

### Common Addresses

| Configuration | Address |
|---------------|---------|
| Default (all open) | 0x5D |
| Various jumper combos | 0x58 - 0x61 |

### Verification

```bash
# Check current motor driver address
sudo i2cdetect -y 7

# Should show device at 0x5D by default
```

---

## ENC ADD - Encoder Reader (ATtiny84) Address

### Location

Jumper pads labeled `ENC ADD`

### Address Configuration

The encoder reader (ATtiny84) has a **fixed I2C address of 0x73** in the default firmware.

**Note**: Unlike other components, the encoder address is typically not hardware-configurable via jumpers alone. The ENC ADD jumpers may be for:
- Future firmware options
- Hardware ID detection
- Interrupt configuration

### Default

**Address**: **0x73** (fixed in firmware)

### Verification

```bash
# Check encoder reader address
sudo i2cdetect -y 7

# Should show device at 0x73
```

---

## Other Jumpers and Pads

### FUSE JP (if present)

Some boards include a fuse jumper for power protection. Check schematic for specific function.

### Test Points

Various test points on the board are for manufacturing testing and debugging:
- Not intended for jumper configuration
- Can be used for voltage measurement with multimeter

---

## Jumper Configuration Best Practices

### Before Changing Jumpers

1. **Power off completely**: Disconnect all power sources
2. **Document current state**: Take photo or write down current jumper positions
3. **Plan address scheme**: Ensure no conflicts with other I2C devices
4. **Check schematic**: Verify jumper function in official schematic

### After Changing Jumpers

1. **Visual inspection**: Verify solder bridges are clean (no shorts to adjacent pads)
2. **Continuity test**: Use multimeter to confirm closed jumpers have 0Ω resistance
3. **Power cycle**: Reboot system completely
4. **Scan bus**: Use `i2cdetect` to verify new address
5. **Update configuration**: Modify `config/hardware/phat_params.yaml` with new addresses

### Common Mistakes

❌ **Bridging wrong pads**: Creates short circuit or wrong address
❌ **Forgetting to update software config**: Code still tries old address
❌ **Creating address conflict**: Two devices at same address
❌ **Poor solder connection**: Intermittent connection causes random failures

✅ **Use flux**: Helps solder flow cleanly
✅ **Fine tip iron**: Prevents bridging adjacent pads
✅ **Test with multimeter**: Verify each jumper
✅ **Document changes**: Update config files and comments

---

## Troubleshooting Jumper Issues

### Device Not Detected After Changing Address

1. **Verify jumper is actually closed/open**:
   ```bash
   # Power off, use multimeter
   # Closed jumper: ~0Ω
   # Open jumper: infinite resistance
   ```

2. **Check for solder bridges to adjacent pads**:
   - Use magnifying glass or microscope
   - Clean with solder wick if needed

3. **Scan all possible addresses**:
   ```bash
   # For servo controller
   for addr in {0x40..0x47}; do
     printf "0x%02x: " $addr
     sudo i2cget -y 7 $addr 0x00 2>&1 | grep -q "0x" && echo "FOUND" || echo "not found"
   done
   ```

4. **Verify power cycle**:
   - Some chips cache address at power-on
   - Full power cycle required after jumper change

### Multiple Devices at Same Address

**Symptom**: I2C communication errors, random data, bus hangs

**Solution**:
1. Scan bus to identify conflict
2. Change one device to different address
3. Update software configuration
4. Retest

---

## Configuration Examples

### Single Auto pHAT (Default)

All jumpers in factory default positions:

```
Device            Address   Jumpers
─────────────────────────────────────
Servo (PCA9685)   0x40      All open
IMU (ICM-20948)   0x69      Closed (AD0 high)
Motor (PSoC)      0x5D      Default
Encoder (ATtiny)  0x73      N/A
```

**Config file** (`config/hardware/phat_params.yaml`):
```yaml
accelerometer_address: 0x69
servo_i2c_address: 0x40
# Motor and encoder addresses in respective config sections
```

### Two Auto pHATs Stacked

Configure second board to avoid conflicts:

**Board 1** (bottom):
```
Servo: 0x40 (all open)
IMU: 0x69 (closed)
Motor: 0x5D (default)
Encoder: 0x73
```

**Board 2** (top):
```
Servo: 0x41 (A0 closed)
IMU: 0x68 (open)
Motor: 0x5E (change MTR ADD)
Encoder: Must use different I2C bus or disable
```

---

## Reference

### I2C Address Space Map (Auto pHAT Default)

```
0x00-0x3F: Reserved / Other devices
0x40:      PCA9685 Servo Controller ← Default
0x41-0x47: PCA9685 alternative addresses
0x48-0x57: Available
0x58-0x61: PSoC Motor Driver range
0x5D:      PSoC Motor Driver ← Default
0x62-0x67: Available
0x68:      ICM-20948 alternative
0x69:      ICM-20948 IMU ← Default
0x6A-0x72: Available
0x73:      ATtiny84 Encoder Reader ← Default
0x74-0x7F: Available / Reserved
```

---

## Additional Resources

- **Schematic**: See `datasheets/SparkFun_Auto_pHAT_Schematic.pdf` for complete jumper connections
- **PCA9685 Datasheet**: See address configuration details in `datasheets/PCA9685_NXP_Datasheet.pdf`
- **I2C Troubleshooting**: See [I2C_TROUBLESHOOTING.md](I2C_TROUBLESHOOTING.md) for debugging help

---

**Next Steps**: If changing addresses, update your configuration in `config/hardware/phat_params.yaml` to match the new hardware settings.
