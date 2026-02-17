# I2C Troubleshooting Guide for SparkFun Auto pHAT

Comprehensive guide for debugging I2C communication issues, with specific focus on the PCA9685 servo controller.

## Table of Contents

1. [I2C Fundamentals](#i2c-fundamentals)
2. [Pull-up vs Pull-down Resistors](#pull-up-vs-pull-down-resistors)
3. [Quick Diagnostic Checklist](#quick-diagnostic-checklist)
4. [Systematic Debugging Procedure](#systematic-debugging-procedure)
5. [PCA9685 Specific Troubleshooting](#pca9685-specific-troubleshooting)
6. [Common Issues and Solutions](#common-issues-and-solutions)
7. [Advanced Debugging with Oscilloscope](#advanced-debugging-with-oscilloscope)

---

## I2C Fundamentals

### How I2C Works

I2C (Inter-Integrated Circuit) is a two-wire serial communication protocol:

- **SDA (Serial Data)**: Bidirectional data line
- **SCL (Serial Clock)**: Clock signal from master

**Key Characteristics**:
- **Open-drain/Open-collector**: Devices can only pull lines LOW, never drive HIGH
- **Pull-up resistors required**: External resistors pull lines HIGH when no device is active
- **Multi-master capable**: Multiple devices can share the same bus
- **Addressable**: Each device has a unique 7-bit or 10-bit address

### Signal Levels

| Condition | SDA/SCL Voltage |
|-----------|----------------|
| **Idle** (no communication) | HIGH (~VCC) |
| **Active LOW** (device transmitting 0) | LOW (~0V) |
| **Active HIGH** (device releasing line) | HIGH via pull-up resistor |

**Critical**: If pull-up resistors are missing or incorrect, the bus cannot idle HIGH and communication fails.

---

## Pull-up vs Pull-down Resistors

### The Critical Difference

#### ❌ Pull-down Resistors (INCORRECT for I2C)

```
VCC (3.3V or 5V)
    |
  Device (can pull LOW)
    |
    +--- Signal Line (SDA or SCL)
    |
  [Resistor] ← PULL-DOWN resistor
    |
   GND
```

**Effect**: Signal line is always pulled toward GND (LOW)
- When device releases line → stays LOW (can't go HIGH)
- Bus cannot idle HIGH → **Communication fails**
- Typical symptom: No devices detected on I2C scan

#### ✅ Pull-up Resistors (CORRECT for I2C)

```
VCC (3.3V or 5V)
    |
  [Resistor] ← PULL-UP resistor
    |
    +--- Signal Line (SDA or SCL)
    |
  Device (can pull LOW)
    |
   GND
```

**Effect**: Signal line is pulled toward VCC (HIGH) when idle
- When device releases line → goes HIGH via resistor
- Bus idles HIGH → **Communication works**
- Devices can pull LOW to transmit 0 bits

### Correct Resistor Values for I2C

| Bus Speed | Recommended Pull-up | Range |
|-----------|---------------------|-------|
| Standard (100 kHz) | 4.7 kΩ | 2.2 kΩ - 10 kΩ |
| Fast (400 kHz) | 2.2 kΩ | 1 kΩ - 4.7 kΩ |
| Fast-mode Plus (1 MHz) | 1 kΩ | 500 Ω - 2.2 kΩ |

**Jetson Orin Nano with Auto pHAT**: Typically 3.3V logic, 400 kHz → Use 2.2 kΩ - 4.7 kΩ

### Built-in Pull-ups on Auto pHAT

The SparkFun Auto pHAT **includes built-in pull-up resistors** on the I2C bus. You should NOT need external resistors unless:

1. Bus capacitance is very high (long wires, many devices)
2. You're operating at high speed (>400 kHz)
3. The built-in resistors are damaged

**To verify built-in pull-ups are working**:
```bash
# Measure voltage on SDA/SCL pins with multimeter
# Should read ~3.3V (Jetson) or ~5V (RPi) when idle
```

---

## Quick Diagnostic Checklist

Before deep troubleshooting, verify these basics:

- [ ] **Power**: Board has 5V and 3.3V power (check LED if present)
- [ ] **GPIO pins aligned**: pHAT seated correctly on 40-pin header
- [ ] **I2C enabled**: `ls /dev/i2c-*` shows `/dev/i2c-7` (Jetson) or `/dev/i2c-1` (RPi)
- [ ] **Permissions**: User in `i2c` group (`groups | grep i2c`)
- [ ] **Tools installed**: `i2cdetect`, `i2cget`, `i2cset` available
- [ ] **No shorts**: Visual inspection for solder bridges or damage

---

## Systematic Debugging Procedure

### Step 1: Verify I2C Bus Exists

```bash
# List I2C buses
ls -l /dev/i2c-*

# Expected output (Jetson Orin Nano):
# /dev/i2c-0
# /dev/i2c-1
# /dev/i2c-7  ← This is the GPIO header I2C bus
```

**If `/dev/i2c-7` missing**:
- Check device tree configuration
- Verify I2C is enabled in `/boot/config.txt` (RPi) or device tree (Jetson)

### Step 2: Scan I2C Bus

```bash
# Scan bus 7 (Jetson Orin Nano GPIO I2C)
sudo i2cdetect -y 7

# Expected output with working Auto pHAT:
#      0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
# 00:          -- -- -- -- -- -- -- -- -- -- -- -- --
# 10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
# 20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
# 30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
# 40: 40 -- -- -- -- -- -- -- -- -- -- -- -- -- -- --  ← PCA9685 servo
# 50: -- -- -- -- -- -- -- -- -- -- -- -- -- 5d -- --  ← Motor driver
# 60: -- -- -- -- -- -- -- -- -- 69 -- -- -- -- -- --  ← ICM-20948 IMU
# 70: -- -- -- 73 -- -- -- --                          ← ATtiny84 encoder
```

### Step 3: Interpret Scan Results

| Result | Meaning | Next Step |
|--------|---------|-----------|
| `UU` | Kernel driver has claimed device | Normal, device is in use |
| Address (e.g., `69`) | Device detected and responding | Good |
| `--` | No device at this address | Device missing, powered off, or faulty |

### Step 4: Check Specific Addresses

If a device is missing (e.g., PCA9685 at 0x40):

```bash
# Try reading from the device
sudo i2cget -y 7 0x40 0x00

# Possible results:
# - "0x??" with a hex value: Device is working!
# - "Error: Read failed": Device not responding
# - "Error: Remote I/O error": No ACK from device
```

---

## PCA9685 Specific Troubleshooting

### Symptom: PCA9685 Not Detected at 0x40

**Observation**: IMU (0x69) works, but servo controller (0x40) doesn't appear on I2C scan.

### Diagnostic Steps

#### 1. Verify Power Supply

The PCA9685 requires 5V on VDD (pin 28):

```bash
# If you have a multimeter:
# - Measure voltage between VDD (pin 28) and GND on PCA9685 IC
# - Should read ~5V
# - If 0V → power supply issue
```

**Check**:
- USB-C power connected to Auto pHAT?
- 5V rail from Jetson providing enough current?
- Reverse polarity protection diode working?

#### 2. Check Output Enable (OE) Pin

The PCA9685 has an active-LOW output enable pin (pin 23).

**Requirements**:
- `/OE` must be pulled LOW (connected to GND) for chip to respond
- If floating or HIGH → chip is in standby, won't respond on I2C

**On the schematic**: Check if `/OE` is:
- Hardwired to GND (always enabled)
- Controlled by GPIO pin (needs software enable)
- Pulled up by resistor (incorrect - chip stays disabled)

**Test**:
```bash
# Check schematic: docs/hardware/sparkfun-auto-phat/datasheets/SparkFun_Auto_pHAT_Schematic.pdf
# Look for pin 23 of PCA9685 (labeled OE or /OE)
# Verify connection to GND or GPIO
```

#### 3. Verify I2C Address Jumpers

Default address is 0x40 when all address pins (A0-A5) are open.

**Check SRV ADD jumpers on back of board**:
- All pads open (not bridged) → 0x40
- Any pads bridged → address changes (see [JUMPER_CONFIGURATION.md](JUMPER_CONFIGURATION.md))

**Scan all possible addresses**:
```bash
# Try addresses 0x40 through 0x47
for addr in 40 41 42 43 44 45 46 47; do
  echo -n "Trying 0x$addr: "
  sudo i2cget -y 7 0x$addr 0x00 2>&1 | grep -q "0x" && echo "FOUND!" || echo "not found"
done
```

#### 4. Software Reset via General Call

The PCA9685 supports a software reset via I2C general call address:

```bash
# Send SWRST command (Software Reset)
sudo i2cset -y 7 0x00 0x06

# Wait a moment
sleep 0.5

# Rescan bus
sudo i2cdetect -y 7
```

**Note**: This sends the reset command to ALL PCA9685 devices on the bus (address 0x00 is "general call").

#### 5. Check for Hardware Damage

If you attempted to use **pull-down resistors** (incorrect), you may have:

1. **Overstressed I/O pins**: Constant current flow when devices tried to drive HIGH
2. **Damaged level shifters**: If board has 3.3V/5V translation
3. **Blown ESD protection diodes**: Excessive current through protection circuits

**Symptoms of damage**:
- Device never responds, even after power cycle
- Short circuit between SDA/SCL and GND (measure with multimeter)
- Excessive heat from IC

**Test for short**:
```bash
# Power off board
# Use multimeter in resistance mode
# Measure between SDA (pin 3 of GPIO) and GND
# Should read several kΩ (pull-up resistor), not 0Ω
```

#### 6. Clock Source Check

The PCA9685 uses an internal 25 MHz oscillator. If this fails, the chip won't operate.

**Symptom**: Device powers on but doesn't respond to I2C
**Check**: Typically requires oscilloscope to verify clock on internal pins

---

## Common Issues and Solutions

### Issue 1: No Devices Detected on Bus

**Symptoms**:
- `i2cdetect` shows all `--`
- No devices respond

**Possible Causes**:
1. Wrong I2C bus number (use 7 on Jetson, 1 on RPi)
2. Pull-up resistors missing or incorrect value
3. SDA/SCL pins shorted together
4. Power supply not connected
5. Incorrect device tree configuration

**Solutions**:
```bash
# Try all buses
for bus in 0 1 7 8; do
  echo "Bus $bus:"
  sudo i2cdetect -y $bus 2>/dev/null || echo "  (not available)"
done

# Check pull-up voltage
# Multimeter: SDA/SCL should read ~3.3V when idle

# Verify power
# Check that Auto pHAT has power LED lit (if present)
```

### Issue 2: Some Devices Work, Others Don't

**Symptoms**:
- IMU (0x69) detected
- PCA9685 (0x40) not detected

**Possible Causes**:
1. Individual device power issue
2. Specific device damaged
3. Output enable pin not asserted (for PCA9685)
4. Address conflict (check jumpers)

**Solutions**:
- Verify power to specific IC (measure VDD pin)
- Check device-specific enable pins
- Review schematic for that component

### Issue 3: Intermittent Communication

**Symptoms**:
- Devices appear/disappear on scans
- Read/write operations sometimes fail

**Possible Causes**:
1. Loose connection on GPIO header
2. Pull-up resistors too weak (value too high)
3. Bus capacitance too high (long wires)
4. Electrical noise
5. Ground loops

**Solutions**:
- Reseat pHAT on GPIO header
- Add stronger pull-ups (lower resistance value)
- Shorten I2C wires if using external devices
- Add 100nF ceramic caps near device VDD pins

### Issue 4: Wrong Device at Expected Address

**Symptoms**:
- Device appears at unexpected address
- Multiple devices at same address (conflict)

**Possible Causes**:
1. Address jumpers configured incorrectly
2. Multiple boards stacked with same addresses
3. Qwiic device connected with conflicting address

**Solutions**:
- Check all jumper configurations
- Use address scan to find actual address
- Reconfigure one device to different address

---

## Advanced Debugging with Oscilloscope

If you have access to an oscilloscope or logic analyzer:

### Setup

1. **Probe Points**:
   - Connect CH1 to SDA (GPIO pin 3)
   - Connect CH2 to SCL (GPIO pin 5)
   - Connect GND to any GND pin

2. **Trigger Settings**:
   - Trigger on SCL falling edge
   - Time base: 10-20 μs/div for 100 kHz, 2-5 μs/div for 400 kHz

### What to Look For

#### Normal I2C Transaction

```
     START          ADDRESS (7 bits)    R/W  ACK     DATA (8 bits)     ACK   STOP
       ↓               ↓  ↓  ↓            ↓    ↓        ↓  ↓  ↓          ↓     ↓
SDA: _/‾‾‾‾‾‾‾‾‾‾‾‾\___\_/_\_/________\____/__\‾‾‾‾\_/_\_/_\________/____/‾‾‾‾
SCL: ‾‾\_/\_/\_/\_/\_/\_/\_/\_/‾‾‾‾‾‾‾‾\_/\_/\_/\_/\_/\_/\_/\_/\_/‾‾‾‾‾‾‾‾‾‾‾
```

#### Common Problems Visible on Scope

**1. SDA/SCL Stuck LOW**:
```
SDA: ________________
SCL: ________________
```
- Pull-down resistors present (wrong!)
- Short circuit to GND
- Device holding bus (bus hang)

**2. SDA/SCL Stuck HIGH**:
```
SDA: ‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
SCL: ‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
```
- No communication occurring
- Check if controller is sending data

**3. No ACK from Device**:
```
SDA: \___START___ADDRESS(7)_W_/‾‾‾(no ACK)‾‾‾\_____...
                                ↑ Should pull LOW for ACK
```
- Device not present at that address
- Device not powered
- Device in sleep/standby mode

**4. Slow Rise Time**:
```
SDA:     ____/‾‾‾‾‾‾  ← Should be sharp corner
         ___/‾‾‾‾‾‾‾  ← This is slow (RC time constant too large)
```
- Pull-up resistors too weak (value too high)
- Too much bus capacitance
- Solution: Use lower resistance pull-ups (e.g., 2.2 kΩ instead of 10 kΩ)

### Measuring Signal Quality

**Voltage Levels** (3.3V I2C):
- HIGH: 2.4V - 3.6V (min 0.7 × VDD)
- LOW: 0V - 0.4V (max 0.3 × VDD)

**Timing** (Fast mode, 400 kHz):
- Clock frequency: ~400 kHz
- Clock HIGH time: ≥600 ns
- Clock LOW time: ≥1300 ns
- Rise time: ≤300 ns
- Fall time: ≤300 ns

---

## Summary: PCA9685 Troubleshooting Flowchart

```
PCA9685 not detected at 0x40
            ↓
    Is IMU (0x69) working?
       /          \
     YES           NO
      ↓             ↓
  Good! Bus OK   Check bus wiring
      ↓          and pull-ups
      ↓
Check PCA9685 power (VDD = 5V?)
      |
      ├─ NO → Fix power supply
      ↓
     YES
      ↓
Check /OE pin (should be LOW)
      |
      ├─ HIGH or floating → Connect to GND or enable via GPIO
      ↓
     LOW (correct)
      ↓
Check address jumpers (all open = 0x40?)
      |
      ├─ Jumpers closed → Try other addresses (0x41-0x47)
      ↓
    All open (correct)
      ↓
Try software reset (i2cset -y 7 0x00 0x06)
      |
      ├─ Still not working → Possible hardware damage
      ↓
     Working!
```

---

## Getting Help

If you've exhausted these troubleshooting steps:

1. **Check SparkFun Forums**: https://forum.sparkfun.com/
2. **Review GitHub Issues**: https://github.com/sparkfun/SparkFun_Auto_pHAT/issues
3. **Measure and document**:
   - Voltages on PCA9685 VDD, /OE, SDA, SCL
   - I2C bus scan results
   - Jumper configurations
   - Oscilloscope captures (if available)

**Before replacing hardware**, verify that pull-down resistors (if added) are removed and replaced with correct pull-ups.

---

**Next Steps**: See [JUMPER_CONFIGURATION.md](JUMPER_CONFIGURATION.md) for detailed address configuration options.
