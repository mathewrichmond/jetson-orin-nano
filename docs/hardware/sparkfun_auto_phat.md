# SparkFun Auto pHAT Setup Guide

## Overview

The SparkFun Auto pHAT is a robotics add-on board for single-board computers that provides:
- Dual DC motor drivers (with encoder support)
- 4 servo motor outputs
- ICM-20948 9DOF IMU (accelerometer, gyroscope, magnetometer)
- I2C communication interface

## Current Hardware Status

**Last Updated**: 2026-01-10
**Board**: SparkFun Auto pHAT for Raspberry Pi (used on Jetson Orin Nano)

### Working Components
- ✅ **Power**: Board receives power from GPIO header (light blinking)
- ✅ **I2C Bus 1**: Functional and detecting devices
- ✅ **Servo Controller**: Detected at I2C address 0x40 (PCA9685)
- ✅ **PHAT Node**: ROS 2 node running and publishing status
- ✅ **GPIO Header**: Properly connected (confirmed by servo controller detection)
- ✅ **Compatibility**: Board works on Jetson (servo controller proves I2C/power working)

### Components Not Detected
- ✅ **ICM-20948 IMU**: **NOW WORKING** on I2C bus 7, address 0x69 (Jetson uses bus 7, not bus 1 like Raspberry Pi)
- ❌ **Motor Driver (SCMD)**: Not detected at address 0x5D (may use GPIO instead)
- ❌ **Encoder**: Not detected at address 0x73

### Status Summary
The SparkFun Auto pHAT **IS compatible** with Jetson Orin Nano. The board is properly connected and powered. I2C communication is working.

**Key Finding**: The ICM-20948 IMU uses **I2C bus 7** on Jetson Orin Nano (not bus 1 like Raspberry Pi). This is a Jetson-specific I2C bus mapping difference.

**Solution**: Update configuration to use `i2c_bus: 7` instead of `i2c_bus: 1`.

## Hardware Setup

### Physical Connection

1. **Orientation**: The Auto pHAT should overhang the **right-hand side** of the Jetson when properly aligned
2. **GPIO Header**: Align with the 40-pin GPIO header
3. **Power**:
   - **GPIO Header Power**: The Auto pHAT gets logic power (3.3V/5V) from the GPIO header pins
   - **USB-C Power** (Optional): For higher current motors/servos or power isolation
   - **Power Pins**: Pin 1 (3.3V), Pin 2/4 (5V), Pin 6/9 (GND)
   - If only using I2C/IMU, GPIO power should be sufficient
   - Motors/servos may need USB-C or separate power supply

### I2C Connections

The Auto pHAT uses I2C for communication:
- **I2C Address**: Controlled by **ADR jumper** on the board
  - **ADR jumper OPEN**: Address = 0x68
  - **ADR jumper CLOSED**: Address = 0x69 (default)
- **I2C Bus**:
  - **Raspberry Pi**: Bus 1 (GPIO header I2C)
  - **Jetson Orin Nano**: Bus 7 (GPIO header I2C)
  - **Note**: Jetson uses different I2C bus numbering than Raspberry Pi
- **SDA/SCL**: Connected via GPIO header

**IMPORTANT**:
- Check the ADR jumper position on your Auto pHAT board to determine the correct I2C address
- Jetson Orin Nano uses **bus 7** for GPIO header I2C (not bus 1 like Raspberry Pi)

### Jetson Orin Nano Pinout

Verify I2C pins on your Jetson Orin Nano:
- **I2C Bus 1**: Usually pins 3 (SDA) and 5 (SCL)
- **I2C Bus 0**: May be on different pins

## Software Configuration

### Current Configuration

The PHAT motor controller node is configured for SparkFun Auto pHAT:
- **Accelerometer Type**: ICM20948
- **I2C Address**: 0x69
- **I2C Bus**: 7 (Jetson Orin Nano) - **Note**: Raspberry Pi uses bus 1, Jetson uses bus 7

### Configuration File

Edit `config/hardware/phat_params.yaml`:

```yaml
i2c_bus: 0  # Try 0 or 1
accelerometer_address: 0x69  # ICM-20948 default
accelerometer_type: 'ICM20948'
```

## Troubleshooting

### Current Issue: ICM-20948 Not Detected

**Symptoms**:
- Servo controller (0x40) is detected ✅
- ICM-20948 (0x69) not responding ❌
- Board is powered (light blinking) ✅

**What This Means**:
- I2C connection is working (servo controller proves this)
- Power is working
- Board is properly seated
- ICM-20948 needs special initialization or hardware enable

**Troubleshooting Steps**:
1. **Check ADR Jumper**: The Auto pHAT has an ADR jumper that controls the ICM-20948 I2C address:
   - ADR jumper **OPEN** = address 0x68
   - ADR jumper **CLOSED** = address 0x69 (default)
   - Verify jumper position and test both addresses
2. **Check Power**: ICM-20948 may need separate power enable (check board for power jumpers)
3. **Physical Inspection**: Verify ICM-20948 chip is present and properly soldered
4. **Check SparkFun Documentation**: Review Auto pHAT hookup guide for enable steps
5. **Try Both Addresses**: Test both 0x68 and 0x69 in case jumper is different than expected

### No I2C Devices Detected

1. **Check Power**:
   ```bash
   # Verify board is powered (check for LEDs if present)
   ```

2. **Check I2C Buses**:
   ```bash
   i2cdetect -y 0
   i2cdetect -y 1
   ```

3. **Check Connections**:
   - Verify SDA/SCL connections
   - Check ground connection
   - Ensure proper GPIO header alignment

4. **Check I2C Permissions**:
   ```bash
   groups | grep i2c  # Should show 'i2c'
   ls -l /dev/i2c-*   # Check permissions
   ```

5. **Enable I2C** (if needed):
   ```bash
   # Jetson may need I2C enabled in device tree
   # Check: /boot/extlinux/extlinux.conf
   ```

### ICM-20948 Not Detected

1. **Try Different I2C Bus**:
   - Change `i2c_bus` in config to 1 if currently 0
   - Or try bus 2, 4, 5

2. **Check Address**:
   - ICM-20948 default is 0x69
   - Some boards may use 0x68

3. **Verify Board Orientation**:
   - Auto pHAT should overhang right side
   - GPIO pins should align correctly

### GPIO Issues

The Auto pHAT motor drivers use I2C, not GPIO. GPIO is only needed if:
- Using additional GPIO-based motor drivers
- Controlling other GPIO peripherals

## Testing

### Run Diagnostics

```bash
./scripts/hardware/diagnose_phat.sh
```

### Current Test Results

**I2C Bus 1 Scan**:
```
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:                         -- -- -- -- -- -- -- --
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
20: -- -- -- -- -- UU -- -- -- -- -- -- -- -- -- --
30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
40: UU -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
60: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
70: -- -- -- -- -- -- -- --
```

**Detected Devices**:
- `0x25`: Unknown device (in use, may be motor controller)
- `0x40`: Servo Controller (PCA9685) ✅

**Expected but Not Detected**:
- `0x5D`: Motor Driver (SCMD) - May use GPIO instead
- `0x69`: ICM-20948 IMU - Not responding
- `0x73`: Encoder - Not detected

### Check ROS 2 Topics

```bash
# Check status
ros2 topic echo /phat/status

# Check IMU data (when detected)
ros2 topic echo /phat/imu

# Check publication rate
ros2 topic hz /phat/imu
```

### Manual I2C Test

```bash
# Test ICM-20948 directly
python3 << 'EOF'
from smbus2 import SMBus
bus = SMBus(1)  # Try bus 0 or 1
addr = 0x69

# Read WHO_AM_I
bus.write_byte_data(addr, 0x7F, 0x00)  # Bank 0
whoami = bus.read_byte_data(addr, 0x00)
print(f"WHO_AM_I: 0x{whoami:02X} (expected 0xEA for ICM-20948)")
bus.close()
EOF
```

## Orientation Check

Once the accelerometer is detected, you can verify orientation:

1. **Flat on table**: Z-axis should read ~9.81 m/s²
2. **Tilted**: Gravity vector should shift accordingly
3. **Rotated**: X/Y axes should change

Check IMU data:
```bash
ros2 topic echo /phat/imu
```

Look for `linear_acceleration` values. When flat:
- One axis should read close to ±9.81 m/s²
- Other axes should read close to 0

## Software Status

### Installed Libraries
- ✅ `smbus2`: Installed (for I2C communication)
- ✅ `sparkfun-qwiic`: Installed (SparkFun QWIIC library)
- ✅ `sparkfun-qwiic-icm20948`: Installed (ICM-20948 driver)

### ROS 2 Node Status
- **Node**: `/hardware/phat_motor_controller` ✅ Running
- **Status Topic**: `/phat/status` ✅ Publishing
- **IMU Topic**: `/phat/imu` ⚠️ Not publishing (ICM-20948 not detected)
- **Current Status**: `motors:disabled|accel:not_found`

### Configuration
- **I2C Bus**: 1 (GPIO header I2C)
- **ICM-20948 Address**: 0x69
- **Accelerometer Type**: ICM20948
- **Config File**: `config/hardware/phat_params.yaml`

## Alternative Accelerometer Solutions

If the Auto pHAT's built-in ICM-20948 cannot be made to work on Jetson Orin Nano, consider these alternatives:

### Option 1: Standalone ICM-20948 Module
- **SparkFun QWIIC ICM-20948 Breakout**: Connect via QWIIC connector on Auto pHAT or directly to Jetson I2C
- **I2C Address**: 0x69 (same as Auto pHAT)
- **Advantage**: Guaranteed to work, same sensor
- **Connection**: Use QWIIC connector on Auto pHAT, or connect to Jetson GPIO I2C pins

### Option 2: MPU6050 Breakout Board
- **Common and well-supported**: Very reliable on Jetson
- **I2C Address**: 0x68 (default) or 0x69 (if AD0 pin high)
- **Advantage**: Already fully supported in PHAT node, very common
- **Connection**: Connect to Jetson GPIO I2C (pins 3=SDA, 5=SCL on bus 1)
- **Configuration**: Change `accelerometer_type` to `MPU6050` and `accelerometer_address` to `0x68` in config

### Option 3: LSM6DS3 Module
- **Also supported**: In PHAT node
- **I2C Address**: 0x6A
- **Connection**: Same as MPU6050
- **Configuration**: Change `accelerometer_type` to `LSM6DS3` and address to `0x6A`

### Option 4: Other I2C IMU Modules
- **BNO055**: 9DOF sensor with sensor fusion
- **ADXL345**: 3-axis accelerometer
- **LSM303**: Accelerometer + magnetometer

All can connect via I2C to Jetson GPIO header (pins 3=SDA, 5=SCL on bus 1).

### Recommendation
If you need accelerometer data immediately, **MPU6050** is the most reliable option:
1. Very common and well-tested on Jetson
2. Already fully supported in the PHAT node
3. Simple I2C connection
4. Inexpensive and widely available

The PHAT node can be configured to use any of these sensors by updating `config/hardware/phat_params.yaml`.

## Resources

- [SparkFun Auto pHAT Hookup Guide](https://learn.sparkfun.com/tutorials/sparkfun-auto-phat-hookup-guide)
- [ICM-20948 Datasheet](https://www.invensense.com/products/motion-tracking/9-axis/icm-20948/)
- SparkFun GitHub: Search for "Auto pHAT" repositories
- [SparkFun Auto pHAT Product Page](https://www.sparkfun.com/products/15443)
- [SparkFun QWIIC ICM-20948 Breakout](https://www.sparkfun.com/products/15335) (alternative)
- [MPU6050 Breakout Boards](https://www.sparkfun.com/products/11028) (alternative)

## Troubleshooting ICM-20948 Detection

### Comprehensive Testing Performed

We have tried the following methods to detect the ICM-20948:

1. ✅ **I2C Bus Scans**: Scanned all I2C buses (0-8) for addresses 0x68 and 0x69
2. ✅ **Direct I2C Access**: Attempted direct register reads/writes
3. ✅ **SparkFun QWIIC Library**: Tested SparkFun's official library
4. ✅ **Reset Sequences**: Tried hardware reset and wake-up sequences
5. ✅ **Corrected Initialization**: Fixed initialization to set bank register first
6. ✅ **Servo Controller Check**: Verified if IMU needs enable via servo controller
7. ✅ **QWIIC Bus Scan**: Checked QWIIC connector I2C addresses

**All attempts resulted in "Remote I/O error" (errno 121)**, indicating the ICM-20948 is not responding on the I2C bus.

### Diagnostic Script

Run the comprehensive diagnostic:
```bash
./scripts/hardware/diagnose_icm20948.sh
```

### Possible Causes

Based on testing, the ICM-20948 may not be detected due to:

1. **Hardware Enable Required**:
   - Jumper or solder bridge needs to be set
   - Power enable switch/jumper
   - I2C address selection jumper

2. **Firmware/Initialization**:
   - May need special firmware initialization
   - May require Raspberry Pi-specific initialization sequence
   - May need to be enabled via another device on the board

3. **Hardware Issue**:
   - ICM-20948 chip may not be properly connected
   - May be a hardware defect on this specific board
   - May require different power supply

4. **Platform Compatibility**:
   - Designed for Raspberry Pi, may have Pi-specific requirements
   - May need different initialization on Jetson

### Next Steps

1. **Physical Inspection**:
   - Check for jumpers/solder bridges near ICM-20948
   - Verify chip is physically present and properly soldered
   - Check for QWIIC connector that needs connection

2. **SparkFun Resources**:
   - Check [SparkFun Auto pHAT Hookup Guide](https://learn.sparkfun.com/tutorials/sparkfun-auto-phat-hookup-guide)
   - Review [SparkFun Community Forum](https://community.sparkfun.com) for Jetson compatibility
   - Check for firmware updates or initialization requirements

3. **Test on Raspberry Pi** (if available):
   - Verify ICM-20948 works on intended platform
   - If it works on Pi but not Jetson, it's a compatibility issue
   - If it doesn't work on Pi either, it's a hardware issue

4. **Alternative Solutions**:
   - Consider standalone accelerometer (see [Alternative Accelerometer Solutions](#alternative-accelerometer-solutions))
   - MPU6050 is most reliable and already supported
   - Access ICM-20948 through servo controller (if supported)
   - Check if different I2C address is used
   - Verify if QWIIC connector needs to be used instead

4. **Hardware Verification**:
   - Verify board orientation (right side overhang)
   - Check all GPIO pins are fully seated
   - Verify I2C pull-up resistors (usually built-in)

5. **Software Verification**:
   - Run diagnostics: `./scripts/hardware/diagnose_phat.sh`
   - Check ROS 2 logs: `journalctl --user -u isaac-robot.service | grep phat`
   - Monitor status topic: `ros2 topic echo /phat/status`

### Current Status Monitoring

The PHAT node is running and will automatically detect the ICM-20948 when it becomes accessible. Monitor the status topic:

```bash
ros2 topic echo /phat/status
```

When ICM-20948 is detected, status will change from:
- `motors:disabled|accel:not_found`
- To: `motors:disabled|accel:ok` (or `motors:ok|accel:ok` if GPIO works)

IMU data will then be published on `/phat/imu` topic.
