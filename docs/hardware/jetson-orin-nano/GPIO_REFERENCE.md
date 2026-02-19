# GPIO Reference: 40-Pin Header

Complete pinout and reference for the Jetson Orin Nano Developer Kit 40-pin GPIO header.

## Overview

The Jetson Orin Nano Developer Kit includes a **40-pin GPIO header** that is electrically and mechanically compatible with Raspberry Pi HATs and pHATs.

**Key Features**:
- Physical Raspberry Pi compatibility
- 3.3V logic levels
- Multiple I2C, SPI, UART, and PWM interfaces
- GPIO pins for general digital I/O

**CRITICAL DIFFERENCE**: While physically compatible with Raspberry Pi, the **I2C bus numbering is different**:
- Raspberry Pi: I2C bus 1
- Jetson Orin Nano: **I2C bus 7**

## Complete Pinout

```
           3.3V  [ 1] [2 ]  5V
  I2C Bus 7 SDA  [ 3] [4 ]  5V
  I2C Bus 7 SCL  [ 5] [6 ]  GND
GPIO   4  GPIO4  [ 7] [8 ]  GPIO14 (UART1 TX)
            GND  [ 9] [10]  GPIO15 (UART1 RX)
GPIO  17  GPIO17 [11] [12]  GPIO18 (I2S SCLK)
GPIO  27  GPIO27 [13] [14]  GND
GPIO  22  GPIO22 [15] [16]  GPIO23
           3.3V  [17] [18]  GPIO24
GPIO  10  SPI MOSI [19] [20]  GND
GPIO   9  SPI MISO [21] [22]  GPIO25
GPIO  11  SPI SCLK [23] [24]  GPIO8 (SPI CE0)
            GND  [25] [26]  GPIO7 (SPI CE1)
  I2C Bus 1 SDA  [27] [28]  I2C Bus 1 SCL (EEPROM)
GPIO   5  GPIO5  [29] [30]  GND
GPIO   6  GPIO6  [31] [32]  GPIO12 (PWM0)
GPIO  13  GPIO13 (PWM1) [33] [34]  GND
GPIO  19  GPIO19 (I2S FS) [35] [36]  GPIO16
GPIO  26  GPIO26 [37] [38]  GPIO20 (I2S DIN)
            GND  [39] [40]  GPIO21 (I2S DOUT)
```

## Detailed Pin Table

| Physical Pin | Jetson GPIO | Function | Raspberry Pi GPIO | Alt Functions |
|--------------|-------------|----------|-------------------|---------------|
| 1 | - | **3.3V Power** | 3.3V | Max 50mA per pin |
| 2 | - | **5V Power** | 5V | From DC jack |
| 3 | GPIO 2 | **I2C Bus 7 SDA** | GPIO 2 (I2C1 SDA) | |
| 4 | - | **5V Power** | 5V | |
| 5 | GPIO 3 | **I2C Bus 7 SCL** | GPIO 3 (I2C1 SCL) | |
| 6 | - | **Ground** | GND | |
| 7 | GPIO 4 | GPIO | GPIO 4 | |
| 8 | GPIO 14 | UART TX | GPIO 14 | UART1 TXD |
| 9 | - | **Ground** | GND | |
| 10 | GPIO 15 | UART RX | GPIO 15 | UART1 RXD |
| 11 | GPIO 17 | GPIO | GPIO 17 | |
| 12 | GPIO 18 | GPIO | GPIO 18 | I2S SCLK |
| 13 | GPIO 27 | GPIO | GPIO 27 | |
| 14 | - | **Ground** | GND | |
| 15 | GPIO 22 | GPIO | GPIO 22 | |
| 16 | GPIO 23 | GPIO | GPIO 23 | |
| 17 | - | **3.3V Power** | 3.3V | |
| 18 | GPIO 24 | GPIO | GPIO 24 | |
| 19 | GPIO 10 | SPI MOSI | GPIO 10 | SPI1 MOSI |
| 20 | - | **Ground** | GND | |
| 21 | GPIO 9 | SPI MISO | GPIO 9 | SPI1 MISO |
| 22 | GPIO 25 | GPIO | GPIO 25 | |
| 23 | GPIO 11 | SPI SCLK | GPIO 11 | SPI1 SCLK |
| 24 | GPIO 8 | SPI CE0 | GPIO 8 | SPI1 CS0 |
| 25 | - | **Ground** | GND | |
| 26 | GPIO 7 | SPI CE1 | GPIO 7 | SPI1 CS1 |
| 27 | GPIO 0 | I2C Bus 1 SDA | GPIO 0 | EEPROM/HAT ID |
| 28 | GPIO 1 | I2C Bus 1 SCL | GPIO 1 | EEPROM/HAT ID |
| 29 | GPIO 5 | GPIO | GPIO 5 | |
| 30 | - | **Ground** | GND | |
| 31 | GPIO 6 | GPIO | GPIO 6 | |
| 32 | GPIO 12 | PWM | GPIO 12 | PWM0 |
| 33 | GPIO 13 | PWM | GPIO 13 | PWM1 |
| 34 | - | **Ground** | GND | |
| 35 | GPIO 19 | GPIO | GPIO 19 | I2S FS |
| 36 | GPIO 16 | GPIO | GPIO 16 | |
| 37 | GPIO 26 | GPIO | GPIO 26 | |
| 38 | GPIO 20 | GPIO | GPIO 20 | I2S DIN |
| 39 | - | **Ground** | GND | |
| 40 | GPIO 21 | GPIO | GPIO 21 | I2S DOUT |

## Critical Pins for Isaac Robot

### I2C Bus 7 (SparkFun Auto pHAT)

| Pin | Function | Connects To |
|-----|----------|-------------|
| **3** | **SDA** | SparkFun Auto pHAT I2C data (all devices) |
| **5** | **SCL** | SparkFun Auto pHAT I2C clock (all devices) |

**Bus Number**: 7 (not 1 like Raspberry Pi!)

**Devices on Bus**:
- PCA9685 Servo Controller (0x40)
- ICM-20948 IMU (0x69)
- PSoC 4245 Motor Driver (0x5D)
- ATtiny84 Encoder Reader (0x73)

**Access I2C Bus 7**:
```bash
# Scan I2C bus 7
sudo i2cdetect -y 7

# NOT bus 1 (that's for HAT EEPROM)
```

### Power Pins

| Pin | Voltage | Max Current | Usage |
|-----|---------|-------------|-------|
| **1, 17** | 3.3V | 50mA total | Low-power peripherals |
| **2, 4** | 5V | 2A total | HATs, motors, servos |
| **6, 9, 14, 20, 25, 30, 34, 39** | GND | - | Ground reference |

**IMPORTANT**: The 5V pins are connected directly to the DC jack power input. Do not exceed 2A total draw.

### Interrupt Pins (Auto pHAT)

| Pin | Jetson GPIO | Used By |
|-----|-------------|---------|
| **7** | GPIO 4 | Encoder Reader (ATtiny84) |
| **26** | GPIO 7 | IMU (ICM-20948) |

## I2C Buses

The Jetson Orin Nano has multiple I2C buses available:

| Bus | Physical Pins | Purpose | Linux Device |
|-----|---------------|---------|--------------|
| **Bus 1** | 27 (SDA), 28 (SCL) | HAT EEPROM ID | `/dev/i2c-1` |
| **Bus 7** | **3 (SDA), 5 (SCL)** | **Primary GPIO I2C** | `/dev/i2c-7` |
| Bus 8 | Internal | CSI camera | `/dev/i2c-8` |

**For Raspberry Pi HATs/pHATs**: Use bus 7, not bus 1!

### I2C Bus Scanning

```bash
# Scan I2C bus 7 (GPIO header primary)
sudo i2cdetect -y 7

# Expected output with SparkFun Auto pHAT:
#      0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
# 00:          -- -- -- -- -- -- -- -- -- -- -- -- --
# 10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
# 20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
# 30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
# 40: 40 -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
# 50: -- -- -- -- -- -- -- -- -- -- -- -- -- 5d -- --
# 60: -- -- -- -- -- -- -- -- -- 69 -- -- -- -- -- --
# 70: -- -- -- 73 -- -- -- --

# Scan I2C bus 1 (HAT EEPROM)
sudo i2cdetect -y 1
```

### I2C Tools

```bash
# Install i2c-tools
sudo apt-get install -y i2c-tools

# List all I2C buses
i2cdetect -l

# Scan specific bus
sudo i2cdetect -y 7

# Read byte from device
sudo i2cget -y 7 0x40 0x00

# Write byte to device
sudo i2cset -y 7 0x40 0x00 0x01
```

## SPI Interface

### SPI1 (Main SPI Bus)

| Pin | Function | Jetson GPIO |
|-----|----------|-------------|
| **19** | MOSI (TX) | GPIO 10 |
| **21** | MISO (RX) | GPIO 9 |
| **23** | SCLK (Clock) | GPIO 11 |
| **24** | CE0 (Chip Select 0) | GPIO 8 |
| **26** | CE1 (Chip Select 1) | GPIO 7 |

**Linux Device**: `/dev/spidev0.0` and `/dev/spidev0.1`

**Access**:
```bash
# Enable SPI (if not enabled)
# Edit /boot/extlinux/extlinux.conf or use device tree overlay

# Test SPI loopback (connect MOSI to MISO)
sudo apt-get install -y python3-spidev
```

## UART Interface

### UART1 (Primary Serial)

| Pin | Function | Jetson GPIO |
|-----|----------|-------------|
| **8** | TX (Transmit) | GPIO 14 |
| **10** | RX (Receive) | GPIO 15 |

**Linux Device**: `/dev/ttyTHS0` (Tegra High Speed UART)

**Access**:
```bash
# Read from UART
sudo cat /dev/ttyTHS0

# Write to UART
echo "Hello" | sudo tee /dev/ttyTHS0

# Use with minicom/screen
sudo minicom -D /dev/ttyTHS0 -b 115200
```

## PWM Outputs

| Pin | Jetson GPIO | PWM Channel | Linux Device |
|-----|-------------|-------------|--------------|
| **32** | GPIO 12 | PWM0 | `/sys/class/pwm/pwmchip0/pwm0` |
| **33** | GPIO 13 | PWM1 | `/sys/class/pwm/pwmchip0/pwm1` |

**Example Usage**:
```bash
# Enable PWM0
echo 0 | sudo tee /sys/class/pwm/pwmchip0/export

# Set period (100 Hz = 10ms = 10000000 ns)
echo 10000000 | sudo tee /sys/class/pwm/pwmchip0/pwm0/period

# Set duty cycle (50% = 5ms = 5000000 ns)
echo 5000000 | sudo tee /sys/class/pwm/pwmchip0/pwm0/duty_cycle

# Enable output
echo 1 | sudo tee /sys/class/pwm/pwmchip0/pwm0/enable

# Disable
echo 0 | sudo tee /sys/class/pwm/pwmchip0/pwm0/enable
```

## GPIO Digital I/O

### Available GPIO Pins

All non-power, non-special-function pins can be used as digital I/O:
- Pins: 7, 11, 13, 15, 16, 18, 22, 29, 31, 37

### GPIO via sysfs (Legacy)

```bash
# Export GPIO (example: GPIO 4 = pin 7)
echo 4 | sudo tee /sys/class/gpio/export

# Set direction (out or in)
echo out | sudo tee /sys/class/gpio/gpio4/direction

# Set value (0 or 1)
echo 1 | sudo tee /sys/class/gpio/gpio4/value

# Read value
cat /sys/class/gpio/gpio4/value

# Unexport
echo 4 | sudo tee /sys/class/gpio/gpio4/unexport
```

### GPIO via libgpiod (Modern, Recommended)

```bash
# Install tools
sudo apt-get install -y gpiod

# List GPIO chips and lines
gpiodetect
gpioinfo

# Set GPIO high (example: GPIO 4)
gpioset gpiochip0 4=1

# Read GPIO
gpioget gpiochip0 4

# Monitor GPIO changes
gpiomon gpiochip0 4
```

### GPIO via Python (Jetson.GPIO)

```bash
# Install Jetson.GPIO library
sudo apt-get install -y python3-pip
sudo pip3 install Jetson.GPIO
```

**Example Python code**:
```python
import Jetson.GPIO as GPIO
import time

# Set mode (BOARD = physical pin, BCM = GPIO number)
GPIO.setmode(GPIO.BOARD)

# Setup pin 7 as output
GPIO.setup(7, GPIO.OUT)

# Blink LED
while True:
    GPIO.output(7, GPIO.HIGH)
    time.sleep(1)
    GPIO.output(7, GPIO.LOW)
    time.sleep(1)

# Cleanup
GPIO.cleanup()
```

## Voltage Levels and Electrical Specifications

| Specification | Value | Notes |
|---------------|-------|-------|
| **Logic High (VIH)** | 2.0V minimum | 3.3V nominal |
| **Logic Low (VIL)** | 0.8V maximum | 0V nominal |
| **Output High (VOH)** | 2.4V minimum | 3.3V typical |
| **Output Low (VOL)** | 0.4V maximum | 0V typical |
| **Max GPIO current** | 16mA | Per pin, not recommended > 10mA |
| **3.3V rail max current** | 50mA total | All pins combined |
| **5V rail max current** | 2A total | Shared with DC jack input |

**CRITICAL**: Do NOT connect 5V signals directly to GPIO pins. Use level shifter if needed.

## Raspberry Pi Compatibility

### What Works

✅ **Physical fit** - HATs and pHATs mount correctly  
✅ **3.3V logic** - Same voltage levels  
✅ **I2C devices** - Change bus number from 1 to 7  
✅ **SPI devices** - Same pinout and configuration  
✅ **GPIO libraries** - Jetson.GPIO is RPi.GPIO compatible  
✅ **Power pins** - Same 3.3V and 5V distribution

### What Requires Changes

⚠️ **I2C bus number** - Use bus 7, not bus 1  
⚠️ **GPIO numbering** - Some GPIO numbers differ  
⚠️ **PWM frequencies** - May differ from RPi  
⚠️ **UART devices** - `/dev/ttyTHS0` instead of `/dev/ttyAMA0`  
⚠️ **Device tree** - Different overlay system

### Adapting Raspberry Pi Software

1. **I2C bus**: Change `1` to `7` in all I2C calls
   ```python
   # Raspberry Pi
   bus = smbus.SMBus(1)
   
   # Jetson Orin Nano
   bus = smbus.SMBus(7)
   ```

2. **GPIO library**: Use Jetson.GPIO (RPi.GPIO-compatible)
   ```python
   # Works on both
   import Jetson.GPIO as GPIO  # or: import RPi.GPIO as GPIO
   GPIO.setmode(GPIO.BOARD)
   ```

3. **UART device**: Update device path
   ```python
   # Raspberry Pi
   ser = serial.Serial('/dev/ttyAMA0', 9600)
   
   # Jetson Orin Nano
   ser = serial.Serial('/dev/ttyTHS0', 9600)
   ```

## Troubleshooting

### I2C Device Not Detected

```bash
# Check I2C bus is enabled
ls /dev/i2c-*

# Should show: /dev/i2c-0 /dev/i2c-1 /dev/i2c-7 ...

# Scan correct bus (7 for GPIO)
sudo i2cdetect -y 7

# If not detected, check:
# - Device is powered (3.3V or 5V)
# - Wiring: SDA to pin 3, SCL to pin 5, GND connected
# - Pull-up resistors present (usually built-in to HATs)
```

### GPIO Permission Denied

```bash
# Add user to gpio group
sudo groupadd -f gpio
sudo usermod -a -G gpio $USER

# Log out and back in, or:
newgrp gpio

# Set GPIO permissions (add to /etc/udev/rules.d/99-gpio.rules)
SUBSYSTEM=="gpio", KERNEL=="gpiochip*", GROUP="gpio", MODE="0660"
```

### PWM Not Working

```bash
# Check PWM device exists
ls /sys/class/pwm/

# May need device tree overlay to enable PWM
# See JetPack documentation for enabling PWM
```

## Related Documentation

- [README.md](README.md) - Jetson Orin Nano overview
- [SparkFun Auto pHAT](../sparkfun-auto-phat/) - Compatible robotics HAT using I2C bus 7
- [ARCHITECTURE.md](../../ARCHITECTURE.md) - System architecture including GPIO usage

## External Resources

- **Jetson GPIO Library**: https://github.com/NVIDIA/jetson-gpio
- **40-pin Header Spec**: NVIDIA Jetson Orin Nano Carrier Board Specification
- **Raspberry Pi Pinout**: https://pinout.xyz (for comparison)
