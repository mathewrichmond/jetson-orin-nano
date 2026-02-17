# SparkFun Auto pHAT Technical Specifications

Detailed technical specifications for all components on the SparkFun Auto pHAT (ROB-16328).

## PCA9685 - 16-Channel PWM Servo Controller

### Overview

The PCA9685 is the primary servo controller IC, providing 16 channels of 12-bit PWM control. On the Auto pHAT, only the first 4 channels are broken out to the servo headers.

### Electrical Specifications

| Parameter | Value | Notes |
|-----------|-------|-------|
| Operating Voltage (VDD) | 2.3V - 5.5V | Hardwired to 5V on Auto pHAT |
| Operating Temperature | -40°C to 85°C | |
| Supply Current | ~10mA (typical) | Excluding servo loads |
| I2C Clock Frequency | Up to 1 MHz | Fast-mode Plus compatible |

### PWM Specifications

| Parameter | Value | Notes |
|-----------|-------|-------|
| PWM Channels | 16 | 4 available on Auto pHAT |
| Resolution | 12-bit | 4096 steps of control |
| PWM Frequency Range | 24 Hz - 1526 Hz | Default: 200 Hz (register 0x1E) |
| Duty Cycle | 0% - 100% | Fully adjustable |
| Output Type | Totem pole | 25mA sink, 10mA source at 5V |

### I2C Configuration

| Parameter | Value | Notes |
|-----------|-------|-------|
| **Default Address** | **0x40** | All address pins open |
| Address Range | 0x40 - 0x47 | Via SRV ADD jumpers (A0-A5) |
| Address Pins | 6 (A0-A5) | Each adds binary value to 0x40 |
| General Call Address | 0x00 | For software reset (SWRST) |

#### Address Calculation

```
Address = 0x40 + (A5<<5 | A4<<4 | A3<<3 | A2<<2 | A1<<1 | A0)

Examples:
- All open:    0x40 + 0b000000 = 0x40 (default)
- A0 closed:   0x40 + 0b000001 = 0x41
- A1 closed:   0x40 + 0b000010 = 0x42
- A0+A1 close: 0x40 + 0b000011 = 0x43
- All closed:  0x40 + 0b111111 = 0x7F (invalid, use 0x40-0x47)
```

### Key Registers

| Address | Register | Function |
|---------|----------|----------|
| 0x00 | MODE1 | Mode register 1 (sleep, restart, AI) |
| 0x01 | MODE2 | Mode register 2 (output drive, invert) |
| 0x06-0x09 | LED0_ON_L/H, LED0_OFF_L/H | Channel 0 PWM control |
| 0xFE | PRESCALE | PWM frequency prescaler |

### Servo Control

For standard hobby servos (50 Hz PWM, 1000-2000μs pulse width):

| Parameter | Value | Calculation |
|-----------|-------|-------------|
| PWM Frequency | 50 Hz | PRESCALE = 121 (0x79) |
| Min Pulse (1000μs) | ~205 | (1000 / 20000) * 4096 |
| Center (1500μs) | ~307 | (1500 / 20000) * 4096 |
| Max Pulse (2000μs) | ~409 | (2000 / 20000) * 4096 |

## ICM-20948 - 9-DoF IMU

### Overview

The ICM-20948 is a 9-axis motion tracking device combining a 3-axis gyroscope, 3-axis accelerometer, and 3-axis magnetometer (via AK09916 internal IC).

### Electrical Specifications

| Parameter | Value | Notes |
|-----------|-------|-------|
| Operating Voltage (VDD) | 1.71V - 3.6V | Hardwired to 1.8V on Auto pHAT |
| VDDIO (I/O Voltage) | 1.71V - 3.6V | Connected to 3.3V via level shifter |
| Operating Temperature | -40°C to 85°C | |
| Supply Current | ~3.2mA | All sensors active |

### Gyroscope Specifications

| Parameter | Value |
|-----------|-------|
| Full Scale Ranges | ±250, ±500, ±1000, ±2000 dps |
| Sensitivity (±250 dps) | 131 LSB/(dps) |
| Sensitivity (±500 dps) | 65.5 LSB/(dps) |
| Sensitivity (±1000 dps) | 32.8 LSB/(dps) |
| Sensitivity (±2000 dps) | 16.4 LSB/(dps) |
| Output Data Rate | 4.4 Hz - 9 kHz |

### Accelerometer Specifications

| Parameter | Value |
|-----------|-------|
| Full Scale Ranges | ±2g, ±4g, ±8g, ±16g |
| Sensitivity (±2g) | 16,384 LSB/g |
| Sensitivity (±4g) | 8,192 LSB/g |
| Sensitivity (±8g) | 4,096 LSB/g |
| Sensitivity (±16g) | 2,048 LSB/g |
| Output Data Rate | 4.4 Hz - 4.5 kHz |

### Magnetometer Specifications

| Parameter | Value |
|-----------|-------|
| Full Scale Range | ±4900 µT |
| Sensitivity | 0.15 µT/LSB |
| Output Data Rate | 100 Hz (max) |

### I2C Configuration

| Parameter | Value | Notes |
|-----------|-------|-------|
| **Default Address** | **0x69** | AD0 pin pulled HIGH (jumper closed) |
| Alternative Address | 0x68 | AD0 pin pulled LOW (jumper open) |
| I2C Clock Frequency | Up to 400 kHz | Standard/Fast mode |

**Auto pHAT Configuration**: The IMU ADD jumper is **closed by default**, setting the address to **0x69**.

### Key Registers

| Address | Register | Function |
|---------|----------|----------|
| 0x00 | WHO_AM_I | Device ID (should read 0xEA) |
| 0x06 | PWR_MGMT_1 | Power management |
| 0x1C | ACCEL_CONFIG | Accelerometer configuration |
| 0x1B | GYRO_CONFIG | Gyroscope configuration |

## PSoC 4245 + DRV8835 - Motor Driver

### Overview

The motor driver consists of a PSoC 4245 microcontroller for I2C control and a DRV8835 dual H-bridge for motor drive.

### DRV8835 H-Bridge Specifications

| Parameter | Value | Notes |
|-----------|-------|-------|
| Operating Voltage (VM) | 2.0V - 11V | Auto pHAT: up to 9V (11V with heatsink) |
| Logic Voltage (VCC) | 2.0V - 7V | 5V on Auto pHAT |
| Peak Current per Channel | 1.5A | Short duration |
| Steady-State Current | 1.2A | Per channel |
| Drive Channels | 2 | Independent control |

### PSoC 4245 I2C Configuration

| Parameter | Value | Notes |
|-----------|-------|-------|
| **Default Address** | **0x5D** | Configurable via MTR ADD jumpers |
| Address Range | 0x58 - 0x61 | 10 possible addresses |
| Drive Control Levels | 127 | Granular speed control |

### Thermal Considerations

The DRV8835 and supporting components generate heat under load. For sustained high-current operation:

1. **Thermal Pad**: The bottom of the board has a thermally conductive area
2. **Heatsink**: Use Theragrip thermal tape + small heatsinks
3. **Active Cooling**: Add a fan for heavy loads
4. **Power Limits**:
   - 9V max without heatsink
   - 11V max with heatsink + cooling

**Temperature Curve**: See SparkFun hookup guide for load vs. temperature graph.

## ATtiny84 - Dual Encoder Reader

### Overview

The ATtiny84 microcontroller reads two quadrature encoders and provides counts via I2C.

### Specifications

| Parameter | Value | Notes |
|-----------|-------|-------|
| Operating Voltage | 3.3V | From board supply |
| **I2C Address** | **0x73** | Fixed (not configurable) |
| Encoder Inputs | 2 channels | Quadrature A/B per channel |
| Update Rate | Dependent on firmware | See SparkFun firmware |

### Encoder Connections

Terminal blocks on bottom of board:
- **ENC1**: Channel 1 (A, B, GND, 3.3V)
- **ENC2**: Channel 2 (A, B, GND, 3.3V)

### Fuse Settings

If reprogramming the ATtiny84:
- Low Fuse: 0xE2 (8 MHz internal clock)
- High Fuse: 0xDF (Brown-out detect disabled)
- Extended Fuse: 0xFF

## Power Budget

### Typical Current Draw (Motors and Servos OFF)

| Component | Current | Notes |
|-----------|---------|-------|
| PCA9685 | ~10 mA | Idle, no servo load |
| ICM-20948 | ~3.2 mA | All sensors active |
| PSoC 4245 | ~10 mA | Idle |
| ATtiny84 | ~5 mA | Running |
| **Total (Idle)** | **~30 mA** | From 5V rail |

### Under Load

- **Servos**: 100-500 mA each (depends on servo and load)
- **Motors**: Up to 1.2A per channel (steady state)
- **Peak**: 1.5A per motor channel (short duration)

**Power Supply Recommendation**: 2-3A capable supply for motor operation, additional capacity for servos.

## Environmental Specifications

| Parameter | Value |
|-----------|-------|
| Operating Temperature | -40°C to 85°C (component limited) |
| Storage Temperature | -40°C to 85°C |
| Humidity | Non-condensing |

## Compliance and Certifications

See SparkFun product page for current certifications and compliance information.

## Revision History

This documentation is based on the standard production version. Check SparkFun GitHub for hardware revision updates.

---

For additional technical details, refer to component datasheets in the `datasheets/` directory.
