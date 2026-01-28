# Electrical Documentation

## Overview

This document provides an organized reference to all electrical components, wiring, power requirements, and interfaces in the Isaac robot system. This consolidates the electrical aspects from the BOM and provides quick reference to detailed component documentation.

**Last Updated**: 2026-01-27

---

## 1. System Architecture

### 1.1 Block Diagram

```
                    ┌─────────────────────┐
                    │  5V Power Supply    │
                    │  (USB-C or Barrel)  │
                    └──────────┬──────────┘
                               │
                               ▼
        ┌──────────────────────────────────────────┐
        │                                          │
        │        Jetson Orin Nano (8GB)            │
        │         Ubuntu 22.04, ROS 2              │
        │                                          │
        └────────────┬───────────────┬─────────────┘
                     │               │
              USB3.0 │        40-pin │ GPIO
                     │               │
                     ▼               ▼
              ┌──────────┐    ┌──────────┐
              │ RSHTECH  │    │SparkFun  │
              │ USB Hub  │    │Auto pHAT │
              │ 7-Port   │    └─────┬────┘
              └─┬────────┘          │
                │                   ▼
      ┌─────────┼────────────┐   ┌─────────────┐
      │         │            │   │4x MG996R    │◄── 6V Power
      ▼         ▼            ▼   │Servos       │   Supply
   ┌────┐   ┌────┐     ┌─────┐  │ICM-20948 IMU│   (separate)
   │D435│   │D455│     │USB  │  └─────────────┘
   │Cam │   │Cam │     │Mic/ │
   └─┬──┘   └─┬──┘     │iRobot
     │        │        └─────┘
     │Ethernet│
     │Harness │
     └────┬───┘
       Sync Signal
```

### 1.2 Power Tree

```
5V/4A Power Supply          6V/10A Servo Power Supply
│                           │ (separate, recommended)
├─ Jetson Orin Nano (~15W)  │
│  │                        │
│  ├─ GPIO → Auto pHAT      │
│  │   ├─ ICM-20948 IMU     │
│  │   └─ PCA9685 (logic)   │
│  │                        │
│  └─ USB → RSHTECH Hub     │
│     ├─ RealSense D435 (~2W)
│     ├─ RealSense D455 (~3W)
│     ├─ USB Microphone (<1W)
│     └─ iRobot USB-Serial (<1W)
│
└─ iRobot Create 2 (internal battery)
                            └─ Auto pHAT USB-C → 4x MG996R
                               Servos (~10A peak)
```

**Power Budget**:
- **Main System** (5V): ~22W (Jetson + USB hub + devices)
- **Servo System** (6V): Up to 60W (4 servos @ 2.5A each = 10A peak)
- **iRobot Create**: Internal 14.4V battery (independent)

**Recommendations**:
- **Jetson**: Use 5V/4A (20W) minimum power supply
- **USB Hub**: RSHTECH 7-port with dedicated 5V adapter (powered hub)
- **Servos**: Use separate 6V/10A supply connected to Auto pHAT USB-C
  - MG996R stall current: 2.5A each × 4 = 10A peak
  - Typical operation: 3-4A (much lower than peak)
- **Isolation**: Separate servo power prevents noise on digital logic

---

## 2. Component Electrical Specifications

### 2.1 Computing

#### Jetson Orin Nano

| Parameter | Specification | Notes |
|-----------|---------------|-------|
| **Power Input** | 5V @ 4A (USB-C) or 7-20V (barrel jack) | USB-C preferred |
| **Power Consumption** | Idle: ~5W, Load: ~15W | Depends on workload |
| **GPIO Voltage** | 3.3V logic | 5V tolerant on some pins |
| **USB Ports** | 4× USB 3.0 Type-A | USB 3.0 Gen 2 (10 Gbps) |
| **GPIO Header** | 40-pin (Raspberry Pi compatible) | UART, I2C, SPI, PWM |

**Documentation**: [Jetson Orin Nano Datasheet](https://developer.nvidia.com/embedded/learn/jetson-orin-nano-devkit-user-guide)

### 2.2 Vision System

#### Intel RealSense D435

| Parameter | Specification | Notes |
|-----------|---------------|-------|
| **Interface** | USB 3.0 Type-C | USB 3.0 required for full performance |
| **Power** | USB-powered (~2W) | - |
| **Sync Pins** | GPIO pins (3.3V) | For hardware frame synchronization |
| **Depth Sensor** | Active stereo (850nm IR) | - |
| **RGB Sensor** | 1920×1080 @ 30fps | - |

**Pinout** (for sync):
- Pin 5: Ground
- Pin 9: Sync signal (GPIO output)

#### Intel RealSense D455

| Parameter | Specification | Notes |
|-----------|---------------|-------|
| **Interface** | USB 3.0 Type-C | USB 3.0 required for full performance |
| **Power** | USB-powered (~3W) | Higher power than D435 |
| **Sync Pins** | GPIO pins (3.3V) | For hardware frame synchronization |
| **Depth Sensor** | Active stereo (850nm IR) | - |
| **RGB Sensor** | 1920×1080 @ 30fps | Wider FOV than D435 |

**Documentation**: 
- [realsense.md](realsense.md)
- [CAMERA_FRAME_SYNC.md](CAMERA_FRAME_SYNC.md)

#### Camera Sync Cable

**Purpose**: Hardware frame synchronization between two RealSense cameras

**Wiring**:
- Camera 1 Pin 5 (GND) ↔ Camera 2 Pin 5 (GND)
- Camera 1 Pin 9 (Sync) ↔ Camera 2 Pin 9 (Sync)

**Cable Type**: Female-female jumper wires, 22AWG or better

### 2.3 Motor Control & IMU

#### SparkFun Auto pHAT (ROB-15443)

| Parameter | Specification | Notes |
|-----------|---------------|-------|
| **Interface** | 40-pin GPIO header (I2C) | Stackable |
| **Power** | 3.3V/5V from GPIO header | Logic power only |
| **I2C Address (IMU)** | 0x68 or 0x69 (ADR jumper) | Default: 0x69 |
| **I2C Address (Servo)** | 0x40 (PCA9685) | Fixed |
| **I2C Bus** | Bus 7 on Jetson (not Bus 1) | Different from Raspberry Pi |
| **Servo Outputs** | 4× PWM outputs | Via PCA9685 |
| **Servo Power** | External 5-6V recommended | Via USB-C on pHAT |

**Key Components**:
- **PCA9685**: 16-channel PWM controller (12-bit resolution)
- **ICM-20948**: 9DOF IMU (accel, gyro, magnetometer)

**Jumpers**:
- **ADR**: Controls IMU I2C address (open=0x68, closed=0x69)

**Documentation**: [sparkfun_auto_phat.md](sparkfun_auto_phat.md)

#### Servo Motors (MG996R)

**Model**: MG996R Digital Servo (4-pack)  
**Manufacturer**: Towerpro/compatible

| Parameter | @ 4.8V | @ 6V (optimal) | @ 7.2V | Notes |
|-----------|--------|----------------|--------|-------|
| **Operating Voltage** | 4.8V | 6V | 7.2V | Range: 4.8-7.2V |
| **Stall Torque** | 9.4 kg·cm | 12 kg·cm | 13 kg·cm | Digital, fully copper gear |
| **Speed** | 0.17 sec/60° | 0.14 sec/60° | ~0.12 sec/60° | Faster at higher voltage |
| **Current Usage** | 3A | 3A | 3A | Operating/stall current |
| **Control Signal** | PWM (50Hz, 1000-2000μs) | - | - | Via PCA9685 |
| **Connector** | 3-pin (Signal, +V, GND) | - | - | Standard servo, 290mm wire |
| **Connector Wire** | 290mm (11.4 in) | - | - | Heavy duty |
| **Weight** | 56g | - | - | Fully copper gear |

**Wiring**:
- **Signal** (Yellow/Orange): From PCA9685 channels 0-3 (via pHAT servo headers)
- **Power (+V)** (Red): External 6V supply (via pHAT USB-C port)
- **Ground** (Brown/Black): Common ground with pHAT

**Power Requirements**:
- **Peak**: 4 servos × 3A = 12A @ 6V = 72W
- **Typical**: 4-6A @ 6V = 24-36W (much lower than peak in normal operation)
- **Recommended Supply**: 6V/12A+ regulated power supply (or 5V/12A if running at 5V)

**Compatibility**:
- **Receiver types**: Futaba, Hitec, Sanwa, GWS (standard 3-pin connector)
- **Mounting**: 40×20×36mm footprint (compatible with S3003, HS322, etc.)

**Safety**: See [SERVO_SAFETY.md](SERVO_SAFETY.md)

### 2.4 Audio

#### USB Microphone

**Model**: TBD (to be specified)

| Parameter | Specification | Notes |
|-----------|---------------|-------|
| **Interface** | USB 2.0 | - |
| **Power** | USB-powered (<1W) | - |
| **Sampling Rate** | 16 kHz or 44.1 kHz | Configurable |
| **Bit Depth** | 16-bit | - |
| **Channels** | Mono or Stereo | - |

**Documentation**: [usb_microphone.md](usb_microphone.md)

### 2.5 Mobile Base

#### iRobot Create 2

| Parameter | Specification | Notes |
|-----------|---------------|-------|
| **Interface** | Serial (115200 baud) | Via USB-to-Serial adapter |
| **Protocol** | Open Interface (OI) | - |
| **Power** | Internal battery (14.4V NiMH) | ~3000mAh typical |
| **Battery Capacity** | ~16000 units (OI) | 100% charge |
| **Serial Connector** | 7-pin Mini-DIN | Requires adapter |

**Pin Functions**:
- Pin 1: Vpwr (14.4V, can power external devices)
- Pin 3: RXD (to iRobot)
- Pin 4: TXD (from iRobot)
- Pin 5: BRC (baud rate change)
- Pin 6/7: GND

**USB-to-Serial Adapter**: FTDI or CH340-based, 5V/3.3V logic

**Documentation**: [irobot.md](irobot.md)

---

## 3. Wiring & Connections

### 3.1 USB Hub Configuration

**Hub**: RSHTECH 7-Port Powered USB 3.2/USB-C Hub

**Hub Specifications**:
- **3× USB 3.2 Gen 2** (10 Gbps): 2× USB-C + 1× USB-A
- **4× USB 3.0** (5 Gbps): 2× USB-C + 2× USB-A
- **Power**: Dedicated 5V adapter (powered hub)
- **Per-port switches**: Individual on/off switches
- **Cable**: 3.3ft attached cable to Jetson

**Device Connections** (recommended):

| Device | Hub Port | USB Type | Speed | Power | Notes |
|--------|----------|----------|-------|-------|-------|
| RealSense D435 (front) | USB 3.2 Port 1 | USB-C | 10 Gbps | ~2W | High bandwidth |
| RealSense D455 (rear) | USB 3.2 Port 2 | USB-C | 10 Gbps | ~3W | High bandwidth |
| USB Microphone | USB 3.0 Port 1 | USB-A | 480 Mbps | <1W | Low bandwidth OK |
| iRobot USB-Serial | USB 3.0 Port 2 | USB-A | 480 Mbps | <1W | Low bandwidth OK |
| *Available* | USB 3.2 Port 3 | USB-A | 10 Gbps | - | For expansion |
| *Available* | USB 3.0 Port 3 | USB-C | 5 Gbps | - | For expansion |
| *Available* | USB 3.0 Port 4 | USB-C | 5 Gbps | - | For expansion |

**Advantages of Powered Hub**:
- Dedicated power supply reduces load on Jetson
- Individual port switches for device power cycling
- Allows future expansion (3 ports available)
- Better power distribution for high-power USB devices

### 3.2 GPIO Connections

#### 40-Pin Header (SparkFun Auto pHAT)

| Pin | Function | Connection | Notes |
|-----|----------|------------|-------|
| 1 | 3.3V Power | Auto pHAT | Logic power |
| 2 | 5V Power | Auto pHAT | Optional |
| 3 | I2C SDA (Bus 7) | Auto pHAT | I2C data |
| 5 | I2C SCL (Bus 7) | Auto pHAT | I2C clock |
| 6 | Ground | Auto pHAT | Common ground |
| 9, 14, 20, 25, 30, 34, 39 | Ground | Auto pHAT | Additional grounds |

**Full pinout**: See [Jetson GPIO Header](https://jetsonhacks.com/nvidia-jetson-orin-nano-gpio-header-pinout/)

### 3.3 Camera Sync Wiring

**Implementation**: Custom Ethernet harness with RJ45 breakout boards

**Configuration**: Master-Slave sync

| Camera | Role | Configuration |
|--------|------|---------------|
| Front (D435/D455) | Master | Sync pin configured as output |
| Rear (D435/D455) | Slave | Sync pin configured as input |

**Custom Ethernet Harness**:
```
Front Camera                           Rear Camera
Pin 5 (GND) ─────┐                ┌──── Pin 5 (GND)
                 │                │
                 └─── Ethernet ───┘
                 ┌─── Cable     ───┐
                 │   (shielded)    │
Pin 9 (Sync) ────┘                └──── Pin 9 (Sync)

RJ45 Breakout ──── Ethernet Cable ──── RJ45 Breakout
(Screw Term)       (Cat5e or better)   (Screw Term)
```

**RJ45 Breakout Board Wiring**:
- **Pin 5 (GND)**: Connect to any Ethernet pair (e.g., pins 1-2)
- **Pin 9 (Sync)**: Connect to another Ethernet pair (e.g., pins 3-6)
- **Note**: Only 2 pairs needed (4 wires total: 2 for GND, 2 for signal)

**Advantages**:
- **Robust**: Ethernet cable is shielded, rated for signal integrity
- **Easy to replace**: Standard Ethernet cable, any length
- **Clean**: RJ45 connectors are compact and secure
- **Flexible**: Screw terminals on breakout boards for easy wiring

**Cable Length**: Can be longer than jumper wires (<2m recommended)

**Documentation**: [CAMERA_FRAME_SYNC.md](CAMERA_FRAME_SYNC.md)

### 3.4 I2C Bus

#### Jetson I2C Bus 7 (GPIO Header)

| Address | Device | Notes |
|---------|--------|-------|
| 0x40 | PCA9685 (Servo Controller) | Fixed address |
| 0x69 | ICM-20948 (IMU) | Default with ADR jumper closed |

**Commands**:
```bash
# Scan I2C bus 7
i2cdetect -y 7

# Read IMU register
i2cget -y 7 0x69 0x00  # WHO_AM_I register
```

---

## 4. Power Management

### 4.1 Power Budget

#### Main System (5V)

| Component | Idle | Active | Peak | Notes |
|-----------|------|--------|------|-------|
| Jetson Orin Nano | 5W | 12W | 15W | Depends on workload |
| RSHTECH USB Hub | 1W | 1W | 2W | Hub logic + pass-through |
| RealSense D435 | - | 2W | 2.5W | Via hub |
| RealSense D455 | - | 3W | 3.5W | Via hub |
| USB Microphone | - | 0.5W | 1W | Via hub |
| USB-Serial Adapter | - | 0.25W | 0.5W | Via hub |
| Auto pHAT (logic) | - | 0.2W | 0.3W | Via GPIO, excludes servos |
| **Total (5V)** | **~6W** | **~19W** | **~25W** | **Main system only** |

**Recommended 5V Supply**: 5V/5A (25W) minimum for main system

#### Servo System (6V, separate)

| Component | Idle | Typical | Peak | Notes |
|-----------|------|---------|------|-------|
| MG996R Servo (each) | 0W | 4-6W | 18W | 3A current usage |
| 4× MG996R Servos | 0W | 16-24W | 72W | All servos active |
| **Total (6V)** | **0W** | **16-24W** | **72W** | **4 servos @ 3A each** |

**Recommended 6V Supply**: 6V/12A (72W) minimum for servo system

**Alternative**: 5V/12A supply if running servos at 5V (lower torque: 10 kg·cm vs 12 kg·cm)

**Note**: Servos typically draw much less than stall current during normal operation

#### iRobot Create 2 (independent)

- **Battery**: 14.4V NiMH, ~3000mAh
- **Power**: Independent, internal battery

### 4.2 Power Supply Selection

#### Recommended Configuration: Dual Supply System

**Supply 1: Main System (5V)**
- **Output**: 5V @ 5A (25W) minimum
- **Connector**: USB-C or barrel jack for Jetson
- **Powers**: Jetson, USB hub (or separate hub adapter), all USB devices
- **Example**: Standard Jetson power adapter + RSHTECH hub adapter

**Supply 2: Servo System (6V)** 
- **Output**: 6V @ 12A (72W) minimum
- **Connector**: USB-C on SparkFun Auto pHAT
- **Powers**: 4× MG996R servos via PCA9685
- **Requirement**: Regulated 5-6V supply (4.8-7.2V acceptable)
- **Example**: Adjustable DC power supply set to 6V, or 5V/12A+ regulated supply
- **Note**: Can use 5V supply (lower torque: 10 kg·cm vs 12 kg·cm @ 6V)

**Advantages of Dual Supply**:
- **Isolation**: Servo motor noise doesn't affect Jetson/digital logic
- **Capacity**: Each supply sized for its specific load
- **Safety**: Servo overcurrent won't affect main system
- **Flexibility**: Can power servos independently for testing

#### Alternative: Single High-Capacity Supply (not recommended)

If using a single supply, you would need:
- **Output**: 12V @ 8A (96W) minimum
- **Voltage regulators**: 
  - 12V → 5V buck converter (25W) for main system
  - 12V → 6V buck converter (60W) for servos
- **Disadvantage**: More complex, less isolation, harder to debug

### 4.3 Power Distribution

**Recommended Dual Supply Configuration**:

```
Wall Power (AC)
│
├─ 5V/5A Supply (25W)
│  │
│  ├─ Jetson Orin Nano (USB-C or barrel jack)
│  │  │
│  │  ├─ GPIO Header → Auto pHAT (logic power only)
│  │  │                  │
│  │  │                  ├─ ICM-20948 IMU
│  │  │                  └─ PCA9685 (servo controller logic)
│  │  │
│  │  └─ USB-A → RSHTECH Hub (or hub has separate adapter)
│  │             │
│  │             ├─ RealSense D435 (~2W)
│  │             ├─ RealSense D455 (~3W)
│  │             ├─ USB Microphone (<1W)
│  │             └─ iRobot USB-Serial (<1W)
│  │
│  └─ (Hub may have separate 5V adapter)
│
└─ 6V/10A Supply (60W, separate)
   │
   └─ Auto pHAT USB-C Port → Servo Power Rails
                              │
                              ├─ MG996R Servo 0 (channel 0)
                              ├─ MG996R Servo 1 (channel 1)
                              ├─ MG996R Servo 2 (channel 2)
                              └─ MG996R Servo 3 (channel 3)
```

**Key Points**:
- **Main system** (5V): Jetson + all USB devices via powered hub
- **Servo system** (6V): Completely separate power supply
- **Common ground**: Servo ground connected via Auto pHAT
- **Signal isolation**: PCA9685 provides logic-level isolation

### 4.4 Power Sequencing

**Startup**:
1. Connect Jetson power supply
2. Power on Jetson (auto-starts if configured)
3. USB devices enumerated automatically
4. Auto pHAT powered via GPIO header
5. (If separate) Connect servo power supply

**Shutdown**:
1. Stop ROS 2 nodes (servos move to safe position)
2. Shutdown Jetson: `sudo shutdown -h now`
3. Wait for Jetson to power off
4. (If separate) Disconnect servo power supply
5. Disconnect Jetson power

---

## 5. Communication Interfaces

### 5.1 Interface Summary

| Interface | Devices | Speed | Notes |
|-----------|---------|-------|-------|
| USB 3.0 | 2× RealSense | 5 Gbps | Depth + RGB streams |
| USB 2.0 | Microphone, Serial | 480 Mbps | - |
| I2C (Bus 7) | Auto pHAT (IMU, Servos) | 400 kHz | Via GPIO header |
| Serial (USB-TTL) | iRobot Create | 115200 baud | Open Interface protocol |

### 5.2 USB Bandwidth

**USB 3.0 Bandwidth** (per port):
- Theoretical: 5 Gbps (625 MB/s)
- Practical: ~400 MB/s

**RealSense Bandwidth** (per camera):
- Color: 1920×1080 @ 30fps, YUY2 = ~60 MB/s
- Depth: 640×480 @ 30fps, 16-bit = ~18 MB/s
- Total per camera: ~78 MB/s
- **Two cameras**: ~156 MB/s (well within USB 3.0 capacity)

### 5.3 I2C Bus

**Jetson I2C Bus 7** (via GPIO header):
- **Speed**: 400 kHz (fast mode)
- **Voltage**: 3.3V logic
- **Devices**: 2 (PCA9685, ICM-20948)
- **Conflicts**: None (fixed addresses)

**Note**: Jetson Orin Nano uses different I2C bus numbers than Raspberry Pi. The GPIO header I2C is **Bus 7**, not Bus 1.

---

## 6. Grounding & EMI

### 6.1 Ground Strategy

**Single-Point Ground**:
- All devices share common ground via Jetson
- USB devices: Ground via USB cable shield
- Auto pHAT: Ground via GPIO header (multiple pins)
- iRobot: Ground via USB-serial adapter

**Ground Loops**:
- Avoid: Connecting iRobot Vpwr ground to Jetson ground directly
- OK: USB-serial adapter provides isolated ground path

### 6.2 EMI Mitigation

**USB Cables**:
- Use shielded USB 3.0 cables for cameras
- Keep cables <2m to minimize signal integrity issues
- Route power and data cables separately when possible

**Servo Wiring**:
- Use separate power supply for servos (reduces digital noise)
- Twist servo power wires together
- Add capacitors near servo power input (100μF + 0.1μF)

**I2C Pull-ups**:
- Auto pHAT has built-in pull-ups (no external resistors needed)
- Typical: 4.7kΩ to 3.3V

---

## 7. Safety & Protection

### 7.1 Overcurrent Protection

**USB Ports**: Jetson has built-in overcurrent protection (per-port)

**Servos**: Use fuse or current-limited supply for servo power
- **Recommendation**: 5A fast-blow fuse on servo power rail

### 7.2 ESD Protection

**Best Practices**:
- Handle boards with anti-static wrist strap
- Store in anti-static bags when not in use
- Avoid touching GPIO pins directly

### 7.3 Reverse Polarity Protection

**USB**: Inherently protected by connector design

**Servos**: Verify polarity before connecting power supply
- Red/Orange: +V
- Brown/Black: GND

### 7.4 Thermal Management

**Jetson Orin Nano**:
- Operating temp: 0°C to 50°C
- Heatsink required
- Consider fan if sustained high load

**RealSense Cameras**:
- Operating temp: 0°C to 45°C
- Passive cooling (adequate airflow required)

---

## 8. Troubleshooting

### 8.1 Power Issues

**Symptom**: Jetson won't boot or reboots randomly
- Check power supply capacity (≥4A @ 5V)
- Verify barrel jack or USB-C connection
- Check for undervoltage warnings in `dmesg`

**Symptom**: USB device not detected
- Check USB cable (try different cable)
- Check power budget (use powered USB hub if needed)
- Verify USB port not disabled: `lsusb`

### 8.2 USB Issues

**Symptom**: RealSense camera low FPS
- Verify USB 3.0 connection: `lsusb -t` (look for "5000M")
- Use USB 3.0 cable (blue connectors)
- Check USB bandwidth: reduce resolution or framerate

**Symptom**: USB device permission denied
- Add user to `dialout` group: `sudo usermod -a -G dialout $USER`
- Log out and back in

### 8.3 I2C Issues

**Symptom**: I2C device not detected
- Check bus number (Bus 7 on Jetson, not Bus 1)
- Verify connections: `i2cdetect -y 7`
- Check pull-ups (should be on pHAT board)
- Check I2C permissions: `sudo chmod 666 /dev/i2c-7`

### 8.4 Serial Issues

**Symptom**: iRobot not responding
- Check serial port: `ls -l /dev/ttyUSB*`
- Verify baud rate (115200 for Create 2)
- Check dialout group membership
- Test with minicom: `minicom -D /dev/ttyUSB0 -b 115200`

---

## 9. Testing & Verification

### 9.1 Electrical Verification Script

Run comprehensive hardware checks:

```bash
cd ~/src/jetson-orin-nano
./scripts/hardware/verify_all_hardware.sh
```

### 9.2 Manual Tests

#### USB Devices
```bash
# List USB devices
lsusb

# Expected devices:
# - Intel RealSense (8086:0b07 or similar)
# - USB microphone
# - USB-serial adapter
```

#### I2C Devices
```bash
# Scan I2C bus 7
i2cdetect -y 7

# Expected devices:
# - 0x40: PCA9685 (servo controller)
# - 0x69: ICM-20948 (IMU)
```

#### Serial Ports
```bash
# List serial devices
ls -l /dev/ttyUSB* /dev/ttyACM*

# Test serial loopback (if available)
# Connect TX to RX and use minicom
```

---

## 10. References

### 10.1 Component Documentation

- [BOM.md](BOM.md) - Complete bill of materials
- [MECHANICAL.md](MECHANICAL.md) - Mechanical specifications
- [realsense.md](realsense.md) - RealSense camera setup
- [sparkfun_auto_phat.md](sparkfun_auto_phat.md) - Auto pHAT setup
- [irobot.md](irobot.md) - iRobot Create setup
- [usb_microphone.md](usb_microphone.md) - Microphone setup

### 10.2 Datasheets & Manuals

- [Jetson Orin Nano Developer Kit User Guide](https://developer.nvidia.com/embedded/learn/jetson-orin-nano-devkit-user-guide)
- [Intel RealSense D400 Series Datasheet](https://www.intelrealsense.com/depth-camera-d435/)
- [SparkFun Auto pHAT Hookup Guide](https://learn.sparkfun.com/tutorials/sparkfun-auto-phat-hookup-guide)
- [iRobot Create 2 Open Interface Spec](https://edu.irobot.com/learning-library/create-2-oi-spec)
- [PCA9685 Datasheet](https://www.nxp.com/docs/en/data-sheet/PCA9685.pdf)
- [ICM-20948 Datasheet](https://invensense.tdk.com/products/motion-tracking/9-axis/icm-20948/)

### 10.3 Standards & Specifications

- USB 2.0: 480 Mbps, 500mA per port
- USB 3.0: 5 Gbps, 900mA per port
- I2C: 100 kHz (standard), 400 kHz (fast mode)
- PWM Servo: 50 Hz, 1000-2000μs pulse width
- RS-232: Typically 115200 baud for robotics

---

**Last Updated**: 2026-01-27

**Next Steps**: 
1. Verify all electrical connections match this documentation
2. Run electrical verification tests
3. Update with actual component part numbers and specifications
