# Bill of Materials (BOM)

## Overview

This document lists all hardware components used in the Isaac robot system, organized by category. Each component includes specifications, part numbers, and references to detailed documentation.

**Last Updated**: 2026-01-27

---

## 1. Computing Hardware

### 1.1 Main Computer

| Item | Description | Qty | Part Number | Specs | Documentation |
|------|-------------|-----|-------------|-------|---------------|
| Jetson Orin Nano | Main compute platform | 1 | 945-13766-0000-000 | 8GB RAM, Ubuntu 22.04 | [HARDWARE.md](HARDWARE.md) |
| microSD Card | System storage | 1 | TBD | ≥128GB, Class 10, UHS-I | - |
| Power Supply | Jetson power | 1 | TBD | 5V/4A USB-C or barrel jack | - |

---

## 2. Electrical Components

### 2.1 Vision System

| Item | Description | Qty | Part Number | Specs | Documentation |
|------|-------------|-----|-------------|-------|---------------|
| Intel RealSense D435 | Depth camera (front) | 1 | 82635D435DKH1 | USB 3.0, depth + RGB | [realsense.md](realsense.md) |
| Intel RealSense D455 | Depth camera (rear) | 1 | 82635D455DKH1 | USB 3.0, depth + RGB | [realsense.md](realsense.md) |
| Camera Sync Cable | Hardware frame sync | 1 | Custom/DIY | Ground + GPIO wire | [CAMERA_FRAME_SYNC.md](CAMERA_FRAME_SYNC.md) |

**Note**: Camera models may vary (D435 or D455), both are supported.

### 2.2 Motor Control & IMU

| Item | Description | Qty | Part Number | Specs | Documentation |
|------|-------------|-----|-------------|-------|---------------|
| SparkFun Auto pHAT | Servo controller + IMU | 1 | ROB-15443 | 4x servo outputs, ICM-20948 9DOF | [sparkfun_auto_phat.md](sparkfun_auto_phat.md) |
| MG996R Servo Motor | Camera pan/tilt actuation | 4 | MG996R (4-pack) | Digital, 180°, 13 kg·cm @ 7.2V | [SERVO_SAFETY.md](SERVO_SAFETY.md) |

**MG996R Servo Specifications** (from manufacturer):
- Operating voltage: 4.8-7.2V (recommended: 5-6V)
- Stall torque: 9.4 kg·cm @ 4.8V, 12 kg·cm @ 6V, 13 kg·cm @ 7.2V
- Speed: 0.17 sec/60° @ 4.8V, 0.14 sec/60° @ 6V
- Current usage: 3A (operating/stall)
- PWM range: 1000-2000μs (standard)
- Connector wire: 290mm (11.4 in)
- Weight: 56g (fully copper gear)
- Control angle: 180° (±90°)
- Compatibility: Futaba, Hitec, Sanwa, GWS receivers

**Reference Images**: See [images/mg996r_specs.png](images/mg996r_specs.png) and [images/mg996r_dimensions.png](images/mg996r_dimensions.png)

### 2.3 Audio System

| Item | Description | Qty | Part Number | Specs | Documentation |
|------|-------------|-----|-------------|-------|---------------|
| USB Microphone | Audio input | 1 | TBD | USB 2.0, ALSA/PulseAudio | [usb_microphone.md](usb_microphone.md) |

### 2.4 USB Hub & Power Distribution

| Item | Description | Qty | Part Number | Specs | Documentation |
|------|-------------|-----|-------------|-------|---------------|
| RSHTECH USB Hub | 7-port powered USB 3.2/USB-C hub | 1 | RSHTECH 7-Port | 3x USB 3.2 (2C+1A), 4x USB 3.0 (2C+2A), 5V adapter | [ELECTRICAL.md](ELECTRICAL.md) |

**Hub Specifications**:
- Ports: 3x USB 3.2 Gen 2 (10 Gbps), 4x USB 3.0 (5 Gbps)
- Connectors: 4x USB-C, 3x USB-A
- Power: 5V power adapter (included), individual port switches
- Cable: 3.3ft attached cable

### 2.5 Mobile Base

| Item | Description | Qty | Part Number | Specs | Documentation |
|------|-------------|-----|-------------|-------|---------------|
| iRobot Create 2 | Mobile robot base | 1 | TBD | Serial OI, differential drive | [irobot.md](irobot.md) |
| USB-to-Serial Adapter | iRobot communication | 1 | TBD | USB 2.0, 115200 baud | [irobot.md](irobot.md) |

### 2.6 Cables & Connectors

| Item | Description | Qty | Part Number | Specs | Notes |
|------|-------------|-----|-------------|-------|-------|
| USB 3.0 Cable | RealSense cameras | 2 | TBD | ≥1m, USB-A to USB-C | Must be USB 3.0 |
| USB 2.0 Cable | Microphone | 1 | TBD | ≥1m, USB-A | - |
| USB 2.0 Cable | iRobot serial | 1 | TBD | ≥1m, USB-A | - |
| IDC GPIO Ribbon Cable | Auto pHAT connection | 1 | 40Pin M-F 8" | 40-pin, Male-Female, 8 inch | For Raspberry Pi compatible |
| Custom Ethernet Harness | Camera sync | 1 | Custom/DIY | RJ45-based, ground + sync | See § 2.6.1 |
| RJ45 Breakout Board | Ethernet to terminal | 2 | 2.54mm pitch | Screw terminal adapter | For camera sync harness |
| JST SH 1.0mm Kit | Small signal connectors | 1 set | 28AWG, 150mm | 3-12 pin housing, pre-crimped | Optional for custom wiring |

#### 2.6.1 Camera Sync Cable Details

**Custom Ethernet Harness**:
- **Base cable**: Standard Ethernet cable (Cat5e or better)
- **Breakout**: RJ45 female breakout boards (2x, screw terminals)
- **Wiring**: 
  - Connect Ground (any pair) to both cameras Pin 5
  - Connect Sync signal (one wire) to both cameras Pin 9
- **Advantage**: Robust, shielded, easy to replace
- **Alternative**: Could use JST connectors or direct jumper wires

See [CAMERA_FRAME_SYNC.md](CAMERA_FRAME_SYNC.md) for wiring details.

---

## 3. Mechanical Components

### 3.1 Structural Components

| Item | Description | Qty | Part Number | Material | Dimensions | Documentation |
|------|-------------|-----|-------------|----------|------------|---------------|
| iRobot Create 2 Chassis | Robot base platform | 1 | iRobot Create 2 | ABS plastic | 340mm dia × 92mm H | [irobot.md](irobot.md) |
| TBD | Main mounting plate/deck | TBD | TBD | TBD | TBD | [MECHANICAL.md](MECHANICAL.md) |
| TBD | Jetson mounting bracket | TBD | TBD | TBD | TBD | [MECHANICAL.md](MECHANICAL.md) |

**To be filled in**: Add mounting plate/deck and Jetson bracket details

### 3.2 Fasteners & Hardware

| Item | Description | Qty | Part Number | Specs | Notes |
|------|-------------|-----|-------------|-------|-------|
| M3 Screws | General mounting | TBD | TBD | Length: TBD | For brackets, camera mounts |
| M2.5 Screws | Board mounting | TBD | TBD | Length: TBD | For Jetson, pHAT |
| Standoffs | Board spacing | TBD | TBD | Height: TBD | For Jetson, electronics |
| Washers | Load distribution | TBD | TBD | M3/M2.5 | - |

**To be measured**: Count and document fasteners during assembly

### 3.3 Servo Mounting Hardware

| Item | Description | Qty | Part Number | Specs | Notes |
|------|-------------|-----|-------------|-------|-------|
| Pan Tilt Servo Bracket | 2-DOF pan/tilt mount | 2 sets | elechawk 88065 | Aluminum, 170×120×26mm | Includes all hardware |
| Servo Horns | Servo output arms | 4 | Included with MG996R | Plastic, multi-hole | Included with servos |
| Mounting Hardware | Screws, bearings, spacers | 2 sets | Included with 88065 | M3 screws, ball bearings | Per image |

**elechawk 88065 Pan/Tilt Bracket Specifications**:
- **Material**: Aluminum alloy, matte black coating
- **Dimensions**: 6.69 × 4.72 × 1.02 inches (170 × 120 × 26 mm)
- **Weight**: 110g total (both sets combined, ~55g per set)
- **Bearings**: Ball bearings for smooth rotation
- **DOF**: 2-axis (pan and tilt for horizontal surface)
- **Compatibility**: MG996R, MG996, SG5010, HS322, HS422, Futaba S3003, etc. (40×20×36mm servos)
- **Included Hardware**: 
  - Aluminum U-brackets (pan and tilt)
  - Servo horns/discs (white plastic)
  - Ball bearings
  - M3 screws (black)
  - Spacers and washers
- **Applications**: Camera/sensor mounting, robot joints (shoulders, knees), humanoid robots, rovers
- **Quantity**: 2 complete sets (one for front camera, one for rear camera)

**Reference Images**: 
- Assembly stages: [images/pan_tilt_bracket_assembly.png](images/pan_tilt_bracket_assembly.png)
- Parts breakdown: [images/pan_tilt_bracket_parts.png](images/pan_tilt_bracket_parts.png)

---

## 4. Tools & Equipment

### 4.1 Assembly Tools

| Item | Description | Notes |
|------|-------------|-------|
| Phillips screwdriver | M3/M2.5 screws | - |
| Hex key set | Metric | For standoffs |
| Wire strippers | 22-24 AWG | For custom cables |
| Soldering iron | Optional | For permanent connections |
| Multimeter | Testing | For debugging |

### 4.2 Calibration Tools

| Item | Description | Notes |
|------|-------------|-------|
| Calibration target | AprilTag or checkerboard | For camera calibration |
| Ruler/caliper | Mechanical measurements | For tolerance verification |
| Level | Alignment verification | For IMU calibration |

---

## 5. Software Dependencies

See [../setup/SETUP.md](../setup/SETUP.md) for full software installation.

### 5.1 Core Software

- **OS**: Ubuntu 22.04 (JetPack 5.x)
- **ROS**: ROS 2 Humble
- **Intel RealSense SDK**: librealsense2
- **Python**: 3.10+

### 5.2 Hardware-Specific Libraries

- **RealSense**: `pyrealsense2`, `librealsense2`
- **SparkFun**: `smbus2`, `sparkfun-qwiic-icm20948`
- **iRobot**: `pyserial`
- **Audio**: ALSA, PulseAudio

---

## 6. Procurement Notes

### 6.1 Recommended Vendors

- **Computing**: NVIDIA (Jetson), major electronics distributors
- **Cameras**: Intel RealSense authorized distributors
- **Components**: SparkFun, Adafruit, DigiKey, Mouser
- **Mechanical**: McMaster-Carr, local hardware suppliers

### 6.2 Cost Estimates

| Category | Item | Estimated Cost |
|----------|------|----------------|
| Computing | Jetson Orin Nano + storage | $400-500 |
| Vision | 2x RealSense D435/D455 | $350-500 |
| Motor Control | SparkFun Auto pHAT | $30-40 |
| Motor Control | 4x MG996R Servos | $25-35 |
| USB Hub | RSHTECH 7-Port Powered Hub | $35-45 |
| Audio | USB Microphone | $20-50 |
| Mobile Base | iRobot Create 2 | $200-300 (or reuse) |
| Servo Mounts | 2 sets RC + 2 sets Pan/Tilt | $30-50 |
| Cables | USB cables, GPIO ribbon, etc. | $20-30 |
| Connectors | RJ45 breakouts, JST kit | $15-25 |
| Fasteners | M3/M2.5 screws, standoffs | $10-20 |
| Mounting Hardware | Plates, brackets (TBD) | $20-50 |
| **Total** | | **~$1,155-1,645** |

*Prices are estimates based on typical online retailers (Amazon, SparkFun, etc.)*

---

## 7. Notes & Maintenance

### 7.1 Component Lifecycle

- **Jetson microSD**: Replace annually or on failure
- **Camera cables**: Inspect quarterly for wear
- **Servo motors**: Verify torque quarterly
- **Fasteners**: Check tightness monthly

### 7.2 Spare Parts

Recommended spares:
- microSD card (1x)
- USB cables (1x each type)
- Servo motors (1x)
- Fasteners (assorted)

### 7.3 Revision History

| Date | Version | Changes | Author |
|------|---------|---------|--------|
| 2026-01-27 | 1.0 | Initial BOM created | - |

---

## Next Steps

1. **Fill in TBD mechanical components**: Document actual brackets, mounts, fasteners in use
2. **Add part numbers**: Record specific part numbers for purchased components
3. **Document servo specifications**: Record actual servo model, torque, speed after testing
4. **Create mechanical drawings**: See [MECHANICAL.md](MECHANICAL.md) for detailed mechanical documentation
5. **Update cost estimates**: Track actual procurement costs
6. **Photograph assembly**: Take reference photos for documentation

See [MECHANICAL.md](MECHANICAL.md) for detailed mechanical specifications and tolerances.
