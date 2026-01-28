# Mechanical Documentation

## Overview

This document describes the mechanical design, dimensions, tolerances, and calibration values for the Isaac robot system. The mechanical model defined here is used for hardware verification, calibration, and simulation.

**Last Updated**: 2026-01-27

---

## 1. Mechanical Architecture

### 1.1 Design Goals

- **Modularity**: Easy to assemble/disassemble for maintenance
- **Stability**: Rigid mounting for cameras and sensors
- **Accessibility**: Easy access to electronics for debugging
- **Calibration**: Design supports calibration target visibility
- **Simulation**: Model is compatible with Isaac Sim, Gazebo, MuJoCo

### 1.2 Coordinate Systems

#### 1.2.1 Robot Base Frame

- **Origin**: Center of iRobot Create base, ground level
- **X-axis**: Forward (robot front)
- **Y-axis**: Left
- **Z-axis**: Up
- **Frame ID**: `base_link`

#### 1.2.2 Camera Frames

- **Front Camera**: `camera_front_link`
  - Origin: TBD (relative to base_link)
  - Orientation: TBD
  
- **Rear Camera**: `camera_rear_link`
  - Origin: TBD (relative to base_link)
  - Orientation: TBD

#### 1.2.3 IMU Frame

- **IMU**: `imu_link`
  - Origin: SparkFun Auto pHAT IMU location
  - Orientation: Aligned with robot base (verify with calibration)

### 1.3 Assembly Overview

```
┌─────────────────────────────────────────┐
│                                         │
│   [Front Camera]    [Rear Camera]       │  ← Camera mounting level
│         │                │              │
│         └────────┬───────┘              │
│                  │                      │
│         [Servo Pan/Tilt Mount]          │  ← Actuation layer
│                  │                      │
├─────────────────────────────────────────┤
│                                         │
│     [Jetson Orin Nano + Auto pHAT]      │  ← Computing layer
│     [USB Hub]  [Microphone]             │
│                                         │
├─────────────────────────────────────────┤
│                                         │
│          [iRobot Create Base]           │  ← Mobile base
│                                         │
└─────────────────────────────────────────┘
```

---

## 2. Component Mechanical Specifications

### 2.1 Intel RealSense Cameras

#### D435/D455 Physical Dimensions

| Parameter | D435 | D455 | Tolerance |
|-----------|------|------|-----------|
| Length | 90 mm | 124 mm | ±0.5 mm |
| Width | 25 mm | 26 mm | ±0.5 mm |
| Height | 25 mm | 29 mm | ±0.5 mm |
| Mass | 72 g | 126 g | ±5 g |

#### Mounting Holes

- **Thread**: 1/4"-20 (standard camera tripod mount)
- **Location**: Bottom center
- **Depth**: TBD mm

#### Camera Optical Center

**To be calibrated**: Offset from mounting hole to optical center
- **X offset**: TBD mm
- **Y offset**: TBD mm
- **Z offset**: TBD mm

#### Field of View (FOV)

| Parameter | D435 | D455 |
|-----------|------|------|
| Depth FOV (H×V) | 87°×58° | 87°×58° |
| RGB FOV (H×V) | 69°×42° | 90°×65° |

### 2.2 Jetson Orin Nano

#### Physical Dimensions

| Parameter | Value | Tolerance |
|-----------|-------|-----------|
| Length | 100 mm | ±0.5 mm |
| Width | 79 mm | ±0.5 mm |
| Height (with heatsink) | ~40 mm | ±2 mm |
| Mass (module only) | ~200 g | ±10 g |

#### Mounting Holes

- **Thread**: M3
- **Hole pattern**: See [official carrier board docs](https://developer.nvidia.com/embedded/learn/jetson-orin-nano-devkit-user-guide)
- **Hole locations**: TBD (measure from actual board)

### 2.3 SparkFun Auto pHAT

#### Physical Dimensions

| Parameter | Value | Tolerance |
|-----------|-------|-----------|
| Length | 65 mm | ±0.5 mm |
| Width | 56 mm | ±0.5 mm |
| Height (above GPIO) | ~15 mm | ±2 mm |
| Mass | ~25 g | ±5 g |

#### Connection

- **Interface**: 40-pin GPIO header (stackable)
- **Orientation**: Overhangs right side of Jetson when properly aligned

#### IMU Location

- **Chip**: ICM-20948
- **Location on board**: TBD (measure from board edge)
- **Orientation**: To be verified during IMU calibration

### 2.4 Servo Motors

**Model**: MG996R Digital Servo (4-pack)  
**Manufacturer**: Towerpro/compatible

#### Physical Specifications

| Parameter | Value | Tolerance | Notes |
|-----------|-------|-----------|-------|
| Length | 40.0 mm | ±0.5 mm | Body only |
| Width | 19.0 mm | ±0.5 mm | Body only |
| Height | 43.0 mm | ±0.5 mm | With mounting tabs |
| Mass | 56 g | ±5 g | Fully copper gear |
| Connector wire | 290 mm | ±10 mm | 11.4 inches |

#### Performance Specifications

| Parameter | @ 4.8V | @ 6V | @ 7.2V | Notes |
|-----------|--------|------|--------|-------|
| Operating voltage | 4.8V | 6V | 7.2V | Range: 4.8-7.2V |
| Stall torque | 9.4 kg·cm | 12 kg·cm | 13 kg·cm | Digital servo, copper gear |
| Speed (no load) | 0.17 sec/60° | 0.14 sec/60° | ~0.12 sec/60° | Faster at higher voltage |
| Current usage | 3A | 3A | 3A | Operating/stall current |
| Operating angle | 180° (±90°) | 180° (±90°) | 180° (±90°) | ±2° tolerance |

#### PWM Specifications

| Parameter | Min | Center | Max | Tolerance |
|-----------|-----|--------|-----|-----------|
| Pulse width | 1000 μs | 1500 μs | 2000 μs | ±10 μs |
| Corresponding angle | 0° | 90° | 180° | ±2° |
| PWM Frequency | 50 Hz | - | - | Standard servo |

#### Compatibility

- **Receiver types**: Futaba, Hitec, Sanwa, GWS
- **Compatible servos**: MG996, SG5010, HS322, HS422, Futaba S3003 (same footprint: 40×20×36mm)
- **Applications**: 1:10 RC cars, robots, RC helicopters, aircraft, DIY projects

**Notes**:
- Digital servo with fully copper gears (good wear resistance, noise reduction)
- Higher torque and precision than analog servos (SG90, MG90S)
- **Recommended operating voltage**: 5-6V for balance of torque and longevity
- **To be calibrated**: Actual PWM-to-angle mapping may vary per unit
- **Power requirement**: 4 servos @ 3A each = 12A peak (use separate 6V/12A+ power supply)

### 2.5 iRobot Create 2

#### Physical Dimensions

| Parameter | Value | Tolerance |
|-----------|-------|-----------|
| Diameter | 340 mm | ±2 mm |
| Height | 92 mm | ±2 mm |
| Mass (with battery) | ~3.5 kg | ±100 g |
| Wheel diameter | 72 mm | ±1 mm |
| Wheelbase | 235 mm | ±2 mm |

#### Kinematic Parameters

| Parameter | Value | Tolerance | Notes |
|-----------|-------|-----------|-------|
| Max linear velocity | 500 mm/s | - | OI protocol limit |
| Max angular velocity | TBD rad/s | - | Depends on radius |
| Wheel radius | 36 mm | ±0.5 mm | Half of diameter |
| Wheel separation | 235 mm | ±2 mm | Track width |

---

## 3. Mounting & Brackets

### 3.1 Camera Mounting Brackets

**Implementation**: elechawk 88065 pan/tilt servo brackets (2 sets) with MG996R servos

#### Design Requirements

- Support camera mass (72-126g per camera) + cable forces
- Allow access to camera sync pins (for custom ethernet harness)
- Minimize vibration (ball bearing design helps)
- Allow adjustment for calibration
- Pan/tilt actuation via 2-axis (2-DOF) servo mounts

#### Bracket Specifications (elechawk 88065)

| Parameter | Value | Notes |
|-----------|-------|-------|
| **Model** | elechawk 88065 | 2 sets included |
| **Material** | Aluminum alloy | Matte black coating |
| **Overall Dimensions** | 170 × 120 × 26 mm | 6.69 × 4.72 × 1.02 inches |
| **Mass** | ~55g per set | 110g total for both sets |
| **DOF** | 2-axis (pan + tilt) | Horizontal surface orientation |
| **Bearings** | Ball bearings | Smooth rotation |
| **Servo Compatibility** | 40×20×36mm footprint | MG996R, SG5010, HS322, HS422, S3003 |

#### Included Hardware (per set)

- **Aluminum U-brackets**: Pan base and tilt arm (black anodized)
- **Servo horns/discs**: White plastic, multi-hole (25mm diameter)
- **Ball bearings**: For smooth pan/tilt rotation
- **M3 screws**: Black socket head (various lengths)
- **Spacers & washers**: For proper servo alignment
- **Assembly brackets**: Multiple mounting holes for flexibility

#### Mounting Configuration

**Front Camera Assembly**:
- **Pan servo** (MG996R, channel 0): Base rotation, mounted vertically
- **Tilt servo** (MG996R, channel 1): Camera tilt, mounted on pan arm
- **Camera mount**: Attached to tilt arm (1/4"-20 threaded mount or custom)

**Rear Camera Assembly**:
- **Pan servo** (MG996R, channel 2): Base rotation, mounted vertically
- **Tilt servo** (MG996R, channel 3): Camera tilt, mounted on pan arm
- **Camera mount**: Attached to tilt arm (1/4"-20 threaded mount or custom)

**Pivot Points** (to be measured after assembly):
- **Pan axis**: Vertical rotation center (Z-axis rotation)
- **Tilt axis**: Horizontal rotation center (Y-axis rotation)
- **Camera offset**: Distance from tilt pivot to camera optical center

**To be measured**: 
- Actual camera position relative to pan/tilt pivot points
- Pan servo pivot height above robot base
- Camera optical center offset from tilt pivot

### 3.2 Jetson Mounting

**To be documented**: Describe actual mounting solution in use

#### Design Requirements

- Secure Jetson to base platform
- Provide thermal clearance for heatsink
- Allow access to USB ports
- Support Auto pHAT via GPIO header

### 3.3 Assembly Hardware

**To be documented**: List actual fasteners in use

See [BOM.md](BOM.md) for complete list of fasteners and hardware.

---

## 4. Tolerances & Calibration

### 4.1 Manufacturing Tolerances

#### Critical Dimensions

| Parameter | Nominal | Tolerance | Notes |
|-----------|---------|-----------|-------|
| Camera-to-camera baseline | TBD mm | ±1 mm | For stereo accuracy |
| Camera height (Z) | TBD mm | ±2 mm | Affects ground plane |
| Camera pitch angle | TBD° | ±1° | Horizon alignment |
| IMU orientation | 0° | ±1° | Alignment with base |

#### Non-Critical Dimensions

| Parameter | Nominal | Tolerance | Notes |
|-----------|---------|-----------|-------|
| Fastener holes | M3/M2.5 | +0.2 mm | Clearance fit |
| Cable routing clearance | ≥10 mm | ±5 mm | Prevent snagging |

### 4.2 Calibration Values

**Location**: `/home/nano/.config/robot/calibration.yaml`

Calibration values are automatically generated and updated by the calibration nodes. The nominal values below are initial estimates to be refined through calibration.

#### 4.2.1 Camera Extrinsics

**Front Camera to Base Link** (`camera_front_link`)

```yaml
camera_front_extrinsics:
  translation:  # meters, [x, y, z] relative to base_link
    x: TBD  # forward from base center
    y: TBD  # left from base center
    z: TBD  # up from base ground level
    tolerance: [0.005, 0.005, 0.005]  # ±5mm in each axis
  
  rotation:  # quaternion [x, y, z, w] or euler [roll, pitch, yaw] in radians
    roll: TBD   # rotation about x-axis
    pitch: TBD  # rotation about y-axis (camera tilt)
    yaw: TBD    # rotation about z-axis (camera pan)
    tolerance: [0.017, 0.017, 0.017]  # ±1° in radians
```

**Rear Camera to Base Link** (`camera_rear_link`)

```yaml
camera_rear_extrinsics:
  translation:  # meters
    x: TBD
    y: TBD
    z: TBD
    tolerance: [0.005, 0.005, 0.005]
  
  rotation:  # euler angles in radians
    roll: TBD
    pitch: TBD
    yaw: TBD
    tolerance: [0.017, 0.017, 0.017]
```

#### 4.2.2 Camera Intrinsics

**To be calibrated using ROS 2 camera calibration or RealSense SDK**

```yaml
camera_front_intrinsics:
  color:
    fx: TBD  # focal length x (pixels)
    fy: TBD  # focal length y (pixels)
    cx: TBD  # principal point x (pixels)
    cy: TBD  # principal point y (pixels)
    distortion_model: "plumb_bob"  # or "rational_polynomial"
    distortion_coefficients: [k1, k2, p1, p2, k3]  # TBD
  
  depth:
    fx: TBD
    fy: TBD
    cx: TBD
    cy: TBD
    distortion_model: "plumb_bob"
    distortion_coefficients: [k1, k2, p1, p2, k3]  # TBD
```

#### 4.2.3 IMU Calibration

```yaml
imu_calibration:
  # IMU to base_link transform
  translation:  # meters
    x: TBD  # IMU location on Auto pHAT
    y: TBD
    z: TBD
    tolerance: [0.005, 0.005, 0.005]
  
  rotation:  # euler angles in radians
    roll: TBD   # IMU alignment with base
    pitch: TBD
    yaw: TBD
    tolerance: [0.017, 0.017, 0.017]
  
  # Accelerometer bias (measured at rest, gravity removed)
  accel_bias:  # m/s²
    x: 0.0
    y: 0.0
    z: 0.0
    tolerance: [0.1, 0.1, 0.1]
  
  # Gyroscope bias (measured at rest)
  gyro_bias:  # rad/s
    x: 0.0
    y: 0.0
    z: 0.0
    tolerance: [0.01, 0.01, 0.01]
  
  # Magnetometer calibration (hard/soft iron)
  mag_offset:  # μT
    x: 0.0
    y: 0.0
    z: 0.0
  
  mag_scale:  # dimensionless
    x: 1.0
    y: 1.0
    z: 1.0
```

#### 4.2.4 Servo Calibration

```yaml
servo_calibration:
  pan_servo:
    pwm_min: TBD  # microseconds for -90° or 0°
    pwm_center: TBD  # microseconds for 0° or 90°
    pwm_max: TBD  # microseconds for +90° or 180°
    angle_min: TBD  # degrees (mechanical limit)
    angle_max: TBD  # degrees (mechanical limit)
    angle_center: TBD  # degrees (level position)
    tolerance: 2.0  # degrees
  
  tilt_servo:
    pwm_min: TBD
    pwm_center: TBD
    pwm_max: TBD
    angle_min: TBD
    angle_max: TBD
    angle_center: TBD
    tolerance: 2.0
```

#### 4.2.5 Robot Base Calibration

```yaml
robot_base_calibration:
  # iRobot Create odometry calibration
  wheel_radius: 0.036  # meters (nominal: 36mm)
  wheel_separation: 0.235  # meters (nominal: 235mm)
  
  # Odometry error correction factors
  linear_scale_factor: 1.0  # correct for wheel wear/slip
  angular_scale_factor: 1.0  # correct for wheelbase variance
  
  tolerance:
    wheel_radius: 0.001  # ±1mm
    wheel_separation: 0.002  # ±2mm
```

### 4.3 Calibration Procedures

#### 4.3.1 Camera Calibration

**Method**: ROS 2 camera calibration using checkerboard or AprilTag target

**Procedure**:
1. Print calibration target (checkerboard: 8×6, square size TBD mm)
2. Launch camera nodes
3. Run `camera_calibration` ROS 2 node
4. Move target through camera FOV
5. Save calibration to YAML
6. Update `config/hardware/realsense_params.yaml`

**Reference**: [ROS 2 Camera Calibration](http://wiki.ros.org/camera_calibration)

#### 4.3.2 IMU Calibration

**Method**: Automated calibration via `calibration_manager_node`

**Procedure**:
1. Place robot on level surface
2. Launch calibration manager
3. Wait for accelerometer bias estimation (~30 seconds)
4. Rotate robot 360° slowly for magnetometer calibration
5. Calibration values automatically saved

**Reference**: See `src/modules/chassis_control/chassis_control/calibration_manager_node.py`

#### 4.3.3 Servo Calibration

**Method**: Manual testing and configuration

**Procedure**:
1. Start with conservative PWM range (1000-2000 μs)
2. Command servo to center position
3. Verify mechanical alignment (use level or protractor)
4. Test min/max positions, verify no binding
5. Record actual PWM values for desired angles
6. Update `config/hardware/phat_params.yaml`

**Safety**: See [SERVO_SAFETY.md](SERVO_SAFETY.md)

#### 4.3.4 Odometry Calibration

**Method**: Measured distance test

**Procedure**:
1. Mark starting position on ground
2. Command robot to drive straight 1 meter
3. Measure actual distance traveled
4. Calculate `linear_scale_factor = actual / commanded`
5. Command robot to rotate 360°
6. Measure actual rotation
7. Calculate `angular_scale_factor = actual / commanded`
8. Update calibration.yaml

---

## 5. Simulation Model

### 5.1 URDF/SDF Model

**Location**: `config/robot/isaac_robot.urdf` (to be created)

The URDF model describes the robot kinematic chain and is used for:
- RViz visualization
- TF tree generation
- Gazebo/Isaac Sim simulation

#### Model Requirements

- All links and joints defined
- Accurate mass and inertia values
- Collision and visual meshes
- Joint limits and dynamics

#### Calibration Integration

The simulation model should load calibrated values from `calibration.yaml`:
- Camera extrinsics → joint transforms
- IMU extrinsics → link transform
- Servo limits → joint limits

### 5.2 Isaac Sim Integration

**To be implemented**: Integration with NVIDIA Isaac Sim for physics simulation

#### Model Export

- Convert URDF to USD format
- Add physics properties (mass, friction, damping)
- Include sensor models (cameras, IMU)
- Define actuation (servo joints, wheels)

#### Calibrated Simulation

Simulation should use calibrated values:
- Camera positions from calibration.yaml
- IMU orientation from calibration.yaml
- Servo ranges from calibration.yaml
- Wheel parameters from calibration.yaml

**Workflow**:
1. Hardware calibration updates `calibration.yaml`
2. URDF generation script reads calibration.yaml
3. Generated URDF used for RViz and TF
4. USD export for Isaac Sim includes calibrated transforms

---

## 6. Verification & Testing

### 6.1 Mechanical Verification Checklist

- [ ] All fasteners tight and secure
- [ ] No loose cables or snagging hazards
- [ ] Camera mounts stable (no wobble)
- [ ] Servo motion clear of obstructions
- [ ] IMU level with base (verify with level)
- [ ] Cable strain relief adequate
- [ ] Thermal clearance for Jetson heatsink
- [ ] USB ports accessible

### 6.2 Calibration Verification

- [ ] Camera intrinsics calibrated (low reprojection error)
- [ ] Camera extrinsics measured or calibrated
- [ ] IMU accelerometer bias < 0.1 m/s²
- [ ] IMU gyroscope bias < 0.01 rad/s
- [ ] Servo PWM ranges tested and safe
- [ ] Odometry linear error < 5%
- [ ] Odometry angular error < 5%

### 6.3 Simulation Verification

- [ ] URDF loads without errors
- [ ] TF tree complete and correct
- [ ] RViz displays robot model correctly
- [ ] Joint limits match hardware
- [ ] Masses and inertias reasonable

---

## 7. Maintenance & Updates

### 7.1 Mechanical Maintenance

**Monthly**:
- Check all fasteners for tightness
- Inspect camera mounts for stability
- Verify servo operation (no binding, unusual sounds)

**Quarterly**:
- Re-run IMU calibration (check for drift)
- Verify odometry calibration (1m distance test)
- Check cable condition (wear, fraying)

**Annually**:
- Full camera calibration
- Disassemble and inspect all mechanical parts
- Replace worn components

### 7.2 Calibration Updates

When to recalibrate:
- After disassembly/reassembly
- After component replacement
- If sensor drift detected
- Before important missions/tests

### 7.3 Documentation Updates

Update this document when:
- New mechanical components added
- Design changes made
- Calibration procedure refined
- Tolerance requirements changed

---

## 8. Future Work

### 8.1 Immediate Tasks

- [ ] Measure and document all mechanical components in use
- [ ] Fill in TBD dimensions and part numbers
- [ ] Perform initial calibration and record values
- [ ] Create URDF model with calibrated values
- [ ] Take assembly photos for documentation

### 8.2 Medium-Term Improvements

- [ ] Create CAD models of custom brackets
- [ ] Design modular mounting system
- [ ] Improve cable management
- [ ] Add vibration damping for cameras
- [ ] Design calibration jig for repeatable assembly

### 8.3 Long-Term Goals

- [ ] Full Isaac Sim integration with calibrated model
- [ ] Automated calibration pipeline
- [ ] Digital twin with real-time calibration updates
- [ ] Manufacturing drawings for reproducibility

---

## References

- [BOM.md](BOM.md) - Complete parts list
- [HARDWARE.md](HARDWARE.md) - Electronics overview
- [SERVO_SAFETY.md](SERVO_SAFETY.md) - Servo safety guidelines
- [ROS REP 103](https://www.ros.org/reps/rep-0103.html) - Standard coordinate frames
- [ROS REP 105](https://www.ros.org/reps/rep-0105.html) - Coordinate frames for mobile platforms
- [Intel RealSense Datasheet](https://www.intelrealsense.com/depth-camera-d435/)
- [iRobot Create 2 Open Interface Spec](https://edu.irobot.com/learning-library/create-2-oi-spec)

---

**Next Steps**: Fill in TBD values by measuring actual hardware, then run calibration procedures to populate calibration.yaml.
