# Hardware Documentation

## Overview

This directory contains comprehensive hardware documentation for the Isaac robot system, organized for hardware verification, calibration, and simulation.

**Last Updated**: 2026-01-27

---

## Quick Reference

### Essential Documents

| Document | Purpose | When to Use |
|----------|---------|-------------|
| **[BOM.md](BOM.md)** | Complete parts list | Procurement, inventory, assembly |
| **[ELECTRICAL.md](ELECTRICAL.md)** | Electrical specs, wiring, power | Wiring, troubleshooting, power planning |
| **[MECHANICAL.md](MECHANICAL.md)** | Mechanical specs, tolerances, calibration | Assembly, calibration, simulation |
| **[HARDWARE.md](HARDWARE.md)** | System overview | Starting point, high-level architecture |

### Setup Guides

| Document | Purpose |
|----------|---------|
| [HARDWARE_SETUP.md](HARDWARE_SETUP.md) | Unified hardware installation |
| **[CHASSIS_MOUNTING.md](CHASSIS_MOUNTING.md)** | **Mounting servo brackets to iRobot Create chassis** |
| [realsense.md](realsense.md) | RealSense camera setup |
| [sparkfun_auto_phat.md](sparkfun_auto_phat.md) | SparkFun Auto pHAT setup |
| [irobot.md](irobot.md) | iRobot Create setup |
| [usb_microphone.md](usb_microphone.md) | USB microphone setup |

### Advanced Topics

| Document | Purpose |
|----------|---------|
| [MOBILE_POWER_DESIGN.md](MOBILE_POWER_DESIGN.md) | Mobile power system design (battery-powered operation) |
| [POWER_MODULE_DESIGN.md](POWER_MODULE_DESIGN.md) | Custom power module PCB design (DC-DC converters) |
| [CAMERA_FRAME_SYNC.md](CAMERA_FRAME_SYNC.md) | Hardware camera synchronization |
| [SERVO_SAFETY.md](SERVO_SAFETY.md) | Servo safety guidelines |
| [BENCH_SETUP.md](BENCH_SETUP.md) | Bench testing configuration |

---

## Documentation Organization

### 1. Bill of Materials (BOM)

**File**: [BOM.md](BOM.md)

**Contains**:
- Complete list of all hardware components
- Part numbers and specifications
- Electrical and mechanical components
- Tools and equipment needed
- Cost estimates
- Procurement recommendations

**Update When**:
- Adding new hardware components
- Changing component specifications
- Updating part numbers or vendors

### 2. Electrical Documentation

**File**: [ELECTRICAL.md](ELECTRICAL.md)

**Contains**:
- System electrical architecture
- Component electrical specifications
- Wiring diagrams and pinouts
- Power budget and distribution
- Communication interfaces (USB, I2C, Serial)
- Troubleshooting guides

**Related**: [MOBILE_POWER_DESIGN.md](MOBILE_POWER_DESIGN.md) for battery-powered mobile operation

**Update When**:
- Changing wiring or connections
- Adding new electrical components
- Modifying power distribution
- Updating interface configurations

### 3. Mechanical Documentation

**File**: [MECHANICAL.md](MECHANICAL.md)

**Contains**:
- Mechanical design and dimensions
- Coordinate systems and frames
- Mounting specifications
- Manufacturing tolerances
- Calibration values and procedures
- Simulation model integration

**Update When**:
- Changing mechanical assembly
- Running calibration procedures
- Updating tolerance specifications
- Creating simulation models

---

## Hardware Workflow

### Phase 1: Planning & Procurement

1. **Review BOM**: [BOM.md](BOM.md)
   - Identify required components
   - Check for missing specifications (TBD items)
   - Plan procurement

2. **Review Electrical**: [ELECTRICAL.md](ELECTRICAL.md)
   - Verify power requirements
   - Plan wiring and connections
   - Identify tools needed

3. **Review Mechanical**: [MECHANICAL.md](MECHANICAL.md)
   - Understand assembly requirements
   - Check mechanical constraints
   - Plan mounting strategy

### Phase 2: Assembly

1. **Follow Setup Guides**:
   - [HARDWARE_SETUP.md](HARDWARE_SETUP.md) - Overall setup
   - Component-specific guides (realsense.md, etc.)

2. **Refer to Electrical**:
   - [ELECTRICAL.md](ELECTRICAL.md) - Wiring diagrams
   - Verify connections match documentation

3. **Refer to Mechanical**:
   - [MECHANICAL.md](MECHANICAL.md) - Mounting specs
   - Check alignment and tolerances

### Phase 3: Verification

1. **Run Verification Scripts**:
   ```bash
   cd ~/src/jetson-orin-nano
   ./scripts/hardware/verify_all_hardware.sh
   ```

2. **Manual Checks**:
   - Electrical: Use [ELECTRICAL.md § Testing](ELECTRICAL.md#9-testing--verification)
   - Mechanical: Use [MECHANICAL.md § Verification](MECHANICAL.md#6-verification--testing)

3. **Update Documentation**:
   - Fill in TBD values with actual measurements
   - Update BOM with actual part numbers
   - Document any deviations from nominal design

### Phase 4: Calibration

1. **Camera Calibration**:
   - Follow procedure in [MECHANICAL.md § 4.3.1](MECHANICAL.md#431-camera-calibration)
   - Results saved to `/home/nano/.config/robot/calibration.yaml`

2. **IMU Calibration**:
   - Follow procedure in [MECHANICAL.md § 4.3.2](MECHANICAL.md#432-imu-calibration)
   - Automated via `calibration_manager_node`

3. **Servo Calibration**:
   - Follow procedure in [MECHANICAL.md § 4.3.3](MECHANICAL.md#433-servo-calibration)
   - Safety guidelines: [SERVO_SAFETY.md](SERVO_SAFETY.md)

4. **Odometry Calibration**:
   - Follow procedure in [MECHANICAL.md § 4.3.4](MECHANICAL.md#434-odometry-calibration)
   - Distance and rotation tests

### Phase 5: Simulation

1. **Create URDF Model**:
   - Use calibrated values from `calibration.yaml`
   - See [MECHANICAL.md § 5.1](MECHANICAL.md#51-urdfsdf-model)

2. **Export to Isaac Sim**:
   - Convert URDF to USD format
   - Include calibrated transforms
   - See [MECHANICAL.md § 5.2](MECHANICAL.md#52-isaac-sim-integration)

3. **Verify Simulation**:
   - Compare simulation to real robot
   - Verify transforms match calibration
   - Test sensor models

---

## Calibration Files

### Production Calibration

**Location**: `/home/nano/.config/robot/calibration.yaml`

**Created By**: Calibration nodes (automated)

**Contains**:
- Camera extrinsics and intrinsics
- IMU calibration (biases, alignment)
- Servo PWM-to-angle mapping
- Odometry correction factors

### Calibration Template

**Location**: `config/robot/calibration.yaml.example`

**Purpose**: Template with nominal values and tolerances

**Usage**:
```bash
# Copy template to production location
mkdir -p /home/nano/.config/robot
cp config/robot/calibration.yaml.example /home/nano/.config/robot/calibration.yaml
```

---

## Hardware States

### 1. Documented (Current State)

**Status**: Documentation exists, but contains TBD values

**Next Step**: Fill in TBD values with actual measurements

**Checklist**:
- [ ] BOM: Fill in TBD part numbers and specifications
- [ ] ELECTRICAL: Measure actual voltages and currents
- [ ] MECHANICAL: Measure actual dimensions and positions

### 2. Assembled

**Status**: Physical hardware assembled and connected

**Next Step**: Run verification tests

**Checklist**:
- [ ] All components physically mounted
- [ ] All electrical connections made
- [ ] Fasteners tightened
- [ ] Cables secured and strain-relieved

### 3. Verified

**Status**: Hardware passes electrical and mechanical tests

**Next Step**: Run calibration procedures

**Checklist**:
- [ ] Electrical verification passed
- [ ] Mechanical verification passed
- [ ] All devices detected by software
- [ ] No obvious issues (loose connections, binding, etc.)

### 4. Calibrated

**Status**: Calibration complete, values saved

**Next Step**: Create simulation model

**Checklist**:
- [ ] Camera calibration complete
- [ ] IMU calibration complete
- [ ] Servo calibration complete
- [ ] Odometry calibration complete
- [ ] `calibration.yaml` file populated

### 5. Simulated

**Status**: Simulation model created with calibrated values

**Next Step**: Use for development and testing

**Checklist**:
- [ ] URDF model created
- [ ] Calibrated values integrated
- [ ] Simulation verified against real hardware
- [ ] Isaac Sim integration (if applicable)

---

## Maintenance

### Regular Checks

**Monthly**:
- [ ] Verify all fasteners tight
- [ ] Inspect cables for wear
- [ ] Check servo operation
- [ ] Verify camera stability

**Quarterly**:
- [ ] Re-run IMU calibration
- [ ] Verify odometry calibration (1m test)
- [ ] Check mechanical tolerances
- [ ] Update documentation with any changes

**Annually**:
- [ ] Full camera calibration
- [ ] Disassemble and inspect
- [ ] Replace worn components
- [ ] Update all documentation

### When to Recalibrate

- After disassembly/reassembly
- After component replacement
- If sensor drift detected (IMU, odometry)
- Before important missions or demos
- If simulation no longer matches real robot

---

## Common Tasks

### Adding a New Component

1. **Update BOM**: Add component to [BOM.md](BOM.md)
2. **Update Electrical**: Add to [ELECTRICAL.md](ELECTRICAL.md) (if electronic)
3. **Update Mechanical**: Add to [MECHANICAL.md](MECHANICAL.md) (if mechanical)
4. **Create Setup Guide**: Document installation procedure
5. **Update Verification**: Add checks to verification script
6. **Update HARDWARE.md**: Add to system overview

### Changing Wiring

1. **Plan Change**: Review [ELECTRICAL.md](ELECTRICAL.md)
2. **Update Diagram**: Modify wiring diagrams
3. **Document**: Update connection tables
4. **Verify**: Run electrical verification tests
5. **Update BOM**: If cables/connectors changed

### Running Calibration

1. **Prepare Robot**: Follow [MECHANICAL.md § 4.3](MECHANICAL.md#43-calibration-procedures)
2. **Run Calibration Nodes**: Use ROS 2 calibration tools
3. **Verify Results**: Check `calibration.yaml` for reasonable values
4. **Update URDF**: Regenerate robot model with calibrated values
5. **Test**: Verify improved accuracy

---

## Tools & Scripts

### Verification Scripts

**Location**: `scripts/hardware/`

```bash
# Comprehensive hardware verification
./scripts/hardware/verify_all_hardware.sh

# Component-specific diagnostics
./scripts/hardware/diagnose_realsense.sh
./scripts/hardware/diagnose_phat.sh
```

### Calibration Tools

**ROS 2 Calibration**:
```bash
# Camera calibration (manual)
ros2 run camera_calibration cameracalibrator

# IMU calibration (automated)
ros2 launch chassis_control calibration_manager.launch.py
```

### Monitoring

```bash
# System monitor (includes hardware health)
ros2 topic echo /system/health

# Individual component status
ros2 topic echo /realsense/status
ros2 topic echo /phat/status
ros2 topic echo /irobot/status
```

---

## Troubleshooting

### Documentation Issues

**Problem**: TBD values prevent assembly
- **Solution**: Measure actual hardware, update docs

**Problem**: Specifications don't match actual hardware
- **Solution**: Update docs to match reality, note deviations

**Problem**: Missing part numbers
- **Solution**: Research and add to BOM

### Hardware Issues

See component-specific troubleshooting:
- Electrical: [ELECTRICAL.md § 8](ELECTRICAL.md#8-troubleshooting)
- Cameras: [realsense.md § Troubleshooting](realsense.md)
- SparkFun pHAT: [sparkfun_auto_phat.md § Troubleshooting](sparkfun_auto_phat.md)
- iRobot: [irobot.md § Troubleshooting](irobot.md)

---

## Contributing

### Documentation Standards

1. **Keep docs updated**: Update when hardware changes
2. **Fill in TBDs**: Replace TBD with actual values as soon as known
3. **Add measurements**: Document all physical measurements
4. **Include photos**: Reference photos in `docs/hardware/images/` (if created)
5. **Version control**: Track changes in git commit messages

### Documentation Updates

When updating hardware docs:
1. Update the specific file (BOM, ELECTRICAL, MECHANICAL)
2. Update this README if structure changes
3. Update HARDWARE.md if system overview changes
4. Update Last Updated date
5. Commit with descriptive message

---

## References

### Internal Documentation

- [HARDWARE.md](HARDWARE.md) - System overview
- [BOM.md](BOM.md) - Bill of materials
- [ELECTRICAL.md](ELECTRICAL.md) - Electrical specifications
- [MECHANICAL.md](MECHANICAL.md) - Mechanical specifications

### External Resources

- [ROS REP 103](https://www.ros.org/reps/rep-0103.html) - Standard coordinate frames
- [ROS REP 105](https://www.ros.org/reps/rep-0105.html) - Mobile platform coordinate frames
- [ROS 2 Camera Calibration](http://wiki.ros.org/camera_calibration)
- [Jetson Orin Nano User Guide](https://developer.nvidia.com/embedded/learn/jetson-orin-nano-devkit-user-guide)
- [Intel RealSense Documentation](https://www.intelrealsense.com/)

---

## Next Steps

1. **Immediate**:
   - [ ] Measure and document mechanical components in use
   - [ ] Fill in TBD values in BOM
   - [ ] Take reference photos of assembly
   - [ ] Run hardware verification script

2. **Short-Term**:
   - [ ] Complete all calibration procedures
   - [ ] Populate `calibration.yaml` with real values
   - [ ] Create URDF model with calibrated values
   - [ ] Verify simulation matches real robot

3. **Medium-Term**:
   - [ ] Create CAD models of custom brackets
   - [ ] Design improved mounting system
   - [ ] Add vibration damping for cameras
   - [ ] Create calibration jig for repeatability

4. **Long-Term**:
   - [ ] Full Isaac Sim integration
   - [ ] Automated calibration pipeline
   - [ ] Digital twin with real-time updates
   - [ ] Manufacturing drawings for reproducibility

---

**Questions?** See [HARDWARE.md](HARDWARE.md) for system overview or component-specific guides for detailed information.
