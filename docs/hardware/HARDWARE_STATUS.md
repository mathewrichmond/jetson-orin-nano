# Hardware Documentation Status

**Last Updated**: 2026-01-27

This document tracks the completion status of hardware documentation for verification, calibration, and simulation.

---

## Current Assembly Status (2026-01-27)

### ✅ Assembled and Connected
- Servos connected to SparkFun Auto pHAT (PCA9685)
- Pan/tilt brackets assembled with MG996R servos
- Electronics powered and functional
- Ready for chassis mounting

### 🔧 Current Task: Chassis Mounting
**Issue**: Securing pan/tilt servo base to iRobot Create chassis
- Currently: Brackets on stand (temporary)
- Need: Permanent mounting solution to chassis
- Once mounted: Can attach cameras and IMU for calibration

### Next Immediate Steps
1. **Mount servo brackets to chassis** → See **[CHASSIS_MOUNTING.md](CHASSIS_MOUNTING.md)**
2. Attach cameras to pan/tilt brackets
3. Secure IMU (already on pHAT, verify orientation is level with robot)
4. Route and secure cables (camera sync, USB, servo power)
5. Run calibration procedures: `./scripts/hardware/calibrate_hardware.sh`

### Chassis Mounting Options

See **[CHASSIS_MOUNTING.md](CHASSIS_MOUNTING.md)** for detailed guidance on:
- iRobot Create mounting points
- Mounting plate design and materials
- Quick mounting solutions (L-brackets, flat bar, 3D printed)
- Professional solutions (laser-cut acrylic/aluminum)
- Assembly procedures
- Cable management
- Verification checklist

---

## Documentation Complete ✅

### Reference Images

Hardware component images stored in `docs/hardware/images/`:
- ✅ `mg996r_specs.png` - MG996R servo specifications diagram
- ✅ `mg996r_dimensions.png` - Detailed dimensional drawing
- ✅ `pan_tilt_bracket_assembly.png` - Bracket assembly stages
- ✅ `pan_tilt_bracket_parts.png` - Complete parts breakdown

### Bill of Materials (BOM.md)

**Electronics - Fully Documented**:
- ✅ Jetson Orin Nano (main computer)
- ✅ 2× Intel RealSense cameras (D435/D455)
- ✅ SparkFun Auto pHAT (servo controller + IMU)
- ✅ 4× MG996R servo motors (with full specs)
- ✅ RSHTECH 7-port powered USB hub
- ✅ USB microphone
- ✅ iRobot Create 2
- ✅ IDC GPIO ribbon cable (40-pin, 8")
- ✅ Custom ethernet harness for camera sync
- ✅ 2× RJ45 breakout boards
- ✅ JST SH 1.0mm connector kit
- ✅ 2 sets RC servo mount brackets (aluminum)
- ✅ 2 sets pan/tilt servo mounts

**Specifications Added**:
- ✅ MG996R servo specifications:
  - Operating voltage: 4.8-7.2V (optimal 6V)
  - Stall torque: 11 kg·cm @ 6V
  - Speed: 0.14 sec/60° @ 6V
  - Stall current: 2.5A per servo
  - Weight: 55g
- ✅ USB hub specifications:
  - 3× USB 3.2 (10 Gbps)
  - 4× USB 3.0 (5 Gbps)
  - Powered with dedicated 5V adapter
- ✅ Power budget with dual supply system
- ✅ Cost estimates updated

### Electrical Documentation (ELECTRICAL.md)

**Fully Documented**:
- ✅ System block diagram with USB hub
- ✅ Power tree (dual supply: 5V main + 6V servo)
- ✅ MG996R servo electrical specs
- ✅ USB hub configuration and port assignments
- ✅ Custom ethernet harness for camera sync (with RJ45 breakouts)
- ✅ Power budget:
  - Main system: ~25W @ 5V
  - Servo system: ~60W @ 6V (peak)
- ✅ Power supply recommendations (5V/5A + 6V/10A)
- ✅ I2C bus assignments (Bus 7 on Jetson)
- ✅ GPIO pinout for Auto pHAT

### Mechanical Documentation (MECHANICAL.md)

**Servo Specifications - Documented**:
- ✅ MG996R physical dimensions
- ✅ MG996R performance specifications
- ✅ PWM specifications (1000-2000μs)
- ✅ Servo mounting bracket specifications
- ✅ Pan/tilt mount details

**Calibration Framework - Complete**:
- ✅ Camera extrinsics (translation + rotation)
- ✅ Camera intrinsics (fx, fy, cx, cy, distortion)
- ✅ IMU calibration (bias, alignment)
- ✅ Servo calibration (4 servos, PWM-to-angle mapping)
- ✅ Odometry calibration (wheel parameters, scale factors)
- ✅ Tolerances specified for all parameters
- ✅ Calibration procedures documented

### Configuration Files

**Complete**:
- ✅ `calibration.yaml.example` - Template with all parameters and tolerances
  - Updated with 4× MG996R servo configuration
  - Includes safe angle limits to prevent collisions
  - Ready to copy to `/home/nano/.config/robot/calibration.yaml`

### Scripts

**Complete**:
- ✅ `scripts/hardware/calibrate_hardware.sh` - Guided calibration script
  - Camera calibration guidance
  - IMU calibration (automated)
  - Servo calibration (manual with safety checks)
  - Odometry calibration (distance tests)

---

## Still To Be Documented ⏳

### Mechanical Components (in BOM.md and MECHANICAL.md)

**Need to measure and document**:
- ⏳ Main mounting plate/deck
  - Material (acrylic, aluminum, 3D printed?)
  - Dimensions (L × W × H)
  - Mounting hole pattern
  - Weight
  
- ⏳ Jetson mounting bracket/standoffs
  - How is Jetson secured to structure?
  - Standoff height
  - Fastener type and count
  
- ⏳ Fasteners inventory
  - Count M3 screws (and lengths used)
  - Count M2.5 screws (and lengths used)
  - Count standoffs (and heights used)
  - Count washers

- ⏳ Camera mounting positions
  - Measure actual camera position relative to robot base
  - Document pan/tilt servo pivot points
  - Measure camera-to-servo-pivot offset
  - Update `MECHANICAL.md` § 2.1 with actual values

- ⏳ Assembly photos
  - Take reference photos of assembly
  - Document cable routing
  - Show servo mounting configuration

### Calibration Values (in calibration.yaml)

**Need to run calibration procedures to fill in**:
- ⏳ Camera extrinsics (actual X, Y, Z positions)
- ⏳ Camera intrinsics (run ROS 2 camera_calibration)
- ⏳ IMU biases (run `calibration_manager_node`)
- ⏳ Servo PWM-to-angle mapping (test each servo)
- ⏳ Servo safe angle limits (verify mechanical range)
- ⏳ Odometry scale factors (1m distance test)

### Simulation Model

**Need to create**:
- ⏳ URDF model (robot_description package)
  - Define all links and joints
  - Use calibrated values from `calibration.yaml`
  - Include collision and visual meshes
  
- ⏳ Isaac Sim integration
  - Convert URDF to USD format
  - Test simulation matches real robot

---

## Next Steps

### Immediate (This Week)

1. **Measure mechanical components**:
   ```bash
   # Measure and document:
   # - Main mounting plate dimensions
   # - Jetson mounting method
   # - Camera positions (X, Y, Z from base center)
   # - Count fasteners used
   ```

2. **Take photos**:
   ```bash
   # Take photos of:
   # - Overall assembly (front, back, side views)
   # - Camera mounting with servos
   # - Jetson + Auto pHAT mounting
   # - Cable routing (especially USB hub connections)
   # - Custom ethernet sync harness
   ```

3. **Update BOM and MECHANICAL.md** with actual measurements

### Short-Term (Next 2 Weeks)

4. **Run calibration script**:
   ```bash
   cd ~/src/jetson-orin-nano
   ./scripts/hardware/calibrate_hardware.sh
   ```
   This will:
   - Guide through camera calibration
   - Run IMU calibration automatically
   - Test servo safe limits
   - Run odometry calibration

5. **Verify calibration values**:
   ```bash
   # Review calibrated values
   cat /home/nano/.config/robot/calibration.yaml
   
   # Test with robot
   ros2 launch isaac_robot robot.launch.py
   ```

### Medium-Term (Next Month)

6. **Create URDF model**:
   - Use measured dimensions
   - Load calibrated transforms from `calibration.yaml`
   - Test in RViz

7. **Test simulation**:
   - Verify TF tree matches real robot
   - Test sensor models
   - Compare simulation to reality

---

## Documentation Files Summary

| File | Purpose | Status |
|------|---------|--------|
| `BOM.md` | Complete parts list | ✅ Electronics complete, ⏳ mechanical TBD |
| `ELECTRICAL.md` | Wiring, power, electrical specs | ✅ Complete |
| `MECHANICAL.md` | Dimensions, tolerances, calibration | ✅ Framework complete, ⏳ need actual values |
| `calibration.yaml.example` | Calibration template | ✅ Complete |
| `HARDWARE.md` | System overview | ✅ Updated with new docs |
| `README.md` | Hardware doc hub | ✅ Complete workflow guide |
| `calibrate_hardware.sh` | Calibration script | ✅ Complete |

---

## Questions to Answer

As you fill in the documentation, answer these:

### Mechanical

1. **Mounting plate**: What material and dimensions are you using for the main structure?
2. **Jetson mounting**: How is the Jetson secured? (standoffs, bracket, direct mount?)
3. **Camera position**: What is the actual X, Y, Z position of each camera?
4. **Servo range**: What are the safe angle limits for pan/tilt (to avoid collisions)?

### Assembly

5. **Cable management**: How are cables routed and secured?
6. **Servo power**: Are you using the separate 6V supply via Auto pHAT USB-C?
7. **USB hub**: Is the hub mounted to the robot, or external?

### Calibration

8. **IMU orientation**: Is the Auto pHAT oriented flat with the robot base?
9. **Camera intrinsics**: Have you run ROS 2 camera calibration?
10. **Odometry**: Does the iRobot track straight for 1 meter? (test for scale factor)

---

## Success Criteria

You'll know documentation is complete when:

- ✅ **BOM**: All TBD values filled in with actual part numbers/dimensions
- ✅ **MECHANICAL.md**: All TBD values replaced with measurements
- ✅ **calibration.yaml**: All values populated from calibration procedures
- ✅ **Photos**: Reference photos taken and stored
- ✅ **URDF**: Robot model created and tested in RViz
- ✅ **Verification**: All checklists in `MECHANICAL.md` § 6 completed

At that point, you'll have:
1. ✅ Hardware fully documented for reproducibility
2. ✅ Calibrated model for accurate control
3. ✅ Simulation model for testing and development

---

**Ready to start?** Begin with measuring and documenting the mechanical components you're using!
