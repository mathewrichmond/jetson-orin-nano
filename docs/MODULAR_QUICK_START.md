# Modular Architecture - Quick Start

**Status**: Phases 1-5 Complete ✅ | Ready for Phase 6 (Integration Testing) or Hardware Deployment

---

## Build & Test

```bash
# Build all modules (Phases 1-4)
cd /home/nano/src/jetson-orin-nano
colcon build --packages-select custom_msgs chassis_control vision_pipeline power_management audio_pipeline vla_planner --symlink-install
source install/setup.bash

# Verify installation
ros2 pkg executables chassis_control vision_pipeline power_management audio_pipeline vla_planner
ros2 interface list | grep custom_msgs
```

---

## Launch Modules

```bash
# Individual modules
ros2 launch chassis_control chassis_control.launch.py
ros2 launch vision_pipeline vision_pipeline.launch.py
ros2 launch power_management power_management.launch.py mock_mode:=true
ros2 launch audio_pipeline audio_pipeline.launch.py
ros2 launch vla_planner vla_planner.launch.py

# Full system (modular graph - Phases 1-4)
ros2 launch isaac_robot graph.launch.py graph:=modular_graph.yaml group:=all

# Deployment modes
ros2 launch isaac_robot graph.launch.py graph:=modular_graph.yaml group:=roomba_plus    # Pi only
ros2 launch isaac_robot graph.launch.py graph:=modular_graph.yaml group:=vision_only    # Jetson only
ros2 launch isaac_robot graph.launch.py graph:=modular_graph.yaml group:=chassis_test  # Test
ros2 launch isaac_robot graph.launch.py graph:=modular_graph.yaml group:=audio_test    # Audio test
ros2 launch isaac_robot graph.launch.py graph:=modular_graph.yaml group:=vla_test      # VLA test
```

---

## Module Overview

### Chassis Control (Phase 1)
- **`imu_processor_node`** - Kalman filtering @ 50Hz
- **`chassis_controller_node`** - Wheel odometry + pose
- **`calibration_manager_node`** - Auto-calibration

**Topics**:
- Subscribes: `/hardware/phat/imu`, `/control/cmd_vel`, `/vision/global_pose`
- Publishes: `/rpi/imu/filtered`, `/rpi/chassis/odometry`, `/rpi/calibration/status`

### Vision Pipeline (Phase 2)
- **`visual_slam_node`** - RTAB-Map wrapper (structure ready)
- **`camera_calibration_node`** - Camera-IMU extrinsics
- **`vision_pipeline_node`** - Orchestrator + power mgmt

**Topics**:
- Subscribes: Camera RGB-D, IMU, battery
- Publishes: `/vision/global_pose`, `/jetson/health/vision_pipeline`, `/jetson/power/request`

### Audio Pipeline (Phase 4)
- **`audio_feature_extractor_node`** - MFCC, spectrograms, VAD
- **`speech_recognition_node`** - Speech-to-text (placeholder)
- **`audio_pipeline_node`** - Audio orchestrator

**Topics**:
- Subscribes: `/sensor_fusion/audio/raw`
- Publishes: `/audio/features/mfcc`, `/audio/transcription`, `/jetson/health/audio_pipeline`

### VLA Planner (Phase 4)
- **`vla_controller_node`** - VLA model inference (placeholder)
- **`action_executor_node`** - Action execution with safety
- **`planner_node`** - Task planning and coordination

**Topics**:
- Subscribes: Vision features, audio features, robot state, transcription
- Publishes: `/vla/actions`, `/control/cmd_vel`, `/vla/plan_status`

**Send commands**:
```bash
ros2 topic pub /vla/command std_msgs/msg/String "data: 'move forward 1 meter'" -1
```

### Power Management (Phase 3)
- **`power_manager_node`** - 6-mode FSM (AUTO/FULL/BALANCED/ECONOMY/CRITICAL/CHARGING)
- **`gpio_controller_node`** - Jetson GPIO control
- **`battery_monitor_node`** - Runtime estimation

**Topics**:
- Subscribes: `/irobot/battery`, temperatures, power requests
- Publishes: `/jetson/power/mode`, `/jetson/battery/alerts`, `/jetson/power/performance`

---

## Testing

```bash
# Smoke tests
cd src/modules/chassis_control/test && python3 test_node_imports.py
cd src/modules/vision_pipeline/test && python3 test_node_imports.py
cd src/modules/power_management/test && python3 test_node_imports.py
cd src/modules/audio_pipeline/test && python3 test_node_imports.py
cd src/modules/vla_planner/test && python3 test_node_imports.py

# Functional tests
cd src/modules/chassis_control/test && ./run_all_tests.sh
```

---

## Configuration

**Primary Config**: `config/robot/modular_graph.yaml`

**Calibration File**: `/home/nano/.config/robot/calibration.yaml` (auto-created)

**Groups**:
- `all` - Full system (Phases 1-4)
- `roomba_plus` - Raspberry Pi (chassis + power)
- `vision_only` - Jetson (vision + power)
- `chassis_test`, `vision_test`, `power_test`, `audio_test`, `vla_test` - Individual testing

---

## Documentation

- **Module Guide**: `src/modules/README.md`
- **Completion Reports**: `docs/PHASE1_COMPLETION.md`, `docs/PHASE2_COMPLETION.md`, `docs/PHASE4_COMPLETION.md`
- **Progress Report**: `docs/MODULAR_ARCHITECTURE_PROGRESS.md`
- **Refactor Summary**: `docs/REFACTOR_COMPLETION.md`

---

## Next Steps

### Option 1: Continue to Phase 6 (Integration Testing) ⭐ RECOMMENDED
```
Hardware integration testing
Performance benchmarking
Failure mode testing
Production readiness validation
```

### Option 2: Deploy to Dual-Compute (When Pi Ready)
```
# On both hosts
sudo ./scripts/network/setup_network.sh dual_compute

# Launch distributed system
./scripts/deployment/deploy.sh dual_compute launch
```

### Option 3: Integrate Real Models
```
Install Whisper for speech recognition
Install OpenVLA/RT-1 for VLA control
Test model inference performance
```

### Option 3: Hardware Validation
```
Test IMU processing on hardware
Test vision pipeline with cameras
Test power management with battery
```

### Option 4: Install RTAB-Map
```bash
sudo apt install ros-humble-rtabmap-ros
# Update visual_slam_node implementation
```

---

## Commit Changes

```bash
git add config/robot/modular_graph.yaml src/modules/ docs/
git commit -m "feat: Complete modular architecture refactor (Phases 1-4)"
git push origin main
```

---

**Status**: ✅ Phase 5 Complete | 5 of 6 Phases Done (83%)  
**Build**: ✅ All packages compile | ✅ Launch files installed  
**Deploy**: ✅ 5 deployment configs | ✅ Network setup with sudo
**Test**: ✅ Smoke tests pass | ⏳ Hardware tests pending  
**Next**: Phase 6 (Integration Testing) or Model Integration
