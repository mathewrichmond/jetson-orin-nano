# Modular Robot Architecture

This directory contains the modular components of the Isaac robot system, organized as independent ROS 2 packages that can be deployed flexibly across different compute platforms.

## Architecture Overview

The modular architecture separates the robot system into:
- **Custom Messages** - Shared message definitions
- **Chassis Control** - IMU processing, wheel odometry, calibration
- **Vision Pipeline** - Visual SLAM, camera calibration, 3D perception
- **Power Management** - Battery monitoring, GPU control, thermal management
- **Audio Pipeline** - Audio features, speech recognition, audio orchestration
- **VLA Planner** - VLA model integration, action execution, task planning

### Design Principles

1. **Independence**: Each module can be built, tested, and deployed separately
2. **Cross-Platform**: Modules can run on Jetson Orin Nano, Raspberry Pi, or other platforms
3. **Health Monitoring**: All nodes publish health status using `ModuleHealth` messages
4. **Power Awareness**: Vision and compute-intensive modules respect power modes
5. **Calibration**: Auto-calibration with persistent storage

## Packages

### 1. `custom_msgs` - Shared Message Definitions

Custom ROS 2 messages used across all modules.

**Messages:**
- `CalibrationStatus` - Sensor/actuator calibration parameters
- `PowerRequest` - GPU power control requests
- `SystemPerformance` - CPU/GPU/battery/pipeline metrics
- `ModuleHealth` - Module-level health aggregation
- `NodeHealth` - Per-node health status
- `WatchdogStatus` - Input topic monitoring

**Build:**
```bash
colcon build --packages-select custom_msgs --symlink-install
```

**Verify:**
```bash
ros2 interface list | grep custom_msgs
```

---

### 2. `chassis_control` - Chassis Control Module (Phase 1)

Three nodes for mobile robot chassis control with sensor fusion.

#### Nodes

**`imu_processor_node`** - Kalman filtering and dead reckoning
- Subscribes: `/hardware/phat/imu` (raw IMU @ 50Hz)
- Publishes: `/rpi/imu/filtered` (Kalman-filtered IMU)
- Features: Bias compensation, health monitoring

**`chassis_controller_node`** - Wheel odometry and pose integration
- Subscribes: `/control/cmd_vel`, `/rpi/imu/filtered`, `/vision/global_pose`
- Publishes: `/rpi/chassis/odometry`, `/rpi/chassis/pose_estimate`
- Features: Dead reckoning, vision drift correction

**`calibration_manager_node`** - Auto-calibration system
- Subscribes: `/vision/global_pose`, `/rpi/chassis/odometry`
- Publishes: `/rpi/calibration/status`
- Features: Drift estimation, parameter updates, persistent storage

#### Usage

**Launch all chassis control nodes:**
```bash
ros2 launch chassis_control chassis_control.launch.py
```

**Launch individual nodes:**
```bash
ros2 run chassis_control imu_processor_node
ros2 run chassis_control chassis_controller_node
ros2 run chassis_control calibration_manager_node
```

**Calibration file location:**
```
/home/nano/.config/robot/calibration.yaml
```

**Documentation:** [README.md](chassis_control/README.md)

---

### 3. `vision_pipeline` - Vision Pipeline Module (Phase 2)

Three nodes for visual SLAM, camera calibration, and pipeline orchestration.

#### Nodes

**`visual_slam_node`** - RTAB-Map visual SLAM wrapper
- Subscribes: Camera RGB-D, IMU
- Publishes: `/vision/global_pose`, `/vision/odometry`, `/vision/trajectory`
- Features: Visual odometry, loop closure, global pose estimation

**`camera_calibration_node`** - Online camera-IMU extrinsics
- Subscribes: IMU, camera info, visual odometry
- Publishes: `/rpi/calibration/status` (with camera transforms)
- Features: Visual-inertial alignment, quality assessment

**`vision_pipeline_node`** - Vision system orchestrator
- Subscribes: Camera health, battery, temperature
- Publishes: `/jetson/health/vision_pipeline`, `/jetson/power/request`
- Features: Mode switching (full/tracking/sleep), power management

#### Usage

**Launch all vision pipeline nodes:**
```bash
ros2 launch vision_pipeline vision_pipeline.launch.py
```

**Launch with RTAB-Map enabled (requires installation):**
```bash
ros2 launch vision_pipeline vision_pipeline.launch.py rtabmap_enabled:=true
```

**Launch individual nodes:**
```bash
ros2 run vision_pipeline visual_slam_node
ros2 run vision_pipeline camera_calibration_node
ros2 run vision_pipeline vision_pipeline_node
```

**Install RTAB-Map (optional):**
```bash
sudo apt install ros-humble-rtabmap-ros
```

**Documentation:** [README.md](vision_pipeline/README.md)

---

### 4. `audio_pipeline` - Audio Pipeline Module (Phase 4)

Three nodes for audio processing and speech recognition.

#### Nodes

**`audio_feature_extractor_node`** - Audio feature extraction for VLA models
- Subscribes: `/sensor_fusion/audio/raw`
- Publishes: `/audio/features/mfcc`, `/audio/features/spectrogram`, `/audio/features/vad`
- Features: MFCC extraction, spectrograms, voice activity detection

**`speech_recognition_node`** - Speech-to-text conversion
- Subscribes: `/sensor_fusion/audio/raw`
- Publishes: `/audio/transcription`, `/audio/transcription_confidence`
- Features: Placeholder for Whisper/DeepSpeech, continuous recognition, VAD triggering

**`audio_pipeline_node`** - Audio system orchestrator
- Subscribes: Battery, temperature
- Publishes: `/jetson/health/audio_pipeline`, `/jetson/power/request`
- Features: Mode switching (full/reduced/sleep), power management

#### Usage

**Launch all audio pipeline nodes:**
```bash
ros2 launch audio_pipeline audio_pipeline.launch.py
```

**Launch individual nodes:**
```bash
ros2 run audio_pipeline audio_feature_extractor_node
ros2 run audio_pipeline speech_recognition_node
ros2 run audio_pipeline audio_pipeline_node
```

**Documentation:** [README.md](audio_pipeline/README.md)

---

### 5. `vla_planner` - VLA Planner Module (Phase 4)

Three nodes for VLA model integration and robot control.

#### Nodes

**`vla_controller_node`** - VLA model inference
- Subscribes: Vision features, audio features, robot state, transcription
- Publishes: `/vla/actions`, `/control/cmd_vel`
- Features: Multimodal input processing, model inference, action generation

**`action_executor_node`** - Action execution with safety
- Subscribes: `/vla/actions`
- Publishes: `/control/cmd_vel`, `/vla/execution_feedback`
- Features: Safety checks, action sequencing, execution monitoring

**`planner_node`** - High-level task planning
- Subscribes: `/vla/command`, transcription, execution feedback
- Publishes: `/vla/plan_status`, `/jetson/health/vla_planner`
- Features: Task queue, priority scheduling, retry logic

#### Usage

**Launch all VLA planner nodes:**
```bash
ros2 launch vla_planner vla_planner.launch.py
```

**Launch individual nodes:**
```bash
ros2 run vla_planner vla_controller_node
ros2 run vla_planner action_executor_node
ros2 run vla_planner planner_node
```

**Send commands:**
```bash
ros2 topic pub /vla/command std_msgs/msg/String "data: 'move forward'" -1
```

**Documentation:** [README.md](vla_planner/README.md)

---

### 6. `power_management` - Power Management Module (Phase 3)

Three nodes for system-wide power control, battery monitoring, and thermal management.

#### Nodes

**`power_manager_node`** - Power state machine
- Subscribes: Battery, temperatures, power requests
- Publishes: `/jetson/power/mode`, `/jetson/power/performance`
- Features: 6 power modes, thermal throttling, request arbitration

**Power Modes:**
| Mode | GPU | Battery | Use Case |
|------|-----|---------|----------|
| FULL | On | >45% | All features active |
| BALANCED | On | >30% | Normal operation |
| ECONOMY | Sleep | 15-30% | Power saving |
| CRITICAL | Off | <15% | Minimal, seek charger |
| CHARGING | Sleep | Any | Docked and charging |
| MANUAL | User | Any | User-controlled |

**`gpio_controller_node`** - Jetson GPIO power control
- Subscribes: `/jetson/power/gpu_control`
- Publishes: `/jetson/gpio/status`
- Features: Safe power sequencing, mock mode for testing

**`battery_monitor_node`** - Battery health and runtime estimation
- Subscribes: `/irobot/battery`
- Publishes: `/jetson/battery/alerts`, `/jetson/battery/runtime_estimate`
- Features: Runtime estimation, power trend tracking, health assessment

#### Usage

**Launch all power management nodes:**
```bash
ros2 launch power_management power_management.launch.py
```

**Launch with mock GPIO (testing without hardware):**
```bash
ros2 launch power_management power_management.launch.py mock_mode:=true
```

**Launch individual nodes:**
```bash
ros2 run power_management power_manager_node
ros2 run power_management gpio_controller_node --ros-args -p mock_mode:=true
ros2 run power_management battery_monitor_node
```

---

## Building

### Build all modules:
```bash
cd /home/nano/src/jetson-orin-nano

# Build dependencies first
colcon build --packages-select custom_msgs --symlink-install

# Build all modules (Phases 1-4)
colcon build --packages-select chassis_control vision_pipeline power_management audio_pipeline vla_planner --symlink-install

# Source the workspace
source install/setup.bash
```

### Build individual modules:
```bash
colcon build --packages-select chassis_control --symlink-install
colcon build --packages-select vision_pipeline --symlink-install
colcon build --packages-select power_management --symlink-install
colcon build --packages-select audio_pipeline --symlink-install
colcon build --packages-select vla_planner --symlink-install
```

### Verify installation:
```bash
# Check packages
ros2 pkg list | grep -E "chassis_control|vision_pipeline|power_management|audio_pipeline|vla_planner"

# Check executables
ros2 pkg executables chassis_control
ros2 pkg executables vision_pipeline
ros2 pkg executables power_management
ros2 pkg executables audio_pipeline
ros2 pkg executables vla_planner

# Check messages
ros2 interface list | grep custom_msgs
```

---

## Testing

### Smoke Tests

Each module includes a smoke test to verify imports and structure:

```bash
# Chassis control
cd src/modules/chassis_control/test
python3 test_node_imports.py

# Vision pipeline
cd src/modules/vision_pipeline/test
python3 test_node_imports.py

# Power management
cd src/modules/power_management/test
python3 test_node_imports.py

# Audio pipeline
cd src/modules/audio_pipeline/test
python3 test_node_imports.py

# VLA planner
cd src/modules/vla_planner/test
python3 test_node_imports.py
```

### Functional Tests

Chassis control includes functional tests (requires ROS 2 environment):

```bash
cd src/modules/chassis_control/test
./run_all_tests.sh
```

---

## Graph Configurations

The modular architecture supports multiple deployment configurations via graph files.

### Available Configurations

**`modular_graph.yaml`** - Complete modular system (recommended)
- All modules: chassis, vision, power management
- Full hardware drivers integration
- Multiple deployment groups defined

**Launch with graph:**
```bash
ros2 launch isaac_robot graph.launch.py graph:=modular_graph.yaml group:=all
```

### Deployment Groups

**Full System (`all`)** - All modules active
```bash
ros2 launch isaac_robot graph.launch.py graph:=modular_graph.yaml group:=all
```

**Roomba+ Mode (`roomba_plus`)** - Raspberry Pi only
- Chassis control + power management
- No vision (Jetson inactive)
```bash
ros2 launch isaac_robot graph.launch.py graph:=modular_graph.yaml group:=roomba_plus
```

**Vision Only (`vision_only`)** - Jetson only
- Vision pipeline + power management
- No chassis (Raspberry Pi inactive)
```bash
ros2 launch isaac_robot graph.launch.py graph:=modular_graph.yaml group:=vision_only
```

**Testing Groups:**
- `chassis_test` - Chassis control only
- `vision_test` - Vision pipeline only
- `power_test` - Power management only
- `minimal_test` - System monitor + IMU processor only

---

## Development Status

### Completed (Phases 1-4) ✅

- ✅ **Phase 1: Chassis Control** - IMU processing, odometry, calibration
- ✅ **Phase 2: Vision Pipeline** - SLAM structure, camera calibration, orchestration
- ✅ **Phase 3: Power Management** - Power modes, GPIO control, battery monitoring
- ✅ **Phase 4: Audio + VLA Planner** - Audio features, speech recognition, VLA integration
- ✅ **Custom Messages** - 6 message definitions
- ✅ **Build System** - All packages build successfully
- ✅ **Launch Files** - Module-specific launch files
- ✅ **Documentation** - READMEs and completion reports

### Pending (Phases 5-6) ⏳

- ⏳ **Phase 5: Distributed Deployment** - FastDDS, multi-host configuration
- ⏳ **Phase 6: Integration Testing** - End-to-end hardware validation

### Known Limitations

1. **RTAB-Map Integration**: Structure ready, backend implementation incomplete
2. **Hardware Testing**: All modules tested in simulation only
3. **Calibration Algorithms**: Placeholder implementations
4. **GPIO Control**: Mock mode only (real Jetson.GPIO testing pending)

---

## Integration

### With Existing System

The modular nodes integrate with existing hardware drivers:

**Hardware Drivers:**
- `realsense_camera_node` - RGB-D camera data
- `phat_motor_controller_node` - IMU data source
- `irobot_serial_node` - Battery and chassis status
- `nvblox_processor_node` - 3D reconstruction

**Shared Utilities:**
- `isaac_utils` - Health monitoring, Kalman filter, watchdogs

### Topic Flow

```
Hardware Layer:
  phat_motor_controller → /hardware/phat/imu
  realsense_camera → /hardware/camera_*/color/image_raw
  irobot_serial → /irobot/battery

Chassis Control:
  imu_processor → /rpi/imu/filtered
  chassis_controller → /rpi/chassis/odometry

Vision Pipeline:
  visual_slam → /vision/global_pose
  vision_pipeline → /jetson/health/vision_pipeline

Power Management:
  power_manager → /jetson/power/mode
  battery_monitor → /jetson/battery/alerts
```

---

## Documentation

### Module Documentation
- [Chassis Control README](chassis_control/README.md)
- [Vision Pipeline README](vision_pipeline/README.md)
- [Phase 1 Completion Report](../../docs/PHASE1_COMPLETION.md)
- [Phase 2 Completion Report](../../docs/PHASE2_COMPLETION.md)
- [Modular Architecture Progress](../../docs/MODULAR_ARCHITECTURE_PROGRESS.md)

### System Documentation
- [Main README](../../README.md)
- [Architecture Documentation](../../docs/architecture/)
- [Hardware Setup](../../docs/hardware/)

---

## Contributing

When adding new nodes to modules:

1. Follow existing node patterns (health monitoring, parameter handling)
2. Use custom messages for cross-module communication
3. Add smoke tests to `test/` directory
4. Update module README and graph configuration
5. Add launch file parameters for flexibility

---

## License

MIT License - See repository LICENSE file for details.

---

**Last Updated**: 2026-01-27  
**Status**: Phases 1-4 Complete (67% of architecture plan)  
**Next**: Phase 5 (Distributed Deployment)
