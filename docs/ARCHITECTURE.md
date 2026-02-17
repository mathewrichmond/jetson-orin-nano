# System Architecture - Articulated Cameras with UDM Worldgraph

Robot system with independently actuated cameras projecting into a unified digital twin.

## Core Design

### Problem
- **Articulated cameras**: Independent pan-tilt servos per camera
- **Simulation challenge**: Moving cameras make planning/simulation difficult
- **Model flexibility**: Want to run pre-trained models with different sensor configs

### Solution: UDM Worldgraph
**Universal Data Model worldgraph** = Canonical digital twin representation

```
Real Sensors (articulated) → Project → UDM Worldgraph
                                            ↓
                           Render ← Simulated Sensors (any config)
                                            ↓
                                    Planning/Models/Training
```

**Benefits**:
1. Plan in worldgraph, independent of sensor poses
2. Emulate any sensor configuration from worldgraph
3. Generate offline RL training data
4. Single ground truth regardless of camera motion

---

## Data Flow

```
┌─────────────────────────────────────────────────────────┐
│ Physical Sensors (Moving)                               │
├─────────────────────────────────────────────────────────┤
│                                                         │
│  Camera Left (pan-tilt)  Camera Right (pan-tilt)       │
│       ↓ depth+RGB              ↓ depth+RGB             │
│       └──────────┬──────────────┘                      │
│                  ↓                                      │
│         + Robot Pose (odometry/SLAM)                   │
│         + Servo Angles (pan, tilt)                     │
│         + Kinematic Chain (calibrated)                 │
│                  ↓                                      │
└──────────────────┼──────────────────────────────────────┘
                   ↓
         ┌─────────────────────┐
         │ Forward Projection  │
         └─────────────────────┘
                   ↓
┌─────────────────────────────────────────────────────────┐
│ UDM Worldgraph (Digital Twin)                          │
├─────────────────────────────────────────────────────────┤
│                                                         │
│  - 3D occupancy grid / TSDF                            │
│  - Object detections (3D bounding boxes)               │
│  - Semantic labels                                      │
│  - Dynamic objects (tracked)                           │
│  - Confidence maps                                      │
│                                                         │
│  Canonical representation, independent of sensors       │
└─────────────────────────────────────────────────────────┘
                   ↓
         ┌─────────────────────┐
         │ Inverse Rendering   │
         └─────────────────────┘
                   ↓
┌─────────────────────────────────────────────────────────┐
│ Virtual Sensors (Any Configuration)                     │
├─────────────────────────────────────────────────────────┤
│                                                         │
│  - Fixed forward camera (for pre-trained models)       │
│  - Bird's eye view (for planning)                      │
│  - Multi-view for offline RL training                  │
│  - Any intrinsics, any pose, any frequency             │
│                                                         │
└─────────────────────────────────────────────────────────┘
                   ↓
         ┌─────────────────────┐
         │ Planning/Models     │
         └─────────────────────┘
                   ↓
              Commands
                   ↓
         ┌─────────────────────┐
         │ Robot Control       │
         └─────────────────────┘
```

---

## Components

### 1. Robot Control Service

**One container** (for bringup):
- Chassis control
- Camera pipeline (RealSense capture)
- Servo control (pan-tilt actuation)
- Forward kinematics (compute camera poses)
- SLAM/odometry (robot base localization)
- UDM worldgraph (projection & rendering)

### 2. Kinematic Chain (per camera)

```
robot_base → pan_servo → tilt_servo → camera_optical_frame
```

**Calibrated parameters**:
- Static transforms between joints
- Servo zero positions, scales, limits
- Camera intrinsics

**Runtime**:
- Read servo angles
- Compute forward kinematics
- Get camera pose in world frame

### 3. UDM Worldgraph

**Representation**:
- TSDF (Truncated Signed Distance Field) or occupancy grid
- RGB-D fusion from multiple views
- Object detections (3D bounding boxes)
- Semantic labels
- Dynamic object tracking

**Properties**:
- Multi-resolution (fine detail to coarse structure)
- Temporal fusion (integrate over time)
- Confidence weighting (per voxel)
- Online updates (real-time integration)

**Technologies**:
- nvblox (NVIDIA TSDF on GPU)
- Octomap (if CPU-based)
- Custom occupancy grid

### 4. Forward Projection

**Per camera, per frame**:
1. Get current camera pose (kinematics + localization)
2. Deproject depth image to 3D points (camera frame)
3. Transform points to world frame
4. Integrate into worldgraph with confidence

**Handles**:
- Moving cameras (tracked via kinematics)
- Overlapping views (fused with weights)
- Outliers (reject based on consistency)

### 5. Inverse Rendering

**On demand** (for planning/models):
1. Define virtual camera (pose + intrinsics)
2. Raycast worldgraph from virtual camera
3. Generate synthetic depth + RGB
4. Apply noise/degradation if modeling real sensors

**Enables**:
- Fixed camera for models expecting static sensors
- Multi-view for training
- Any resolution, any FOV

---

## Hardware Platform

### SparkFun Auto pHAT (ROB-16328)

The robot control is implemented using the SparkFun Auto pHAT for Raspberry Pi, connected to a Jetson Orin Nano via the 40-pin GPIO header.

**Key Components**:
- **PCA9685 Servo Controller**: 4-channel PWM for pan-tilt servos (I2C address 0x40)
- **ICM-20948 IMU**: 9-DoF motion sensing for odometry and SLAM (I2C address 0x69)
- **PSoC 4245 Motor Driver**: 2-channel H-bridge for chassis motors (I2C address 0x5D)
- **ATtiny84 Encoder Reader**: Dual quadrature encoder inputs (I2C address 0x73)

**I2C Bus Configuration**:
- All devices communicate via I2C bus 7 on Jetson Orin Nano (bus 1 on Raspberry Pi)
- GPIO pins 3 (SDA) and 5 (SCL) on 40-pin header
- Built-in pull-up resistors (2.2kΩ-10kΩ to 3.3V)

**Documentation**:
- Complete hardware reference: [hardware/sparkfun-auto-phat/](hardware/sparkfun-auto-phat/)
- I2C troubleshooting: [I2C_TROUBLESHOOTING.md](hardware/sparkfun-auto-phat/I2C_TROUBLESHOOTING.md)
- Configuration: `config/hardware/phat_params.yaml`

**Diagnostics**:
```bash
# Scan I2C bus
sudo i2cdetect -y 7

# Test servo controller
./scripts/hardware/diagnose_pca9685.sh

# General PHAT diagnostics
./scripts/hardware/diagnose_phat.sh
```

---

## Use Cases

### Use Case 1: Planning with Worldgraph

```python
# Planner operates on worldgraph, not raw sensors
planner = HighLevelPlanner(worldgraph)

# Plan path in 3D space
path = planner.plan_path(current_pose, goal_pose)

# Execute (robot control handles details)
robot.execute_path(path)
```

### Use Case 2: Run Model with Different Sensor Config

```python
# Pre-trained model expects fixed forward-facing camera at 640x480
# But our cameras are articulated and different resolution

# Define virtual camera matching model's expectations
virtual_pose = get_fixed_forward_camera(robot_pose)
virtual_intrinsics = model.get_expected_intrinsics()

# Render from worldgraph
synthetic_image = worldgraph.render(virtual_pose, virtual_intrinsics)

# Run model
action = model.infer(synthetic_image)
```

### Use Case 3: Offline RL Training

```python
# Generate training data in simulated environment
sim = WorldgraphSimulator(worldgraph)

for episode in range(num_episodes):
    obs = sim.reset()
    
    for step in range(max_steps):
        # Render observation from current robot pose
        # Can render any sensor configuration
        obs = sim.render_observation()
        
        action = policy(obs)
        next_obs, reward = sim.step(action)
        
        buffer.add(obs, action, reward, next_obs)

# Train policy on simulated data
policy.train(buffer)
```

### Use Case 4: Active Perception

```python
# Use camera articulation for object tracking

# Detect object in worldgraph
object_pos_3d = worldgraph.get_object_position(object_id)

# Compute servo angles to look at object
pan, tilt = compute_look_at(camera_pose, object_pos_3d)

# Actuate servos
servo_controller.set_target(pan, tilt)

# Camera automatically tracks object
```

---

## Calibration

See [CALIBRATION.md](CALIBRATION.md) for complete details.

**Key calibrations**:
1. **Camera intrinsics** (per camera, fixture-based)
2. **Kinematic chain** (per camera, marker-based)
3. **Servo calibration** (zero positions, scales)
4. **Odometry** (SLAM ground truth)
5. **Worldgraph parameters** (runtime adaptation)

---

## Storage

```
/data/
├── config/
│   └── factory_calibration.yaml       # Kinematics, intrinsics, servos
│
├── worldgraph/
│   ├── map_YYYYMMDD_HHMMSS.tsdf      # Saved worldgraph maps
│   └── current.tsdf                   # Active worldgraph
│
└── sessions/
    └── worldgraph_build_*/            # Worldgraph construction data
```

---

## Performance

### Real-Time Requirements

**Sensor integration**:
- Camera capture: 30 FPS
- Forward projection: 30 Hz (per camera)
- Worldgraph update: 30 Hz

**Planning/inference**:
- Worldgraph queries: < 10ms
- Inverse rendering: 10-30 Hz (depends on resolution)
- Model inference: 10-20 Hz

### GPU Usage

- TSDF integration (nvblox): Primary GPU load
- Depth processing: Secondary
- Inverse rendering: On-demand

**Memory**:
- Worldgraph: 500MB - 2GB (depends on resolution, map size)
- Camera buffers: 100-200MB
- Model: 500MB - 2GB

---

## Future Extensions

### Additional Sensors
- Add LIDAR → project into worldgraph
- Add thermal camera → add temperature layer to worldgraph
- Add microphone array → add audio layer

### Dynamic Objects
- Track moving objects in worldgraph
- Predict motion, update predictions
- Handle occlusions

### Multi-Robot
- Multiple robots build shared worldgraph
- Coordinate observations
- Distributed SLAM

---

## Technology Stack

**Worldgraph**:
- nvblox (NVIDIA TSDF on GPU)
- PCL (Point Cloud Library)
- Open3D (visualization, processing)

**Kinematics**:
- ROS 2 TF2 (transform broadcasting)
- URDF (robot description)

**Rendering**:
- OpenGL / CUDA raycasting
- nvblox rendering utilities

**Communication**:
- ROS 2 (sensor streams, commands)
- Shared memory (worldgraph access)

---

## See Also

- **Hardware Documentation**: [hardware/sparkfun-auto-phat/](hardware/sparkfun-auto-phat/) - Complete SparkFun Auto pHAT specifications
  - [Hardware Overview](hardware/sparkfun-auto-phat/README.md) - Board identification and component layout
  - [Technical Specifications](hardware/sparkfun-auto-phat/SPECIFICATIONS.md) - Detailed IC specs and addresses
  - [I2C Troubleshooting](hardware/sparkfun-auto-phat/I2C_TROUBLESHOOTING.md) - Systematic debugging guide
  - [Jumper Configuration](hardware/sparkfun-auto-phat/JUMPER_CONFIGURATION.md) - Address configuration
- **Calibration**: [CALIBRATION.md](CALIBRATION.md) - Camera and servo calibration procedures
- **Setup**: [SETUP.md](../SETUP.md) - Initial system setup
- **Recovery**: [RECOVERY.md](RECOVERY.md) - System recovery procedures
