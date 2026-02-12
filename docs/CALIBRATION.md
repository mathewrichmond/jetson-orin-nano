# Calibration System - Articulated Cameras with UDM Worldgraph

Calibration for independently actuated pan-tilt cameras projecting into a unified digital twin.

## System Architecture

### Key Design
- **Cameras**: Independently articulated (pan-tilt servos per camera)
- **Depth**: RealSense built-in depth (not stereo)
- **Articulation**: For object tracking and active perception
- **Challenge**: Moving cameras make simulation/planning difficult
- **Solution**: Project all sensor data into **UDM worldgraph** (digital twin)

### UDM Worldgraph Benefits
1. **Simulation independence**: Plan without exact sensor poses/kinematics
2. **Sensor config flexibility**: Run pre-trained models with different sensors by emulating inputs from worldgraph
3. **Offline training**: Generate RL training data from virtual world
4. **Canonical representation**: Single ground truth regardless of sensor motion

```
Real Sensors → Forward Projection → UDM Worldgraph (digital twin)
                                           ↓
                        Inverse Rendering → Simulated Sensors
                                           ↓
                                    Models/Planning/Training
```

---

## Calibration Categories

### 1. Camera Intrinsics (Fixture-Based)
**Per-camera calibration** (each camera is independent):

- **Camera intrinsics**: Focal length, principal point, distortion
  - Method: Checkerboard calibration per camera
  - Frequency: Once per camera, or after changes
  - Why: Needed for accurate depth projection
  
- **Depth calibration**: RealSense depth accuracy
  - Method: Known distances, planar surfaces
  - Frequency: Rarely (factory calibrated)
  - Why: Depth error affects worldgraph accuracy

### 2. Kinematic Chain (Fixture-Based + Verification)
**Camera articulation kinematics**:

#### Camera Mount Chain (per camera):
```
robot_base → pan_servo → tilt_servo → camera_optical_frame
```

**Parameters to calibrate**:
- `base_to_pan`: Static transform (camera mount position on robot)
- `pan_servo_offset`: Zero position of pan servo
- `pan_to_tilt`: Static transform (tilt servo relative to pan)
- `tilt_servo_offset`: Zero position of tilt servo  
- `tilt_to_camera`: Static transform (camera relative to tilt servo)

**Calibration methods**:

#### Option A: Manual measurement + verification
```bash
# Measure physical offsets with calipers
./calibrate.sh camera-kinematics --camera left --measure

# Verify with known markers in view
./calibrate.sh camera-kinematics --camera left --verify
```

#### Option B: Marker-based (preferred)
```bash
# Place ArUco markers at known positions in world
# Move camera through range of motion
# System observes markers, optimizes kinematic chain

./calibrate.sh camera-kinematics --camera left --markers
```

**Output**: Forward kinematics for each camera

### 3. Servo Calibration (Auto + Verification)

**Per-servo parameters**:
- `zero_position`: Encoder value at mechanical zero
- `angle_scale`: Encoder ticks to radians conversion
- `direction`: +1 or -1 (servo direction)
- `joint_limits`: Min/max safe angles

**Calibration**:
```bash
# Home servos to known positions (mechanical stops or markers)
./calibrate.sh servo-calibration --camera left --home

# Verify range of motion
./calibrate.sh servo-calibration --camera left --verify-range
```

### 4. Odometry (Auto-Calibration)

**Robot base motion** (same as before):
- Method: SLAM ground truth comparison
- Parameters: Wheel radius, wheelbase, encoder scales
- See previous odometry calibration section

### 5. UDM Worldgraph Projection (Runtime)

**Projection parameters** (learned online):
- Depth confidence thresholds
- Outlier rejection
- Temporal fusion weights
- Occupancy grid resolution

These adapt during operation based on observed consistency.

---

## Calibration Data Structure

```yaml
# /data/config/factory_calibration.yaml

camera_left:
  intrinsics:
    fx: 615.234
    fy: 615.987
    cx: 320.123
    cy: 240.456
    distortion: [k1, k2, p1, p2, k3]
  
  depth:
    depth_scale: 0.001  # RealSense depth units to meters
    min_depth: 0.3
    max_depth: 5.0
  
  kinematics:
    # Robot base to pan servo mount
    base_to_pan:
      translation: [0.15, 0.10, 0.25]  # x, y, z (meters)
      rotation: [0.0, 0.0, 0.0]        # roll, pitch, yaw (radians)
    
    # Pan servo properties
    pan_servo:
      zero_position: 2048           # Encoder value at 0°
      ticks_per_radian: 651.74      # Encoder resolution
      direction: 1                   # +1 or -1
      joint_limits: [-1.57, 1.57]   # ±90° range
    
    # Pan servo to tilt servo mount
    pan_to_tilt:
      translation: [0.0, 0.0, 0.05]
      rotation: [0.0, 0.0, 0.0]
    
    # Tilt servo properties
    tilt_servo:
      zero_position: 2048
      ticks_per_radian: 651.74
      direction: 1
      joint_limits: [-0.785, 0.785]  # ±45° range
    
    # Tilt servo to camera optical frame
    tilt_to_camera:
      translation: [0.04, 0.0, 0.0]
      rotation: [0.0, 0.0, 0.0]

camera_right:
  # Same structure for right camera
  intrinsics: {...}
  depth: {...}
  kinematics: {...}

odometry:
  # Robot base motion calibration
  wheel_radius_left: 0.0652
  wheel_radius_right: 0.0648
  wheelbase: 0.322
```

---

## Forward Kinematics

**Compute camera pose in world frame**:

```python
def get_camera_pose(pan_angle, tilt_angle, robot_pose):
    """
    Compute camera optical frame in world coordinates.
    
    Args:
        pan_angle: Pan servo angle (radians)
        tilt_angle: Tilt servo angle (radians)
        robot_pose: Robot base pose in world frame
    
    Returns:
        camera_pose: Camera optical frame in world frame
    """
    T_world_base = robot_pose
    T_base_pan = calib.base_to_pan
    T_pan_joint = rotation_z(pan_angle)  # Pan rotation
    T_pan_tilt = calib.pan_to_tilt
    T_tilt_joint = rotation_y(tilt_angle)  # Tilt rotation
    T_tilt_camera = calib.tilt_to_camera
    
    T_world_camera = (T_world_base @ 
                      T_base_pan @ 
                      T_pan_joint @ 
                      T_pan_tilt @ 
                      T_tilt_joint @ 
                      T_tilt_camera)
    
    return T_world_camera
```

---

## UDM Worldgraph Projection

### Real Sensors → Worldgraph

```python
def project_to_worldgraph(depth_image, color_image, camera_pose, intrinsics):
    """
    Project sensor observations into UDM worldgraph.
    
    Args:
        depth_image: RealSense depth (H x W)
        color_image: RGB image (H x W x 3)
        camera_pose: Camera pose in world frame
        intrinsics: Camera intrinsics
    
    Returns:
        worldgraph: Updated 3D occupancy/TSDF representation
    """
    # Deproject to camera frame 3D points
    points_camera = deproject_depth(depth_image, intrinsics)
    
    # Transform to world frame
    points_world = camera_pose @ points_camera
    
    # Integrate into worldgraph (TSDF, occupancy grid, etc.)
    worldgraph.integrate(points_world, color_image, confidence)
    
    return worldgraph
```

### Worldgraph → Simulated Sensors

```python
def render_from_worldgraph(worldgraph, virtual_camera_pose, virtual_intrinsics):
    """
    Render synthetic sensor observations from worldgraph.
    Allows emulating different sensor configurations.
    
    Args:
        worldgraph: UDM worldgraph (3D scene representation)
        virtual_camera_pose: Desired camera pose (can be anywhere)
        virtual_intrinsics: Desired camera intrinsics (can differ from real)
    
    Returns:
        synthetic_depth: Rendered depth image
        synthetic_rgb: Rendered RGB image
    """
    # Ray casting or mesh rendering from worldgraph
    synthetic_depth = raycast_tsdf(worldgraph, virtual_camera_pose, virtual_intrinsics)
    synthetic_rgb = render_color(worldgraph, virtual_camera_pose, virtual_intrinsics)
    
    return synthetic_depth, synthetic_rgb
```

---

## Use Cases

### 1. Planning in Virtual World

```python
# Plan using worldgraph, not raw sensors
planner.set_scene(worldgraph)
path = planner.plan(start, goal)

# Execute path in real world
robot.execute_path(path)
```

### 2. Run Pre-Trained Model with Different Sensor Config

```python
# Model expects fixed forward-facing camera
# But we have articulated cameras

# Render what the model expects from worldgraph
virtual_pose = get_forward_camera_pose(robot_pose)
virtual_image = render_from_worldgraph(worldgraph, virtual_pose, model_intrinsics)

# Feed to model
action = model.infer(virtual_image)
```

### 3. Offline RL Training

```python
# Generate training data from worldgraph
for episode in range(num_episodes):
    # Simulate robot motion in worldgraph
    robot_sim.reset()
    
    for step in range(max_steps):
        # Render observations from current pose
        obs = render_from_worldgraph(worldgraph, robot_sim.get_camera_pose(), ...)
        
        # Get action from policy
        action = policy(obs)
        
        # Simulate action, update worldgraph
        robot_sim.step(action)
        worldgraph.update_dynamic_objects(...)
        
        # Store transition
        replay_buffer.add(obs, action, reward, next_obs)
```

---

## Calibration Workflow

### Phase 1: Camera Intrinsics

```bash
# Per-camera intrinsics (independent)
./calibrate.sh camera-intrinsics --camera left
./calibrate.sh camera-intrinsics --camera right
```

### Phase 2: Kinematic Chain

```bash
# Per-camera kinematic chain
# Option A: Manual measurement
./calibrate.sh camera-kinematics --camera left --measure

# Option B: Marker-based (preferred)
./calibrate.sh camera-kinematics --camera left --markers
# Repeat for right camera
```

### Phase 3: Servo Calibration

```bash
# Home servos, verify range
./calibrate.sh servo-calibration --camera left --home
./calibrate.sh servo-calibration --camera left --verify-range
# Repeat for right
```

### Phase 4: Odometry (Robot Base)

```bash
./calibrate.sh odometry --start
# Drive robot
./calibrate.sh odometry --stop
```

### Phase 5: Worldgraph Verification

```bash
# Verify projection accuracy
./calibrate.sh worldgraph-verify

# Move cameras, check consistency
# Worldgraph should remain consistent across camera motions
```

---

## Verification

### Kinematic Chain Verification

```bash
# Place markers at known positions
# Move cameras through range of motion
# Check: Observed marker positions match expected from kinematics

./calibrate.sh verify-kinematics --camera left
```

**Metrics**:
- Marker reprojection error: < 5mm over 2m distance
- Consistency across camera poses: < 1cm variation

### Worldgraph Consistency

```bash
# Observe same scene from different camera poses
# Check: Worldgraph projections agree

./calibrate.sh verify-worldgraph
```

**Metrics**:
- Point cloud alignment: < 2cm RMS
- Surface consistency: < 5mm deviation

---

## Runtime Calibration

### Servo Drift Compensation

**Automatically track**:
- Encoder drift (compare commanded vs observed poses)
- Backlash (hysteresis in servo response)
- Thermal expansion (servo zero shifts with temperature)

**Method**: Use worldgraph consistency as ground truth
- Observe known feature from different poses
- If reprojection error increases, update servo offsets

### Depth Calibration Refinement

**Continuously monitor**:
- Depth vs worldgraph consistency
- Planar surface fit quality
- Feature alignment across cameras

**Adapt**: Depth confidence weights, outlier rejection thresholds

---

## Calibration Quality Metrics

### Camera Intrinsics
- Reprojection error: < 0.5 pixels

### Kinematic Chain  
- Marker reprojection: < 5mm at 2m
- Cross-pose consistency: < 1cm

### Servo Calibration
- Repeatability: < 0.5° (< 1cm at 2m)
- Backlash: < 1°

### Worldgraph
- Point cloud alignment: < 2cm RMS
- Surface consistency: < 5mm
- Multi-view consistency: > 95% overlap agreement

---

## Calibration Data Persistence

### Storage Location

All calibration data stored in `/data/config/` for persistence across reboots and syncing:

```
/data/config/
├── calibration/
│   ├── factory_calibration.yaml       # Master calibration file
│   ├── odometry_calibration.yaml      # Wheel parameters
│   ├── camera_imu_refined.yaml        # Refined extrinsics
│   └── runtime_calibration.yaml       # Continuous updates
│
├── calibration_history/               # Versioned backups
│   ├── factory_calibration_20260212.yaml
│   ├── odometry_calibration_20260212.yaml
│   └── ...
│
└── calibration_sessions/              # Raw calibration data
    ├── camera_intrinsics_left_20260212/
    ├── odometry_20260212_143022/
    └── ...
```

### Mount Points

Container mounts calibration as **read-write** to save results:

```yaml
# docker-compose.yml
volumes:
  - /data/config:/data/config        # Read-write for calibration updates
```

During calibration, new files written to `/data/config/calibration/`.

### Loading Calibration

System loads calibration at startup:

```python
# In robot-control container startup
calibration = CalibrationManager('/data/config')
calibration.load_all()  # Loads all .yaml files

# Use in runtime
camera_pose = calibration.get_camera_pose(pan, tilt, robot_pose)
depth_corrected = calibration.apply_depth_correction(depth_raw)
```

### Versioning

**Automatic versioning** on calibration updates:

```python
# Before overwriting calibration
old_file = '/data/config/calibration/factory_calibration.yaml'
timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
backup = f'/data/config/calibration_history/factory_calibration_{timestamp}.yaml'
shutil.copy(old_file, backup)

# Then write new calibration
save_calibration(new_data, old_file)
```

**Benefits**:
- Never lose previous calibrations
- Rollback if new calibration is bad
- Track calibration drift over time

### Sync Strategy

Calibration data synced to remote backup via `isaac-sync.service`:

```bash
# /usr/local/bin/isaac-sync.sh includes:
rsync -avz --delete /data/config/ user@server:/backup/isaac/config/
```

**Important**: Syncs after each calibration to prevent data loss.

### Loading Priority

System loads calibration in order:

1. `/data/config/calibration/factory_calibration.yaml` (required)
2. `/data/config/calibration/odometry_calibration.yaml` (optional, uses defaults if missing)
3. `/data/config/calibration/runtime_calibration.yaml` (optional, starts fresh if missing)

### Calibration States

```yaml
# /data/config/calibration/factory_calibration.yaml

metadata:
  calibration_date: "2026-02-12"
  calibration_version: "1.0"
  robot_id: "isaac"
  calibrated_by: "auto"
  
status:
  camera_left_intrinsics: "complete"    # or "pending", "failed"
  camera_right_intrinsics: "complete"
  camera_left_kinematics: "complete"
  camera_right_kinematics: "complete"
  servo_left: "complete"
  servo_right: "complete"
  odometry: "complete"
  
# ... actual calibration data ...
```

System checks `status` fields at startup:
- If any "pending": Prompt user to complete calibration
- If any "failed": Load defaults, warn user
- If all "complete": Use calibrated values

### Manual Backup

```bash
# Backup all calibration before re-calibrating
tar czf ~/calibration_backup_$(date +%Y%m%d).tar.gz /data/config/calibration/

# Restore if needed
tar xzf ~/calibration_backup_20260212.tar.gz -C /
```

### Migration

When moving calibration to new robot or after hardware changes:

```bash
# Copy calibration from old system
scp -r old-robot:/data/config/calibration/ /data/config/

# Mark what needs re-calibration
# Edit /data/config/calibration/factory_calibration.yaml
# Set status fields to "pending" for changed components

# Re-run only affected calibrations
./calibrate.sh camera-kinematics --camera left --markers
```

---

## Tools

```bash
# Camera calibration
./calibrate.sh camera-intrinsics --camera [left|right]

# Kinematic chain
./calibrate.sh camera-kinematics --camera [left|right] --markers

# Servo calibration
./calibrate.sh servo-calibration --camera [left|right] --home

# Odometry
./calibrate.sh odometry [--start|--stop]

# Verification
./calibrate.sh verify-kinematics --camera [left|right]
./calibrate.sh verify-worldgraph
./calibrate.sh verify-all
```

---

## See Also

- Forward kinematics: `src/isaac_robot/kinematics/camera_kinematics.py`
- Worldgraph projection: `src/isaac_robot/worldgraph/projection.py`
- Rendering: `src/isaac_robot/worldgraph/rendering.py`
- Config manager: `src/isaac_robot/calibration/calibration_manager.py`
- Storage: `/data/config/calibration/`
