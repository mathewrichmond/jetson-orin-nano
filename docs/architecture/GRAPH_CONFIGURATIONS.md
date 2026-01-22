# Graph Configurations

This document describes the sub-graph architecture and available graph configurations for the Isaac robot system.

## Sub-Graph Architecture

The Isaac robot system is organized into four sub-graphs that communicate through well-defined interfaces:

### 1. System Sub-Graph (Core Interface)

**Purpose:** Gather all sensor data, time-align and resample it, publish synchronized fused sensor data, and execute control plans.

**Key Components:**
- Hardware driver nodes (cameras, IMU, chassis, microphones, servos)
- Sensor fusion node (synchronization, filtering, time-alignment)
- Control planner node (executes control plans - not yet implemented)
- System monitor (health, temperatures, system state)

**Publishes (Stable Interface):**
- `/sensor_fusion/*` - Fused, synchronized sensor data
  - Camera images (time-aligned, resampled)
  - IMU data (filtered, synchronized)
  - Chassis state (battery, status, synchronized)
  - 3D data (pointclouds, mesh, TSDF - downsampled for consumers)
  - **System status** (`/sensor_fusion/system/*`) - Comprehensive system health and state
    - Traditional metrics: CPU/GPU usage, memory usage, disk usage, network I/O rates
    - System-specific: camera sync status, temperatures (all thermal zones), device errors, missing hardware, voltages, power consumption
    - Hardware status: device connectivity, error states, calibration status
    - System alerts and warnings
  - Synchronization metadata (timestamps, alignment info, frame sync status)

**Consumes:**
- `/control/plan` - Timestamped control plan from planner sub-graph
  - Motion commands (chassis, servos)
  - Sound/audio commands
  - Mode switches
  - Frame rate/frequency changes
  - Other control actions

**Interface Stability:**
⚠️ **CRITICAL**: The `/sensor_fusion/*` topics form the **stable sense vector** that models train on. Once model training begins, these topics must remain unchanged. Internal system graph implementation can change, but the interface must remain stable.

### 2. Planner Sub-Graph

**Purpose:** Generate control plans from fused sensor data.

**Key Components:**
- Control planner node (not yet implemented)
- VLA controller (future)
- Other planning/control nodes

**Consumes:**
- `/sensor_fusion/*` - Fused sensor data from system sub-graph

**Publishes:**
- `/control/plan` - Timestamped control plan
  - Motion commands
  - Sound/audio commands
  - Mode switches
  - Frame rate/frequency changes
  - Other control actions

**Note:** Can be replaced with human control, remote control, or other control sources that publish to `/control/plan`.

### 3. Visualization Sub-Graph

**Purpose:** Visualize robot state and sensor data for monitoring and debugging.

**Key Components:**
- Foxglove Bridge (remote visualization)
- RViz2 (local visualization)
- Other visualization tools

**Consumes:**
- `/sensor_fusion/*` - Fused sensor data from system sub-graph
- `/viz/remote/*` - Pre-downsampled visualization topics (optional)

**Publishes:**
- Visualization data (via bridge or local display)

**Note:** Can be enabled/disabled independently. Does not affect system or planner operation.

### 4. Logging Sub-Graph

**Purpose:** Record sensor data and system state for analysis and training.

**Key Components:**
- ROS bag recorder
- Custom logging nodes
- Data export tools

**Consumes:**
- `/sensor_fusion/*` - Fused sensor data from system sub-graph

**Publishes:**
- Log files, bag files, exported data

**Note:** Can be enabled/disabled independently. Does not affect system or planner operation.

## Data Flow

```
┌─────────────────────────────────────────────────────────────┐
│                    Hardware Layer                            │
│  Cameras, IMU, Chassis, Microphones, Servos                 │
└───────────────────────┬─────────────────────────────────────┘
                        │
                        ▼
┌─────────────────────────────────────────────────────────────┐
│              System Sub-Graph (Core Interface)               │
│                                                              │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐      │
│  │ Hardware     │  │ Sensor       │  │ Control      │      │
│  │ Drivers      │→ │ Fusion       │  │ Planner      │      │
│  │              │  │ Node         │← │ Node         │      │
│  └──────────────┘  └──────┬───────┘  └──────┬───────┘      │
│                            │                  │              │
│                            │                  │              │
│                            ▼                  │              │
│                    /sensor_fusion/*           │              │
│                    (Stable Sense Vector)     │              │
│                            │                  │              │
│                            │                  │              │
│                            │                  ▼              │
│                            │            /control/plan        │
│                            │            (Control Plan)       │
└────────────────────────────┼──────────────────┼──────────────┘
                             │                  │
                             │                  │
        ┌────────────────────┼──────────────────┼────────────────────┐
        │                    │                  │                    │
        ▼                    ▼                  ▼                    ▼
┌───────────────┐   ┌───────────────┐   ┌───────────────┐   ┌───────────────┐
│   Planner     │   │ Visualization │   │   Logging    │   │   Other       │
│  Sub-Graph   │   │   Sub-Graph   │   │   Sub-Graph  │   │  Consumers    │
│               │   │               │   │               │   │               │
│ Consumes:     │   │ Consumes:     │   │ Consumes:     │   │ Consumes:     │
│ /sensor_      │   │ /sensor_      │   │ /sensor_      │   │ /sensor_      │
│ fusion/*      │   │ fusion/*      │   │ fusion/*      │   │ fusion/*      │
│               │   │               │   │               │   │               │
│ Publishes:    │   │ Publishes:    │   │ Publishes:    │   │               │
│ /control/plan │   │ Viz data      │   │ Log files     │   │               │
└───────────────┘   └───────────────┘   └───────────────┘   └───────────────┘
```

## Graph Configurations

Graph configurations define which sub-graphs are enabled and how they're configured:

### `robot` Graph (Full System)

**Purpose:** Development, testing, and remote monitoring

**Enabled Sub-Graphs:**
- ✅ **System** - All hardware drivers, sensor fusion, control planner
- ✅ **Planner** - Control planning (when implemented)
- ✅ **Visualization** - Foxglove Bridge for remote monitoring
- ✅ **Logging** - ROS bag recording (optional)

**Use Cases:**
- Development and debugging
- Remote monitoring
- Testing with visualization
- Human-in-the-loop operation

**Configuration File:** `config/robot/robot_graph.yaml`

### `minimal` Graph (Production)

**Purpose:** Autonomous operation without visualization/logging overhead

**Enabled Sub-Graphs:**
- ✅ **System** - All hardware drivers, sensor fusion, control planner
- ✅ **Planner** - Control planning (when implemented)
- ❌ **Visualization** - Disabled
- ❌ **Logging** - Disabled (or minimal)

**Use Cases:**
- Autonomous operation
- Production deployment
- Maximum performance mode
- When visualization/logging not needed

**Configuration File:** `config/robot/minimal_graph.yaml`

### `bench_test` Graph (Hardware Validation)

**Purpose:** Hardware testing and validation

**Enabled Sub-Graphs:**
- ✅ **System** - Hardware drivers only (no sensor fusion)
- ❌ **Planner** - Disabled
- ✅ **Visualization** - Foxglove Bridge for monitoring
- ❌ **Logging** - Disabled

**Use Cases:**
- Hardware validation
- Individual component testing
- Bench testing before integration

**Configuration File:** `config/robot/bench_test_graph.yaml`

## Topic Architecture

### System Sub-Graph Outputs (Stable Interface)

**Fused Sensor Data (`/sensor_fusion/*`):**
- `/sensor_fusion/imu/filtered` - Filtered IMU data (15 Hz)
- `/sensor_fusion/chassis/battery` - Chassis battery state (15 Hz)
- `/sensor_fusion/chassis/status` - Chassis status (15 Hz)
- `/sensor_fusion/camera_front/color/image_raw` - Front camera (15 Hz, 480×360)
- `/sensor_fusion/camera_rear/color/image_raw` - Rear camera (15 Hz, 480×360)
- `/sensor_fusion/three_d/camera_front/pointcloud` - Front pointcloud (downsampled)
- `/sensor_fusion/three_d/camera_rear/pointcloud` - Rear pointcloud (downsampled)
- `/sensor_fusion/three_d/mesh` - Fused mesh (decimated)
- `/sensor_fusion/three_d/tsdf` - TSDF voxel grid (coarsened)
- `/sensor_fusion/vlm_features` - VLM-ready features (15 Hz)
- `/sensor_fusion/status` - Fusion node status

**System Status (`/sensor_fusion/system/*`):**
- `/sensor_fusion/system/status` - System health status summary
- `/sensor_fusion/system/temperature/cpu` - CPU temperature
- `/sensor_fusion/system/temperature/gpu` - GPU temperature
- `/sensor_fusion/system/temperature/*` - All thermal zones
- `/sensor_fusion/system/cpu/usage` - CPU usage percentage
- `/sensor_fusion/system/gpu/usage` - GPU usage percentage
- `/sensor_fusion/system/memory/usage` - Memory usage percentage
- `/sensor_fusion/system/disk/usage` - Disk usage percentage
- `/sensor_fusion/system/network/rx_rate` - Network receive rate
- `/sensor_fusion/system/network/tx_rate` - Network transmit rate
- `/sensor_fusion/system/power` - Power consumption (watts)
- `/sensor_fusion/system/voltage/*` - System voltages (if available)
- `/sensor_fusion/system/hardware/camera_sync_status` - Camera frame sync status
- `/sensor_fusion/system/hardware/device_errors` - Device error states
- `/sensor_fusion/system/hardware/missing_devices` - Missing hardware detection
- `/sensor_fusion/system/hardware/connectivity` - Device connectivity status
- `/sensor_fusion/system/alerts` - System alerts and warnings

**Note:** System status is synchronized and resampled to match sensor fusion rate (typically 15 Hz for controller, 10 Hz for visualization). The same information is available to both controller and visualization, following the "single point of understanding" principle.

**Visualization Topics (`/viz/remote/*`):**
- `/viz/remote/imu/filtered` - IMU for visualization (10 Hz)
- `/viz/remote/chassis/battery` - Battery for visualization (10 Hz)
- `/viz/remote/camera_front/color/image_raw` - Camera for visualization (10 Hz, 320×240)
- `/viz/remote/three_d/pointcloud` - Pointcloud for visualization (5 Hz, heavily downsampled)
- `/viz/remote/three_d/mesh` - Mesh for visualization (5 Hz, heavily decimated)
- `/viz/remote/three_d/tsdf_mesh` - TSDF mesh for visualization (2 Hz)

### Planner Sub-Graph Outputs

**Control Plan (`/control/plan`):**
- Timestamped control plan message containing:
  - Motion commands (chassis velocity, servo positions)
  - Sound/audio commands
  - Mode switches
  - Frame rate/frequency changes
  - Other control actions

## Switching Between Graphs

### Select Graph Configuration

```bash
# Select robot graph (full system)
echo "robot" > config/robot/selected_graph.txt

# Select minimal graph (production)
echo "minimal" > config/robot/selected_graph.txt

# Select bench test graph (hardware validation)
echo "bench_test" > config/robot/selected_graph.txt
```

### Start Graph

```bash
# Start selected graph
systemctl --user start isaac-robot.service

# Or start specific graph
ROBOT_GRAPH=robot systemctl --user start isaac-robot.service
ROBOT_GRAPH=minimal systemctl --user start isaac-robot.service
ROBOT_GRAPH=bench_test systemctl --user start isaac-robot.service
```

### Stop Graph

```bash
systemctl --user stop isaac-robot.service
```

## Interface Stability Guarantees

### Stable Interface (Must Not Change)

Once model training begins, these interfaces **must remain stable**:

1. **Sense Vector (`/sensor_fusion/*` topics):**
   - Topic names
   - Message types
   - Update frequencies
   - Data formats
   - Coordinate frames
   - **System status topics** (`/sensor_fusion/system/*`) - All system status information must be included in the sense vector

2. **Control Plan (`/control/plan` topic):**
   - Topic name
   - Message type
   - Command structure
   - Timestamp format

**System Status Requirements:**
- System status must be part of the fused sense vector (not separate topics)
- Same system status information exposed to controller and visualization
- Frequency can be adjusted for bandwidth, but data completeness must be maintained
- All system-specific status (camera sync, device errors, voltages, etc.) must be included

### Flexible Implementation

These can change without affecting trained models:

- Internal system graph node implementation
- Hardware driver implementations
- Sensor fusion algorithms (as long as output format stays the same)
- Control planner execution (as long as it consumes sense vector and publishes control plan)
- Visualization and logging implementations

## Recommendations

1. **For Development:** Use `robot` graph with all sub-graphs enabled
2. **For Production:** Use `minimal` graph with only system and planner sub-graphs
3. **For Testing:** Use `bench_test` graph for hardware validation
4. **For Training:** Use `robot` graph with logging enabled to capture training data
5. **For Autonomous:** Use `minimal` graph to maximize performance

## Future Enhancements

- **Dynamic sub-graph enabling/disabling** - Enable/disable sub-graphs without full restart
- **Multiple planner support** - Switch between different planners (VLA, remote, human)
- **Bandwidth monitoring** - Automatic sub-graph disabling based on bandwidth
- **Interface versioning** - Support for interface evolution while maintaining backward compatibility
