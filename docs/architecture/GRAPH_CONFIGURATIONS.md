# Graph Configurations

This document describes the available graph configurations and their use cases.

## Overview

The Isaac robot system supports multiple graph configurations for different operational modes:

1. **`robot`** - Full system with visualization (development/testing)
2. **`minimal`** - Production mode without visualization (autonomous operation)

## Graph Configurations

### `robot` Graph (Full System)

**Purpose:** Development, testing, and remote monitoring

**Components:**
- ✅ All sensors (cameras, IMU, chassis)
- ✅ Sensor fusion node
- ✅ Feature builder (VLA controller)
- ✅ Bridge (remote visualization)
- ✅ Visualization topics (downsampled for bridge)

**Bandwidth:**
- Feature Builder: ~27.6 MB/s (camera images at 15 Hz, full resolution)
- Bridge: ~2.3 MB/s (visualization topics at 10 Hz, low resolution)
- **Total**: ~30 MB/s

**Use Cases:**
- Development and debugging
- Remote monitoring
- Testing with visualization
- Human-in-the-loop operation

**Configuration File:** `config/robot/robot_graph.yaml`

### `minimal` Graph (Production)

**Purpose:** Autonomous operation without visualization overhead

**Components:**
- ✅ All sensors (cameras, IMU, chassis)
- ✅ Sensor fusion node
- ✅ Feature builder (VLA controller)
- ❌ Bridge **disabled**
- ❌ Visualization topics **disabled**

**Bandwidth:**
- Feature Builder: ~27.6 MB/s (camera images at 15 Hz, full resolution)
- Bridge: 0 MB/s (disabled)
- **Total**: ~28 MB/s

**Use Cases:**
- Autonomous operation
- Production deployment
- When visualization is not needed
- Maximum performance mode

**Configuration File:** `config/robot/minimal_graph.yaml`

## Topic Architecture

### Raw Sensors (High Frequency)
- `/phat/imu` - 50 Hz
- `/irobot/battery` - 10 Hz
- `/irobot/status` - 10 Hz
- `/hardware/camera_*/color/image_raw` - 30 Hz

### Fusion Node Outputs

#### Feature Topics (for VLM/Feature Builder)
- `/sensor_fusion/imu/filtered` - 15 Hz
- `/sensor_fusion/chassis/battery` - 15 Hz
- `/sensor_fusion/chassis/status` - 15 Hz
- `/sensor_fusion/camera_*/color/image_raw` - 15 Hz, 640×480
- `/sensor_fusion/vlm_features` - 15 Hz

#### Visualization Topics (for Bridge)
- `/viz/remote/imu/filtered` - 10 Hz
- `/viz/remote/chassis/battery` - 10 Hz
- `/viz/remote/camera_front/color/image_raw` - 10 Hz, 320×240

## Switching Between Graphs

### Select Graph Configuration
```bash
# Select robot graph (full system)
./scripts/system/manage_graph.sh select robot

# Select minimal graph (production)
./scripts/system/manage_graph.sh select minimal
```

### Start Graph
```bash
# Start selected graph
./scripts/system/manage_graph.sh start

# Or start specific graph
./scripts/system/manage_graph.sh start robot
./scripts/system/manage_graph.sh start minimal
```

### Stop Graph
```bash
./scripts/system/manage_graph.sh stop
```

## Bandwidth Allocation

### Feature Builder Budget: 20 MB/s
- Camera images: 640×480 @ 15 Hz × 2 cameras = ~27.6 MB/s
- **Note**: Currently exceeds budget, needs optimization

### Bridge Budget: 5 MB/s
- Camera: 320×240 @ 10 Hz = ~2.3 MB/s
- IMU/Chassis: ~0.005 MB/s
- System stats: ~0.001 MB/s
- **Total**: ~2.3 MB/s (within budget)

## Recommendations

1. **For Development:** Use `robot` graph with bridge for remote monitoring
2. **For Production:** Use `minimal` graph to maximize performance
3. **For Testing:** Use `robot` graph to verify all systems
4. **For Autonomous:** Use `minimal` graph to reduce overhead

## Future Enhancements

- **`debug` graph**: Additional logging and diagnostics
- **`bench_test` graph**: Hardware validation mode
- **Dynamic graph switching**: Change graphs without full restart
- **Bandwidth monitoring**: Automatic graph switching based on bandwidth
