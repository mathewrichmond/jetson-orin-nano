# VLA Planner Module

**Status**: Phase 4 ✅ | VLA model integration, action execution, task planning

---

## Overview

The VLA (Vision-Language-Action) planner module integrates VLA models for embodied AI control, providing model inference, action execution, and high-level task planning.

### Architecture

```
Sensor Inputs → VLA Controller → Action Executor → Robot Hardware
      ↓                 ↑                ↑
 Audio Features    Planner Node    Execution Feedback
```

---

## Nodes

### 1. VLA Controller Node

Runs VLA model inference for action prediction.

**Executable**: `vla_controller_node`

**Features**:
- Multimodal input processing (vision, audio, proprioception, language)
- VLA model inference (placeholder for OpenVLA, RT-1, etc.)
- Action sequence generation
- GPU power management integration

**Topics**:
- Subscribes:
  - `/sensor_fusion/vlm_features` - Vision features
  - `/audio/features/mfcc` - Audio features
  - `/rpi/chassis/pose_estimate` - Robot state
  - `/audio/transcription` - Speech commands
- Publishes:
  - `/vla/actions` - Raw action predictions
  - `/control/cmd_vel` - Velocity commands

**Parameters**:
```yaml
model_path: ""  # Path to VLA weights
model_type: placeholder  # placeholder, openvla, rt1
device: cuda  # cuda or cpu
inference_rate: 10.0  # Hz
action_horizon: 10  # Future actions to predict
context_window: 5  # Past observations to use
confidence_threshold: 0.5
request_gpu: true
gpu_priority: 4  # Highest priority
```

**Supported Models** (future):
- **OpenVLA** - Open source VLA model
- **RT-1** - Robotics Transformer
- **RT-2** - Robotics Transformer 2
- **PaLM-E** - Embodied multimodal LLM

---

### 2. Action Executor Node

Executes actions from VLA controller with safety checks.

**Executable**: `action_executor_node`

**Features**:
- Action validation and safety limits
- Multi-step action sequencing
- Execution monitoring
- Failure recovery

**Topics**:
- Subscribes: `/vla/actions`
- Publishes:
  - `/control/cmd_vel` - Velocity commands
  - `/vla/execution_feedback` - Execution status

**Parameters**:
```yaml
action_topic: /vla/actions
cmd_vel_topic: /control/cmd_vel
max_linear_velocity: 0.5  # m/s
max_angular_velocity: 2.0  # rad/s
action_timeout_sec: 5.0
enable_safety_checks: true
action_buffer_size: 10
execution_rate: 50.0  # Hz (control loop)
```

**Safety Features**:
- Velocity limiting
- Action timeout
- Emergency stop
- Collision avoidance (future)

---

### 3. Planner Node

High-level task planning and coordination.

**Executable**: `planner_node`

**Features**:
- Task queue management
- Priority-based scheduling
- Execution monitoring
- Retry logic for failed tasks

**Topics**:
- Subscribes:
  - `/vla/command` - Explicit task commands
  - `/audio/transcription` - Parse voice commands
  - `/vla/execution_feedback` - Monitor execution
- Publishes:
  - `/vla/plan_status` - Planning status
  - `/jetson/health/vla_planner` - Module health

**Parameters**:
```yaml
task_queue_size: 10
max_retries: 3
task_timeout_sec: 60.0
planning_rate: 1.0  # Hz
```

**Task States**:
- `PENDING` - Waiting in queue
- `ACTIVE` - Currently executing
- `COMPLETE` - Successfully completed
- `FAILED` - Failed after retries
- `CANCELLED` - User cancelled

---

## Usage

### Build

```bash
cd /home/nano/src/jetson-orin-nano
colcon build --packages-select vla_planner --symlink-install
source install/setup.bash
```

### Launch All Nodes

```bash
ros2 launch vla_planner vla_planner.launch.py
```

### Launch Individual Nodes

```bash
# VLA controller
ros2 run vla_planner vla_controller_node

# Action executor
ros2 run vla_planner action_executor_node

# Planner
ros2 run vla_planner planner_node
```

### Send Commands

```bash
# Explicit command
ros2 topic pub /vla/command std_msgs/msg/String "data: 'move forward 1 meter'" -1

# Or use voice commands via audio transcription
# (automatically parsed by planner)
```

### Test

```bash
cd src/modules/vla_planner/test
python3 test_node_imports.py
```

---

## Integration

### With Existing System

**Input Sources**:
- `sensor_sync_node` - Synchronized sensor data
- `audio_feature_extractor_node` - Audio features
- `chassis_controller_node` - Robot state
- `speech_recognition_node` - Voice commands

**Outputs**:
- `chassis_controller_node` - Velocity commands
- `power_manager_node` - GPU power requests

### Full Pipeline

```
Cameras → sensor_sync → VLA Controller → Action Executor → Motors
                              ↑                 ↓
Audio → audio_features ────┘           execution_feedback
                                              ↓
Speech → transcription → Planner ───────────┘
```

---

## VLA Model Integration

### Placeholder Mode (Current)

The default `placeholder` mode provides a mock VLA implementation for testing:
- Returns simple forward motion
- Responds to basic voice commands ("stop", "turn")
- No GPU required

### OpenVLA Integration (Future)

To integrate OpenVLA:

```bash
# Install OpenVLA
pip install openvla

# Download model
python -c "from openvla import OpenVLA; OpenVLA.download('openvla-7b')"

# Configure node
ros2 param set /vla_controller_node model_type openvla
ros2 param set /vla_controller_node model_path /path/to/openvla-7b
```

### RT-1 Integration (Future)

To integrate RT-1:

```bash
# Install RT-1
pip install tensorflow tensorflow_hub
# Download RT-1 model from TensorFlow Hub

# Configure node
ros2 param set /vla_controller_node model_type rt1
ros2 param set /vla_controller_node model_path /path/to/rt1
```

---

## Configuration

### Action Format

Actions are represented as Float32MultiArray:

```python
# Mobile base control (3D)
[linear_x, linear_y, angular_z]

# With gripper (4D)
[linear_x, linear_y, angular_z, gripper]

# Full 7-DOF arm (10D)
[linear_x, linear_y, angular_z, joint1, joint2, joint3, joint4, joint5, joint6, gripper]
```

### Performance Tuning

**Low latency** (real-time control):
```yaml
inference_rate: 20.0  # 50ms latency
execution_rate: 100.0  # 10ms control loop
action_buffer_size: 5
```

**Balanced** (default):
```yaml
inference_rate: 10.0  # 100ms latency
execution_rate: 50.0  # 20ms control loop
action_buffer_size: 10
```

**High throughput** (batch processing):
```yaml
inference_rate: 5.0  # 200ms latency
execution_rate: 50.0  # 20ms control loop
action_buffer_size: 20
```

---

## Safety

### Safety Checks

The action executor implements multiple safety layers:

1. **Velocity Limiting**: Actions clamped to max velocities
2. **Action Timeout**: Abort if execution exceeds timeout
3. **Emergency Stop**: Immediate stop on failure
4. **Buffer Management**: Drop old actions if buffer full

### Emergency Stop

```bash
# Stop all actions
ros2 topic pub /vla/command std_msgs/msg/String "data: 'stop'" -1

# Or kill action executor (will stop robot)
ros2 node kill /action_executor_node
```

---

## Future Enhancements

### Planned Features

1. **Model Integration** - OpenVLA, RT-1, RT-2 support
2. **Collision Avoidance** - Integrate with navigation stack
3. **Visual Servoing** - Fine-grained manipulation control
4. **Multi-Task Learning** - Learn from demonstrations
5. **Sim-to-Real Transfer** - Train in simulation, deploy on robot

### Advanced Planning

- **Hierarchical Planning** - Break complex tasks into subtasks
- **Plan Visualization** - Show planned trajectory in RViz
- **Plan Optimization** - Optimize for time, energy, safety
- **Plan Repair** - Dynamically adjust plan on failures

---

## Troubleshooting

### No Actions Published

**Check**:
1. VLA controller is running: `ros2 node list | grep vla_controller`
2. Vision features available: `ros2 topic echo /sensor_fusion/vlm_features`
3. Model loaded successfully: Check logs for errors

### Actions Not Executing

**Check**:
1. Action executor is running: `ros2 node list | grep action_executor`
2. Actions are being received: `ros2 topic echo /vla/actions`
3. Safety limits not exceeded: Check velocity parameters

### High GPU Usage

**Solutions**:
- Use smaller model (`model_type: placeholder` for testing)
- Reduce inference rate (`inference_rate: 5.0`)
- Enable model quantization (future)

### Poor Control Performance

**Solutions**:
- Increase execution rate (`execution_rate: 100.0`)
- Tune safety limits (`max_linear_velocity: 1.0`)
- Reduce action buffer lag (`action_buffer_size: 5`)
- Improve vision/audio features quality

---

## Dependencies

### Python Packages (Optional)

For VLA models:
```bash
# OpenVLA
pip install openvla transformers torch

# RT-1/RT-2
pip install tensorflow tensorflow_hub

# General ML
pip install numpy scipy
```

### ROS 2 Dependencies

- `rclpy`
- `std_msgs`
- `geometry_msgs`
- `sensor_msgs`
- `nav_msgs`
- `custom_msgs` (Phases 1-3)
- `isaac_utils`

---

## License

MIT License - See repository LICENSE file for details.

---

**Last Updated**: 2026-01-27  
**Status**: Phase 4 Complete  
**Next**: Phase 5 (Distributed Deployment)
