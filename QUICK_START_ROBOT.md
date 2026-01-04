# Quick Start - Robot System

## Launch Robot System

### Minimal System (System Monitor Only)
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch isaac_robot minimal.launch.py
```

### Full System (All Components)
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch isaac_robot full.launch.py
```

## Monitor System

In another terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Check nodes
ros2 node list

# Check topics
ros2 topic list

# Monitor system status
ros2 topic echo /system/status

# Monitor CPU temperature
ros2 topic echo /system/temperature/cpu

# Monitor alerts
ros2 topic echo /system/alerts
```

## Graph Configurations

Graph configs define the robot topology:
- `config/robot/robot_graph.yaml` - Default
- `config/robot/minimal_graph.yaml` - Minimal
- `config/robot/full_graph.yaml` - Full

See `docs/robot/GRAPH_CONFIG.md` for details.

## Verify Installation

```bash
# Check packages
ros2 pkg list | grep -E "(system_monitor|isaac_robot)"

# Check launch files
ros2 launch isaac_robot --help
```

## Troubleshooting

If launch fails:
1. Ensure packages are built: `cd ~/ros2_ws && colcon build`
2. Source workspace: `source ~/ros2_ws/install/setup.bash`
3. Check ROS 2: `ros2 --help`
