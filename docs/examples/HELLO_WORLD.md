# Hello World Example

## Overview

The `hello_world` package is a simple ROS 2 package that verifies the Isaac robot system is working correctly.

## Quick Test

```bash
# Build the package
cd ~/ros2_ws
colcon build --packages-select hello_world
source install/setup.bash

# Run the node
ros2 run hello_world hello_world_node
```

Or use the convenience script:

```bash
./run_hello_world.sh
```

## What It Does

The hello_world node:
- Starts a ROS 2 node
- Publishes messages every 2 seconds to `/hello_world/message`
- Logs messages to console
- Verifies ROS 2 is working

## Expected Output

```
[INFO] [hello_world]: Hello World Node started!
[INFO] [hello_world]: Isaac robot system is operational!
[INFO] [hello_world]: Published: Hello from Isaac Robot! Message #0
[INFO] [hello_world]: Published: Hello from Isaac Robot! Message #1
...
```

## Test in Another Terminal

```bash
# Source ROS 2
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Listen to messages
ros2 topic echo /hello_world/message

# Check node is running
ros2 node list

# Check topics
ros2 topic list
```

## What This Verifies

✅ ROS 2 is installed and working
✅ Python environment is set up
✅ Package build system works
✅ Node can publish messages
✅ System is operational

## Package Structure

```
src/hello_world/
├── package.xml          # Package metadata
├── setup.py            # Python package setup
├── CMakeLists.txt      # Build configuration
├── hello_world/        # Python package
│   ├── __init__.py
│   └── hello_world_node.py  # Main node
└── launch/             # Launch files
    └── hello_world.launch.py
```

## Next Steps

After verifying hello_world works:
1. Build system_monitor: `colcon build --packages-select system_monitor`
2. Launch system monitor: `ros2 launch system_monitor system_monitor.launch.py`
3. Start developing your own packages!
