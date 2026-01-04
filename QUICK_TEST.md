# Quick Test - Hello World

## Test Development Install

```bash
# 1. Source ROS 2
source /opt/ros/humble/setup.bash

# 2. Build hello_world package
cd ~/ros2_ws
colcon build --packages-select hello_world
source install/setup.bash

# 3. Run hello world node
ros2 run hello_world hello_world_node
```

Expected: Node starts and publishes messages every 2 seconds.

## Verify Installation

```bash
# Check package is installed
ros2 pkg list | grep hello_world

# Check node exists
ros2 run hello_world hello_world_node --help

# Check topics (in another terminal)
ros2 topic list
ros2 topic echo /hello_world/message
```

## Success Criteria

✅ Package builds without errors  
✅ Node runs and publishes messages  
✅ Topics are visible  
✅ No import errors  

If all checks pass, development environment is working!
