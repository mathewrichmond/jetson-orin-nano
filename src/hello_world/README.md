# Hello World Package

Simple ROS 2 package to verify the Isaac robot system is working.

## Build

```bash
cd ~/ros2_ws
colcon build --packages-select hello_world
source install/setup.bash
```

## Run

```bash
# Run node directly
ros2 run hello_world hello_world_node

# Or launch
ros2 launch hello_world hello_world.launch.py
```

## Test

In another terminal:

```bash
# Listen to messages
ros2 topic echo /hello_world/message

# Check node is running
ros2 node list

# Check topics
ros2 topic list
```

## Expected Output

The node should:
- Start successfully
- Print "Hello World Node started!"
- Print "Isaac robot system is operational!"
- Publish messages every 2 seconds
- Log published messages

This verifies:
- ✅ ROS 2 is installed and working
- ✅ Python environment is set up
- ✅ Package build system works
- ✅ Node can publish messages
- ✅ System is operational
