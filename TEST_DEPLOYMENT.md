# Testing Deployment & Graph Selection

## Test Graph Selection

```bash
# Test graph selection script
./scripts/utils/get_graph.sh
./scripts/utils/select_graph.sh minimal
./scripts/utils/select_graph.sh full
./scripts/utils/select_graph.sh robot

# Test environment variable override
ROBOT_GRAPH=full ./scripts/utils/get_graph.sh
```

## Test Launch Files

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Test minimal
ros2 launch isaac_robot minimal.launch.py

# Test full
ros2 launch isaac_robot full.launch.py

# Test robot (with config)
ros2 launch isaac_robot robot.launch.py
```

## Test Start Script

```bash
# Test with different graphs
./scripts/utils/select_graph.sh minimal
./scripts/system/start_robot.sh

# Or with env var
ROBOT_GRAPH=full ./scripts/system/start_robot.sh
```

## Test Quick Deploy (Local)

```bash
# Make a test change
echo "# Test" >> src/system_monitor/README.md

# Rebuild locally
./scripts/deployment/rebuild_local.sh

# Verify
source ~/ros2_ws/install/setup.bash
ros2 pkg list | grep system_monitor
```

## Expected Results

✅ Graph selection works  
✅ Launch files start correctly  
✅ Start script uses selected graph  
✅ Rebuild script works  
✅ Quick deploy syncs changes  

All tests should pass!
