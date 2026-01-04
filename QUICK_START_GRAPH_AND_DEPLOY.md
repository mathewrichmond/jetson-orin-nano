# Quick Start - Graph Selection & Deployment

## Graph Selection

### Select Graph at Startup
```bash
# Set graph (saves to config file)
./scripts/utils/select_graph.sh minimal
./scripts/utils/select_graph.sh full
./scripts/utils/select_graph.sh robot

# Check current selection
./scripts/utils/get_graph.sh
```

### Start with Selected Graph
```bash
# Uses config file or default
./scripts/system/start_robot.sh

# Or override with environment variable
ROBOT_GRAPH=full ./scripts/system/start_robot.sh
```

### Launch Directly
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch isaac_robot minimal.launch.py
ros2 launch isaac_robot full.launch.py
ros2 launch isaac_robot robot.launch.py
```

## Quick Deployment

### Local Development
```bash
# 1. Make code changes
vim src/system_monitor/system_monitor/system_monitor_node.py

# 2. Rebuild locally (fast)
./scripts/deployment/rebuild_local.sh

# 3. Test
source ~/ros2_ws/install/setup.bash
ros2 launch isaac_robot minimal.launch.py
```

### Deploy to Remote Target
```bash
# Quick deploy (rsync - fastest)
./scripts/deployment/quick_deploy.sh

# Or full deploy (tar/scp - more reliable)
./scripts/deployment/deploy_dev.sh
```

## Systemd Service

### Set Graph for Service
```bash
# Edit service to set graph
sudo systemctl edit isaac-robot.service

# Add:
[Service]
Environment="ROBOT_GRAPH=full"

# Restart
sudo systemctl restart isaac-robot.service
```

### Check Service Status
```bash
sudo systemctl status isaac-robot.service
sudo journalctl -u isaac-robot.service -f
```

## Priority Order

1. **Environment Variable** (`ROBOT_GRAPH`) - Highest
2. **Config File** (`config/robot/selected_graph.txt`)
3. **Default** (`minimal`)

## Examples

### Development Workflow
```bash
# Select minimal graph for testing
./scripts/utils/select_graph.sh minimal

# Make changes and rebuild
vim src/system_monitor/system_monitor/system_monitor_node.py
./scripts/deployment/rebuild_local.sh

# Test
source ~/ros2_ws/install/setup.bash
./scripts/system/start_robot.sh
```

### Production Deployment
```bash
# Select full graph
./scripts/utils/select_graph.sh full

# Deploy
./scripts/deployment/quick_deploy.sh

# Restart service
ssh nano@isaac.local
sudo systemctl restart isaac-robot.service
```
