# Robot Graph Selection

## Overview

The Isaac robot system supports multiple graph configurations that define which nodes are launched. You can select the graph at startup using several methods.

## Available Graphs

- **`minimal`** - Only essential nodes (system monitor)
- **`full`** - Complete system with all components
- **`robot`** - Configurable graph with custom settings

## Selection Methods

### 1. Environment Variable (Highest Priority)

```bash
export ROBOT_GRAPH=full
./scripts/system/start_robot.sh
```

Or inline:
```bash
ROBOT_GRAPH=full ./scripts/system/start_robot.sh
```

### 2. Config File

Edit the config file:
```bash
echo "full" > config/robot/selected_graph.txt
```

Or use the helper script:
```bash
./scripts/utils/select_graph.sh full
```

### 3. Systemd Service Override

Edit the service file or create an override:
```bash
sudo systemctl edit isaac-robot.service
```

Add:
```ini
[Service]
Environment="ROBOT_GRAPH=full"
```

Then restart:
```bash
sudo systemctl restart isaac-robot.service
```

## Quick Reference

### Select Graph
```bash
# Set to minimal (default)
./scripts/utils/select_graph.sh minimal

# Set to full
./scripts/utils/select_graph.sh full

# Set to robot
./scripts/utils/select_graph.sh robot
```

### Check Current Selection
```bash
./scripts/utils/get_graph.sh
```

### Start with Specific Graph
```bash
# Using environment variable
ROBOT_GRAPH=full ./scripts/system/start_robot.sh

# Or launch directly
ROBOT_GRAPH=full ros2 launch isaac_robot full.launch.py
```

## Priority Order

1. **Environment Variable** (`ROBOT_GRAPH`) - Highest priority
2. **Config File** (`config/robot/selected_graph.txt`)
3. **Default** (`minimal`)

## Examples

### Development - Minimal Graph
```bash
./scripts/utils/select_graph.sh minimal
./scripts/system/start_robot.sh
```

### Production - Full Graph
```bash
./scripts/utils/select_graph.sh full
sudo systemctl restart isaac-robot.service
```

### Testing - Custom Graph
```bash
ROBOT_GRAPH=robot ros2 launch isaac_robot robot.launch.py
```

## Graph Configurations

Graph configurations are defined in:
- `config/robot/minimal_graph.yaml`
- `config/robot/full_graph.yaml`
- `config/robot/robot_graph.yaml`

See `docs/robot/GRAPH_CONFIG.md` for details on graph structure.

## Troubleshooting

### Check Current Graph
```bash
./scripts/utils/get_graph.sh
cat config/robot/selected_graph.txt
```

### Verify Launch
```bash
# Check what will be launched
ROBOT_GRAPH=$(./scripts/utils/get_graph.sh)
echo "Will launch: $ROBOT_GRAPH"

# Test launch
ros2 launch isaac_robot ${ROBOT_GRAPH}.launch.py
```

### Reset to Default
```bash
./scripts/utils/select_graph.sh minimal
```
