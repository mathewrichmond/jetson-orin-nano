# Platform Bridge

Integration package for connecting the Jetson Orin Nano robot to the cross-platform robotics system.

## Overview

The platform bridge adapts the Jetson's ROS 2-based architecture to the platform's Zenoh-based communication layer, enabling:

- **Multi-robot coordination** through the dispatch service
- **Digital twin integration** with TSDF/ESDF map merging
- **Mission-based control** replacing direct VLA commands
- **PTP time synchronization** for precise sensor fusion

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    Jetson Robot (ROS 2)                     │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  ┌───────────────┐        ┌─────────────────────────────┐  │
│  │ ROS 2 Nodes   │        │   Platform Bridge           │  │
│  │               │        │                             │  │
│  │ - nvblox      │───────▶│ JetsonPerception           │──┼──▶ Zenoh
│  │ - RealSense   │        │ - TSDF/ESDF serialization  │  │   /robot/jetson-01/
│  │ - Visual SLAM │        │ - Pose publishing          │  │   perception/*
│  │               │        │ - Image forwarding         │  │
│  └───────────────┘        └─────────────────────────────┘  │
│                                                             │
│  ┌───────────────┐        ┌─────────────────────────────┐  │
│  │ VLA Controller│◀───────│ JetsonControl              │◀─┼── Zenoh
│  │ - Model       │        │ - Mission translation      │  │   /robot/jetson-01/
│  │ - Actions     │        │ - Status reporting         │  │   mission/command
│  │               │        │ - Subsystem commands       │  │
│  └───────────────┘        └─────────────────────────────┘  │
│                                                             │
│  ┌───────────────────────────────────────────────────────┐  │
│  │              JetsonRobot (Main)                       │  │
│  │  - Manages lifecycle                                  │  │
│  │  - Registers capabilities                             │  │
│  │  - Coordinates perception & control                   │  │
│  └───────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────┘
```

## Components

### 1. JetsonRobot (`jetson_robot.py`)

Main entry point that:
- Loads capability configuration
- Initializes Zenoh session
- Manages perception and control adapters
- Registers with platform dispatch service

### 2. JetsonPerception (`jetson_perception.py`)

Perception adapter that:
- Subscribes to ROS 2 perception topics (nvblox, cameras, poses)
- Converts to platform message formats (TSDFUpdate, ESDF, RobotPose)
- Publishes to Zenoh with PTP timestamps
- Handles image compression and serialization

### 3. JetsonControl (`jetson_control.py`)

Control adapter that:
- Subscribes to platform mission commands via Zenoh
- Forwards commands to VLA controller
- Monitors VLA execution feedback
- Reports mission status/results to platform

### 4. Configuration

- **`config/capability.yaml`**: Robot capability descriptor
  - Sensors: RealSense cameras (left/right), IMU, microphone
  - Actuators: Pan-tilt units, differential drive base
  - Capabilities: 3D vision, stereo, navigation, detection, etc.

- **`config/bridge_config.yaml`**: Topic mapping configuration
  - ROS 2 to Zenoh topic mappings
  - QoS profiles
  - Compression settings
  - Performance tuning

## Installation

### Prerequisites

```bash
# In robotics repo (platform)
cd /path/to/robotics
pip install -e common/robotics_common
pip install -e common/robotics_sdk

# Install Zenoh Python
pip install eclipse-zenoh
```

### Jetson Setup

```bash
cd /path/to/jetson-orin-nano/src/platform_bridge

# Install package
pip install -e .

# Optional: Install compression libraries
pip install zstandard lz4
```

### PTP Setup (Required for time sync)

See [`docs/setup/PTP_SETUP.md`](../../docs/setup/PTP_SETUP.md) for detailed instructions.

Quick setup:
```bash
# Install linuxptp
sudo apt install linuxptp

# Configure PTP (edit /etc/linuxptp/ptp4l.conf)
# Start PTP daemon
sudo systemctl start ptp4l
sudo systemctl enable ptp4l
```

## Usage

### Standalone Mode

```bash
# Set robot ID (or use default "jetson-01")
export HOST_ID=jetson-01

# Optional: Set Zenoh router address
export ZENOH_ROUTER=tcp/192.168.1.100:7447

# Run platform bridge
python3 -m platform_bridge.jetson_robot
```

### With ROS 2 Launch

```bash
# Add to existing launch file
ros2 launch isaac_robot graph.launch.py

# In separate terminal, start platform bridge
python3 -m platform_bridge.jetson_robot
```

### Systemd Service (Production)

```bash
# Create systemd service
sudo cp config/systemd/platform_bridge.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable platform_bridge.service
sudo systemctl start platform_bridge.service

# View logs
journalctl -u platform_bridge.service -f
```

## Testing

### 1. Verify Zenoh Connection

```bash
# In one terminal (on server or laptop)
zenoh-cli sub "/robot/**"

# In another terminal (on Jetson)
python3 -m platform_bridge.jetson_robot

# You should see capability registration message
```

### 2. Test Capability Registration

```python
# On server (Python)
import zenoh

session = zenoh.open(zenoh.Config())
replies = session.get("/capabilities/robot/*")
for reply in replies:
    print(reply.payload.decode('utf-8'))
```

### 3. Send Test Mission

```python
# On server (Python)
import zenoh
import json

session = zenoh.open(zenoh.Config())

mission = {
    "mission_id": "test-001",
    "command": "Look at the door",
    "required_capabilities": ["pan_tilt_cameras"],
    "priority": 3,
    "timeout": 30.0,
    "parameters": "{}"
}

session.put(
    "/robot/jetson-01/mission/command",
    json.dumps(mission).encode('utf-8')
)

# Monitor status
def status_callback(sample):
    print(f"Status: {sample.payload.decode('utf-8')}")

session.declare_subscriber(
    "/robot/jetson-01/mission/status",
    status_callback
)
```

## Configuration

### Environment Variables

- `HOST_ID`: Robot identifier (default: "jetson-01")
- `ZENOH_ROUTER`: Zenoh router address (default: auto-discovery)
- `ROBOT_CONFIG`: Path to capability.yaml (default: `config/capability.yaml`)

### Capability Configuration

Edit [`config/capability.yaml`](config/capability.yaml) to:
- Update robot_id and robot_name
- Adjust sensor specifications (resolution, frame rate, etc.)
- Modify actuator limits (velocity, pan/tilt ranges)
- Add/remove capabilities

### Bridge Configuration

Edit [`config/bridge_config.yaml`](config/bridge_config.yaml) to:
- Add/remove topic mappings
- Adjust QoS profiles
- Enable/disable compression
- Tune performance settings

## Troubleshooting

### Robot not appearing in dispatch service

1. Check Zenoh connection:
   ```bash
   # On Jetson
   zenoh-cli scout
   ```

2. Verify capability registration:
   ```bash
   # On server
   zenoh-cli sub "/capabilities/robot/*"
   ```

3. Check logs:
   ```bash
   journalctl -u platform_bridge.service -n 100
   ```

### Mission commands not reaching VLA controller

1. Verify VLA controller is running:
   ```bash
   ros2 node list | grep vla_controller
   ```

2. Check topic connectivity:
   ```bash
   ros2 topic echo /vla/command
   ```

3. Monitor bridge logs for errors

### TSDF not updating in digital twin

1. Check nvblox is running and publishing
2. Verify perception adapter is receiving data
3. Check Zenoh topic:
   ```bash
   zenoh-cli sub "/robot/jetson-01/perception/tsdf"
   ```

### Time synchronization issues

1. Verify PTP is running:
   ```bash
   sudo systemctl status ptp4l
   ```

2. Check PTP sync status:
   ```bash
   pmc -u -b 0 'GET CURRENT_DATA_SET'
   ```

3. Check offset_from_master < 100μs

## Development

### Adding New Sensors

1. Update `config/capability.yaml` with sensor specs
2. Add ROS 2 subscription in `JetsonPerception`
3. Implement serialization/forwarding logic
4. Update `config/bridge_config.yaml` if needed

### Adding New Capabilities

1. Add capability string to `config/capability.yaml`
2. Update dispatch service capability matching if custom logic needed
3. Implement capability-specific logic in control adapter

### Debugging

Enable debug logging:
```python
import logging
logging.basicConfig(level=logging.DEBUG)
```

Or set environment variable:
```bash
export ROS_LOG_LEVEL=debug
```

## See Also

- [Platform Architecture](../../../robotics/docs/ARCHITECTURE.md)
- [Integration Plan](~/.cursor/plans/jetson_platform_integration_*.plan.md)
- [TSDF Message Spec](../../../robotics/interfaces/sensor_data/msg/TSDFUpdate.msg)
- [Mission Message Spec](../../../robotics/interfaces/missions/msg/Mission.msg)
