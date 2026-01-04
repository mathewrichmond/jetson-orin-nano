# Raspberry Pi Sub-module Integration

## Overview

This guide covers integrating Raspberry Pi modules as sub-controllers for the Isaac robot system.

## Use Cases

Raspberry Pi modules can be used for:
- Distributed sensor processing
- Local motor control
- Auxiliary computing tasks
- Network-based subsystems

## Communication

### ROS 2 Over Network

1. **Configure ROS 2 Discovery**: Set `ROS_DISCOVERY_SERVER` environment variable
2. **Network Setup**: Ensure Pi and Jetson are on same network
3. **Firewall**: Configure firewall rules if needed

### Direct Network Connection

- SSH access: `ssh pi@<pi-hostname>.local`
- ROS 2 topics: Accessible across network with proper configuration

## Setup Process

1. **Flash Raspberry Pi**: Install Ubuntu 22.04 or Raspberry Pi OS
2. **Install ROS 2**: Install ROS 2 Humble on Raspberry Pi
3. **Network Configuration**: Configure hostname and network
4. **ROS 2 Configuration**: Set up ROS 2 discovery and communication
5. **Integration**: Create ROS 2 packages for Pi-specific functionality

## Configuration Files

Raspberry Pi module configurations:
- `config/hardware/raspberry_pi_modules/` - Module-specific configs
- `hardware/raspberry_pi_modules/` - Setup scripts and documentation

## ROS 2 Discovery Server (Optional)

For better performance with multiple nodes:
```bash
# On Jetson (discovery server)
fastdds discovery --server-id 0

# On Raspberry Pi (client)
export ROS_DISCOVERY_SERVER=192.168.1.100:11811
```

## Testing

```bash
# From Jetson, check if Pi nodes are visible
ros2 node list

# Check topics from Pi
ros2 topic list

# Monitor Pi topics
ros2 topic echo /pi/sensor_data
```

## Troubleshooting

- **Nodes not visible**: Check network connectivity, ROS_DOMAIN_ID
- **Topics not accessible**: Verify firewall rules, ROS 2 discovery
- **Performance issues**: Consider using ROS 2 discovery server
- **Connection issues**: Check hostname resolution, network configuration

## Future Enhancements

- Automated Pi module discovery
- Health monitoring of Pi modules
- Over-the-air updates for Pi modules
- Redundancy and failover support

