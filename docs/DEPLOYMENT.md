# Deployment Guide - Isaac Robot System

**Status**: Phase 5 ✅ | Multi-environment deployment with network configuration

---

## Overview

The Isaac robot system supports multiple deployment configurations:

| Deployment | Description | Use Case | Hosts |
|------------|-------------|----------|-------|
| **local** | Single Jetson Orin Nano | Development, testing | 1 |
| **dual_compute** | Pi 4 + Jetson | Production robot | 2 |
| **test** | Mock hardware | CI/CD, unit testing | 1 |
| **sim** | Gazebo/Isaac Sim | Algorithm development | 1 |
| **cloud_dev** | Remote development | Cloud-based dev | 2+ |

---

## Quick Start

### Local Deployment (Current Setup)

**Your current setup** - Everything runs on Jetson Orin Nano:

```bash
cd /home/nano/src/jetson-orin-nano

# Build everything
colcon build --symlink-install
source install/setup.bash

# Launch full system
ros2 launch isaac_robot graph.launch.py graph:=modular_graph.yaml group:=all

# Or use deployment script
./scripts/deployment/deploy.sh local launch
```

**No network configuration needed** - Uses localhost/multicast.

---

## Dual-Compute Deployment (Future)

**When you add the Raspberry Pi** - Distributed across Pi 4 + Jetson:

### Architecture

```
Raspberry Pi 4                    Jetson Orin Nano
├── Chassis Control               ├── Vision Pipeline
│   ├── IMU Processor             │   ├── Visual SLAM
│   ├── Chassis Controller        │   ├── Camera Calibration
│   └── Calibration Manager       │   └── Vision Pipeline
├── Power Management              ├── Audio Pipeline
│   └── Battery Monitor           │   ├── Feature Extractor
├── Hardware Drivers              │   ├── Speech Recognition
│   ├── iRobot Serial             │   └── Audio Pipeline
│   └── PHAT Motor Controller     ├── VLA Planner
                                  │   ├── VLA Controller
                                  │   ├── Action Executor
                                  │   └── Planner
                                  ├── Power Management
                                  │   ├── Power Manager
                                  │   └── GPIO Controller
                                  └── Hardware Drivers
                                      ├── RealSense Camera
                                      └── nvblox Processor

         FastDDS Discovery Server (on Jetson)
         ├── Cross-host communication
         └── Network: 192.168.1.0/24
```

### Network Setup (Requires sudo)

**Step 1: Configure static IPs**

```bash
# On Raspberry Pi
sudo ./scripts/network/setup_network.sh dual_compute

# On Jetson Orin Nano
sudo ./scripts/network/setup_network.sh dual_compute
```

This configures:
- Pi: `192.168.1.10/24`
- Jetson: `192.168.1.20/24`
- FastDDS discovery server on Jetson
- Firewall rules for ROS 2 DDS traffic

**Step 2: Verify connectivity**

```bash
# From Pi
ping 192.168.1.20  # Should reach Jetson

# From Jetson
ping 192.168.1.10  # Should reach Pi
```

### Deployment

**On Raspberry Pi:**
```bash
cd /home/pi/src/jetson-orin-nano
source install/setup.bash

# Launch Pi nodes
ros2 launch isaac_robot distributed.launch.py deployment:=dual_compute host:=pi

# Or use deployment script
./scripts/deployment/deploy.sh dual_compute launch
```

**On Jetson Orin Nano:**
```bash
cd /home/nano/src/jetson-orin-nano
source install/setup.bash

# Launch Jetson nodes
ros2 launch isaac_robot distributed.launch.py deployment:=dual_compute host:=jetson

# Or use deployment script
./scripts/deployment/deploy.sh dual_compute launch
```

**Verify cross-host communication:**
```bash
# From either host
ros2 node list  # Should see nodes from both hosts
ros2 topic list  # Should see topics from both hosts
ros2 topic hz /rpi/imu/filtered  # Should see data from Pi
ros2 topic hz /vision/global_pose  # Should see data from Jetson
```

---

## Test Deployment

**For CI/CD and unit testing** - No real hardware required:

```bash
# Set environment
export DEPLOYMENT=test
export ROS_DOMAIN_ID=42  # Separate domain from production

# Launch with mock hardware
ros2 launch isaac_robot test.launch.py mock_hardware:=true headless:=true

# Or use deployment script
./scripts/deployment/deploy.sh test launch
```

**Features**:
- Mock GPIO, cameras, IMU, motors
- No sudo required
- Isolated ROS domain (42)
- Headless operation
- Fast startup for CI

---

## Simulation Deployment

**For algorithm development** - Gazebo or Isaac Sim:

### Gazebo Simulation

```bash
# Install Gazebo (if not already installed)
sudo apt install ros-humble-gazebo-ros-pkgs

# Launch simulation
ros2 launch isaac_robot graph.launch.py \
    graph:=modular_graph.yaml \
    group:=all \
    deployment:=sim \
    use_sim_time:=true

# Or use deployment script
./scripts/deployment/deploy.sh sim launch
```

### Isaac Sim (Future)

```bash
# Requires NVIDIA Isaac Sim installation
# Launch Isaac Sim with robot model
# Then launch nodes
ros2 launch isaac_robot graph.launch.py \
    graph:=modular_graph.yaml \
    group:=all \
    deployment:=sim \
    use_sim_time:=true
```

---

## Cloud Development Deployment

**For remote development** - Work from anywhere, robot at home/lab:

### Prerequisites

1. **VPN or SSH Tunnel**: Secure connection to robot
2. **Static IP or DDNS**: For reliable robot access
3. **Port Forwarding**: Ports 11811 (FastDDS), 8765 (Foxglove), 22 (SSH)

### Setup

**Step 1: Configure robot for remote access**

On robot (Jetson):
```bash
# Enable FastDDS discovery server
sudo ./scripts/network/setup_network.sh cloud_dev

# Start robot nodes
./scripts/deployment/deploy.sh cloud_dev launch
```

**Step 2: Connect from development machine**

On dev machine:
```bash
# Set up SSH tunnel
ssh -L 11811:localhost:11811 nano@robot.example.com -N &

# Set FastDDS to use tunnel
export FASTRTPS_DEFAULT_PROFILES_FILE=/path/to/fastdds_client.xml

# Launch development tools
ros2 launch isaac_robot cloud_dev.launch.py remote_host:=robot.example.com

# Open Foxglove
# Connect to ws://robot.example.com:8765
```

### Development Workflow

```bash
# 1. Develop VLA models locally
#    Edit vla_controller_node.py on dev machine

# 2. Deploy to robot
scp -r src/modules/vla_planner/ nano@robot.example.com:~/src/jetson-orin-nano/src/modules/

# 3. Build on robot
ssh nano@robot.example.com "cd ~/src/jetson-orin-nano && colcon build --packages-select vla_planner"

# 4. Test remotely
#    Monitor via Foxglove or rostopic
```

---

## Deployment Configurations

### Configuration Files

All deployment configs are in `config/deployment/`:

- `local.yaml` - Single Jetson (current)
- `dual_compute.yaml` - Pi + Jetson (future)
- `test.yaml` - Mock hardware
- `sim.yaml` - Simulation
- `cloud_dev.yaml` - Remote development

### Graph Configurations

Host-specific graphs in `config/robot/`:

- `modular_graph.yaml` - Full system (all modules)
- `pi_graph.yaml` - Raspberry Pi nodes only
- `jetson_graph.yaml` - Jetson nodes only
- `minimal_graph.yaml` - Minimal system
- `stable_graph.yaml` - Stable/conservative config

---

## Network Configuration

### Local Mode (Default)

- **Discovery**: Multicast (automatic)
- **Network**: Localhost or LAN
- **Sudo**: Not required
- **Configuration**: None needed

### Distributed Mode (Dual-Compute)

- **Discovery**: FastDDS server
- **Network**: Static IPs on private subnet
- **Sudo**: Required for network setup
- **Configuration**: Run `setup_network.sh`

**Network Layout**:
```
192.168.1.0/24 subnet
├── 192.168.1.10 - Raspberry Pi (isaac-pi.local)
├── 192.168.1.20 - Jetson Orin Nano (isaac-jetson.local)
└── 192.168.1.1 - Gateway/Router
```

**Ports Used**:
- `7400-7500/udp` - ROS 2 DDS traffic
- `11811/tcp` - FastDDS discovery server
- `8765/tcp` - Foxglove Bridge
- `22/tcp` - SSH

### Cloud Mode

- **Discovery**: FastDDS server with SSH tunnel
- **Network**: Public internet or VPN
- **Sudo**: Not required on dev machine
- **Configuration**: SSH tunnel setup

---

## Launch Commands Reference

### By Deployment Type

```bash
# Local (current setup)
ros2 launch isaac_robot graph.launch.py graph:=modular_graph.yaml group:=all

# Dual-compute - Pi
ros2 launch isaac_robot distributed.launch.py deployment:=dual_compute host:=pi

# Dual-compute - Jetson
ros2 launch isaac_robot distributed.launch.py deployment:=dual_compute host:=jetson

# Test environment
ros2 launch isaac_robot test.launch.py mock_hardware:=true

# Simulation
ros2 launch isaac_robot graph.launch.py graph:=modular_graph.yaml deployment:=sim use_sim_time:=true

# Cloud development
ros2 launch isaac_robot distributed.launch.py deployment:=cloud_dev
```

### By Module Group

```bash
# Full system (all modules)
ros2 launch isaac_robot graph.launch.py graph:=modular_graph.yaml group:=all

# Chassis control only
ros2 launch isaac_robot graph.launch.py graph:=modular_graph.yaml group:=chassis_control

# Vision pipeline only
ros2 launch isaac_robot graph.launch.py graph:=modular_graph.yaml group:=vision_pipeline

# Audio pipeline only
ros2 launch isaac_robot graph.launch.py graph:=modular_graph.yaml group:=audio_pipeline

# VLA planner only
ros2 launch isaac_robot graph.launch.py graph:=modular_graph.yaml group:=vla_planner

# Testing groups
ros2 launch isaac_robot graph.launch.py graph:=modular_graph.yaml group:=chassis_test
ros2 launch isaac_robot graph.launch.py graph:=modular_graph.yaml group:=vision_test
ros2 launch isaac_robot graph.launch.py graph:=modular_graph.yaml group:=audio_test
ros2 launch isaac_robot graph.launch.py graph:=modular_graph.yaml group:=vla_test
```

### Individual Modules

```bash
# Chassis control
ros2 launch chassis_control chassis_control.launch.py

# Vision pipeline
ros2 launch vision_pipeline vision_pipeline.launch.py

# Power management
ros2 launch power_management power_management.launch.py mock_mode:=true

# Audio pipeline
ros2 launch audio_pipeline audio_pipeline.launch.py

# VLA planner
ros2 launch vla_planner vla_planner.launch.py model_type:=placeholder
```

---

## Deployment Script Usage

The unified deployment script simplifies common operations:

```bash
# Setup deployment environment
./scripts/deployment/deploy.sh <deployment> setup

# Launch deployment
./scripts/deployment/deploy.sh <deployment> launch

# Check status
./scripts/deployment/deploy.sh <deployment> status

# Stop deployment
./scripts/deployment/deploy.sh <deployment> stop
```

**Examples**:
```bash
# Local development
./scripts/deployment/deploy.sh local launch

# Test environment
./scripts/deployment/deploy.sh test launch

# Dual-compute (auto-detects host from hostname)
./scripts/deployment/deploy.sh dual_compute launch
```

---

## Network Setup Process

### Automatic Network Configuration

The setup script now includes network configuration:

```bash
# For deployments that require network setup
export DEPLOYMENT=dual_compute
export SETUP_NETWORK=true

# Run unified setup (includes network configuration)
./setup.sh
```

### Manual Network Configuration

If you need to reconfigure network later:

```bash
# Configure network for dual-compute deployment
sudo ./scripts/network/setup_network.sh dual_compute

# Configure for cloud development
sudo ./scripts/network/setup_network.sh cloud_dev
```

### Network Configuration Details

The `setup_network.sh` script:
1. Parses deployment configuration
2. Determines network mode (local/distributed/cloud)
3. Configures static IPs (if required)
4. Sets up FastDDS discovery server
5. Configures firewall rules
6. Tests connectivity

**Requires sudo** for:
- Static IP configuration (netplan or NetworkManager)
- Firewall rules (ufw)
- Network interface management

---

## Transitioning to Dual-Compute

### Current State (Single Jetson)

You're currently running everything on the Jetson Orin Nano:
- All 15 nodes running locally
- No network configuration needed
- Simple multicast discovery

### Future State (Pi + Jetson)

When you're ready to add the Raspberry Pi:

**Step 1: Prepare Raspberry Pi**

```bash
# On Raspberry Pi
git clone <your-repo> /home/pi/src/jetson-orin-nano
cd /home/pi/src/jetson-orin-nano

# Run setup
export DEPLOYMENT=dual_compute
export SETUP_NETWORK=true
./setup.sh

# Build (Pi modules only)
colcon build --packages-select custom_msgs chassis_control power_management --symlink-install
```

**Step 2: Configure Network (both hosts)**

```bash
# On Pi
sudo ./scripts/network/setup_network.sh dual_compute

# On Jetson
sudo ./scripts/network/setup_network.sh dual_compute

# Verify connectivity
ping 192.168.1.20  # From Pi to Jetson
ping 192.168.1.10  # From Jetson to Pi
```

**Step 3: Start Discovery Server (Jetson)**

```bash
# On Jetson - start FastDDS discovery server
fastdds discovery --server-id 0 --port 11811 &

# Or it will auto-start with launch file
```

**Step 4: Launch Nodes**

```bash
# On Pi
ros2 launch isaac_robot distributed.launch.py deployment:=dual_compute host:=pi

# On Jetson
ros2 launch isaac_robot distributed.launch.py deployment:=dual_compute host:=jetson
```

**Step 5: Verify**

```bash
# From either host
ros2 node list  # Should see nodes from both hosts
ros2 topic echo /rpi/imu/filtered  # Pi node
ros2 topic echo /vision/global_pose  # Jetson node
```

---

## Cloud Development Setup

### When Developing Remotely

**Prerequisites**:
1. Robot accessible via public IP or VPN
2. SSH access to robot
3. Stable internet connection

**Setup on Robot** (one-time):

```bash
# On robot (Jetson)
cd /home/nano/src/jetson-orin-nano

# Configure for cloud access
sudo ./scripts/network/setup_network.sh cloud_dev

# Start robot nodes
./scripts/deployment/deploy.sh cloud_dev launch
```

**Setup on Dev Machine**:

```bash
# Clone repository
git clone <your-repo> ~/isaac-robot-dev
cd ~/isaac-robot-dev

# Install ROS 2 (if not already)
sudo apt install ros-humble-ros-base

# Build development tools only (no hardware drivers)
colcon build --packages-select isaac_robot custom_msgs --symlink-install

# Set up SSH tunnel
ssh -L 11811:localhost:11811 -L 8765:localhost:8765 nano@robot.example.com -N &

# Configure FastDDS client (points to tunnel)
export FASTRTPS_DEFAULT_PROFILES_FILE=$PWD/config/network/fastdds_client.xml

# Launch development tools
source install/setup.bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765

# Open Foxglove Studio
# Connect to ws://localhost:8765
```

**Development Workflow**:

```bash
# 1. Develop locally
code src/modules/vla_planner/vla_planner/vla_controller_node.py

# 2. Deploy to robot
rsync -av src/modules/vla_planner/ nano@robot.example.com:~/src/jetson-orin-nano/src/modules/vla_planner/

# 3. Build on robot (via SSH)
ssh nano@robot.example.com "cd ~/src/jetson-orin-nano && colcon build --packages-select vla_planner"

# 4. Restart module (via SSH)
ssh nano@robot.example.com "ros2 lifecycle set /vla/vla_controller restart"

# 5. Monitor results
ros2 topic echo /vla/actions
# Or view in Foxglove
```

---

## Testing Each Configuration

### Local Deployment Test

```bash
# Build
colcon build --symlink-install
source install/setup.bash

# Launch
ros2 launch isaac_robot graph.launch.py graph:=modular_graph.yaml group:=all

# Verify
ros2 node list | wc -l  # Should show ~15 nodes
ros2 topic list | wc -l  # Should show many topics
```

### Test Environment Validation

```bash
# Launch test environment
./scripts/deployment/deploy.sh test launch

# Run tests
cd src/modules/chassis_control/test && ./run_all_tests.sh
cd src/modules/vision_pipeline/test && python3 test_node_imports.py
# etc.

# Stop
./scripts/deployment/deploy.sh test stop
```

### Dual-Compute Readiness Check

**Before transitioning to dual-compute**:

```bash
# Check prerequisites
./scripts/deployment/preflight_check.sh dual_compute

# This would check:
# - Both hosts accessible
# - Static IPs configured
# - Firewall rules in place
# - FastDDS server running
# - ROS 2 installed on both hosts
# - Repository cloned on both hosts
```

---

## Troubleshooting

### No Cross-Host Communication (Dual-Compute)

**Check**:
1. Can hosts ping each other: `ping 192.168.1.10/20`
2. Firewall allows DDS traffic: `sudo ufw status`
3. Same ROS_DOMAIN_ID on both hosts: `echo $ROS_DOMAIN_ID`
4. FastDDS server running on Jetson: `ps aux | grep fastdds`
5. Correct discovery server IP in environment

**Fix**:
```bash
# Re-run network setup
sudo ./scripts/network/setup_network.sh dual_compute

# Restart nodes
./scripts/deployment/deploy.sh dual_compute stop
./scripts/deployment/deploy.sh dual_compute launch
```

### High Network Latency (Cloud Dev)

**Check**:
1. Bandwidth available: Run speed test
2. SSH tunnel stable: Check `ssh` process
3. Topic rates reasonable: `ros2 topic hz /vision/global_pose`

**Fix**:
- Use VPN instead of SSH tunnel for better performance
- Reduce topic rates in configuration
- Enable QoS policies for lossy networks

### Mock Hardware Not Working (Test)

**Check**:
1. ROS_DOMAIN_ID set to 42: `echo $ROS_DOMAIN_ID`
2. Mock mode enabled in parameters
3. No real hardware nodes running

**Fix**:
```bash
# Set correct domain
export ROS_DOMAIN_ID=42

# Kill any existing nodes
ros2 daemon stop

# Relaunch
./scripts/deployment/deploy.sh test launch
```

---

## Environment Variables Reference

```bash
# Deployment configuration
export DEPLOYMENT=local              # Deployment type
export SETUP_NETWORK=auto            # Auto-detect if network setup needed
export ROS_DOMAIN_ID=0               # ROS domain (0=prod, 42=test, 1=sim)

# FastDDS configuration
export FASTRTPS_DEFAULT_PROFILES_FILE=/path/to/fastdds.xml
export ROS_DISCOVERY_SERVER=192.168.1.20:11811

# Robot identification
export ROBOT_HOSTNAME=isaac-jetson.local
export ROBOT_HOST_ROLE=jetson        # pi, jetson, dev_workstation

# Cloud development
export REMOTE_ROBOT_HOST=robot.example.com
export SSH_TUNNEL_LOCAL_PORT=11811
```

---

## Best Practices

### Development

1. **Start Local** - Develop on Jetson first, then distribute
2. **Test Environment** - Use test deployment for CI/CD
3. **Incremental Deployment** - Test individual modules before full system
4. **Mock Hardware** - Use test mode when hardware unavailable

### Production

1. **Dual-Compute** - Use Pi for chassis, Jetson for perception
2. **Static IPs** - Configure static IPs for reliability
3. **Health Monitoring** - Monitor module health continuously
4. **Automatic Recovery** - Use systemd for auto-restart

### Remote Development

1. **VPN Recommended** - More reliable than SSH tunnel
2. **Local Testing First** - Test changes locally in test mode
3. **Incremental Deployment** - Deploy one module at a time
4. **Version Control** - Always commit before deploying

---

## Quick Reference

### Current Setup (Local)

```bash
# Launch
ros2 launch isaac_robot graph.launch.py graph:=modular_graph.yaml group:=all

# Or
./scripts/deployment/deploy.sh local launch
```

### Future Setup (Dual-Compute)

```bash
# One-time network setup (requires sudo)
sudo ./scripts/network/setup_network.sh dual_compute

# Launch on each host
./scripts/deployment/deploy.sh dual_compute launch
```

### Testing

```bash
# Test environment
./scripts/deployment/deploy.sh test launch

# Individual module testing
ros2 launch <module> <module>.launch.py
```

---

## See Also

- **Deployment Configurations**: `config/deployment/`
- **Graph Configurations**: `config/robot/`
- **Network Scripts**: `scripts/network/`
- **Deployment Scripts**: `scripts/deployment/`
- **Architecture Guide**: `src/modules/README.md`
- **Quick Start**: `docs/MODULAR_QUICK_START.md`

---

**Last Updated**: 2026-01-27  
**Status**: Phase 5 Complete  
**Next**: Hardware integration and Phase 6 testing
