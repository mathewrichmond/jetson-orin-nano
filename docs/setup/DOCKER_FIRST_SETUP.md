# Docker-First Setup Guide

Complete guide for setting up Jetson Orin Nano with Docker-first architecture.

## Architecture Overview

**Host (Minimal)**:
- JetPack OS with NVIDIA drivers
- Docker + NVIDIA Container Runtime
- Device permissions (udev rules)
- Network configuration

**Containers (Everything Else)**:
- **Perception**: ROS2, RealSense SDK, nvblox, camera drivers
- **Control**: ROS2, PyTorch, VLA model, subsystem control

## Prerequisites

- Jetson Orin Nano with JetPack 5.x installed on SSD
- SSD mounted at `/data` (see storage setup guide)
- Network connectivity to robotics platform server
- Camera serial numbers and device paths documented

## Full Bringup Procedure

### Phase 1: Flash and Initial Setup (One-Time)

1. **Flash JetPack to SSD** (do this BEFORE hardware assembly):
   ```bash
   # Using NVIDIA SDK Manager on host machine
   # Select Jetson Orin Nano, JetPack 5.x
   # Install to SSD (not eMMC/SD card)
   ```

2. **First boot setup**:
   ```bash
   # Complete NVIDIA first-boot wizard
   # Set username, password, etc.
   # Connect to network (Ethernet recommended for setup)
   ```

3. **Clone repository**:
   ```bash
   cd ~
   git clone https://github.com/your-org/jetson-orin-nano.git
   cd jetson-orin-nano
   ```

### Phase 2: Host Setup (Minimal, One-Time)

Run the Docker-first setup script:

```bash
./setup.sh
```

This installs:
- ✅ Docker and NVIDIA Container Runtime
- ✅ Device permissions (udev rules for cameras/servos)
- ✅ Network configuration (mDNS, optional WiFi/Bluetooth)
- ✅ Data directories on SSD (`/data/*`)

**IMPORTANT**: Reboot after setup for group membership to take effect:

```bash
sudo reboot
```

### Phase 3: Robot Configuration

1. **Create .env file**:
   ```bash
   cp .env.example .env
   nano .env
   ```

   Configure:
   ```bash
   ROBOT_ID=jetson-01  # Unique robot identifier
   ZENOH_ROUTER=tcp/192.168.1.100:7447  # Server IP where dispatch runs
   CAMERA_LEFT_SERIAL=12345678  # From `rs-enumerate-devices`
   CAMERA_RIGHT_SERIAL=87654321
   ```

2. **Update robot_config.yaml**:
   ```bash
   nano config/robot_config.yaml
   ```

   Configure capabilities, sensors, actuators:
   ```yaml
   robot_id: jetson-01
   capabilities:
     - pan_tilt_camera
     - stereo_vision
     - mobile_base
   
   sensors:
     camera_left:
       type: realsense_d435i
       serial: "${CAMERA_LEFT_SERIAL}"
   # ... etc
   ```

3. **Document hardware setup** (BEFORE assembly):
   ```bash
   # List USB devices to identify cameras
   lsusb
   
   # List video devices
   ls -l /dev/video*
   
   # Enumerate RealSense cameras
   rs-enumerate-devices
   
   # List serial devices
   ls -l /dev/ttyUSB* /dev/ttyACM*
   
   # Save this info to config/hardware_map.txt
   ```

### Phase 4: Download VLA Model

Download or train VLA model and place in `/data/models/`:

```bash
# Example: Download pretrained Octo model
mkdir -p /data/models
cd /data/models

# Download or copy your VLA checkpoint
# scp user@server:/models/vla_latest.pth ./vla_latest.pth

# Or use placeholder for testing
touch vla_latest.pth
```

### Phase 5: Build Containers

Build perception and control Docker images:

```bash
cd ~/jetson-orin-nano

# Build both containers
docker compose build

# Or build individually
docker compose build perception
docker compose build control
```

**Expected build time**: 20-40 minutes on first build.

### Phase 6: Test Hardware Access (Before Assembly)

Test camera access:

```bash
# Run perception container interactively
docker compose run --rm perception bash

# Inside container, test RealSense
rs-enumerate-devices
python3 -c "import pyrealsense2 as rs; print(rs.context())"

exit
```

Test servo access:

```bash
# Run control container interactively
docker compose run --rm control bash

# Inside container, test serial
python3 -c "import serial; print(serial.Serial('/dev/ttyUSB0', timeout=1))"

exit
```

Test GPU access:

```bash
# Should show GPU info
docker run --rm --gpus all nvcr.io/nvidia/l4t-base:r35.2.1 nvidia-smi
```

### Phase 7: Hardware Assembly

**Only after testing**, assemble robot hardware:

1. Mount cameras on pan-tilt mounts
2. Connect servos/motors
3. Route cables
4. Mount Jetson in chassis
5. Ensure SSD and flash drive remain accessible

### Phase 8: Start Robot

Start containers:

```bash
cd ~/jetson-orin-nano

# Start in foreground (for debugging)
docker compose up

# Or start in background
docker compose up -d

# View logs
docker compose logs -f perception control
```

### Phase 9: Verify Registration

Check that robot registered with dispatch service:

```bash
# From server or any machine with access to dispatch API
curl http://192.168.1.100:5000/api/v1/robots

# Should show:
# {
#   "robots": [
#     {
#       "robot_id": "jetson-01",
#       "capabilities": ["pan_tilt_camera", "stereo_vision", ...],
#       "last_seen": "2024-01-15T10:30:00Z"
#     }
#   ]
# }
```

### Phase 10: Test Mission Execution

Submit test mission:

```bash
curl -X POST http://192.168.1.100:5000/api/v1/missions \
  -H "Content-Type: application/json" \
  -d '{
    "command": "Look around the room",
    "required_capabilities": ["pan_tilt_camera"],
    "priority": 3
  }'
```

Monitor execution:

```bash
# On Jetson
docker compose logs -f control

# Should see:
# control | Received mission: Look around the room
# control | Generated action: {"subsystem": "pan_tilt", ...}
# control | Mission completed successfully
```

### Phase 11: Auto-Start on Boot (Optional)

Install systemd service:

```bash
cd ~/jetson-orin-nano
./scripts/system/install_services.sh
```

Test reboot:

```bash
sudo reboot

# After reboot, check containers started
docker compose ps
```

## Daily Operation

### Start Robot

```bash
docker compose up -d
```

### Stop Robot

```bash
docker compose down
```

### View Logs

```bash
# All logs
docker compose logs -f

# Specific container
docker compose logs -f perception
docker compose logs -f control
```

### Update Code

```bash
cd ~/jetson-orin-nano
git pull
docker compose build
docker compose up -d
```

### Update VLA Model

```bash
# Copy new model to Jetson
scp new_model.pth jetson-01.local:/data/models/vla_latest.pth

# Restart control container
docker compose restart control
```

## Troubleshooting

### Camera Not Accessible

```bash
# Check USB devices
lsusb | grep Intel

# Check video devices
ls -l /dev/video*

# Check permissions
groups $USER  # Should include: video, plugdev

# Test in container
docker compose run --rm perception bash
rs-enumerate-devices
```

**Fix**: Ensure udev rules applied and user in video group.

### GPU Not Accessible

```bash
# Check NVIDIA runtime
docker run --rm --gpus all nvcr.io/nvidia/l4t-base:r35.2.1 nvidia-smi

# Check daemon.json
cat /etc/docker/daemon.json
```

**Fix**: Reinstall nvidia-docker2, restart Docker daemon.

### Robot Not Registering

```bash
# Check Zenoh connectivity
docker compose logs control | grep "Registered capabilities"

# Test Zenoh router
nc -zv 192.168.1.100 7447

# Check .env file
cat .env | grep ZENOH_ROUTER
```

**Fix**: Verify server IP, check firewall, ensure dispatch service running.

### Permission Denied (Serial Devices)

```bash
# Check groups
groups $USER  # Should include: dialout

# Check device permissions
ls -l /dev/ttyUSB0
```

**Fix**: Add user to dialout group, reboot.

## Architecture Benefits

### vs. Host-Based Setup

**Before** (Host-Based):
- 737-line setup.sh
- 40+ script files
- ROS2 on host
- System package conflicts
- Difficult updates
- Hard to reproduce

**After** (Docker-First):
- 150-line setup.sh (host only)
- 5 host scripts
- ROS2 in containers
- Isolated dependencies
- Easy updates (rebuild container)
- Reproducible (Dockerfile = source of truth)

### Development Workflow

```bash
# Edit code on laptop
vim perception/src/perception_main.py

# Build for Jetson (multi-arch)
docker buildx build --platform linux/arm64 \
  -t myregistry/perception:latest \
  --push ./perception

# Deploy to Jetson
ssh jetson-01
docker compose pull
docker compose up -d
```

## Migration from Host-Based Setup

If you have an existing host-based setup:

1. **Backup current system**:
   ```bash
   # Save ROS2 workspace
   tar czf ~/ros2_ws_backup.tar.gz ~/ros2_ws
   
   # Save configs
   cp -r config config.backup
   ```

2. **Run refactored setup.sh**:
   - Old scripts won't run (Docker-first checks environment)
   - Host setup is now minimal
   - ROS2/packages moved to Dockerfiles

3. **Migrate code to containers**:
   - Perception code → `perception/src/`
   - Control code → `control/src/`
   - Update imports to use `robotics_sdk`

4. **Test before hardware assembly**:
   - Verify camera/GPU/servo access in containers
   - Test on development machine if possible

## Next Steps

- [Hardware Verification](../hardware/VERIFICATION.md) - Verify all hardware
- [Mission Examples](../api/MISSION_EXAMPLES.md) - Try complex missions
- [Deployment Guide](../deployment/DEPLOYMENT.md) - Multi-robot deployment
- [Continuous Learning](../architecture/CONTINUOUS_LEARNING.md) - VLA retraining

## Support

Issues or questions:
- Check [Troubleshooting](#troubleshooting) section
- Review Docker logs: `docker compose logs`
- Check dispatch service health: `curl http://server:5000/health`
- Hardware verification: `./scripts/hardware/setup_hardware.sh verify`
