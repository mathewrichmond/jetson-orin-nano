# Jetson Orin Nano - Robot Target Implementation

Docker-first target implementation for autonomous mobile robot on Jetson Orin Nano, integrating with the [robotics platform](../robotics).

## Architecture

**Docker-First Design** - Host provides hardware access, containers provide functionality:

```
┌─────────────────────────────────────┐
│ Host (Minimal)                      │
│ - JetPack + NVIDIA drivers          │
│ - Docker + NVIDIA runtime           │
│ - Device permissions (udev)         │
│ - Network (mDNS, WiFi)              │
└─────────────────┬───────────────────┘
                  │
        ┌─────────┴──────────┐
        │                    │
┌───────▼────────┐  ┌────────▼────────┐
│   Perception   │  │    Control      │
│   Container    │  │   Container     │
│                │  │                 │
│ - ROS2 Humble  │  │ - ROS2 Humble   │
│ - RealSense SDK│  │ - PyTorch       │
│ - nvblox TSDF  │  │ - VLA model     │
│ - Cameras      │  │ - Servo control │
└────────────────┘  └─────────────────┘
```

**Integration**: Uses [robotics_sdk](../robotics/common/robotics_sdk) base classes to implement `PerceptionBase` and `ControlBase`.

## Quick Start

### 1. Host Setup (One-Time)

```bash
git clone <repository-url>
cd jetson-orin-nano

# Run minimal host setup
./setup.sh

# Reboot for group membership
sudo reboot
```

### 2. Configure Robot

```bash
# Create environment file
cp .env.example .env
nano .env

# Set:
# - ROBOT_ID=jetson-01
# - ZENOH_ROUTER=tcp/192.168.1.100:7447 (server IP)
# - Camera serials from: rs-enumerate-devices

# Edit robot config
nano config/robot_config.yaml
```

### 3. Build and Start

```bash
# Build containers
docker compose build

# Start robot
docker compose up -d

# Monitor logs
docker compose logs -f perception control
```

### 4. Verify Registration

```bash
# Check robot registered with dispatch service
curl http://192.168.1.100:5000/api/v1/robots

# Should show jetson-01 with capabilities
```

### 5. Test Mission

```bash
# Submit test mission
curl -X POST http://192.168.1.100:5000/api/v1/missions \
  -H "Content-Type: application/json" \
  -d '{"command": "Look around", "required_capabilities": ["pan_tilt_camera"]}'
```

## Repository Structure

```
jetson-orin-nano/
├── perception/              # Perception container
│   ├── Dockerfile
│   ├── requirements.txt
│   └── src/
│       └── perception_main.py  # Implements PerceptionBase
├── control/                 # Control container
│   ├── Dockerfile
│   ├── requirements.txt
│   └── src/
│       └── control_main.py     # Implements ControlBase
├── config/
│   └── robot_config.yaml       # Robot-specific configuration
├── docs/
│   ├── setup/
│   │   └── DOCKER_FIRST_SETUP.md  # Complete bringup guide
│   ├── hardware/               # Hardware setup guides
│   └── architecture/           # System design
├── scripts/
│   ├── system/                 # Minimal host scripts
│   │   ├── setup_wifi.sh       # WiFi configuration
│   │   └── setup_bluetooth.sh  # Bluetooth pairing
│   └── hardware/
│       └── setup_hardware.sh   # Hardware verification
├── setup.sh                    # Minimal host setup (~150 lines)
├── docker-compose.yml          # Production orchestration
└── .env.example                # Configuration template
```

## Hardware

**Supported**:
- 2x Intel RealSense D435i cameras (stereo, depth, IMU)
- Pan-tilt mounts (Dynamixel servos)
- Mobile base (differential drive)
- iRobot Create chassis (optional)

**Requirements**:
- Jetson Orin Nano with JetPack 5.x
- 256GB+ SSD (boot drive + data)
- USB 3.0 for cameras
- Serial/I2C for actuators

## Key Features

### Docker-First Benefits

**Before (Host-Based)**:
- 737-line setup.sh
- 40+ script files
- System package conflicts
- Difficult to update

**After (Docker-First)**:
- 150-line setup.sh (host only)
- Isolated dependencies
- Easy updates (rebuild container)
- Reproducible builds

### Integration with Robotics Platform

- **Perception**: Publishes TSDF, ESDF, pose to `/robot/{robot_id}/perception/*`
- **Control**: Subscribes to missions from dispatch, sends subsystem commands
- **Capabilities**: Registers with dispatch service for automatic task routing
- **Logging**: Streams to centralized MCAP logging service
- **Digital Twin**: Contributes to shared 3D world representation

### Continuous Learning

- Novel scenario detection triggers active learning
- MCAP logs uploaded for VLA retraining
- Model updates deployed via volume mount (`/data/models/`)

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
docker compose logs -f perception control
```

### Update Code

```bash
git pull
docker compose build
docker compose up -d
```

### Update Model

```bash
# Copy new VLA model
scp new_model.pth jetson-01.local:/data/models/vla_latest.pth

# Restart control
docker compose restart control
```

### Auto-Start on Boot

```bash
./scripts/system/install_services.sh
sudo reboot  # Test auto-start
```

## Documentation

- **[Docker-First Setup](docs/setup/DOCKER_FIRST_SETUP.md)** - Complete bringup guide
- **[Hardware Verification](docs/hardware/VERIFICATION.md)** - Test all hardware
- **[Troubleshooting](docs/setup/DOCKER_FIRST_SETUP.md#troubleshooting)** - Common issues
- **[Architecture](docs/architecture/)** - System design

## Development

### Local Testing

```bash
# Build for x86 (on laptop)
docker buildx build --platform linux/amd64 ./perception

# Test perception logic without hardware
docker compose run --rm perception python3 -m pytest
```

### Live Development

```bash
# Mount robotics framework locally
# Edit docker-compose.yml:
# volumes:
#   - ../robotics:/robotics:ro

# Rebuild
docker compose build --no-cache
docker compose up
```

### Hardware Verification

```bash
# Test camera access
docker compose run --rm perception bash
rs-enumerate-devices

# Test servo access
docker compose run --rm control bash
python3 -c "import serial; print(serial.Serial('/dev/ttyUSB0'))"

# Test GPU
docker run --rm --gpus all nvcr.io/nvidia/l4t-base:r35.2.1 nvidia-smi
```

## Migration from Host-Based

See [Docker-First Setup Guide](docs/setup/DOCKER_FIRST_SETUP.md#migration-from-host-based-setup) for migration instructions.

**Summary**:
1. Backup existing ROS2 workspace
2. Run new setup.sh (installs Docker only)
3. Move code to `perception/src/` and `control/src/`
4. Update imports to use `robotics_sdk`
5. Build and test containers

## Troubleshooting

### Camera Not Found

```bash
# Check USB
lsusb | grep Intel

# Check permissions
groups $USER  # Should include: video, plugdev

# Test in container
docker compose run --rm perception rs-enumerate-devices
```

### GPU Not Accessible

```bash
# Test NVIDIA runtime
docker run --rm --gpus all nvcr.io/nvidia/l4t-base:r35.2.1 nvidia-smi

# Check daemon config
cat /etc/docker/daemon.json
```

### Robot Not Registering

```bash
# Check Zenoh connection
docker compose logs control | grep "Registered capabilities"

# Test server connectivity
nc -zv 192.168.1.100 7447

# Check .env
cat .env | grep ZENOH_ROUTER
```

See [full troubleshooting guide](docs/setup/DOCKER_FIRST_SETUP.md#troubleshooting).

## Support

- **Issues**: Open GitHub issue
- **Docs**: See `docs/` directory
- **Platform**: See [robotics framework](../robotics)
- **Hardware**: See `docs/hardware/`

## License

MIT
