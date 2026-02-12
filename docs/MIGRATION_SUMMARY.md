# Docker-First Migration Summary

Summary of refactoring from host-based to Docker-first architecture.

## Changes Made

### Files Refactored

#### setup.sh (737 → ~200 lines, 73% reduction)

**Removed** (moved to Dockerfiles):
- `step_install_system_packages()` - System packages → Dockerfiles
- `step_install_python_packages()` - Python packages → requirements.txt
- `step_setup_ros2_workspace()` - ROS2 setup → Dockerfile base image
- `step_setup_venv()` - Virtual environment → Not needed
- `step_install_precommit()` - Dev tools → Not needed on robot
- `step_install_realsense()` - RealSense SDK → perception/Dockerfile
- `step_setup_visualization()` - ROS packages → Dockerfiles
- `step_build_ros2_packages()` - ROS2 build → Docker build

**Kept** (host-level only):
- `step_update_system()` - System updates
- `step_install_docker()` - NEW: Docker + NVIDIA runtime
- `step_setup_device_permissions()` - NEW: Udev rules
- `step_setup_network()` - NEW: mDNS, hostname
- `step_setup_wifi()` - WiFi credentials (host-level)
- `step_setup_bluetooth()` - Bluetooth pairing (host-level)
- `step_verify_hardware()` - Hardware verification

### Files Deleted

- `config/system/packages.yaml` (207 lines, 3.3KB) → Moved to Dockerfiles
- `config/system/python_packages.yaml` (1.5KB) → Moved to requirements.txt
- **Total savings**: 4.8KB config files eliminated

### Files Created

#### Perception Container
- `perception/Dockerfile` - ROS2 + RealSense + nvblox
- `perception/requirements.txt` - Python dependencies
- `perception/src/perception_main.py` - Implements PerceptionBase

#### Control Container
- `control/Dockerfile` - ROS2 + PyTorch + VLA
- `control/requirements.txt` - Python dependencies
- `control/src/control_main.py` - Implements ControlBase

#### Configuration
- `.env.example` - Robot-specific environment variables
- `config/robot_config.yaml` - Robot capabilities, sensors, actuators
- `config/system/README.md` - Deprecation notice

#### Documentation
- `docs/setup/DOCKER_FIRST_SETUP.md` - Complete bringup guide (500+ lines)
- `docs/MIGRATION_SUMMARY.md` - This file
- `README.md` - Rewritten for Docker-first approach

#### Docker Orchestration
- `docker-compose.yml` - Updated for production with device mappings

## Architecture Changes

### Before (Host-Based)

```
Host (Jetson)
├── JetPack OS
├── ROS2 Humble (apt install)
├── RealSense SDK (apt install)
├── Python packages (pip install)
├── System packages (207 packages from YAML)
├── ROS2 workspace (~/ros2_ws)
│   ├── perception packages
│   ├── control packages
│   └── hardware drivers
└── Systemd services
```

**Problems**:
- Package conflicts
- Difficult updates
- Hard to reproduce
- 737-line setup script
- 40+ script files

### After (Docker-First)

```
Host (Jetson) - Minimal
├── JetPack OS
├── Docker + NVIDIA runtime
├── Udev rules (device permissions)
└── Network config (WiFi, mDNS)

Perception Container
├── ROS2 Humble
├── RealSense SDK
├── nvblox
├── robotics_sdk
└── perception_main.py

Control Container
├── ROS2 Humble
├── PyTorch
├── VLA model
├── robotics_sdk
└── control_main.py
```

**Benefits**:
- Isolated dependencies
- Easy updates (rebuild container)
- Reproducible (Dockerfile = source of truth)
- 150-line host setup
- 5 host scripts

## Code Statistics

### Before
- `setup.sh`: 737 lines
- Scripts: 40+ files
- Config files: 207 line packages.yaml + more
- **Total**: ~1000+ lines of setup/config

### After
- `setup.sh`: ~200 lines (73% reduction)
- Scripts: 5 host scripts (87% reduction)
- Config files: robot_config.yaml + .env
- Dockerfiles: 2 × ~60 lines = 120 lines
- **Total**: ~400 lines (60% reduction)

### Savings
- **Setup script**: 537 lines removed (73%)
- **Config files**: 4.8KB deleted
- **Script files**: 35+ files deprecated
- **Maintenance burden**: Dramatically reduced

## Package Migration Map

| Category | Before | After |
|----------|--------|-------|
| ROS2 | Host apt | perception/Dockerfile, control/Dockerfile |
| Python dev tools | Host apt | Container Dockerfiles |
| RealSense SDK | Host apt | perception/Dockerfile |
| PyTorch | Manual install | control/Dockerfile (l4t-pytorch base) |
| System packages | packages.yaml (207 lines) | Dockerfiles |
| Python packages | python_packages.yaml | requirements.txt |
| Docker | Not installed | host setup.sh |
| Network tools | Host | host setup.sh (minimal) |

## Behavioral Changes

### Setup Process

**Before**:
```bash
./setup.sh  # Takes 30-60 minutes, installs everything on host
sudo reboot
ros2 launch isaac_robot full.launch.py
```

**After**:
```bash
./setup.sh  # Takes 5-10 minutes, installs Docker only
sudo reboot
docker compose build  # Takes 20-40 minutes
docker compose up -d
```

### Daily Operation

**Before**:
```bash
# Start robot
ros2 launch isaac_robot full.launch.py

# Update code
git pull
cd ~/ros2_ws
colcon build
source install/setup.bash

# Debugging package conflicts, dependency hell
```

**After**:
```bash
# Start robot
docker compose up -d

# Update code
git pull
docker compose build
docker compose up -d

# Clean separation, no conflicts
```

### Updates

**Before**:
- System updates might break ROS2
- ROS2 updates might break packages
- Package conflicts common
- Hard to rollback

**After**:
- Host updates independent of containers
- Container updates atomic
- No package conflicts (isolation)
- Easy rollback (previous image)

## Integration Points

### With Robotics Framework

Implements interfaces from [robotics_sdk](../robotics/common/robotics_sdk):

- `PerceptionBase` - Perception container publishes TSDF, pose
- `ControlBase` - Control container subscribes to missions
- `RobotBase` - Registers capabilities with dispatch

### With Platform Services

- **Dispatch**: Receives missions, reports status/results
- **Digital Twin**: Publishes TSDF updates, receives global map
- **Training**: Logs uploaded for VLA retraining
- **Logging**: Streams to centralized MCAP service

### Hardware Access

Containers access hardware via device passthrough:

- Cameras: `/dev/video*`, `/dev/bus/usb`
- Servos: `/dev/ttyUSB0`
- I2C: `/dev/i2c-1`
- GPU: NVIDIA runtime

## Migration Checklist

For existing deployments:

- [x] Refactor setup.sh to Docker-first
- [x] Delete deprecated config files
- [x] Create perception/Dockerfile
- [x] Create control/Dockerfile
- [x] Update docker-compose.yml
- [x] Create .env.example
- [x] Create robot_config.yaml template
- [x] Write comprehensive setup documentation
- [x] Update README.md
- [ ] Test on actual hardware (after power-up)
- [ ] Migrate existing perception code to perception/src/
- [ ] Migrate existing control code to control/src/
- [ ] Build and test containers
- [ ] Verify hardware access (cameras, servos, GPU)
- [ ] Test mission execution
- [ ] Deploy to production

## Next Steps

1. **Before Power-Up**:
   - Review all created files
   - Customize robot_config.yaml for your hardware
   - Document camera serials, device paths

2. **Storage Setup** (see main plan):
   - Flash JetPack to SSD
   - Configure /data partition
   - Set up flash recovery

3. **Initial Bringup**:
   - Run setup.sh on Jetson
   - Configure .env
   - Build containers
   - Test hardware access

4. **Code Migration**:
   - Move perception code → perception/src/
   - Move control code → control/src/
   - Update imports to use robotics_sdk
   - Test in containers

5. **Production Deploy**:
   - Verify registration with dispatch
   - Submit test missions
   - Monitor logs
   - Set up auto-start

## Rollback Plan

If needed to revert:

1. **Backup**: Current setup.sh is backed up in git history
2. **Restore**: `git checkout <commit-before-refactor>`
3. **Old setup**: Run original setup.sh

However, Docker-first is recommended for long-term maintainability.

## Questions?

- Docker-first setup: See `docs/setup/DOCKER_FIRST_SETUP.md`
- Hardware verification: See `docs/hardware/VERIFICATION.md`
- Platform integration: See `../robotics/docs/BUILDING_ROBOTS.md`
