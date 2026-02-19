# Setup Guide

## Prerequisites

- **Jetson Orin Nano** with JetPack 5.x
  - For fresh flash: See [docs/hardware/jetson-orin-nano/FLASHING.md](docs/hardware/jetson-orin-nano/FLASHING.md)
  - For storage setup: See [docs/hardware/jetson-orin-nano/STORAGE_SETUP.md](docs/hardware/jetson-orin-nano/STORAGE_SETUP.md)
- **NVMe SSD** (256GB+ recommended) or SD card
- **Intel RealSense cameras** with pan-tilt servos
- **SparkFun Auto pHAT** (ROB-16328) for motor/servo control

## Installation

### 1. Host Setup

```bash
./setup-host.sh
```

Installs:
- Docker + NVIDIA runtime
- Camera drivers (RealSense)
- GPU performance mode
- PTP time sync
- Network (mDNS)
- Data structure with calibration storage

Creates:
```
/data/
├── config/
│   ├── calibration/           # Calibration data (persistent)
│   ├── calibration_history/   # Versioned backups
│   └── calibration_sessions/  # Raw calibration data
├── worldgraph/                # TSDF maps (persistent)
├── logs/                      # Runtime logs
├── sessions/                  # Recordings
└── docker/                    # Docker data root
```

**Time**: ~15 minutes

### 2. Reboot

```bash
sudo reboot
```

### 3. Build & Deploy

```bash
docker compose build
docker compose up -d
```

**Time**: ~30 minutes build on Jetson

### 4. Calibrate

```bash
# Factory calibration (one-time)
./calibrate.sh camera-intrinsics --camera left
./calibrate.sh camera-kinematics --camera left --markers
./calibrate.sh servo-calibration --camera left --home

# Repeat for right camera

# Field calibration
./calibrate.sh odometry --start
# Drive robot
./calibrate.sh odometry --stop

# Verify
./calibrate.sh verify-all
```

Calibration saved to `/data/config/calibration/`, automatically versioned and synced.

### 5. Auto-Start (Optional)

```bash
sudo systemctl enable isaac-containers.service
```

## Configuration

### Calibration Data

Persists in `/data/config/calibration/`:
- Loaded at container startup
- Updated during calibration routines
- Versioned automatically in `calibration_history/`
- Synced hourly to remote backup

### Data Sync

Configure remote backup:
```bash
sudo nano /usr/local/bin/isaac-sync.sh
# Set: REMOTE_USER, REMOTE_HOST, REMOTE_PATH
# Uncomment rsync lines
```

Syncs:
- `/data/config/` (including calibration)
- `/data/logs/`
- `/data/worldgraph/`

### Headless Boot

```bash
sudo systemctl set-default multi-user.target
```

### SD Fallback

```bash
sudo ./scripts/system/add_sd_fallback.sh
```

## Management

```bash
# Status
./deploy.sh status

# Logs
./deploy.sh logs [service]

# Restart
./deploy.sh restart [service]

# Rebuild
docker compose build [service]
./deploy.sh restart [service]
```

## Debugging

```bash
# Exec into container
docker exec -it isaac-robot-control bash

# Check calibration loaded
cat /data/config/calibration/factory_calibration.yaml

# Check ROS nodes
ros2 node list

# Check cameras
rs-enumerate-devices
```

## Calibration Rollback

If new calibration is bad:

```bash
# List previous calibrations
ls -lt /data/config/calibration_history/

# Restore previous version
cp /data/config/calibration_history/factory_calibration_20260212.yaml \
   /data/config/calibration/factory_calibration.yaml

# Restart
docker compose restart
```

## Updates

```bash
git pull
docker compose build
docker compose restart
```

## Troubleshooting

### Calibration Not Loading
```bash
# Check file exists
ls -l /data/config/calibration/factory_calibration.yaml

# Check container can access
docker exec -it isaac-robot-control ls -l /data/config/calibration/
```

### Calibration Lost After Reboot
```bash
# Verify /data is on NVMe (persistent)
df -h /data

# Should show /dev/nvme0n1p1, not tmpfs
```

### Calibration File Corrupted
```bash
# Restore from history
ls /data/config/calibration_history/
cp /data/config/calibration_history/factory_calibration_YYYYMMDD.yaml \
   /data/config/calibration/factory_calibration.yaml
```

## See Also

- Calibration: [docs/CALIBRATION.md](docs/CALIBRATION.md)
- Architecture: [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)
- Recovery: [docs/RECOVERY.md](docs/RECOVERY.md)
- SSD Setup: [docs/SSD_MIGRATION.md](docs/SSD_MIGRATION.md)
