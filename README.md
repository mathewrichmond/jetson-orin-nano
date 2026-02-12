# Isaac Robot - Articulated Cameras with UDM Worldgraph

Robot system with independently actuated cameras projecting into a unified digital twin for simulation-independent planning.

## Key Design

### Problem
- **Articulated cameras**: Pan-tilt servos per camera (for tracking/looking)
- **RealSense depth**: Built-in depth sensing (not stereo)
- **Simulation challenge**: Moving cameras make planning/simulation hard

### Solution: UDM Worldgraph
Project all sensor data into a **unified digital twin** (UDM worldgraph):
- Plan/simulate independent of sensor poses
- Run pre-trained models with different sensor configs
- Generate offline RL training data

```
Articulated Cameras → UDM Worldgraph → Virtual Sensors
                           ↓
                     Planning/Models
```

## Quick Start

```bash
# 1. Setup host
./setup-host.sh

# 2. Reboot
sudo reboot

# 3. Build & deploy
docker compose build
docker compose up -d
```

## Calibration

### Factory Calibration (One-Time)

```bash
# Camera intrinsics (per camera)
./calibrate.sh camera-intrinsics --camera left
./calibrate.sh camera-intrinsics --camera right

# Kinematic chain (base → pan → tilt → camera)
./calibrate.sh camera-kinematics --camera left --markers
./calibrate.sh camera-kinematics --camera right --markers

# Servo zero positions
./calibrate.sh servo-calibration --camera left --home
./calibrate.sh servo-calibration --camera right --home
```

### Field Calibration (Auto)

```bash
# Odometry (robot base motion)
./calibrate.sh odometry --start
# Drive robot 2-3 minutes
./calibrate.sh odometry --stop
```

See [docs/CALIBRATION.md](docs/CALIBRATION.md) for complete guide.

## Calibration Data Storage

All calibration persists in `/data/config/`:

```
/data/config/
├── calibration/
│   ├── factory_calibration.yaml       # Main calibration
│   ├── odometry_calibration.yaml      # Wheel params
│   └── runtime_calibration.yaml       # Auto-updates
│
├── calibration_history/               # Versioned backups
│   ├── factory_calibration_20260212.yaml
│   └── odometry_calibration_20260215.yaml
│
└── calibration_sessions/              # Raw data
    ├── camera_intrinsics_left_*/
    └── odometry_*/
```

**Persistence**:
- Survives reboots (stored on NVMe)
- Synced hourly to remote backup
- Versioned automatically on updates
- Mounted read-write in containers

**Loading**: System loads from `/data/config/calibration/` at startup

## Management

```bash
./deploy.sh [command]
# Commands: build, start, stop, restart, logs, status
```

## System Components

**One Container**: `robot-control`
- Chassis control
- Camera pipeline (RealSense)
- Servo control (pan-tilt)
- Forward kinematics
- SLAM/odometry
- UDM worldgraph (projection & rendering)

## Data Structure

```
/data/
├── config/              # Calibration (synced, versioned)
├── worldgraph/          # TSDF maps (synced)
├── logs/                # Runtime logs (synced)
└── sessions/            # Recordings, calibration raw data
```

## Documentation

- **[docs/CALIBRATION.md](docs/CALIBRATION.md)** - Complete calibration & storage
- **[docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)** - System architecture
- [SETUP.md](SETUP.md) - Setup guide
- [docs/SSD_MIGRATION.md](docs/SSD_MIGRATION.md) - NVMe setup
- [docs/RECOVERY.md](docs/RECOVERY.md) - Recovery procedures
