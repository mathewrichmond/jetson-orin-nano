# Hardware Installation and Testing Scripts

This directory contains scripts for installing, configuring, and testing hardware components.

## Unified Camera Tool

**Recommended**: Use the unified `camera` command for all camera testing and verification:

```bash
# Full verification (publishing + sync)
./scripts/hardware/camera verify

# Check synchronization only
./scripts/hardware/camera verify-sync

# Run diagnostics
./scripts/hardware/camera diagnose

# Identify camera positions
./scripts/hardware/camera identify

# Quick checks
./scripts/hardware/camera check-topics
./scripts/hardware/camera check-rates
./scripts/hardware/camera check-sync-status
./scripts/hardware/camera check-timestamps

# With options
./scripts/hardware/camera verify --duration 20 --tolerance 5.0
```

## Installation Scripts

- `install_realsense.sh` - Install Intel RealSense SDK and ROS 2 wrapper
- `setup_hardware.sh` - Setup all hardware components
- `setup_microphone.sh` - Setup USB microphone

**Usage**:
```bash
sudo ./scripts/hardware/install_realsense.sh
```

## Testing Scripts

### Camera Testing (Use `camera` command above)

Legacy scripts (still available, but prefer `camera` command):
- `verify_camera_sync.sh` - Verify cameras publishing and synced
- `check_camera_sync.sh` - Hardware/firmware sync diagnostics
- `diagnose_realsense.sh` - Comprehensive camera diagnostics
- `identify_cameras.sh` - Identify which camera is front vs rear

### Other Hardware Testing

- `test_hardware_nodes.sh` - Test all hardware ROS nodes
- `test_servo_hardware.sh` - Test servo hardware (PCA9685)
- `verify_all_hardware.sh` - Verify all hardware components
- `diagnose_phat.sh` - Diagnose SparkFun Auto pHAT (IMU, GPIO, general)
- `diagnose_pca9685.sh` - **Comprehensive PCA9685 servo controller diagnostics**
  - Systematic I2C troubleshooting
  - Address scanning and detection
  - Register read/write testing
  - Software reset capability
  - Hardware fault detection

## Organization

The camera testing scripts have been consolidated into a unified `camera` command tool that provides:
- Consistent interface (similar to `isaac` command)
- Modular subcommands for specific checks
- Better discoverability and documentation
- Easier maintenance

Legacy scripts are still available for backward compatibility but new usage should prefer the unified `camera` command.

## Hardware Documentation

Comprehensive hardware documentation is available for the SparkFun Auto pHAT:

**Location**: `docs/hardware/sparkfun-auto-phat/`

**Key Resources**:
- [README.md](../../docs/hardware/sparkfun-auto-phat/README.md) - Board overview and component layout
- [SPECIFICATIONS.md](../../docs/hardware/sparkfun-auto-phat/SPECIFICATIONS.md) - Technical specifications for all ICs
- [I2C_TROUBLESHOOTING.md](../../docs/hardware/sparkfun-auto-phat/I2C_TROUBLESHOOTING.md) - Systematic I2C debugging guide
  - Pull-up vs pull-down resistors explained
  - PCA9685 servo controller troubleshooting
  - Voltage and signal verification
  - Common failure modes and solutions
- [JUMPER_CONFIGURATION.md](../../docs/hardware/sparkfun-auto-phat/JUMPER_CONFIGURATION.md) - I2C address configuration
- [datasheets/](../../docs/hardware/sparkfun-auto-phat/datasheets/) - Official schematics and datasheets

**Quick Diagnostics**:
```bash
# Scan I2C bus for all devices
sudo i2cdetect -y 7

# Test PCA9685 servo controller specifically
./scripts/hardware/diagnose_pca9685.sh

# General Auto pHAT diagnostics
./scripts/hardware/diagnose_phat.sh
```

**Configuration**: All hardware parameters are in `config/hardware/phat_params.yaml`

