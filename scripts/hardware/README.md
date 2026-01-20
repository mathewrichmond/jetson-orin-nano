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
- `diagnose_phat.sh` - Diagnose SparkFun Auto pHAT

## Organization

The camera testing scripts have been consolidated into a unified `camera` command tool that provides:
- Consistent interface (similar to `isaac` command)
- Modular subcommands for specific checks
- Better discoverability and documentation
- Easier maintenance

Legacy scripts are still available for backward compatibility but new usage should prefer the unified `camera` command.

