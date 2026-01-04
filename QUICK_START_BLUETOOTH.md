# Quick Start - Bluetooth Setup

## Setup

### During Unified Setup
```bash
./setup.sh
# Answer 'y' when prompted for Bluetooth setup
```

### Manual Setup
```bash
./scripts/system/setup_bluetooth.sh
```

## Quick Commands

```bash
# Interactive control
bluetoothctl

# Power on
bluetoothctl power on

# Scan for devices
bluetoothctl scan on

# List devices
bluetoothctl devices

# Pair device
bluetoothctl pair <MAC_ADDRESS>

# Connect device
bluetoothctl connect <MAC_ADDRESS>
```

## Check Status

```bash
# Service status
systemctl status bluetooth

# Adapter status
hciconfig
bluetoothctl show
```

## More Info

See `docs/system/BLUETOOTH_SETUP.md` for detailed guide.
