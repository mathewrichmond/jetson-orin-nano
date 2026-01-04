# Bluetooth Setup Guide

## Overview

This guide covers setting up Bluetooth support on the Isaac robot system. Bluetooth can be used for connecting peripherals like keyboards, mice, controllers, or other Bluetooth-enabled devices.

## Quick Setup

### During Unified Setup

Bluetooth setup is integrated into the unified setup script:

```bash
./setup.sh
```

When prompted, answer 'y' to setup Bluetooth.

### Manual Setup

```bash
./scripts/system/setup_bluetooth.sh
```

## What Gets Installed

The setup script installs:
- **bluez** - Bluetooth protocol stack
- **bluez-tools** - Bluetooth command-line tools
- **bluetooth** - Bluetooth service
- **pulseaudio-module-bluetooth** - Audio support for Bluetooth devices

## Bluetooth Service

The Bluetooth service is automatically enabled and started:

```bash
# Check status
systemctl status bluetooth

# Start service
sudo systemctl start bluetooth

# Enable on boot
sudo systemctl enable bluetooth
```

## Using Bluetooth

### Interactive Control

Use `bluetoothctl` for interactive Bluetooth control:

```bash
bluetoothctl
```

### Common Commands

```bash
# Power on Bluetooth
bluetoothctl power on

# Make device discoverable
bluetoothctl discoverable on

# Make device pairable
bluetoothctl pairable on

# Scan for devices
bluetoothctl scan on

# List known devices
bluetoothctl devices

# Pair with device
bluetoothctl pair <MAC_ADDRESS>

# Connect to device
bluetoothctl connect <MAC_ADDRESS>

# Disconnect device
bluetoothctl disconnect <MAC_ADDRESS>

# Trust device (auto-connect)
bluetoothctl trust <MAC_ADDRESS>

# Remove device
bluetoothctl remove <MAC_ADDRESS>
```

### Check Bluetooth Adapter

```bash
# List adapters
hciconfig

# Show adapter info
hciconfig hci0

# Power on adapter
sudo hciconfig hci0 up

# Power off adapter
sudo hciconfig hci0 down
```

## Troubleshooting

### Bluetooth Not Working

1. **Check service status:**
   ```bash
   systemctl status bluetooth
   ```

2. **Check adapter:**
   ```bash
   hciconfig
   bluetoothctl show
   ```

3. **Power on adapter:**
   ```bash
   sudo hciconfig hci0 up
   bluetoothctl power on
   ```

4. **Check permissions:**
   ```bash
   groups | grep bluetooth
   ```
   If not in bluetooth group, add user:
   ```bash
   sudo usermod -a -G bluetooth $USER
   # Log out and back in
   ```

### Device Not Pairing

1. **Make sure device is in pairing mode**

2. **Make adapter discoverable:**
   ```bash
   bluetoothctl discoverable on
   ```

3. **Scan for devices:**
   ```bash
   bluetoothctl scan on
   ```

4. **Try pairing:**
   ```bash
   bluetoothctl pair <MAC_ADDRESS>
   ```

### Audio Not Working

If using Bluetooth audio devices:

1. **Install PulseAudio Bluetooth module** (already included):
   ```bash
   sudo apt install pulseaudio-module-bluetooth
   ```

2. **Restart PulseAudio:**
   ```bash
   pulseaudio -k
   pulseaudio --start
   ```

3. **Connect audio device:**
   ```bash
   bluetoothctl connect <MAC_ADDRESS>
   ```

4. **Set as audio output:**
   ```bash
   pactl list short sinks
   pactl set-default-sink <sink_name>
   ```

## Configuration Files

Bluetooth configuration is stored in:
- `/etc/bluetooth/main.conf` - Main Bluetooth daemon configuration
- `/var/lib/bluetooth/` - Stored device information

## Security Considerations

- **Pairing**: Devices must be paired before connecting
- **Trust**: Trusted devices can auto-connect
- **Discoverable**: Only enable when needed for pairing
- **Pairable**: Only enable when pairing new devices

## Integration with Robot System

Bluetooth can be used for:
- **Input devices**: Keyboards, mice, game controllers
- **Audio devices**: Headphones, speakers
- **Sensors**: Bluetooth-enabled sensors
- **Communication**: Inter-device communication

## Useful Commands Reference

```bash
# Quick status check
systemctl status bluetooth
hciconfig
bluetoothctl show

# Power control
bluetoothctl power on
bluetoothctl power off

# Device management
bluetoothctl devices
bluetoothctl scan on
bluetoothctl pair <MAC>
bluetoothctl connect <MAC>
bluetoothctl disconnect <MAC>
bluetoothctl remove <MAC>

# Adapter control
sudo hciconfig hci0 up
sudo hciconfig hci0 down
```

## See Also

- [WiFi Setup](WIFI_SETUP.md) - Network configuration
- [System Setup](../setup/SETUP.md) - Complete setup guide
- BlueZ documentation: `man bluetoothctl`
