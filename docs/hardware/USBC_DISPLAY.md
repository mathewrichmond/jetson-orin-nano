# USB-C Display and Dock Setup Guide

## Overview

This guide covers setting up USB-C monitors and docking stations with the Jetson Orin Nano. USB-C display support on Jetson boards can be tricky due to hardware limitations.

## Important Limitations

### USB-C Port Capabilities

**Jetson Orin Nano USB-C ports may NOT support DisplayPort Alternate Mode (DP Alt Mode)** by default. This means:
- Standard USB-C to DisplayPort cables may not work
- USB-C monitors that require DP Alt Mode won't work directly
- USB-C docks may have limited functionality

### What This Means

Unlike laptops, Jetson boards often have USB-C ports that are **data-only** and don't support video output via USB-C. The USB-C ports are typically for:
- USB data transfer
- Power delivery (if supported)
- NOT video output

## Solutions

### Option 1: Use Native Display Outputs (Recommended)

The Jetson Orin Nano has dedicated display outputs:
- **HDMI port** - Use HDMI cable directly to monitor
- **DisplayPort** - If available, use DisplayPort cable

**This is the most reliable solution.**

### Option 2: DisplayLink Docks (Software-Based)

If your dock uses **DisplayLink technology**, it can work via USB data connection:

1. **Install DisplayLink driver:**
   ```bash
   sudo apt update
   sudo apt install displaylink-driver
   ```

2. **Check if dock is detected:**
   ```bash
   lsusb | grep -i displaylink
   ```

3. **Restart display manager:**
   ```bash
   sudo systemctl restart gdm3
   ```

4. **Configure display:**
   ```bash
   xrandr --listmonitors
   xrandr --output <display> --auto --primary
   ```

### Option 3: USB-C to HDMI/DisplayPort Adapter

Some USB-C to HDMI adapters work if they:
- Use DisplayLink chipset (software-based)
- Are active adapters with their own chipset
- Don't rely on DP Alt Mode

**Note:** Passive USB-C to HDMI adapters that rely on DP Alt Mode won't work.

## Setup Script

Run the setup script to check your system:

```bash
./scripts/hardware/setup_usbc_display.sh
```

This will:
- Check display status
- Check USB-C ports
- Verify DisplayPort support
- Install required packages
- Provide recommendations

## Display Configuration

### Check Available Displays

```bash
# List displays
xrandr --listmonitors

# Check DRM connectors
ls -la /sys/class/drm/card*/

# Check connector status
cat /sys/class/drm/card*/status
```

### Configure Display

```bash
# Enable a display connector
echo on > /sys/class/drm/card*/<connector>/dpms

# Configure with xrandr
xrandr --output <connector> --auto --primary

# Set resolution
xrandr --output <connector> --mode 1920x1080
```

### Helper Script

Use the display configuration helper:

```bash
./scripts/hardware/configure_display.sh
```

## Troubleshooting

### Monitor Not Detected

1. **Check USB-C port:**
   ```bash
   lsusb
   dmesg | grep -i usb
   ```

2. **Check display connectors:**
   ```bash
   cat /sys/class/drm/card*/status
   ```

3. **Try different USB-C port** (if multiple available)

4. **Check if dock requires external power**

### DisplayLink Dock Not Working

1. **Verify DisplayLink driver:**
   ```bash
   dpkg -l | grep displaylink
   ```

2. **Check DisplayLink service:**
   ```bash
   sudo systemctl status displaylink-driver
   ```

3. **Restart display manager:**
   ```bash
   sudo systemctl restart gdm3
   ```

4. **Check logs:**
   ```bash
   journalctl -u displaylink-driver -f
   ```

### No Video Output

1. **Use HDMI port instead** (most reliable)

2. **Check if monitor supports the connection type**

3. **Try different cable** (some cables don't support video)

4. **Check power** - USB-C docks often need external power

## Recommended Setup

For best compatibility:

1. **Primary Display:** Use HDMI port directly
2. **Secondary Display:** Use DisplayLink dock if needed
3. **USB-C Dock:** Use for USB ports, power, network (not video)

## Power Considerations

- USB-C docks often require external power supply
- Jetson Orin Nano power budget: ~15W
- Ensure dock has adequate power for all connected devices

## Additional Resources

- [NVIDIA Jetson Documentation](https://developer.nvidia.com/embedded/jetson-linux)
- [DisplayLink Linux Driver](https://www.displaylink.com/downloads/ubuntu)
- Jetson forums for specific hardware compatibility

## Quick Reference

```bash
# Setup USB-C display
./scripts/hardware/setup_usbc_display.sh

# Configure display
./scripts/hardware/configure_display.sh

# Check displays
xrandr --listmonitors

# Check USB devices
lsusb

# Check display connectors
cat /sys/class/drm/card*/status
```
