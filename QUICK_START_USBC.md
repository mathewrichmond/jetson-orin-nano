# Quick Start - USB-C Display & Dock

## The Problem

Jetson Orin Nano USB-C ports **may not support DisplayPort Alt Mode**, so USB-C monitors/docks may not work like on a laptop.

## Quick Solutions

### Option 1: Use HDMI (Easiest)
```bash
# Just plug HDMI cable directly into HDMI port
# Most reliable solution
```

### Option 2: DisplayLink Dock
```bash
# Install DisplayLink driver
sudo apt install displaylink-driver

# Restart display manager
sudo systemctl restart gdm3

# Configure display
xrandr --listmonitors
xrandr --output <display> --auto --primary
```

### Option 3: Check Your Setup
```bash
# Run setup script
./scripts/hardware/setup_usbc_display.sh

# Check displays
./scripts/hardware/configure_display.sh
```

## What Works

✅ **HDMI port** - Direct connection  
✅ **DisplayLink docks** - Software-based, works via USB  
✅ **USB-C for data/power** - Standard USB functionality  

## What Doesn't Work

❌ **DP Alt Mode USB-C** - Hardware limitation  
❌ **Passive USB-C to HDMI** - Requires DP Alt Mode  
❌ **Standard USB-C monitors** - If they require DP Alt Mode  

## Check Your Setup

```bash
# Check USB devices
lsusb

# Check display connectors
cat /sys/class/drm/card*/status

# List displays
xrandr --listmonitors
```

## More Info

See `docs/hardware/USBC_DISPLAY.md` for detailed guide.
