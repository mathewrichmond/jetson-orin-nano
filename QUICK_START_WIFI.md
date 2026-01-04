# Quick Start - WiFi Setup

## The Problem

You want WiFi to work when Ethernet is disconnected, with Ethernet preferred when available.

## Solution

WiFi is configured with **lower priority** than Ethernet, so it automatically connects when Ethernet is unavailable.

## Quick Setup

### During Unified Setup
```bash
./setup.sh
# Answer 'y' when prompted for WiFi setup
```

### Manual Setup
```bash
sudo ./scripts/system/setup_wifi.sh
```

## How It Works

- **Ethernet**: Priority 100 (preferred)
- **WiFi**: Priority 200 (fallback)
- **Auto-connect**: WiFi connects automatically when Ethernet is disconnected

## Test

```bash
# Check current status
./scripts/system/check_network.sh

# Disconnect Ethernet - WiFi should connect automatically
# Reconnect Ethernet - Ethernet takes priority
```

## Check Status

```bash
# Quick check
nmcli device status

# Detailed check
./scripts/system/check_network.sh
```

## More Info

See `docs/system/WIFI_SETUP.md` for detailed guide.
