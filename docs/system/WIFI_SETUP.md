# WiFi Setup with Ethernet Priority

## Overview

The Isaac robot system is configured to prefer Ethernet connections but automatically fall back to WiFi when Ethernet is disconnected. This provides reliable network connectivity for robot operations.

## How It Works

### Priority System

NetworkManager uses **route metrics** to determine connection priority:
- **Lower metric = Higher priority**
- **Ethernet**: Metric 100 (preferred)
- **WiFi**: Metric 200 (fallback)

### Automatic Behavior

1. **Ethernet Connected**: Ethernet is used, WiFi stays disconnected
2. **Ethernet Disconnected**: WiFi automatically connects
3. **Ethernet Reconnected**: Ethernet takes priority again, WiFi disconnects

## Setup

### During Unified Setup

The WiFi setup is integrated into the unified setup script:

```bash
./setup.sh
```

When prompted, answer 'y' to setup WiFi. You'll be asked for:
- WiFi SSID (network name)
- WiFi password (if secured)

### Manual Setup

```bash
sudo ./scripts/system/setup_wifi.sh
```

This will:
1. Scan for available WiFi networks
2. Prompt for SSID and password
3. Create WiFi connection with fallback priority
4. Configure Ethernet priority (if connected)

## Check Network Status

```bash
# Quick status check
./scripts/system/check_network.sh

# Or use NetworkManager CLI
nmcli device status
nmcli connection show
```

## Configuration Details

### Route Metrics

- **Ethernet**: `ipv4.route-metric: 100`
- **WiFi**: `ipv4.route-metric: 200`

### Auto-Connect

WiFi is configured with:
- `connection.autoconnect: yes` - Auto-connects when available
- `connection.autoconnect-priority: 10` - Lower priority than Ethernet

## Testing

### Test WiFi Fallback

1. **Check current status:**
   ```bash
   nmcli device status
   ```

2. **Disconnect Ethernet cable**

3. **Wait a few seconds** - WiFi should connect automatically

4. **Check status again:**
   ```bash
   nmcli device status
   ip addr show
   ```

5. **Reconnect Ethernet** - Ethernet should take priority

### Verify Priority

```bash
# Check route metrics
nmcli connection show | grep -E "NAME|route-metric"

# Check default route
ip route | grep default
```

## Troubleshooting

### WiFi Not Connecting

1. **Check WiFi device:**
   ```bash
   nmcli device status
   ```

2. **Check WiFi is enabled:**
   ```bash
   nmcli radio wifi on
   ```

3. **Scan for networks:**
   ```bash
   nmcli device wifi list
   ```

4. **Check connection:**
   ```bash
   nmcli connection show
   ```

### Ethernet Not Taking Priority

1. **Check Ethernet metric:**
   ```bash
   nmcli connection show "Wired connection 1" | grep route-metric
   ```

2. **Set Ethernet metric manually:**
   ```bash
   sudo nmcli connection modify "Wired connection 1" ipv4.route-metric 100
   sudo nmcli connection up "Wired connection 1"
   ```

### WiFi Keeps Disconnecting

- Check WiFi signal strength: `nmcli device wifi list`
- Check power management: `iwconfig wlP1p1s0 | grep Power`
- Disable power saving: `sudo iwconfig wlP1p1s0 power off`

## Manual Configuration

### Update WiFi Connection

```bash
# Modify existing WiFi connection
sudo nmcli connection modify "WiFi-SSID" ipv4.route-metric 200
sudo nmcli connection up "WiFi-SSID"
```

### Update Ethernet Connection

```bash
# Modify existing Ethernet connection
sudo nmcli connection modify "Wired connection 1" ipv4.route-metric 100
sudo nmcli connection up "Wired connection 1"
```

### Create WiFi Connection Manually

```bash
sudo nmcli connection add \
    type wifi \
    con-name "WiFi-MyNetwork" \
    ifname wlP1p1s0 \
    ssid "MyNetwork" \
    wifi-sec.key-mgmt wpa-psk \
    wifi-sec.psk "password" \
    ipv4.method auto \
    ipv4.route-metric 200 \
    connection.autoconnect yes
```

## NetworkManager Commands

### Useful Commands

```bash
# List devices
nmcli device status

# List connections
nmcli connection show

# Show active connections
nmcli connection show --active

# Connect to WiFi
nmcli device wifi connect "SSID" password "password"

# Disconnect WiFi
nmcli device disconnect wlP1p1s0

# Restart NetworkManager
sudo systemctl restart NetworkManager
```

## Best Practices

1. **Use Ethernet for primary connection** - More reliable and faster
2. **WiFi as fallback** - Ensures connectivity when mobile
3. **Test fallback** - Periodically test WiFi fallback works
4. **Monitor connectivity** - Use system monitor to track network status
5. **Keep WiFi credentials secure** - Don't commit passwords to git

## Integration with System Monitor

The system monitor can track network connectivity:

```bash
# Monitor network status via ROS 2
ros2 topic echo /system/status
```

## See Also

- [Network Configuration](NETWORK.md) - General network setup
- [System Setup](../setup/SETUP.md) - Complete setup guide
- NetworkManager documentation: `man nmcli`
