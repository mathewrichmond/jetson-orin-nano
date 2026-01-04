# Automation and Maintenance

## Overview

The Isaac robot system includes comprehensive automation for:
- **Auto-start on boot** - Services launch automatically
- **Periodic updates** - Automated system and package updates
- **Maintenance tasks** - Automated cleanup and health checks
- **Self-healing** - Automatic service restart and recovery

## Systemd Services

### Main Services

- **`isaac-robot.service`** - Main robot system
  - Auto-starts on boot
  - Restarts on failure
  - Watchdog monitoring (60s)

- **`isaac-system-monitor.service`** - System monitoring
  - Auto-starts on boot
  - Restarts on failure

### Maintenance Services

- **`isaac-update.service`** - System updates
  - Triggered by `isaac-update.timer`
  - Runs daily at 3 AM
  - Updates system packages, Python packages

- **`isaac-cleanup.service`** - System cleanup
  - Triggered by `isaac-cleanup.timer`
  - Runs weekly on Sunday at 2 AM
  - Cleans logs, temporary files, manages disk space

- **`isaac-health-check.service`** - Health monitoring
  - Triggered by `isaac-health-check.timer`
  - Runs every 15 minutes
  - Checks services, temperature, disk, memory
  - Automatically restarts failed services

- **`isaac-restore.service`** - System restore
  - Triggered on service failure
  - Attempts to restore from common failures

## Installation

Install all services and timers:

```bash
sudo ./scripts/system/install_services.sh
```

This installs:
- All service files
- All timer files
- Enables auto-start
- Starts maintenance timers

## Timer Schedule

| Timer | Schedule | Purpose |
|-------|----------|---------|
| `isaac-update.timer` | Daily at 3 AM | System and package updates |
| `isaac-cleanup.timer` | Weekly (Sun 2 AM) | Log cleanup, disk management |
| `isaac-health-check.timer` | Every 15 minutes | Health monitoring, service checks |

## Manual Operations

### Check Timer Status

```bash
# List all Isaac timers
systemctl list-timers isaac-*

# Check specific timer
systemctl status isaac-update.timer
```

### Run Maintenance Manually

```bash
# Run update
sudo systemctl start isaac-update.service

# Run cleanup
sudo systemctl start isaac-cleanup.service

# Run health check
sudo systemctl start isaac-health-check.service

# Run restore
sudo systemctl start isaac-restore.service
```

### Check Service Status

```bash
# Check all services
systemctl status isaac-robot
systemctl status isaac-system-monitor

# Check if services are enabled
systemctl is-enabled isaac-robot
systemctl is-enabled isaac-system-monitor
```

## Logs

### Service Logs

```bash
# View service logs
journalctl -u isaac-robot -f
journalctl -u isaac-system-monitor -f

# View maintenance logs
tail -f /var/log/isaac-update.log
tail -f /var/log/isaac-cleanup.log
tail -f /var/log/isaac-health.log
tail -f /var/log/isaac-restore.log
```

### All Isaac Logs

```bash
journalctl -u isaac-* -f
```

## Configuration

### Update Schedule

Edit timer files in `config/systemd/`:
- `isaac-update.timer` - Change `OnCalendar` for update schedule
- `isaac-cleanup.timer` - Change `OnCalendar` for cleanup schedule
- `isaac-health-check.timer` - Change `OnCalendar` for health check frequency

After editing, reinstall services:
```bash
sudo ./scripts/system/install_services.sh
```

### Thresholds

Edit maintenance scripts to adjust thresholds:
- `scripts/maintenance/health_check.sh` - Temperature, disk, memory thresholds
- `scripts/maintenance/self_clean.sh` - Disk thresholds, retention periods

## Self-Healing Features

### Automatic Service Restart

Services are configured with:
- `Restart=on-failure` - Restart on failure
- `RestartSec=10` - Wait 10 seconds before restart
- `StartLimitBurst=5` - Allow 5 restarts before giving up
- `WatchdogSec=60` - Watchdog timeout

### Health Check Actions

Health check automatically:
- Restarts failed services
- Triggers cleanup if disk space is low
- Logs warnings for temperature/memory issues

### Restore Actions

Restore script attempts to:
- Restart failed services
- Recreate ROS 2 workspace if missing
- Recreate virtual environment if missing
- Run health check

## Monitoring

### Check Automation Status

```bash
# Check all timers
systemctl list-timers isaac-*

# Check service status
systemctl status isaac-*

# Check recent maintenance runs
tail -20 /var/log/isaac-*.log
```

### Verify Auto-Start

After reboot, verify services started:
```bash
systemctl status isaac-robot
systemctl status isaac-system-monitor
systemctl list-timers isaac-*
```

## Troubleshooting

### Services Not Starting

1. Check logs: `journalctl -u isaac-robot -n 50`
2. Check ROS 2: `source /opt/ros/humble/setup.bash && ros2 --help`
3. Check permissions: `ls -la /home/nano/src/jetson-orin-nano/setup.sh`
4. Run restore: `sudo systemctl start isaac-restore.service`

### Timers Not Running

1. Check timer status: `systemctl status isaac-update.timer`
2. Check if enabled: `systemctl is-enabled isaac-update.timer`
3. Manually trigger: `sudo systemctl start isaac-update.service`
4. Reinstall: `sudo ./scripts/system/install_services.sh`

### Maintenance Not Running

1. Check timer: `systemctl list-timers isaac-*`
2. Check logs: `tail -f /var/log/isaac-update.log`
3. Run manually: `sudo ./scripts/maintenance/self_update.sh`

## Best Practices

1. **Monitor logs regularly** - Check `/var/log/isaac-*.log`
2. **Review health checks** - Ensure thresholds are appropriate
3. **Test manually first** - Run maintenance scripts manually before relying on automation
4. **Keep backups** - Important configurations should be backed up
5. **Monitor disk space** - Ensure cleanup is working
