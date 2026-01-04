# System Maintenance Scripts

This directory contains scripts for automated system maintenance, updates, and recovery.

## Scripts

### `self_update.sh`

Automated system and package updates.

**Usage**:
```bash
sudo ./scripts/maintenance/self_update.sh [all|system|python|isaac]
```

**What it does**:
- Updates system packages (apt-get)
- Updates Python packages
- Checks for Isaac package updates (if installed)

**Automation**: Runs daily via `isaac-update.timer` at 3 AM

### `self_clean.sh`

Automated system cleanup.

**Usage**:
```bash
sudo ./scripts/maintenance/self_clean.sh
```

**What it does**:
- Cleans old logs (configurable retention)
- Removes temporary files
- Cleans Python cache
- Manages disk space (triggers aggressive cleanup if needed)

**Automation**: Runs weekly via `isaac-cleanup.timer` on Sunday at 2 AM

### `health_check.sh`

Automated health monitoring and service management.

**Usage**:
```bash
sudo ./scripts/maintenance/health_check.sh
```

**What it does**:
- Checks service status (restarts if needed)
- Monitors temperature
- Monitors disk usage (triggers cleanup if needed)
- Monitors memory usage
- Checks ROS 2 status

**Automation**: Runs every 15 minutes via `isaac-health-check.timer`

### `self_restore.sh`

Automated system restore from failures.

**Usage**:
```bash
sudo ./scripts/maintenance/self_restore.sh
```

**What it does**:
- Restarts failed services
- Restores ROS 2 workspace if missing
- Restores virtual environment if missing
- Runs health check

**Automation**: Triggered on service failure

## Automation

All maintenance scripts are automated via systemd timers:

- **Updates**: Daily at 3 AM
- **Cleanup**: Weekly on Sunday at 2 AM
- **Health Check**: Every 15 minutes
- **Restore**: On service failure

Install automation:
```bash
sudo ./scripts/system/install_services.sh
```

## Configuration

Edit scripts to adjust:
- Update frequency and types
- Cleanup retention periods
- Health check thresholds
- Disk space thresholds

See `docs/deployment/AUTOMATION.md` for detailed documentation.
