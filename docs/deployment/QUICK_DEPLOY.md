# Quick Deploy Guide

## Overview

Quick deployment tools for fast iteration during development. These tools sync code changes and rebuild packages efficiently.

## Tools

### 1. Quick Deploy (`quick_deploy.sh`)

Fast deployment using `rsync` - only syncs changed files:

```bash
./scripts/deployment/quick_deploy.sh
```

**What it does:**
- Syncs source code via rsync (faster than tar/scp)
- Rebuilds ROS 2 workspace on target
- Optionally restarts services

**When to use:**
- Deploying to remote target
- After making code changes
- When you want to test on hardware

### 2. Rebuild Local (`rebuild_local.sh`)

Quick rebuild of ROS 2 packages locally:

```bash
./scripts/deployment/rebuild_local.sh
```

**What it does:**
- Links packages from dev sandbox
- Rebuilds ROS 2 workspace
- Sources new build

**When to use:**
- After making local code changes
- Before testing locally
- Quick rebuild without deployment

### 3. Deploy Dev (`deploy_dev.sh`)

Full deployment with archive (slower but more reliable):

```bash
./scripts/deployment/deploy_dev.sh
```

**What it does:**
- Creates tar archive
- Deploys to target
- Optionally restarts services

**When to use:**
- First-time deployment
- When rsync might have issues
- When you want a complete archive

## Workflow

### Local Development

```bash
# 1. Make code changes
vim src/system_monitor/system_monitor/system_monitor_node.py

# 2. Rebuild locally
./scripts/deployment/rebuild_local.sh

# 3. Test locally
source ~/ros2_ws/install/setup.bash
ros2 launch isaac_robot minimal.launch.py
```

### Remote Deployment

```bash
# 1. Make code changes
vim src/system_monitor/system_monitor/system_monitor_node.py

# 2. Quick deploy to target
./scripts/deployment/quick_deploy.sh

# 3. Test on target (SSH in)
ssh nano@isaac.local
source ~/ros2_ws/install/setup.bash
ros2 launch isaac_robot minimal.launch.py
```

## Configuration

### Target Settings

Set environment variables:
```bash
export TARGET_HOST=isaac.local
export TARGET_USER=nano
export TARGET_DEV_DIR=/home/nano/src/jetson-orin-nano
```

Or edit scripts directly.

### Excluded Files

These files/directories are excluded from sync:
- `.git/`
- `__pycache__/`, `*.pyc`
- `build/`, `install/`, `log/`
- `.venv/`, `venv/`
- `*.egg-info/`, `dist/`
- `.setup_state`, `.setup.log`
- `.ros/`
- `*.swp`, `*.swo`, `*~`

## Speed Comparison

- **rsync (quick_deploy.sh)**: ~5-30 seconds (depends on changes)
- **tar/scp (deploy_dev.sh)**: ~30-60 seconds
- **rebuild_local.sh**: ~10-20 seconds

## Tips

1. **Use rebuild_local.sh** for local testing
2. **Use quick_deploy.sh** for remote deployment
3. **Use deploy_dev.sh** for first-time or problematic deployments
4. **Rebuild on target** after deploying to ensure packages are built
5. **Check services** after deployment if using systemd

## Troubleshooting

### rsync fails
```bash
# Fall back to deploy_dev.sh
./scripts/deployment/deploy_dev.sh
```

### Packages not found after deploy
```bash
# Rebuild on target
ssh nano@isaac.local
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### Services not restarting
```bash
# Manually restart
ssh nano@isaac.local
sudo systemctl restart isaac-robot.service
sudo systemctl restart isaac-system-monitor.service
```

## Integration with Graph Selection

After deploying, you can select which graph to use:

```bash
# On target
ssh nano@isaac.local
cd /home/nano/src/jetson-orin-nano
./scripts/utils/select_graph.sh minimal
sudo systemctl restart isaac-robot.service
```
