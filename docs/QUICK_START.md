# Quick Start Guide

**Get up and running in 3 steps**

---

## 1. Install Dependencies

```bash
cd /home/nano/src/jetson-orin-nano

# Install Python dependencies
pip install -e ".[dev]"

# Install ROS 2 (if needed)
# See setup.sh or docs/setup/SETUP.md
```

---

## 2. Set Up Local Testing

```bash
# Install pre-commit hooks (one-time)
make pre-commit-install

# This enables:
# - Automatic lint + unit tests on commit (~30s)
# - Docker-based CI consistency
```

---

## 3. Build and Run

```bash
# Build ROS 2 workspace
colcon build --symlink-install

# Launch system
ros2 launch isaac_robot graph.launch.py graph:=minimal_graph.yaml
```

---

## Daily Workflow

```bash
# 1. Make changes
vim src/some_file.py

# 2. Test locally
make docker-test        # Optional: manual testing

# 3. Commit (hooks run automatically)
git commit -m "feat: your change"

# 4. Push
git push
```

---

## Next Steps

- **Full setup**: See [MODULAR_QUICK_START.md](MODULAR_QUICK_START.md)
- **Local testing**: See [LOCAL_TESTING.md](development/LOCAL_TESTING.md)
- **Deployment**: See [DEPLOYMENT.md](DEPLOYMENT.md)
- **Testing guide**: See [TESTING.md](TESTING.md)

---

**Time to setup**: ~10 minutes  
**Time per commit**: ~30 seconds (after setup)
