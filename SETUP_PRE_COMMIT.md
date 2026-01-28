# Setup Pre-Commit Hooks with Docker

**Quick setup guide for fast local CI testing**

---

## Step 1: Fix Docker Permissions (One-Time)

```bash
# Add yourself to docker group
sudo usermod -aG docker $USER

# Apply group changes (logout/login or use newgrp)
newgrp docker

# Test Docker works without sudo
docker ps
```

---

## Step 2: Install Pre-Commit (One-Time)

```bash
# Install pre-commit
pip install --user pre-commit

# Or use system package manager
sudo apt install pre-commit
```

---

## Step 3: Install Hooks (One-Time)

```bash
cd /home/nano/src/jetson-orin-nano

# Install pre-commit hooks
make pre-commit-install

# This will:
# 1. Install hooks to .git/hooks/
# 2. Build Docker image (first time only, ~2-5 min)
```

---

## Step 4: Test It Works

```bash
# Test Docker build
make docker-build

# Test lint
make docker-lint

# Test unit tests  
make docker-unit

# Test everything
make docker-test
```

---

## Step 5: Normal Usage

Now just commit normally:

```bash
git add .
git commit -m "your message"

# Hooks will automatically:
# 1. Check formatting
# 2. Run lint tests in Docker
# 3. Run unit tests in Docker
# 
# If anything fails, commit is blocked
# Fix issues and try again
```

---

## What You Get

### Before (Slow Loop)
```
Edit code → Push → Wait 5-10 min → CI fails → Fix → Repeat
```

### After (Fast Loop)
```
Edit code → Commit → Hooks run (30s) → Issues caught → Fix → Commit again
                                       ↓
                                    All good? Push → CI validates → ✅
```

---

## Quick Commands

```bash
# Run tests manually (no commit)
make docker-test          # All tests
make docker-lint          # Just lint
make docker-unit          # Just unit tests

# Skip hooks (emergency only)
git commit --no-verify

# Update hooks
pre-commit autoupdate

# Reinstall hooks
make pre-commit-install
```

---

## Troubleshooting

**"Permission denied" for Docker**
```bash
sudo usermod -aG docker $USER
newgrp docker
```

**"Docker image not found"**
```bash
make docker-build
```

**Hooks are slow**
- First run builds Docker image (~2-5 min)
- Subsequent runs are fast (~30s)
- If still slow, run `docker system prune`

**Want to skip unit tests** (keep only lint)
- Edit `.pre-commit-config.yaml`
- Comment out `docker-unit-tests` hook

---

## Full Documentation

See `docs/PRE_COMMIT_DOCKER.md` for:
- Detailed explanation
- Customization options
- Troubleshooting guide
- FAQ

---

**Setup Time**: ~5 minutes one-time  
**Per-Commit Time**: ~30 seconds (after first build)  
**Time Saved**: ~10-20 minutes per failed CI run
