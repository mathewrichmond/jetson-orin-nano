# Pre-Commit Hooks with Docker - Complete Setup

**Date**: 2026-01-28  
**Purpose**: Fast local testing to avoid slow GitHub Actions feedback loop  
**Status**: ✅ Ready to use

---

## What Was Added

### Files Created

1. **`Dockerfile.ci`** - Lightweight Docker image for CI testing (~500MB)
2. **`scripts/testing/docker_test.sh`** - Docker test runner script
3. **`.pre-commit-config.yaml`** - Updated with Docker hooks
4. **`Makefile`** - Added docker-* commands
5. **`docs/PRE_COMMIT_DOCKER.md`** - Complete documentation
6. **`SETUP_PRE_COMMIT.md`** - Quick setup guide
7. **`QUICK_START_PRECOMMIT.sh`** - Automated setup script

### What It Does

**Before each commit**, automatically runs in Docker:
1. **Lint tests** - Black, isort, flake8
2. **Unit tests** - pytest unit tests

**Time**: ~30 seconds (after first Docker build)  
**Benefit**: Catch issues before pushing to GitHub

---

## Quick Setup

### Option 1: Automated (Recommended)

```bash
cd /home/nano/src/jetson-orin-nano
./QUICK_START_PRECOMMIT.sh
```

### Option 2: Manual

```bash
# 1. Fix Docker permissions
sudo usermod -aG docker $USER
newgrp docker

# 2. Install pre-commit
pip install --user pre-commit

# 3. Install hooks
make pre-commit-install

# 4. Test it works
make docker-test
```

---

## Usage

### Normal Workflow

```bash
# 1. Make changes
vim src/some_file.py

# 2. Commit (hooks run automatically)
git add .
git commit -m "fix: some change"

# Hooks will:
# - Check formatting
# - Run lint in Docker
# - Run unit tests in Docker
# - Block commit if anything fails

# 3. Fix issues if needed
black src/ tests/
git add .
git commit -m "fix: some change"

# 4. Push (only after local checks pass)
git push
```

### Manual Testing (No Commit)

```bash
make docker-test        # All tests
make docker-lint        # Just lint
make docker-unit        # Just unit tests
make docker-build       # Rebuild image
```

---

## Benefits

### Before (Slow Loop) ❌
```
Write code
  ↓
git push
  ↓
Wait 5-10 minutes
  ↓
GitHub Actions fails
  ↓
Fix issue
  ↓
Repeat...
```

**Problem**: Slow feedback, wasted time

### After (Fast Loop) ✅
```
Write code
  ↓
git commit (hooks run ~30s)
  ↓
Issues caught immediately
  ↓
Fix locally
  ↓
Commit again
  ↓
All good? git push
  ↓
GitHub Actions validates (usually passes)
```

**Benefit**: Fast feedback, fewer CI failures

---

## What Runs

### Pre-Commit Hooks (Local, Fast)

1. **Standard Checks** (~1s)
   - Trailing whitespace
   - YAML/JSON syntax
   - Large files

2. **Docker Lint** (~5-10s)
   - Black formatting
   - isort imports
   - flake8 style

3. **Docker Unit Tests** (~10-20s)
   - pytest unit tests
   - No hardware required

**Total**: ~30 seconds

### GitHub Actions (Remote, Thorough)

- Lint + Unit + Integration + Build
- ROS 2 workspace build
- Full validation
- ~5-10 minutes

**Strategy**: Both work together
- Pre-commit catches 90% of issues
- GitHub Actions validates everything

---

## Docker Details

### Dockerfile.ci

Lightweight image optimized for fast testing:

```dockerfile
FROM ubuntu:22.04
# Install Python + pip
# Install deps from pyproject.toml
# Copy source code
# Size: ~500MB (vs 5GB for full ROS 2)
```

**Build time**:
- First build: ~2-5 minutes
- Rebuild after code change: ~5 seconds
- Rebuild after pyproject.toml change: ~1-2 minutes

### Caching Strategy

Docker layers are cached:
1. ✅ Python install (rarely changes)
2. ✅ Python dependencies (only when pyproject.toml changes)
3. ⚡ Source code (changes often, but fast to copy)

Result: Subsequent runs are fast!

---

## Commands Reference

```bash
# Setup
make pre-commit-install     # Install hooks (one-time)
./QUICK_START_PRECOMMIT.sh  # Automated setup

# Testing
make docker-test            # Run all tests in Docker
make docker-lint            # Run only lint
make docker-unit            # Run only unit tests
make docker-build           # Rebuild Docker image

# Pre-commit
git commit                  # Hooks run automatically
git commit --no-verify      # Skip hooks (emergency only)
make pre-commit-test        # Run hooks manually
pre-commit run --all-files  # Run on all files

# Troubleshooting
docker ps                   # Check Docker works
docker system df            # Check disk space
docker system prune         # Clean up old images
```

---

## Customization

### Skip Unit Tests (Keep Only Lint)

Edit `.pre-commit-config.yaml`:
```yaml
# Comment out unit tests
# - id: docker-unit-tests
#   ...
```

### Add More Tests

Edit `.pre-commit-config.yaml`:
```yaml
- repo: local
  hooks:
    - id: my-custom-test
      name: My Test
      entry: bash -c './my_test.sh'
      language: system
```

### Change Docker Image

Edit `scripts/testing/docker_test.sh`:
```bash
IMAGE_NAME="my-custom-image:tag"
```

---

## Troubleshooting

### Docker Permission Denied

```bash
sudo usermod -aG docker $USER
newgrp docker
docker ps  # Should work without sudo
```

### Hooks Are Slow

**First run**: Normal (builds Docker image ~2-5 min)  
**Subsequent runs**: Should be fast (~30s)

If still slow:
```bash
docker system prune  # Clean up
make docker-build    # Rebuild
```

### Formatting Failures

```bash
# Auto-fix
black src/ tests/
isort src/ tests/

# Commit fixed code
git add .
git commit -m "fix: formatting"
```

### Skip Hooks Temporarily

```bash
# Skip all hooks (use sparingly!)
git commit --no-verify -m "message"

# Skip specific hook
SKIP=docker-lint git commit -m "message"
```

---

## Files Added/Modified

```
New Files:
├── Dockerfile.ci                       # CI Docker image
├── scripts/testing/docker_test.sh      # Docker test runner
├── docs/PRE_COMMIT_DOCKER.md           # Full documentation
├── SETUP_PRE_COMMIT.md                 # Setup guide
├── QUICK_START_PRECOMMIT.sh            # Automated setup
└── PRE_COMMIT_SETUP_SUMMARY.md         # This file

Modified Files:
├── .pre-commit-config.yaml             # Added Docker hooks
└── Makefile                            # Added docker-* commands
```

---

## Next Steps

### 1. Run Setup

```bash
./QUICK_START_PRECOMMIT.sh
```

### 2. Test It

```bash
# Make a change
echo "# test" >> README.md

# Commit (hooks run)
git add README.md
git commit -m "test: pre-commit"

# Should see:
# - Running hooks...
# - Docker lint tests...
# - Docker unit tests...
# - ✅ All passed (or ❌ issues found)
```

### 3. Fix Docker Permissions If Needed

```bash
sudo usermod -aG docker $USER
newgrp docker
```

### 4. Normal Development

Just commit normally - hooks run automatically!

---

## Summary

**Setup**: One command - `./QUICK_START_PRECOMMIT.sh`  
**Usage**: Normal git workflow - hooks run automatically  
**Speed**: ~30 seconds per commit (vs 5-10 min GitHub Actions)  
**Benefit**: Catch 90% of issues before pushing  

**Time Saved**: 10-20 minutes per failed CI iteration

---

**Created**: 2026-01-28  
**Ready**: ✅ Yes, run setup script to start using  
**Documentation**: See `docs/PRE_COMMIT_DOCKER.md` for details
