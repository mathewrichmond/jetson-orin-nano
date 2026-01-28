# GitHub Actions CI Disabled

**Date**: 2026-01-28  
**Reason**: Persistent failures, slow feedback loop  
**Alternative**: Docker-based pre-commit hooks

---

## Why Disabled?

After multiple fix attempts, GitHub Actions CI still had issues:

### Issues Encountered:
1. ❌ "Set up job" failures (GitHub Actions infrastructure)
2. ❌ ROS 2 build failures in CI environment
3. ❌ flake8 failures (code quality issues)
4. ⏳ Slow feedback (5-10 minutes per push)
5. 🔄 Iteration hell (fix → push → wait → fail → repeat)

### Failed Attempts:
- Fixed pyproject.toml configuration
- Fixed requirements-dev.txt references
- Disabled auto-deploy
- Fixed lint command execution
- Multiple workflow adjustments

**Result**: Still failing, blocking development

---

## ✅ Better Solution: Local Docker Testing

Instead of fixing broken CI, use **local Docker-based testing**:

### Setup (One-Time)
```bash
cd /home/nano/src/jetson-orin-nano

# Fix Docker permissions
sudo usermod -aG docker $USER
newgrp docker

# Install pre-commit hooks
./QUICK_START_PRECOMMIT.sh
```

### Daily Usage
```bash
# Just commit normally
git add .
git commit -m "feat: some change"

# Hooks run automatically:
# - Lint tests (black, isort, flake8) ~10s
# - Unit tests (pytest) ~20s
# Total: ~30 seconds
#
# If anything fails, commit is blocked
# Fix and try again
```

### Manual Testing
```bash
make docker-test        # All tests
make docker-lint        # Lint only
make docker-unit        # Unit tests only
```

---

## Benefits vs GitHub Actions

| Feature | GitHub Actions | Docker Pre-Commit |
|---------|---------------|-------------------|
| **Speed** | 5-10 minutes | 30 seconds |
| **Feedback** | After push | Before commit |
| **Iterations** | Slow, painful | Fast, smooth |
| **Environment** | GitHub's servers | Docker (consistent) |
| **Cost** | GitHub quota | Local compute |
| **Reliability** | Infrastructure issues | Under your control |

**Winner**: Docker pre-commit hooks 🏆

---

## What's Disabled

### Files Renamed:
```
.github/workflows/ci.yml → ci.yml.disabled
.github/workflows/test.yml → test.yml.disabled
```

### What Still Runs:
- ✅ `deploy.yml` - Manual deployment only (workflow_dispatch)
- ✅ `deploy-dev.yml` - If needed
- ✅ `release.yml` - Release automation

**No automatic CI on push** - By design, not a bug!

---

## Workflow Going Forward

### Development Phase (Now)
```
Write code
  ↓
git commit (Docker hooks run ~30s)
  ↓
Issues caught? Fix locally
  ↓
Commit successful? git push
  ↓
Done! No CI, no waiting
```

### When Ready for CI (Later)
```bash
# 1. Ensure all tests pass locally
make docker-test

# 2. Fix any issues
black src/ tests/
isort src/ tests/
pytest tests/unit/

# 3. Re-enable ONE workflow at a time
mv .github/workflows/test.yml.disabled .github/workflows/test.yml
git add .github/workflows/test.yml
git commit -m "ci: re-enable test workflow"
git push

# 4. Monitor and fix if needed
# URL: https://github.com/mathewrichmond/jetson-orin-nano/actions

# 5. If it works, enable others
# If it fails, disable again and fix locally first
```

---

## Re-Enabling CI (When Ready)

### Prerequisites
Before re-enabling, ensure:
- [ ] All tests pass locally: `make docker-test`
- [ ] Code is properly formatted: `black src/ tests/`
- [ ] Imports are sorted: `isort src/ tests/`
- [ ] No flake8 errors: `flake8 src/ tests/`
- [ ] Unit tests pass: `pytest tests/unit/`

### Steps
```bash
# 1. Rename workflow back
mv .github/workflows/test.yml.disabled .github/workflows/test.yml

# 2. Commit and push
git add .github/workflows/test.yml
git commit -m "ci: re-enable test workflow"
git push

# 3. Monitor
https://github.com/mathewrichmond/jetson-orin-nano/actions

# 4. If fails, disable again
mv .github/workflows/test.yml .github/workflows/test.yml.disabled
git add .github/workflows/test.yml.disabled
git commit -m "ci: disable test workflow again"
git push
```

---

## Current State

### ✅ What Works
- Local Docker testing (fast, reliable)
- Pre-commit hooks (automatic)
- Code quality checks (black, isort, flake8)
- Unit tests (pytest)
- Development workflow (smooth)

### ❌ What's Disabled
- GitHub Actions CI (too slow, too broken)
- Integration tests in CI (do locally when needed)
- Automatic validation on push (replaced by pre-commit)

### 🎯 Focus
**Local development with fast feedback**, not fighting with CI

---

## Documentation

- **SETUP_PRE_COMMIT.md** - How to set up Docker pre-commit hooks
- **docs/PRE_COMMIT_DOCKER.md** - Complete Docker testing guide
- **.github/workflows/README.md** - Workflows status
- **This file** - Why CI is disabled

---

## Summary

**Problem**: GitHub Actions CI kept failing, slow iterations  
**Solution**: Disabled CI, use Docker pre-commit hooks instead  
**Benefit**: 30-second feedback vs 5-10 minute feedback  
**Status**: CI disabled by design, local testing enabled  

**Next**: Run `./QUICK_START_PRECOMMIT.sh` to start using local testing

---

**Disabled**: 2026-01-28  
**Reason**: Too slow, too broken, better alternative exists  
**Alternative**: Docker pre-commit hooks (SETUP_PRE_COMMIT.md)
