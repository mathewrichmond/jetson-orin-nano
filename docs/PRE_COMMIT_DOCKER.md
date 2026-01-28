# Local Testing with Docker Pre-Commit Hooks

**Purpose**: Fast local testing before pushing to GitHub  
**Speed**: ~30 seconds vs 5-10 minutes GitHub Actions  
**Benefit**: CI consistency, catch issues immediately

---

## Quick Setup

```bash
# One-time setup
make pre-commit-install

# Use normally
git commit -m "message"  # Hooks run automatically
```

---

## Why Docker for Pre-Commit?

### The Problem
- GitHub Actions feedback is slow (5-10 minutes per push)
- Local environment might differ from CI
- Iterations on failing CI are painful

### The Solution
- **Pre-commit hooks** catch issues before push (< 1 minute)
- **Docker** ensures same environment as CI
- **Incremental builds** make subsequent runs fast

---

## Quick Start

### 1. Install Pre-Commit Hooks

```bash
# One-time setup
make pre-commit-install

# Or manually:
pip install pre-commit
pre-commit install
```

### 2. Test It Works

```bash
# Run hooks manually on all files
make pre-commit-test

# Or just test Docker tests
make docker-test
```

### 3. Commit Code

```bash
git add .
git commit -m "your message"

# Hooks will automatically run:
# 1. Trailing whitespace, YAML checks, etc.
# 2. Black, isort, flake8 (in Docker)
# 3. Unit tests (in Docker)
#
# If any fail, commit is blocked
```

---

## Usage

### Manual Testing (Without Commit)

```bash
# Run all Docker tests (lint + unit)
make docker-test

# Run only lint
make docker-lint

# Run only unit tests
make docker-unit

# Rebuild Docker image (if dependencies changed)
make docker-build
```

### What Runs On Commit

When you run `git commit`, these hooks run automatically:

1. **Standard checks** (fast, < 1s)
   - Trailing whitespace
   - YAML/JSON/TOML syntax
   - Large files check
   - Merge conflict markers

2. **Docker Lint** (~10-30s first run, ~2-5s cached)
   - Black formatting
   - isort import ordering
   - flake8 style checks

3. **Docker Unit Tests** (~10-30s first run, ~5-10s cached)
   - pytest unit tests
   - Fast, no hardware required

**Total time**: ~1 minute first run, ~10-30 seconds subsequent runs

---

## Docker Image Details

### Dockerfile.ci

Lightweight image for fast testing:
- **Base**: Ubuntu 22.04
- **Python**: 3.10
- **Size**: ~500MB (vs 5GB for full ROS 2 image)
- **Includes**: Only Python dev dependencies from pyproject.toml
- **Excludes**: ROS 2, hardware drivers, system deps

### Build Strategy

The Docker build is incremental:
```dockerfile
# Layer 1: Python install (cached)
# Layer 2: pyproject.toml deps (cached unless changed)
# Layer 3: Source code (changes often, but fast to copy)
```

**First build**: ~2-5 minutes  
**Rebuild after code change**: ~5 seconds  
**Rebuild after pyproject.toml change**: ~1-2 minutes

---

## Customization

### Skip Hooks Temporarily

```bash
# Skip all hooks (emergency only!)
git commit --no-verify -m "message"

# Skip specific hook
SKIP=docker-lint git commit -m "message"
SKIP=docker-unit-tests git commit -m "message"
```

### Disable Docker Hooks

Edit `.pre-commit-config.yaml`:
```yaml
# Comment out Docker hooks
# - repo: local
#   hooks:
#     - id: docker-lint
#     ...
```

### Speed Up Hooks

If hooks are too slow, disable unit tests:
```yaml
# Keep lint, remove unit tests
- repo: local
  hooks:
    - id: docker-lint  # Keep this
      ...
    # - id: docker-unit-tests  # Comment this out
```

---

## Troubleshooting

### "Docker not found"
```bash
# Install Docker
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh

# Add user to docker group (no sudo needed)
sudo usermod -aG docker $USER
newgrp docker
```

### "Docker image not found"
```bash
# Build the image
make docker-build
```

### "Pre-commit hook failed"
```bash
# See what failed
git commit -m "test"

# Run hooks manually to see detailed output
make pre-commit-test

# Run specific test
make docker-lint    # Just lint
make docker-unit    # Just unit tests
```

### "Hook takes too long"
```bash
# First run is slow (builds Docker image)
# Subsequent runs should be fast (~10-30s)

# If still slow, check Docker:
docker system df    # Check disk space
docker system prune # Clean up old images
```

### "Formatting issues"
```bash
# Auto-fix formatting (Black, isort)
black src/ tests/
isort src/ tests/

# Then commit
git add .
git commit -m "fix: formatting"
```

---

## Integration with GitHub Actions

### Pre-commit hooks vs CI

**Pre-commit** (Local, Fast):
- Runs before push
- Catches basic issues
- ~30 seconds
- Lint + unit tests only

**GitHub Actions** (Remote, Slow):
- Runs after push
- Full validation
- ~5-10 minutes
- Lint + unit + integration + build

**Strategy**: Use both
1. Pre-commit catches 90% of issues locally
2. GitHub Actions validates everything
3. Much faster iteration

---

## Files

### New Files Added

```
Dockerfile.ci                          # Lightweight CI Docker image
scripts/testing/docker_test.sh         # Docker test runner
.pre-commit-config.yaml                # Updated with Docker hooks
Makefile                               # Added docker-* commands
docs/PRE_COMMIT_DOCKER.md              # This file
```

### Configuration

**`.pre-commit-config.yaml`** - Hook configuration
```yaml
- repo: local
  hooks:
    - id: docker-lint
      name: Docker Lint Tests
      entry: bash -c './scripts/testing/docker_test.sh lint'
      ...
    
    - id: docker-unit-tests
      name: Docker Unit Tests
      entry: bash -c './scripts/testing/docker_test.sh unit'
      ...
```

**`Makefile`** - Shortcuts
```makefile
make docker-test        # Run all tests in Docker
make docker-lint        # Lint only
make docker-unit        # Unit tests only
make docker-build       # Rebuild image
make pre-commit-install # Setup hooks
```

---

## FAQ

**Q: Do I need Docker?**  
A: Yes, for CI consistency. But you can disable Docker hooks and just use standard linters.

**Q: Can I run tests without Docker?**  
A: Yes, use `make test-lint` and `make test-unit` (no Docker).

**Q: How much disk space does Docker use?**  
A: ~500MB for the CI image. Check with `docker system df`.

**Q: Can I use this on the Jetson?**  
A: Yes! Docker works on ARM64. First run will be slower to build.

**Q: What if I don't have internet?**  
A: After first build, Docker works offline. Pre-commit hooks work offline too.

**Q: Can I skip hooks for a quick commit?**  
A: Use `git commit --no-verify` but avoid this - defeats the purpose!

---

## Summary

**Before**:
- Push to GitHub → wait 5-10 min → CI fails → fix → repeat
- Slow feedback loop, frustrating iterations

**After**:
- Commit locally → hooks run in 30s → caught issues immediately
- Push to GitHub → CI validates everything → ✅ passes
- Fast feedback loop, fewer CI failures

**Setup**: One command
```bash
make pre-commit-install
```

**Use**: Normal git workflow
```bash
git add .
git commit -m "message"  # Hooks run automatically
git push
```

---

**Created**: 2026-01-28  
**Purpose**: Fast local testing with CI consistency  
**Time Saved**: ~10-20 minutes per failed CI run
