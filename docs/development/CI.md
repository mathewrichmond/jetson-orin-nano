# Continuous Integration Setup

**Status**: GitHub Actions CI workflows disabled  
**Alternative**: Local Docker-based testing (faster, more reliable)

---

## Current CI Status

### Disabled Workflows

The following workflows are disabled to avoid slow feedback loops:

- `ci.yml.disabled` - CI pipeline
- `test.yml.disabled` - Test suite

**Reason**: Persistent failures, slow iterations (5-10 min per push)

### Active Workflows

- `deploy.yml` - Manual deployment only (workflow_dispatch)
- `release.yml` - Release automation (on version tags)

---

## Local Testing Alternative

Instead of GitHub Actions, use Docker-based pre-commit hooks:

### Setup
```bash
# One-time setup
make pre-commit-install
```

### Usage
```bash
# Hooks run automatically on commit
git commit -m "message"

# Manual testing
make docker-test        # All tests (~30s)
make docker-lint        # Lint only
make docker-unit        # Unit tests only
```

**Benefits**: 30-second feedback vs 5-10 minutes

---

## Re-Enabling GitHub Actions CI

When ready to fix and re-enable:

### 1. Fix Issues Locally First
```bash
# Ensure everything passes
make docker-test
black src/ tests/
isort src/ tests/
```

### 2. Re-Enable One Workflow
```bash
mv .github/workflows/test.yml.disabled .github/workflows/test.yml
git add .github/workflows/test.yml
git commit -m "ci: re-enable test workflow"
git push
```

### 3. Monitor
https://github.com/YOUR_REPO/actions

### 4. If Fails, Disable Again
```bash
mv .github/workflows/test.yml .github/workflows/test.yml.disabled
```

---

## See Also

- [Local Testing](LOCAL_TESTING.md) - Docker pre-commit hooks
- [Testing Guide](../TESTING.md) - All testing options
- [Deployment](../DEPLOYMENT.md) - Manual deployment

---

**Updated**: 2026-01-28  
**Focus**: Local development, disable broken CI
