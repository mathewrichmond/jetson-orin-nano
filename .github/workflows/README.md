# Workflows Status

## Disabled Workflows

The following workflows are temporarily disabled while focusing on local development:

- `ci.yml.disabled` - CI pipeline (had ROS 2 build issues)
- `test.yml.disabled` - Test suite (had setup failures)

## Why Disabled?

These workflows had persistent issues:
1. ROS 2 build failures in GitHub Actions environment
2. Setup job failures (GitHub Actions infrastructure)
3. Slow feedback loop (5-10 minutes per push)

## Alternative: Local Development with Docker

Use the Docker-based pre-commit hooks instead:

```bash
# Setup (one-time)
./QUICK_START_PRECOMMIT.sh

# Use normally - hooks run on commit
git commit -m "message"

# Manual testing
make docker-test        # All tests
make docker-lint        # Lint only
make docker-unit        # Unit tests only
```

**Benefits**:
- Fast feedback (~30 seconds)
- CI consistency (Docker environment)
- No GitHub Actions quota usage
- Iterate quickly

## Active Workflows

- `deploy.yml` - Manual deployment (workflow_dispatch only)
- `deploy-dev.yml` - Dev deployment (if exists)
- `release.yml` - Release automation (on version tags)

## Re-enabling CI Workflows

When ready to fix and re-enable:

1. **Fix the issues locally first** using Docker tests
2. **Rename back**: `mv ci.yml.disabled ci.yml`
3. **Test with a PR** (not direct to main)
4. **Monitor**: https://github.com/mathewrichmond/jetson-orin-nano/actions

## Recommended Workflow

**For daily development**:
1. Use Docker pre-commit hooks (fast, local)
2. Skip GitHub Actions entirely
3. Focus on code quality

**For releases/production**:
1. Enable workflows temporarily
2. Test thoroughly
3. Disable again if issues persist

---

**Date**: 2026-01-28  
**Status**: CI workflows disabled, local Docker testing enabled  
**See**: `SETUP_PRE_COMMIT.md` for local testing guide
