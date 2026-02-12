# Archived Bare-Metal Scripts

**⚠️ DEPRECATED**: These scripts are from the old bare-metal architecture and are **no longer maintained**.

The repository has migrated to a container-first architecture. See [MIGRATION_TO_CONTAINERS.md](../../MIGRATION_TO_CONTAINERS.md) for details.

## What's Here

This directory contains legacy scripts from the bare-metal ROS 2 installation approach:

- `setup-bare-metal.sh` - Old system setup (installed ROS 2 on host)
- `install_services.sh` - Old systemd service installation
- `start_robot.sh` - Old direct ROS 2 launch script
- `manage_graph.sh` - Old node management system

## Do Not Use

These scripts are kept for reference only. They will not work with the new container-first system.

## Current Approach

Use the new container-first system:

1. **Host setup**: `./setup-host.sh` (minimal host configuration)
2. **Container deployment**: `./scripts/deployment/deploy_containers.sh`

See documentation:
- `docs/setup/CONTAINER_FIRST_ARCHITECTURE.md`
- `MIGRATION_TO_CONTAINERS.md`

---

**Last Updated**: 2026-02-12  
**Status**: Archived, not maintained
