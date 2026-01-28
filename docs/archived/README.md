# Archived Documentation

**Purpose**: This directory contains documentation from the pre-modular architecture (before 2026-01-27).

---

## Why Archived?

These documents describe the old monolithic architecture with:
- `sensor_sync` node (replaced by modular architecture)
- Old graph configurations (`robot_graph.yaml`, `stable_graph.yaml`)
- Old launch system (`composable_graph.launch.py`)

The system has been refactored to a modular architecture with:
- 5 independent modules (chassis_control, vision_pipeline, power_management, audio_pipeline, vla_planner)
- New graph configurations (`modular_graph.yaml`, `minimal_graph.yaml`, etc.)
- New launch system (`graph.launch.py`, `distributed.launch.py`)

---

## Current Documentation

See the main `docs/` directory for current documentation:
- **MODULAR_QUICK_START.md** - How to use the new system
- **DEPLOYMENT.md** - Multi-environment deployment
- **TESTING.md** - Testing framework
- **IMPLEMENTATION_STATUS.md** - TODOs and integration guide

---

## Archived Documents

These documents are kept for historical reference only:
- **SENSOR_FUSION.md** - Old sensor_sync architecture
- **COMPUTE_ARCHITECTURE_COMPARISON.md** - Old compute split analysis
- **system/** - Old system management documentation
- **robot/** - Old robot graph documentation

**Note**: Do not use these for current development. They describe a superseded architecture.

---

**Archived**: 2026-01-27  
**Reason**: Modular architecture refactor (Phases 1-6 complete)
