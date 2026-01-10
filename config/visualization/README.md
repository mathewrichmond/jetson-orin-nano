# Visualization Configuration

This directory contains visualization configuration files for the Isaac robot system.

## Foxglove Studio Layouts

Foxglove Studio layout files for visualizing robot data.

**Note**: Foxglove Studio uses a specific layout format with `configById` and panel IDs. Layouts exported from Foxglove Studio will have this format.

### Available Layout

**`foxglove_all_camera_data.json`** - Complete camera data view
- Front camera (color + depth images)
- Rear camera (color + depth images)
- Front & rear pointclouds (if enabled)
- Camera status messages
- System metrics (CPU/GPU temp, CPU/memory usage)
- Uses the correct Foxglove Studio format (should import without errors)

### Usage

**In Foxglove Studio:**
1. File → Import Layout
2. Select one of the layout files from this directory
3. Layout will load with all configured panels

**From command line:**
```bash
# Open layout directory
open config/visualization/

# Or use helper script
./scripts/visualization/load_camera_layout.sh
```

### Layout Features

All layouts include:
- Color images from front/rear cameras
- Depth images with turbo colormap
- Horizontal flip for correct orientation
- Camera info topics for proper display

## Adding New Layouts

To create a new Foxglove Studio layout:

1. Create layout in Foxglove Studio
2. File → Export Layout
3. Save to `config/visualization/` with descriptive name
4. Update this README with layout description

## Configuration Pattern

Following the global config pattern:
- All visualization configs → `config/visualization/`
- Layouts are topic-agnostic (work with any ROS 2 topics)
- Can be customized per use case
