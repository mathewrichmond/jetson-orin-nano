# Visualization Configurations

This directory contains Foxglove Studio layout configurations for visualizing robot sensor data.

## Available Layouts

**`sensor_bridged.json`** - Optimized for remote/bridged connection (low bandwidth)
- Uses `/viz/remote/*` topics
- Aggressive downsampling
- Front camera only
- Essential sensors

**`sensor_raw.json`** - Full data for direct connection (high bandwidth)
- Uses `/hardware/*`, `/nvblox/full/*`, `/phat/*`, `/irobot/*` topics
- Full quality, no downsampling
- Both cameras
- All sensors including raw + filtered comparison

**`camera_debug.json`** - Camera debugging and monitoring layout
- Two side-by-side camera views (front and rear)
- Shows downsampled visualization images (`/viz/remote/camera_*/color/image_raw`)
- Displays camera info messages (calibration data)
- Shows timestamp plots for frame rate monitoring
- Includes sync status from sensor fusion
- Optimized for debugging camera publishing, sync, and frame rates

## Usage

1. Open Foxglove Studio
2. Connect to your data source
3. Import the appropriate layout:
   - Remote monitoring: `sensor_bridged.json`
   - Local development: `sensor_raw.json`

## Topic Structure

See `docs/visualization/FOXGLOVE_LAYOUTS.md` for detailed topic information.
