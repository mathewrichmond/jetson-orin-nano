# Visualization Quick Start

Quick reference for visualizing **any ROS 2 topics** - raw data, processed streams, fused pointclouds, system metrics, or custom topics.

## Cross-Platform (Mac/Windows/Ubuntu)

### Foxglove Studio

1. **Download**: [foxglove.dev/download](https://foxglove.dev/download)

2. **On Robot** (generic, works with any ROS 2 nodes):
   ```bash
   ros2 launch isaac_robot rosbridge.launch.py
   ```

3. **In Foxglove Studio**:
   - Open → ROS 2
   - Host: `<robot-ip>` (or `isaac.local`)
   - Port: `9090`
   - Open

4. **Load Layout** (optional):
   - File → Import Layout
   - Select: `src/hardware_drivers/realsense_camera/config/foxglove_layout.json` (for cameras)
   - Or create your own custom layout

## Web Browser (Any Platform)

1. **On Robot**:
   ```bash
   ros2 launch isaac_robot rosbridge.launch.py
   ```

2. **In Browser**:
   - Open [studio.foxglove.dev](https://studio.foxglove.dev)
   - Connect to `ws://<robot-ip>:9090`

## Docker

```bash
docker-compose up isaac-viz
```

Then connect via browser or Foxglove Studio to `ws://localhost:9090`

## Linux Native (RViz2)

```bash
ros2 launch isaac_robot unified_visualization.launch.py viz_backend:=rviz2
```

## Setup Rosbridge (First Time)

```bash
./scripts/visualization/setup_rosbridge.sh
```

## Troubleshooting

- **Can't connect**: Check `ROS_DOMAIN_ID` matches on both machines
- **Port 9090**: Ensure firewall allows it
- **No data**: Verify camera node is running: `ros2 topic list`

See [Full Visualization Guide](VISUALIZATION.md) for details.
