# Bench Test Launch Instructions

## On the Jetson Hardware

Once you're on the Jetson Orin Nano with ROS 2 installed, run:

```bash
# 1. Select bench test graph
./scripts/system/manage_graph.sh select bench_test

# 2. Start the system
./scripts/system/manage_graph.sh start
```

Or start directly with graph specified:

```bash
./scripts/system/manage_graph.sh start bench_test
```

## What Gets Launched

The bench_test graph launches:

1. **System Monitor** (`system_monitor_node`)
   - Publishes: `/system/status`, `/system/temperature/cpu`, `/system/temperature/gpu`, etc.

2. **RealSense Cameras** (`realsense_camera_node`)
   - Publishes: `/camera_front/color/image_raw`, `/camera_front/depth/image_rect_raw`, `/camera_rear/color/image_raw`, `/camera_rear/depth/image_rect_raw`, `/realsense/status`

3. **USB Microphone** (`usb_microphone_node`)
   - Publishes: `/microphone/status`

4. **ODrive Controller** (`odrive_controller_node`)
   - Publishes: `/odrive/status`, `/odrive/imu`
   - Subscribes: `/cmd_vel`

5. **iRobot Serial** (`irobot_serial_node`)
   - Publishes: `/irobot/status`, `/irobot/battery`
   - Subscribes: `/cmd_vel`

6. **Foxglove Bridge** (`foxglove_bridge`)
   - WebSocket server on port 8765 for visualization

## Verify Launch

After starting, verify all nodes are running:

```bash
# Check status
./scripts/system/manage_graph.sh status

# Verify data streams
./scripts/system/manage_graph.sh verify

# Check ROS 2 nodes
ros2 node list

# Check topics
ros2 topic list
```

## Connect Foxglove Studio

1. Open Foxglove Studio on your computer
2. Connect to robot IP (or `isaac.local`) on port 8765
3. Import layout: `config/visualization/foxglove_bench_test_layout.json`

## Stop the System

```bash
./scripts/system/manage_graph.sh stop
```

## Systemd Management

For production use:

```bash
# Enable auto-start
sudo systemctl enable isaac-robot.service

# Start service
sudo systemctl start isaac-robot.service

# Check status
sudo systemctl status isaac-robot.service

# View logs
sudo journalctl -u isaac-robot.service -f
```

The service will automatically use the selected graph (`bench_test`).
