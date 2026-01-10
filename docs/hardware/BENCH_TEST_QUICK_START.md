# Bench Test Quick Start

## Launch Bench Test

```bash
# Select bench test graph
./scripts/system/manage_graph.sh select bench_test

# Start system
./scripts/system/manage_graph.sh start
```

Or start directly:

```bash
./scripts/system/manage_graph.sh start bench_test
```

This launches:
- All hardware nodes
- System monitor
- Foxglove bridge for visualization

## Connect Foxglove Studio

1. **Open Foxglove Studio** on your computer
2. **Connect**:
   - Framework: **ROS 2**
   - Address: `<robot-ip>` or `isaac.local`
   - Port: `8765`
3. **Import Layout**:
   - File → Import Layout
   - Select: `config/visualization/foxglove_bench_test_layout.json`

## Verify Everything Works

Verify data streams:

```bash
# Unified verification
./scripts/system/manage_graph.sh verify

# Or manually check
ros2 topic list
ros2 topic echo /system/status
ros2 topic echo /realsense/status
```

## What You Should See

### In Foxglove Studio

- **4 Camera Views**: Front/rear color and depth
- **System Plots**: CPU/GPU temp, CPU/memory usage
- **Sensor Plots**: ODrive accelerometer, iRobot battery
- **Status Panels**: All hardware status messages

### Expected Topics

- `/system/status` - System health ✓
- `/camera_front/color/image_raw` - Front camera ✓
- `/camera_rear/color/image_raw` - Rear camera ✓
- `/microphone/status` - Microphone ✓
- `/odrive/status` - ODrive ✓
- `/odrive/imu` - Accelerometer ✓
- `/irobot/status` - iRobot ✓
- `/irobot/battery` - Battery ✓

## Troubleshooting

### Nodes Not Starting

```bash
# Check if packages are built
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select isaac_robot realsense_camera usb_microphone odrive_controller irobot_serial
source install/setup.bash
```

### Can't Connect to Foxglove

```bash
# Check bridge is running
ros2 node list | grep foxglove

# Check port is open
netstat -tuln | grep 8765

# Check firewall
sudo ufw allow 8765/tcp
```

### No Data in Topics

```bash
# Check hardware is connected
./scripts/hardware/verify_all_hardware.sh

# Check nodes are running
ros2 node list

# Check for errors in node logs
```

## Systemd Management

For production use, manage through systemd:

```bash
# Enable auto-start
sudo systemctl enable isaac-robot.service

# Start service
sudo systemctl start isaac-robot.service

# Check status
./scripts/system/manage_graph.sh status
```

## Next Steps

Once verified:
1. Test motor commands: `ros2 topic pub /cmd_vel ...`
2. Record data: `ros2 bag record -a`
3. Integrate with VLA controller

See [Full Verification Guide](BENCH_TEST_VERIFICATION.md) for details.
