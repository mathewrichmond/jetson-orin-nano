# Foxglove Bridge Connection Troubleshooting

## Common Issue: Intermittent Connection Drops

If your Foxglove Bridge connection drops every few seconds, this is typically caused by:

1. **High-frequency camera topics overwhelming the bridge**
2. **Resource constraints on Jetson Orin Nano**
3. **Network bandwidth limitations**

## Solutions

### Solution 1: Reduce Camera Frame Rates (Recommended)

The bridge subscribes to all topics, including high-frequency camera streams. Reducing camera frame rates reduces bandwidth:

**Edit `config/hardware/realsense_params.yaml`:**

```yaml
/**:
  ros__parameters:
    # Reduce frame rates to reduce bridge load
    color_fps: 15  # Reduced from 30
    depth_fps: 15  # Reduced from 30
    publish_rate: 15.0  # Reduced from 30.0
```

Then restart the graph:
```bash
./scripts/system/manage_graph.sh restart robot
```

### Solution 2: Use Topic Filtering (Advanced)

If you need full frame rates, you can filter topics by modifying the bridge launch. However, this requires custom launch file modifications.

### Solution 3: Check System Resources

Monitor CPU and memory usage:

```bash
# Check CPU/memory
htop

# Check if bridge is crashing
./scripts/system/manage_graph.sh logs | grep foxglove

# Check systemd service status
systemctl --user status isaac-robot.service
```

### Solution 4: Network Stability

If using WiFi:
- Switch to wired Ethernet if possible
- Check WiFi signal strength
- Reduce interference (change WiFi channel)

### Solution 5: Use Monitor Graph Instead

The `monitor` graph includes fewer nodes and may be more stable:

```bash
./scripts/system/manage_graph.sh select monitor
./scripts/system/manage_graph.sh start monitor
```

## Verification

After applying fixes:

1. **Check bridge is running:**
   ```bash
   ros2 node list | grep foxglove
   ```

2. **Monitor connection stability:**
   - Watch Foxglove Studio connection indicator
   - Check for error messages in bridge logs

3. **Verify topics are publishing:**
   ```bash
   ros2 topic hz /camera_front/color/image_raw
   ```

## Expected Behavior

- **Stable connection**: Connection indicator stays green
- **Smooth data**: Panels update smoothly without gaps
- **No errors**: No error messages in Foxglove Studio

## If Issues Persist

1. Check bridge logs: `./scripts/system/manage_graph.sh logs`
2. Verify camera nodes are stable: `ros2 node list`
3. Check system resources: `htop` or `top`
4. Try restarting the bridge: `./scripts/system/manage_graph.sh restart robot`
