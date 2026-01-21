# Foxglove Studio Connection Troubleshooting

## Quick Fix: Connection Not Working

If you see a red error indicator on `ws://localhost:8765` in Foxglove Studio:

### Step 1: Verify Bridge is Running

On the robot, check:
```bash
# Check bridge is listening
ss -tlnp | grep 8765

# Should show:
# LISTEN 0 1024 0.0.0.0:8765 ...
```

If not running:
```bash
./scripts/system/manage_graph.sh restart robot
```

### Step 2: Set Up Port Forwarding (If Connecting from Laptop)

**On your laptop**, run:
```bash
ssh -L 8765:localhost:8765 nano@isaac.local
```

**Keep this terminal open** while using Foxglove Studio.

### Step 3: Connect in Foxglove Studio

1. **Connection Type**: Select **Foxglove WebSocket**
2. **Host**:
   - If port forwarding: `localhost`
   - If same network: `isaac.local` or `192.168.0.155`
3. **Port**: `8765`
4. Click **Connect**

### Step 4: Verify Connection

After connecting:
- Red error indicator should turn green
- Topics should appear in the Topics panel (left sidebar)
- Panels should start showing data

## Common Issues

### "Connection Refused" Error

**Cause**: Port forwarding not active or bridge not running

**Fix**:
1. Check port forwarding is active:
   ```bash
   # On laptop
   netstat -an | grep 8765
   # Should show LISTEN on localhost:8765
   ```

2. Restart port forwarding:
   ```bash
   # Kill existing tunnel
   pkill -f "ssh.*8765"
   # Start new tunnel
   ssh -L 8765:localhost:8765 nano@isaac.local
   ```

3. Verify bridge is running on robot:
   ```bash
   # On robot
   ss -tlnp | grep 8765
   ```

### Topics Not Appearing

**Cause**: Topics not being published or bridge not forwarding them

**Check**:
```bash
# On robot
ros2 topic list | grep "/viz/remote"

# Should show:
# /viz/remote/camera_front/color/image_raw
# /viz/remote/camera_rear/color/image_raw
# ... etc
```

**Fix**:
- Restart robot graph: `./scripts/system/manage_graph.sh restart robot`
- Wait 10-15 seconds for topics to initialize
- Refresh connection in Foxglove Studio

### Panels Show "Topics Do Not Exist"

**Cause**: Topics exist but Foxglove can't see them (connection issue)

**Fix**:
1. Verify connection is green (not red)
2. Check Topics panel shows the topics
3. If topics missing, check bridge topic whitelist in `config/robot/robot_graph.yaml`
4. Restart bridge: `./scripts/system/manage_graph.sh restart robot`

### RawMessages Panels Show "No Message Path Entered"

**Cause**: Panel configuration issue (should be auto-configured from layout)

**Fix**:
1. Right-click the panel → **Settings**
2. Enter topic name manually (e.g., `/hardware/camera_front/color/camera_info`)
3. Or re-import the layout file

## Verification Checklist

Before reporting issues, verify:

- [ ] Bridge is running: `ss -tlnp | grep 8765` shows LISTEN
- [ ] Port forwarding active (if remote): `netstat -an | grep 8765` on laptop
- [ ] Topics exist: `ros2 topic list | grep "/viz/remote"` shows topics
- [ ] Topics publishing: `ros2 topic hz /viz/remote/camera_front/color/image_raw` shows rate
- [ ] Connection is green in Foxglove Studio (not red)
- [ ] Topics appear in Foxglove Topics panel

## Direct Connection (Same Network)

If you're on the same network as the robot:

1. **No port forwarding needed**
2. In Foxglove Studio:
   - Host: `isaac.local` or `192.168.0.155`
   - Port: `8765`
3. Connect directly

## Testing Connection

Test the WebSocket connection manually:

```bash
# On laptop (if port forwarded) or robot (if direct)
curl -v http://localhost:8765
# Should connect (may return error, but connection works)
```

Or use `wscat`:
```bash
# Install: npm install -g wscat
wscat -c ws://localhost:8765
# Should connect without errors
```
