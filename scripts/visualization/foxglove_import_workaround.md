# Foxglove Studio Layout Import Workaround

If you're getting `TypeError: l.split is not a function` when importing layouts, try these workarounds:

## Option 1: Create Layout Manually in Foxglove Studio

1. **Open Foxglove Studio** and connect to your robot
2. **Create a new layout**:
   - Click "+" → "New layout"
   - Name it "Isaac Robot - Camera View"
3. **Add panels manually**:
   - Click "+" to add panels
   - Add **Image** panels for:
     - `/camera_front/color/image_raw` (with camera info `/camera_front/color/camera_info`)
     - `/camera_front/depth/image_rect_raw` (with camera info `/camera_front/depth/camera_info`, colorMode: turbo)
     - `/camera_rear/color/image_raw` (with camera info `/camera_rear/color/camera_info`)
     - `/camera_rear/depth/image_rect_raw` (with camera info `/camera_rear/depth/camera_info`, colorMode: turbo)
   - Add **Plot** panels for system metrics
   - Add **RawMessages** panel for `/realsense/status`
4. **Save the layout**: File → Export Layout
5. This will create a working layout file you can reuse

## Option 2: Copy JSON Content Directly

1. Open the layout JSON file in a text editor
2. Copy all the JSON content
3. In Foxglove Studio:
   - Create a new layout
   - Open browser console (if available) or try File → Import
   - Some versions allow pasting JSON directly

## Option 3: Check Foxglove Studio Version

This might be a bug in your Foxglove Studio version:
1. Check your version: Help → About
2. Try updating to the latest version
3. Or try the web version: https://studio.foxglove.dev

## Option 4: Use Web Version

The web version sometimes handles imports differently:
1. Go to https://studio.foxglove.dev
2. Connect to your robot
3. Try importing the layout there

## Layout Files Available

All layout files are in `config/visualization/`:
- `foxglove_all_camera_data.json` - Complete camera view with pointclouds
- `foxglove_cameras_layout.json` - Camera view with metrics
- `foxglove_cameras_only_layout.json` - Cameras only
- `foxglove_layout.json` - Original layout
- `foxglove_minimal_test.json` - Minimal test layout

## Manual Panel Configuration

If import doesn't work, configure panels manually:

### Image Panels:
- **Topic**: `/camera_front/color/image_raw`
- **Camera Topic**: `/camera_front/color/camera_info`
- **Flip**: horizontal (for correct orientation)

### Depth Image Panels:
- **Topic**: `/camera_front/depth/image_rect_raw`
- **Camera Topic**: `/camera_front/depth/camera_info`
- **Color Mode**: turbo
- **Flip**: horizontal

### Plot Panels:
- **Paths**:
  - `/system/temperature/cpu` (red)
  - `/system/temperature/gpu` (green)
  - `/system/cpu/usage` (blue)
  - `/system/memory/usage` (magenta)

### RawMessages Panel:
- **Topic**: `/realsense/status`
