# USB Microphone Setup

## Overview

This guide covers the setup and configuration of USB microphones for the Isaac robot system. USB microphones are used for voice command input and audio capture.

## Prerequisites

- Jetson Orin Nano with Ubuntu 22.04
- USB microphone
- ALSA/PulseAudio audio system

## Hardware Connection

1. **Connect USB microphone**:
   - Plug USB microphone into available USB port
   - Ensure adequate power (use powered USB hub if needed)

2. **Verify device detection**:
   ```bash
   lsusb | grep -i "audio\|microphone\|mic"
   ```

3. **Check audio devices**:
   ```bash
   arecord -l
   ```

## Software Installation

### Install Dependencies

```bash
sudo apt update
sudo apt install -y alsa-utils pulseaudio
```

### Build ROS 2 Package

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select usb_microphone
source install/setup.bash
```

## Configuration

Edit `config/hardware/microphone_params.yaml`:

```yaml
/**:
  ros__parameters:
    device: 'default'  # Use 'default' or specific device name
    sample_rate: 16000  # Hz
    channels: 1  # Mono
    format: 'S16_LE'  # Signed 16-bit little-endian
    chunk_size: 1024  # Bytes per chunk
    publish_rate: 10.0  # Hz
```

### Finding Device Name

To find the correct device name:

```bash
# List recording devices
arecord -l

# Test recording
arecord -D plughw:1,0 -d 5 test.wav
```

Use the device name (e.g., `plughw:1,0`) in the configuration.

## Usage

### Launch Microphone Node

```bash
# Launch standalone
ros2 launch usb_microphone usb_microphone.launch.py

# Or launch as part of full system
ros2 launch isaac_robot robot.launch.py graph_config:=robot_graph.yaml group:=hardware
```

### Monitor Status

```bash
# Check microphone status
ros2 topic echo /microphone/status

# List all topics
ros2 topic list | grep microphone
```

### Test Recording

```bash
# Test with arecord
arecord -d 5 -f cd test.wav

# Play back
aplay test.wav
```

## ROS 2 Topics

### Published Topics

- `/microphone/status` (std_msgs/String) - Microphone status messages

## Troubleshooting

### Microphone Not Detected

1. **Check USB connection**:
   ```bash
   lsusb | grep -i audio
   ```

2. **Check PulseAudio**:
   ```bash
   pactl list sources short
   ```

3. **Check ALSA**:
   ```bash
   arecord -l
   ```

### No Audio Input

1. **Check device permissions**:
   ```bash
   ls -l /dev/snd/*
   ```

2. **Test with arecord**:
   ```bash
   arecord -D default -d 5 test.wav
   ```

3. **Check PulseAudio**:
   ```bash
   pulseaudio --check
   pulseaudio --start
   ```

### Permission Errors

If you get permission errors:

```bash
# Add user to audio group
sudo usermod -a -G audio $USER
# Log out and back in
```

## Audio Format Options

Common audio formats:
- `S16_LE` - Signed 16-bit little-endian (most common)
- `S24_LE` - Signed 24-bit little-endian
- `S32_LE` - Signed 32-bit little-endian
- `FLOAT_LE` - 32-bit float little-endian

## Integration with Voice Recognition

The USB microphone node provides audio capture that can be integrated with:
- Speech recognition systems
- Voice command processing
- Audio streaming
- Audio analysis

## Next Steps

- Integrate with speech recognition (e.g., Whisper, Vosk)
- Configure noise cancellation
- Set up audio streaming for remote monitoring
- Integrate with VLA controller for voice commands
