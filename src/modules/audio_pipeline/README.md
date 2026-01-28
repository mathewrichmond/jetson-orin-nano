# Audio Pipeline Module

**Status**: Phase 4 ✅ | Feature extraction, speech recognition, orchestration

---

## Overview

The audio pipeline module provides audio processing capabilities for the robot system, including feature extraction for VLA models and speech-to-text conversion.

### Architecture

```
Audio Hardware → USB Microphone Node → Sensor Fusion → Audio Pipeline
                                                              ↓
                                          Feature Extractor ←→ Speech Recognition
                                                              ↓
                                                        VLA Planner
```

---

## Nodes

### 1. Audio Feature Extractor Node

Extracts audio features for VLA model consumption.

**Executable**: `audio_feature_extractor_node`

**Features**:
- MFCC (Mel-Frequency Cepstral Coefficients) extraction
- Spectrogram generation
- Voice activity detection (VAD)
- Configurable FFT parameters

**Topics**:
- Subscribes: `/sensor_fusion/audio/raw` (synchronized audio)
- Publishes:
  - `/audio/features/mfcc` - MFCC coefficients
  - `/audio/features/spectrogram` - Audio spectrogram
  - `/audio/features/vad` - Voice activity detection

**Parameters**:
```yaml
audio_topic: /sensor_fusion/audio/raw
output_namespace: /audio
sample_rate: 16000
channels: 2
mfcc_coefficients: 13
fft_size: 2048
hop_length: 512
enable_vad: true
vad_threshold: 0.5
```

---

### 2. Speech Recognition Node

Converts audio to text using speech recognition engines.

**Executable**: `speech_recognition_node`

**Features**:
- Placeholder for Whisper, DeepSpeech, or cloud STT
- Continuous audio stream processing
- Configurable recognition buffer
- Voice activity triggered recognition

**Topics**:
- Subscribes: `/sensor_fusion/audio/raw`
- Publishes:
  - `/audio/transcription` - Text transcription
  - `/audio/transcription_confidence` - Transcription with confidence

**Parameters**:
```yaml
audio_topic: /sensor_fusion/audio/raw
output_namespace: /audio
recognition_engine: placeholder  # whisper, deepspeech, google
language: en-US
model_size: base  # tiny, base, small, medium, large
buffer_duration_sec: 2.0
enable_continuous: true
vad_enabled: true
```

**Supported Engines** (future):
- **Whisper** - OpenAI's speech recognition (install: `pip install openai-whisper`)
- **DeepSpeech** - Mozilla's STT (install: `pip install deepspeech`)
- **Google Cloud STT** - Cloud-based recognition
- **Vosk** - Offline speech recognition

---

### 3. Audio Pipeline Node

Orchestrates audio processing and health monitoring.

**Executable**: `audio_pipeline_node`

**Features**:
- Coordinate feature extraction and speech recognition
- Power-aware operation (reduce processing in low battery)
- Module health aggregation
- Mode switching (FULL/REDUCED/SLEEP)

**Topics**:
- Publishes:
  - `/jetson/health/audio_pipeline` - Module health
  - `/jetson/power/request` - Power requests
- Subscribes:
  - `/irobot/battery` - Battery level
  - `/jetson/system/temperature/cpu` - CPU temperature

**Parameters**:
```yaml
output_namespace: /jetson
audio_namespace: /audio
initial_mode: FULL  # FULL, REDUCED, SLEEP
enable_feature_extraction: true
enable_speech_recognition: true
auto_mode_switching: true
battery_threshold_reduced: 25.0  # %
battery_threshold_sleep: 15.0  # %
temp_threshold_reduce: 75.0  # °C
```

**Modes**:
- **FULL**: All audio processing active
- **REDUCED**: Feature extraction only (no STT)
- **SLEEP**: Minimal processing (buffer only)

---

## Usage

### Build

```bash
cd /home/nano/src/jetson-orin-nano
colcon build --packages-select audio_pipeline --symlink-install
source install/setup.bash
```

### Launch All Nodes

```bash
ros2 launch audio_pipeline audio_pipeline.launch.py
```

### Launch Individual Nodes

```bash
# Feature extractor
ros2 run audio_pipeline audio_feature_extractor_node

# Speech recognition
ros2 run audio_pipeline speech_recognition_node

# Orchestrator
ros2 run audio_pipeline audio_pipeline_node
```

### Test

```bash
cd src/modules/audio_pipeline/test
python3 test_node_imports.py
```

---

## Integration

### With Existing System

**Input Sources**:
- `usb_microphone_node` - Raw audio capture
- `sensor_sync_node` - Audio synchronization to camera frames

**Outputs**:
- `vla_planner` module - Consumes audio features
- Visualization - Audio features for debugging

### Data Flow

```
USB Mic → sensor_sync → audio_feature_extractor → VLA Model
                              ↓
                      speech_recognition → Commands
```

---

## Configuration

### Feature Extraction Configuration

For VLA models, typical configurations:

**Low latency** (real-time interaction):
```yaml
mfcc_coefficients: 13
fft_size: 1024
hop_length: 256
```

**High quality** (transcription, analysis):
```yaml
mfcc_coefficients: 40
fft_size: 4096
hop_length: 1024
```

### Speech Recognition Models

**Lightweight** (edge devices):
- Vosk small models (~50MB)
- Whisper tiny (~75MB)

**Balanced** (recommended):
- Whisper base (~140MB)
- DeepSpeech 0.9.3 (~190MB)

**High accuracy** (if GPU available):
- Whisper medium (~1.5GB)
- Whisper large (~3GB)

---

## Dependencies

### Python Packages (Optional)

For advanced audio processing:
```bash
# MFCC and spectrogram extraction
pip install librosa numpy

# Speech recognition (choose one)
pip install openai-whisper  # Whisper
pip install deepspeech  # DeepSpeech
pip install vosk  # Vosk (offline)
```

### ROS 2 Dependencies

- `rclpy`
- `std_msgs`
- `sensor_msgs`
- `custom_msgs` (Phase 1-3)
- `isaac_utils`

---

## Future Enhancements

### Planned Features

1. **Whisper Integration** - Real Whisper model integration
2. **Emotion Detection** - Extract emotional features from audio
3. **Speaker Diarization** - Identify multiple speakers
4. **Noise Suppression** - Deep learning-based noise reduction
5. **Audio Localization** - Direction of arrival estimation (with mic array)

### Performance Optimization

- GPU acceleration for STT models
- Model quantization for edge deployment
- Streaming inference for low latency

---

## Troubleshooting

### No Audio Features

**Check**:
1. USB microphone is publishing: `ros2 topic echo /microphone/audio`
2. sensor_sync is running: `ros2 topic echo /sensor_fusion/audio/raw`
3. Feature extractor is subscribed: `ros2 node info /audio_feature_extractor_node`

### High CPU Usage

**Solutions**:
- Reduce `fft_size` (e.g., 1024 instead of 2048)
- Increase `hop_length` (less frequent updates)
- Disable speech recognition in low power mode
- Use smaller STT models

### Poor STT Accuracy

**Solutions**:
- Use larger model size (`model_size: medium`)
- Improve audio quality (better microphone, noise suppression)
- Tune VAD threshold (`vad_threshold: 0.3`)
- Increase buffer duration (`buffer_duration_sec: 3.0`)

---

## License

MIT License - See repository LICENSE file for details.

---

**Last Updated**: 2026-01-27  
**Status**: Phase 4 Complete  
**Next**: VLA Planner Integration
