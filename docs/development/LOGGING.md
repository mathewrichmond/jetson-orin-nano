# Data Logging and Session Management

Complete guide to the session-based data logging system for VLA model training and analysis.

---

## Overview

The logging system provides a complete pipeline for collecting, processing, and analyzing robot training data:

1. **Session Management**: Create and track data collection sessions
2. **Data Recording**: ROS bags + VLA inference logs
3. **Quality Assurance**: Automated validation and anomaly detection
4. **Data Processing**: Convert to ML-ready formats (Parquet)
5. **Artifact Generation**: Videos, thumbnails, statistics

All data is organized in a unified session-based structure that works for both real robot and simulation data.

---

## Session Structure

### Directory Layout

```
/mnt/nfs/robot_data/session_20260127_143022_abc123/
├── raw/                          # Original data
│   ├── abc123_part_001.bag      # ROS bags (auto-split)
│   ├── abc123_part_002.bag
│   └── vla_inference_abc123_000.jsonl  # VLA inference logs
├── processed/                    # ML-ready data
│   ├── camera.parquet
│   ├── depth.parquet
│   ├── actions.parquet
│   ├── imu.parquet
│   ├── odometry.parquet
│   ├── audio.parquet
│   └── power.parquet
├── metadata/                     # Session information
│   ├── session_manifest.json
│   ├── data_schema.json
│   ├── processing_manifest.json
│   └── pipeline.log
├── artifacts/                    # Visualizations
│   ├── trajectory_viz.mp4
│   ├── summary_stats.json
│   └── thumbnails/
├── quality/                      # QC reports
│   ├── qc_report.json
│   └── anomaly_flags.json
└── models/                       # Model info
    └── vila_checkpoint_sha256.txt
```

### Storage Locations

- **Primary**: `/mnt/nfs/robot_data` (shared NFS volume)
- **Fallback**: `/home/nano/data` (local when NFS unavailable)

The system automatically uses the fallback if NFS is not mounted.

---

## Recording Data

### Start Recording

The robot automatically starts logging when the `isaac-robot.service` starts (via systemd hooks). For manual control:

```bash
# Start a new session
scripts/logging/start_logging.sh

# This will:
# 1. Create a new session with unique ID
# 2. Export SESSION_ID environment variable
# 3. Start ROS bag recording
# 4. Enable VLA inference logging
```

### Stop Recording

```bash
# Stop the current session
scripts/logging/stop_logging.sh

# This will:
# 1. Stop ROS bag recording gracefully
# 2. Finalize session manifest
# 3. Trigger data pipeline automatically
```

### What Gets Recorded

**ROS Topics** (in bags):
- Camera: RGB and depth from all cameras
- Audio: Raw audio, MFCC features, transcriptions
- Actions: VLA outputs and executed commands
- State: Odometry, IMU, pose estimates
- Power: Battery level, system performance
- Health: Node health status

**VLA Inferences** (in JSONL):
- Timestamp and frame ID
- Input references (camera frames, audio, robot state)
- VLA model output (tokens, confidence, decoded action)
- Executed action
- Metadata (model version, inference time, GPU temp)

### Recording Configuration

Edit `config/logging/session_structure.yaml`:

```yaml
bag_split_size_mb: 1024        # Split bags at 1GB
bag_split_duration_sec: 600    # Or split at 10 minutes

topics:
  camera: [...]                 # Camera topics to record
  audio: [...]                  # Audio topics
  actions: [...]                # Action topics
  # ... etc
```

---

## Session Management

### Create a Session

```bash
# Create session manually (for testing)
python3 scripts/logging/session_manager.py create --env robot

# Output:
#   Session ID: abc123
#   Export with: export SESSION_ID=abc123
```

### List Sessions

```bash
# List recent sessions
python3 scripts/logging/list_sessions.py --limit 10

# Filter by environment
python3 scripts/logging/list_sessions.py --env robot

# Filter by status
python3 scripts/logging/list_sessions.py --status completed

# Filter by date
python3 scripts/logging/list_sessions.py --date 2026-01-27

# Show only failed QC
python3 scripts/logging/list_sessions.py --failed-qc
```

### Check Session Status

```bash
# Show comprehensive status report
scripts/logging/session_status.sh

# Displays:
# - Active recording session
# - Storage location and disk usage
# - Recent sessions
# - Pipeline processing queue
# - Time until disk full (if recording)
```

### Query Active Session

```bash
# Get currently recording session
python3 scripts/logging/session_manager.py active
```

---

## Data Pipeline

### Automatic Processing

The data pipeline runs automatically when a session is stopped. It performs:

1. **Quality Check**: Validate bags, detect anomalies
2. **Bag to Parquet**: Convert to columnar ML format
3. **Generate Artifacts**: Create videos, thumbnails, stats

The pipeline runs at low priority (nice +19, ionice best-effort) to avoid impacting robot operation.

### Manual Processing

```bash
# Process a specific session
python3 scripts/data_pipeline/process_session.py /path/to/session_dir

# Resume interrupted processing (skip completed steps)
python3 scripts/data_pipeline/process_session.py /path/to/session_dir --resume

# Trigger pipeline in background
scripts/data_pipeline/trigger_pipeline.sh <session_id>
```

### Quality Checks

The QC pipeline validates:

- **Bag integrity**: Files not corrupted
- **Topic count**: Expected topics present
- **Data gaps**: No large timestamp gaps
- **Camera data**: No black or frozen frames
- **IMU data**: Rate and outlier detection
- **Battery data**: No sudden drops

Reports are saved to `quality/qc_report.json`:

```json
{
  "status": "pass",
  "checks_passed": 8,
  "checks_failed": 0,
  "warnings": [],
  "errors": []
}
```

### Parquet Conversion

Converts ROS bags to Parquet files optimized for ML training:

```python
# Parquet files created:
camera.parquet      # RGB frames (JPEG encoded)
depth.parquet       # Depth maps
actions.parquet     # VLA outputs and executed actions
imu.parquet         # IMU sensor data
odometry.parquet    # Odometry and pose
audio.parquet       # MFCC features and transcriptions
power.parquet       # Battery and system metrics
```

All data is synchronized to a common time base and compressed (snappy).

Configuration in `config/data_pipeline/pipeline_config.yaml`:

```yaml
parquet:
  compression: "snappy"
  row_group_size: 10000
  image_encoding: "jpeg"
  image_quality: 85
  sync_timestamps: true
```

### Artifact Generation

Generates visualizations for quick review:

- **Trajectory video**: Bird's eye and 3D perspective views
- **Thumbnails**: One per minute for quick browsing
- **Summary stats**: Distance traveled, actions taken, inference count, etc.

---

## VLA Inference Logging

### In VLA Controller Node

The VLA controller automatically logs every inference when a session is active:

```python
from vla_planner.data_logger import get_data_logger

# In __init__
self.data_logger = get_data_logger(enable_logging=True)

# In inference callback
self.data_logger.log_inference(
    timestamp=time.time(),
    frame_id=self.frame_count,
    camera_frames={'front': 'bag_ref'},
    audio_features=audio_array,
    robot_state={'position': [x, y, z]},
    transcription="move forward",
    vla_output={'confidence': 0.87, 'action': [0.1, 0.0, ...]},
    executed_action=[0.1, 0.0, ...],
    metadata={'model_version': 'sha256', 'inference_time_ms': 45.2}
)
```

### Log Format

JSONL (JSON Lines) format, one inference per line:

```json
{
  "timestamp": 1706380922.123,
  "session_id": "abc123",
  "frame_id": 42,
  "inputs": {
    "camera_frames": {"front": "abc123_part_001.bag@/camera_front/123"},
    "audio_features": {"shape": [13, 44], "mean": 0.12, "std": 0.8},
    "robot_state": {"position": [1.2, 0.3, 0.0], "velocity": [0.1, 0.0, 0.0]},
    "transcription": "move forward"
  },
  "vla_output": {
    "action_tokens": [...],
    "confidence": 0.87,
    "action_decoded": [0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  },
  "executed_action": [0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  "metadata": {
    "model_version": "VILA1.5-3b-sha256",
    "inference_time_ms": 45.2,
    "gpu_temp": 68.5
  }
}
```

### Benefits

- **Async I/O**: Buffered writes don't block inference
- **File rotation**: Automatic rotation at 100MB
- **Lightweight**: Only stores references to bag data
- **Complete**: Every inference tracked for debugging

---

## Disk Management

### Automatic Cleanup

The system automatically manages disk space when usage exceeds 85%:

```bash
# Cleanup runs via systemd timer (daily)
# Or manually:
scripts/logging/cleanup_sessions.sh
```

Cleanup policy (configured in `config/logging/session_structure.yaml`):

```yaml
cleanup:
  keep_raw_days: 7              # Keep raw bags for 7 days
  keep_processed_days: 30       # Keep processed for 30 days
  delete_failed_days: 30        # Delete failed sessions after 30 days
  archive_threshold_percent: 85 # Start cleanup at 85% full
  min_free_gb: 10               # Maintain 10GB free minimum
```

### Cleanup Process

1. **Archive old raw data**: Delete bags >7 days old (if processed data exists)
2. **Delete old processed**: Remove processed data >30 days old
3. **Delete failed sessions**: Remove failed sessions >30 days old
4. **Remove empty directories**: Clean up empty session folders

Raw bags are only deleted if processed Parquet files exist, ensuring no data loss.

---

## Best Practices

### Recording Sessions

1. **Always use session manager**: Don't record bags manually
2. **Monitor disk space**: Check `session_status.sh` regularly
3. **Stop cleanly**: Use `stop_logging.sh` to ensure proper finalization
4. **Name appropriately**: Use descriptive metadata when creating sessions

### Data Quality

1. **Review QC reports**: Check `quality/qc_report.json` after processing
2. **Verify completeness**: Ensure all expected topics were recorded
3. **Check artifacts**: View trajectory videos for sanity checks
4. **Monitor anomalies**: Investigate flagged issues

### Storage Management

1. **Use NFS when available**: Shared storage for collaboration
2. **Local fallback works**: System handles NFS unavailability
3. **Archive regularly**: Move old data to cold storage manually if needed
4. **Clean up failed**: Delete failed test sessions promptly

### Training Data Collection

1. **Diverse scenarios**: Record varied environments and tasks
2. **Label sessions**: Add metadata about task, environment, conditions
3. **Balance data**: Record successes and failures
4. **Quality over quantity**: Better to have clean, diverse data

---

## Troubleshooting

### Session Won't Start

```bash
# Check if session already active
python3 scripts/logging/session_manager.py active

# Check disk space
df -h /mnt/nfs/robot_data

# Check NFS mount
mountpoint /mnt/nfs/robot_data

# Check permissions
ls -la /mnt/nfs/robot_data
```

### Recording Not Working

```bash
# Check ROS environment
source install/setup.bash
ros2 topic list

# Check bag recording process
ps aux | grep "ros2 bag record"

# Check session lock file
ls -la /path/to/session/.session_lock
```

### Pipeline Not Running

```bash
# Check pipeline lock
ls -la /path/to/session/.processing_lock

# Check pipeline log
tail -f /path/to/session/metadata/pipeline.log

# Run manually
python3 scripts/data_pipeline/process_session.py /path/to/session
```

### Disk Full

```bash
# Run cleanup immediately
scripts/logging/cleanup_sessions.sh

# Check what's using space
du -sh /mnt/nfs/robot_data/session_*/ | sort -h | tail -20

# Archive old sessions manually
tar -czf session_backup.tar.gz /mnt/nfs/robot_data/session_OLD_ID/
rm -rf /mnt/nfs/robot_data/session_OLD_ID/
```

### Data Logger Not Working

```bash
# Check SESSION_ID is set
echo $SESSION_ID

# If not set, source from environment file
source /tmp/current_session.env

# Check if session directory exists
python3 -c "from scripts.logging.session_manager import SessionManager; m = SessionManager(); print(m.find_session('$SESSION_ID'))"
```

---

## Configuration Reference

### Session Structure Config

`config/logging/session_structure.yaml`:

```yaml
session_structure:
  root: "/mnt/nfs/robot_data"
  local_fallback: "/home/nano/data"
  session_naming: "session_{timestamp}_{session_id}"
  
  directories:
    raw: "raw/"
    processed: "processed/"
    metadata: "metadata/"
    artifacts: "artifacts/"
    quality: "quality/"
    models: "models/"
  
  bag_naming: "{session_id}_part_{num:03d}.bag"
  bag_split_size_mb: 1024
  bag_split_duration_sec: 600
  
  topics:
    camera: [...]
    audio: [...]
    actions: [...]
    state: [...]
    power: [...]
    health: [...]
  
  cleanup:
    keep_raw_days: 7
    keep_processed_days: 30
    delete_failed_days: 30
    archive_threshold_percent: 85
    min_free_gb: 10
```

### Pipeline Config

`config/data_pipeline/pipeline_config.yaml`:

```yaml
pipeline:
  quality:
    max_frame_gap_ms: 200
    min_imu_rate_hz: 40
    max_battery_drop_percent: 5
    check_black_frames: true
    check_frozen_frames: true
  
  parquet:
    compression: "snappy"
    row_group_size: 10000
    image_encoding: "jpeg"
    image_quality: 85
    sync_timestamps: true
  
  artifacts:
    generate_trajectory_video: true
    trajectory_video_fps: 10
    generate_thumbnails: true
    thumbnail_interval_sec: 60
    generate_summary_stats: true
  
  execution:
    priority: 19                # nice priority
    io_class: "best-effort"     # ionice class
    auto_trigger_on_stop: true
    retry_on_failure: true
    max_retries: 3
```

---

## Integration with Systemd

The logging system integrates with systemd services via hooks:

```ini
[Service]
# In isaac-robot.service
ExecStartPost=/home/nano/src/jetson-orin-nano/scripts/logging/start_logging.sh
ExecStopPre=/home/nano/src/jetson-orin-nano/scripts/logging/stop_logging.sh
```

This ensures:
- Sessions start automatically when robot starts
- Sessions stop and process when robot stops
- No manual intervention required for production use

---

## Python API

For programmatic access:

```python
from scripts.logging.session_manager import SessionManager

# Create session
manager = SessionManager()
session = manager.create_session(environment='robot', metadata={'task': 'navigation'})
print(f"Session ID: {session['session_id']}")

# Query sessions
sessions = manager.list_sessions(environment='robot', status='completed', limit=10)
for s in sessions:
    print(f"{s['session_id']}: {s['duration_sec']}s, {s['total_size_bytes']/1e9:.2f}GB")

# Finalize session
manager.finalize_session(session_id='abc123', status='completed')

# Find session path
path = manager.find_session('abc123')
```

---

## See Also

- [TESTING.md](../testing/TESTING.md) - Testing with recorded data
- [SYSTEM_MONITOR.md](../monitoring/SYSTEM_MONITOR.md) - System metrics monitoring
- [DEPLOYMENT.md](../deployment/DEPLOYMENT.md) - Deployment workflows
- Session structure config: `config/logging/session_structure.yaml`
- Pipeline config: `config/data_pipeline/pipeline_config.yaml`
