# SystemPerformance.msg
# System-wide performance metrics

std_msgs/Header header

# CPU metrics
float32 cpu_usage_pct           # 0-100 per host
float32 temperature_cpu         # Celsius
int32 cpu_frequency_mhz         # Current CPU frequency

# GPU metrics (Jetson only)
float32 gpu_usage_pct           # 0-100 (0 if GPU off or not applicable)
float32 temperature_gpu         # Celsius (0 if GPU off)
int32 gpu_frequency_mhz         # Current GPU frequency

# Memory metrics
float32 memory_usage_pct        # 0-100
int64 memory_used_mb            # MB
int64 memory_available_mb       # MB

# Pipeline performance
float32 vision_pipeline_fps     # Actual FPS (0 if vision off)
float32 planner_hz              # Control loop frequency
float32 imu_processor_hz        # IMU processing frequency

# Network metrics
float32 network_latency_ms      # Pi ↔ Jetson latency (0 if single-host)
float32 network_bandwidth_mbps  # Current network utilization

# Battery metrics
float32 battery_level_pct       # 0-100
float32 battery_voltage_v       # Volts
float32 battery_current_a       # Amps (negative = discharging)
float32 estimated_runtime_min   # Based on current usage
