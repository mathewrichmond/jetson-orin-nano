# NodeHealth.msg
# Health status for a single ROS 2 node

# Node identification
string node_name                # Full node name (e.g., "/rpi/chassis/chassis_controller")
string node_namespace           # Node namespace

# Node status
string status                   # "OK", "WARN", "ERROR", "CRITICAL", "STALE"
string status_message           # Human-readable status description

# Watchdog information
WatchdogStatus[] watchdogs      # Status of all input watchdogs

# Resource usage (if available)
float32 cpu_usage_pct           # 0-100
float32 memory_usage_mb         # MB

# Timestamps
builtin_interfaces/Time last_heartbeat
float32 uptime_sec
