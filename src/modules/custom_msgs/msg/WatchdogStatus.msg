# WatchdogStatus.msg
# Status of an input watchdog monitoring a topic or sensor

# Watchdog identification
string watchdog_name            # Name of the monitored input (e.g., "imu", "cmd_vel")
string topic_name               # Topic being monitored

# Watchdog configuration
float32 expected_rate_hz        # Expected message rate
float32 warn_timeout_sec        # Warning threshold
float32 error_timeout_sec       # Error threshold

# Current status
string status                   # "OK", "WARN", "ERROR", "STALE"
float32 actual_rate_hz          # Measured message rate
float32 time_since_last_msg_sec # Time since last message received
int32 message_count             # Total messages received

# Timestamps
builtin_interfaces/Time last_message_time
