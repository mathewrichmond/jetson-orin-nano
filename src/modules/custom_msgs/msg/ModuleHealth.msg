# ModuleHealth.msg
# Health status for a module (group of related nodes)

std_msgs/Header header

# Module identification
string module_name              # "chassis_control", "vision_pipeline", etc.
string host                     # "rpi", "jetson", "external"

# Overall module status
string status                   # "OK", "WARN", "ERROR", "CRITICAL"
string status_message           # Human-readable status description

# Per-node health
NodeHealth[] node_health        # Health of each node in this module

# Resource usage
float32 cpu_usage               # Module-level CPU % (sum of all nodes)
float32 memory_usage            # Module-level memory % (sum of all nodes)
float32 uptime_sec              # Module uptime

# Timestamps
builtin_interfaces/Time started_at
builtin_interfaces/Time last_update
