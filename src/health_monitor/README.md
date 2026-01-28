# Health Monitor ROS 2 Package

Aggregates per-node health topics into system-level health status.

## Topics

- `/system/health/summary` (`std_msgs/String`) - Aggregated health summary (JSON)
- `/system/health/nodes` (`std_msgs/String`) - Raw per-node health payloads (JSON)

## Launch

```bash
ros2 launch health_monitor health_monitor.launch.py
```

Parameters:
- `graph_config`: Graph config YAML file (default `modular_graph.yaml`)
- `group`: Node group (default `all`)
- `health_publish_rate`: Publish rate for summary (Hz)
