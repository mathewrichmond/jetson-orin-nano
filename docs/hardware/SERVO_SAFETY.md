# Camera Servo Safety Guide

## Overview

The camera servo system includes comprehensive safety controls to prevent damage to cameras and servos during operation. This guide explains the safety features and how to use them.

## Safety Features

### 1. Rate Limiting (Velocity Control)

**Purpose**: Prevents servos from moving too quickly, which could cause mechanical stress or damage.

**Configuration**:
- `servo_max_speed_deg_per_sec`: Maximum angular velocity (default: 30°/s)
- `servo_safe_mode_enabled`: Enable/disable rate limiting (default: true)

**Behavior**:
- When enabled, servo movements are limited to the maximum speed
- Movements are smoothed to prevent sudden jerks
- Can be disabled for faster movements once limits are verified

### 2. Acceleration Limiting

**Purpose**: Prevents sudden acceleration/deceleration that could stress mechanical components.

**Configuration**:
- `servo_max_accel_deg_per_sec2`: Maximum angular acceleration (default: 60°/s²)

**Behavior**:
- Servos accelerate and decelerate smoothly
- Prevents rapid direction changes

### 3. Hard Limits (Physical Limits)

**Purpose**: Prevents servos from exceeding mechanical range of motion.

**Configuration**:
- `servo_min_angle_deg`: Minimum angle (default: 0°)
- `servo_max_angle_deg`: Maximum angle (default: 180°)

**Behavior**:
- Commands outside these limits are automatically clamped
- Error messages are logged when limits are hit
- Movement stops at limits

### 4. Soft Limits (Warning Zone)

**Purpose**: Provides early warning before hitting hard limits.

**Configuration**:
- `servo_soft_limit_margin_deg`: Margin before hard limits (default: 5°)

**Behavior**:
- Warnings are logged when approaching soft limits
- Movement continues but operator is alerted
- Example: If hard limit is 180° and margin is 5°, warning at 175°

### 5. Emergency Stop

**Purpose**: Immediately stop all servo movement in case of emergency.

**Methods**:
1. **Topic**: Publish `true` to `/phat/servo_emergency_stop` (std_msgs/Bool)
2. **Service**: Call `/phat/servo_emergency_stop` service with `data: true`

**Behavior**:
- All servo movement stops immediately
- Servos hold current position
- Commands are ignored until emergency stop is released
- Status shows `servo_estop:active`

**Release Emergency Stop**:
- Publish `false` to topic or call service with `data: false`

### 6. Watchdog Timer

**Purpose**: Automatically holds position if commands stop arriving (prevents runaway).

**Configuration**:
- `servo_watchdog_timeout_sec`: Timeout duration (default: 1.0s)

**Behavior**:
- If no command received within timeout, servos hold current position
- Prevents servos from continuing to last commanded position indefinitely
- Warning logged when timeout occurs

### 7. Safe Initialization

**Purpose**: Slowly moves servos to safe startup position to avoid sudden movements.

**Configuration**:
- `servo_initialize_on_start`: Enable initialization (default: false)
- `servo_startup_pan_deg`: Startup pan angle (default: 90°)
- `servo_startup_tilt_deg`: Startup tilt angle (default: 90°)
- `servo_init_speed_deg_per_sec`: Initialization speed (default: 10°/s)
- `servo_init_delay_sec`: Delay between steps (default: 0.1s)

**Behavior**:
- Servos move slowly to startup position in small steps
- Prevents sudden movements on system startup
- Status shows `servo_init:in_progress` during initialization

## Configuration

Edit `config/hardware/phat_params.yaml` to adjust safety parameters:

```yaml
# Servo safety parameters
servo_max_speed_deg_per_sec: 30.0  # Maximum angular velocity (deg/s)
servo_max_accel_deg_per_sec2: 60.0  # Maximum angular acceleration (deg/s²)
servo_watchdog_timeout_sec: 1.0  # Timeout before holding position (seconds)
servo_soft_limit_margin_deg: 5.0  # Margin before hard limits (degrees)
servo_emergency_stop_enabled: true  # Enable emergency stop functionality
servo_safe_mode_enabled: true  # Start in safe mode (rate limited)
servo_init_speed_deg_per_sec: 10.0  # Speed for initialization (deg/s)
servo_init_delay_sec: 0.1  # Delay between init steps (seconds)
```

## Usage

### Starting Servos Safely

1. **Verify Configuration**:
   ```bash
   # Check current safety settings
   ros2 param get /hardware/phat_motor_controller_node servo_safe_mode_enabled
   ros2 param get /hardware/phat_motor_controller_node servo_max_speed_deg_per_sec
   ```

2. **Start with Safe Mode Enabled** (recommended for first use):
   - `servo_safe_mode_enabled: true` (default)
   - `servo_max_speed_deg_per_sec: 30.0` (conservative)
   - `servo_initialize_on_start: false` (until limits verified)

3. **Test Small Movements**:
   ```bash
   # Small pan movement
   ros2 topic pub --once /phat/camera_pan std_msgs/Float32 "{data: 95.0}"
   
   # Small tilt movement
   ros2 topic pub --once /phat/camera_tilt std_msgs/Float32 "{data: 95.0}"
   ```

4. **Gradually Increase Limits**:
   - Once you've verified mechanical limits, you can increase `servo_max_speed_deg_per_sec`
   - Test at each new speed before increasing further

### Emergency Stop

**Activate**:
```bash
# Via topic
ros2 topic pub --once /phat/servo_emergency_stop std_msgs/Bool "{data: true}"

# Via service
ros2 service call /phat/servo_emergency_stop std_srvs/SetBool "{data: true}"
```

**Release**:
```bash
# Via topic
ros2 topic pub --once /phat/servo_emergency_stop std_msgs/Bool "{data: false}"

# Via service
ros2 service call /phat/servo_emergency_stop std_srvs/SetBool "{data: false}"
```

### Monitoring Status

```bash
# Check servo status
ros2 topic echo /phat/status

# Look for:
# - servos:ok - Servos initialized and ready
# - servo_estop:active - Emergency stop is active
# - servo_init:in_progress - Initialization in progress
# - servo_safe_mode:on - Safe mode (rate limiting) enabled
```

## Safety Recommendations

### Initial Setup

1. **Start Conservative**: Use default safety settings initially
2. **Verify Mechanical Limits**: Manually check physical range of motion
3. **Test Small Movements**: Start with 5-10° movements
4. **Check for Binding**: Watch for any mechanical resistance or unusual sounds
5. **Gradually Increase**: Only increase speed limits after verifying safe operation

### During Operation

1. **Monitor Status**: Regularly check `/phat/status` topic
2. **Watch for Warnings**: Soft limit warnings indicate approaching limits
3. **Keep Emergency Stop Accessible**: Know how to activate emergency stop
4. **Test Watchdog**: Stop sending commands and verify servos hold position

### Troubleshooting

**Servos Not Moving**:
- Check if emergency stop is active: `ros2 topic echo /phat/status`
- Verify servos are initialized: Look for `servos:ok` in status
- Check if safe mode is limiting movement too much

**Servos Moving Too Fast**:
- Reduce `servo_max_speed_deg_per_sec`
- Ensure `servo_safe_mode_enabled: true`

**Hitting Limits Unexpectedly**:
- Verify `servo_min_angle_deg` and `servo_max_angle_deg` match physical limits
- Check soft limit warnings in logs
- Adjust limits if needed

**Watchdog Triggering**:
- Increase `servo_watchdog_timeout_sec` if commands arrive slowly
- Ensure command source is publishing regularly

## Advanced Configuration

### Disabling Safe Mode (Not Recommended)

Only disable safe mode after thoroughly testing mechanical limits:

```yaml
servo_safe_mode_enabled: false  # Disables rate limiting - USE WITH CAUTION
```

### Faster Initialization

If you've verified safe operation:

```yaml
servo_init_speed_deg_per_sec: 20.0  # Faster initialization
servo_init_delay_sec: 0.05  # Shorter delays
```

### Tighter Soft Limits

For more conservative operation:

```yaml
servo_soft_limit_margin_deg: 10.0  # Larger warning zone
```

## Integration with Control Systems

The servo safety system integrates with the robot control system:

- **Safe Mode**: Servos respect rate limits and safety checks
- **Emergency Stop**: Can be triggered by system monitor or user
- **Status Reporting**: Servo status included in system health monitoring

## Related Documentation

- [SparkFun Auto pHAT Setup](sparkfun_auto_phat.md) - Hardware setup
- [Hardware Setup Guide](HARDWARE_SETUP.md) - General hardware configuration
- [System Monitoring](../monitoring/SYSTEM_MONITOR.md) - System health monitoring
