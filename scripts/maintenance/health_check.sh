#!/bin/bash
# Automated Health Check Script
# Performs system health checks and takes corrective action

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

LOG_FILE="${LOG_FILE:-/var/log/isaac-health.log}"
ALERT_THRESHOLD_TEMP=85
ALERT_THRESHOLD_DISK=90
ALERT_THRESHOLD_MEM=95

log() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] $*" | tee -a "$LOG_FILE"
}

check_service() {
    local service=$1
    if systemctl is-active --quiet "$service"; then
        log "✓ Service $service is running"
        return 0
    else
        log "✗ Service $service is not running - attempting restart"
        sudo systemctl restart "$service" || log "Failed to restart $service"
        return 1
    fi
}

check_temperature() {
    local max_temp=0
    for zone in /sys/class/thermal/thermal_zone*/temp; do
        if [ -f "$zone" ]; then
            temp=$(cat "$zone")
            temp_c=$((temp / 1000))
            if [ "$temp_c" -gt "$max_temp" ]; then
                max_temp=$temp_c
            fi
        fi
    done

    log "Maximum temperature: ${max_temp}°C"

    if [ "$max_temp" -ge "$ALERT_THRESHOLD_TEMP" ]; then
        log "WARNING: Temperature ${max_temp}°C exceeds threshold"
        # Could trigger cooling measures here
        return 1
    fi
    return 0
}

check_disk() {
    local disk_usage=$(df -h / | awk 'NR==2 {print $5}' | sed 's/%//')
    log "Disk usage: ${disk_usage}%"

    if [ "$disk_usage" -ge "$ALERT_THRESHOLD_DISK" ]; then
        log "WARNING: Disk usage ${disk_usage}% exceeds threshold"
        # Trigger cleanup
        "$SCRIPT_DIR/self_clean.sh" || true
        return 1
    fi
    return 0
}

check_memory() {
    local mem_usage=$(free | awk 'NR==2{printf "%.0f", $3/$2 * 100}')
    log "Memory usage: ${mem_usage}%"

    if [ "$mem_usage" -ge "$ALERT_THRESHOLD_MEM" ]; then
        log "WARNING: Memory usage ${mem_usage}% exceeds threshold"
        return 1
    fi
    return 0
}

log "=========================================="
log "Isaac Robot System Health Check"
log "=========================================="

# Check services
check_service isaac-system-monitor.service || true
check_service isaac-robot.service || true

# Check system health
check_temperature
check_disk
check_memory

# Check ROS 2
if command -v ros2 &> /dev/null; then
    if ros2 node list &> /dev/null; then
        log "✓ ROS 2 is operational"
    else
        log "✗ ROS 2 nodes not responding"
    fi
fi

log "Health check complete"
log "=========================================="
