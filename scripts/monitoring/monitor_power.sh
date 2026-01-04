#!/bin/bash
# Power Monitoring Script
# Monitors power consumption using tegrastats or power sensors

set -e

echo "=========================================="
echo "Power Monitoring"
echo "=========================================="
echo ""

if command -v tegrastats &> /dev/null; then
    echo "Using tegrastats for power monitoring..."
    echo "Press Ctrl+C to exit"
    echo ""
    
    # Run tegrastats with 1 second interval
    tegrastats --interval 1000
else
    echo "tegrastats not found. Checking power sensors..."
    
    # Check for INA3221 power sensors
    POWER_SENSORS=$(find /sys/bus/i2c/drivers/ina3221x -name "in_power*_input" 2>/dev/null | head -5)
    
    if [ -z "$POWER_SENSORS" ]; then
        echo "No power sensors found."
        echo "Install jetson_stats for power monitoring:"
        echo "  sudo pip3 install -U jetson-stats"
        exit 1
    fi
    
    echo "Found power sensors. Monitoring..."
    echo "Press Ctrl+C to exit"
    echo ""
    
    while true; do
        total_power=0
        sensor_count=0
        
        for sensor in $POWER_SENSORS; do
            if [ -f "$sensor" ]; then
                power_mw=$(cat "$sensor" 2>/dev/null || echo "0")
                power_w=$(echo "scale=3; $power_mw / 1000" | bc)
                total_power=$(echo "$total_power + $power_w" | bc)
                sensor_count=$((sensor_count + 1))
            fi
        done
        
        if [ $sensor_count -gt 0 ]; then
            echo "$(date '+%H:%M:%S') - Total Power: ${total_power}W (${sensor_count} sensors)"
        else
            echo "No sensors readable"
        fi
        
        sleep 1
    done
fi

