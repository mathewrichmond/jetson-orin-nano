#!/bin/bash
# Temperature Monitoring Script
# Monitors all thermal zones and reports status

set -e

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

WARNING_THRESHOLD=70
CRITICAL_THRESHOLD=85

echo "=========================================="
echo "Temperature Monitoring"
echo "=========================================="
echo ""

while true; do
    clear
    echo "=========================================="
    echo "Temperature Monitoring - $(date)"
    echo "=========================================="
    echo ""
    
    # Read all thermal zones
    for zone in /sys/class/thermal/thermal_zone*/type; do
        if [ -f "$zone" ]; then
            zone_num=$(echo "$zone" | grep -o '[0-9]*')
            temp_file="/sys/class/thermal/thermal_zone${zone_num}/temp"
            
            if [ -f "$temp_file" ]; then
                zone_name=$(cat "$zone" | tr -d '\n')
                temp=$(cat "$temp_file")
                temp_c=$((temp / 1000))
                
                # Color coding
                if [ "$temp_c" -ge "$CRITICAL_THRESHOLD" ]; then
                    COLOR=$RED
                    STATUS="CRITICAL"
                elif [ "$temp_c" -ge "$WARNING_THRESHOLD" ]; then
                    COLOR=$YELLOW
                    STATUS="WARNING"
                else
                    COLOR=$GREEN
                    STATUS="OK"
                fi
                
                printf "${COLOR}Zone %2d: %-20s %3d°C [%s]${NC}\n" "$zone_num" "$zone_name" "$temp_c" "$STATUS"
            fi
        fi
    done
    
    echo ""
    echo "Press Ctrl+C to exit"
    sleep 2
done

