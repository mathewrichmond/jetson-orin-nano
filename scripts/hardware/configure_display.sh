#!/bin/bash
# Quick display configuration helper

echo "Available displays:"
xrandr --listmonitors 2>/dev/null || echo "X server not running or no displays detected"

echo ""
echo "Display connectors:"
for connector in /sys/class/drm/card*/status; do
    if [ -f "$connector" ]; then
        name=$(basename $(dirname $connector))
        status=$(cat $connector)
        echo "  $name: $status"
    fi
done

echo ""
echo "To configure a display:"
echo "  xrandr --output <connector> --auto --primary"
echo ""
echo "To enable a connector:"
echo "  echo on > /sys/class/drm/card*/<connector>/dpms"
