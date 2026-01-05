#!/bin/bash
# Launch Foxglove Studio with ROS_DOMAIN_ID set
# Run this from your MacBook (viz computer)

export ROS_DOMAIN_ID=0

echo "Launching Foxglove Studio with ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo ""

# Try to find Foxglove Studio
if [ -d "/Applications/Foxglove.app" ]; then
    echo "Opening Foxglove Studio..."
    open -a "Foxglove"
elif [ -d "/Applications/Foxglove Studio.app" ]; then
    echo "Opening Foxglove Studio..."
    open -a "Foxglove Studio"
elif [ -d "$HOME/Applications/Foxglove.app" ]; then
    echo "Opening Foxglove Studio..."
    open -a "$HOME/Applications/Foxglove.app"
elif command -v foxglove-studio &> /dev/null; then
    echo "Starting Foxglove Studio..."
    foxglove-studio &
else
    echo "Foxglove Studio not found in standard locations"
    echo "Please open Foxglove Studio manually"
    echo ""
    echo "Make sure ROS_DOMAIN_ID=0 is set before opening"
    echo "You can set it with: export ROS_DOMAIN_ID=0"
    exit 1
fi

echo ""
echo "Foxglove Studio should now be opening..."
echo ""
echo "Connection settings:"
echo "  Framework: ROS 2"
echo "  URL: ws://192.168.0.129:8765"
echo "  (or ws://isaac.local:8765 if mDNS works)"
echo ""
