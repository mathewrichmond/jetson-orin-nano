#!/bin/bash
# Helper script to open Foxglove camera layout file
# Run this from your MacBook to quickly access the layout file

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

VIZ_CONFIG_DIR="$REPO_ROOT/config/visualization"
LAYOUT_FILE="$VIZ_CONFIG_DIR/foxglove_cameras_layout.json"
LAYOUT_ONLY="$VIZ_CONFIG_DIR/foxglove_cameras_only_layout.json"
LAYOUT_ORIGINAL="$VIZ_CONFIG_DIR/foxglove_layout.json"

echo "Foxglove Camera Layouts"
echo "========================"
echo ""
echo "Available layouts:"
echo "  1. Camera View (with system metrics) - $LAYOUT_FILE"
echo "  2. Cameras Only (clean view) - $LAYOUT_ONLY"
echo "  3. Original Layout - $LAYOUT_ORIGINAL"
echo ""
echo "To load in Foxglove Studio:"
echo "  1. File → Import Layout"
echo "  2. Select one of the files above"
echo ""
echo "Opening visualization config directory..."
open "$VIZ_CONFIG_DIR"
