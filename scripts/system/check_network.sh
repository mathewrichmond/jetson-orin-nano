#!/bin/bash
# Check Network Configuration
# Shows current network status and priority

set -e

echo "=========================================="
echo "Network Status Check"
echo "=========================================="
echo ""

# Device status
echo "=== Network Devices ==="
nmcli device status
echo ""

# Connection details
echo "=== Connection Details ==="
nmcli connection show --active
echo ""

# Route metrics (priority)
echo "=== Connection Priority (Route Metrics) ==="
echo "Lower metric = higher priority"
echo ""
for conn in $(nmcli -t -f NAME connection show --active); do
    metric_v4=$(nmcli -t -f ipv4.route-metric connection show "$conn" 2>/dev/null | cut -d: -f2 || echo "not set")
    metric_v6=$(nmcli -t -f ipv6.route-metric connection show "$conn" 2>/dev/null | cut -d: -f2 || echo "not set")
    type=$(nmcli -t -f connection.type connection show "$conn" 2>/dev/null | cut -d: -f2 || echo "unknown")
    echo "  $conn ($type):"
    echo "    IPv4 metric: $metric_v4"
    echo "    IPv6 metric: $metric_v6"
done
echo ""

# IP addresses
echo "=== IP Addresses ==="
ip -4 addr show | grep -E "^[0-9]+:|inet " | grep -v "127.0.0.1"
echo ""

# Default route
echo "=== Default Route ==="
ip route | grep default || echo "No default route found"
echo ""

# Connectivity test
echo "=== Connectivity Test ==="
if ping -c 1 -W 2 8.8.8.8 &>/dev/null; then
    echo "✓ Internet connectivity: OK"
else
    echo "✗ Internet connectivity: FAILED"
fi
echo ""
