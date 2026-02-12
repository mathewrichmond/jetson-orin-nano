#!/bin/bash
# Production System Setup Script
# Sets up headless autonomous system with synced storage
#
# Run with: sudo ./scripts/system/setup_production.sh

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

if [ "$EUID" -ne 0 ]; then
    echo -e "${RED}Must run with sudo${NC}"
    exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ISAAC_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

echo "=========================================="
echo "Production System Setup"
echo "=========================================="
echo ""

# Step 1: Create data structure
echo -e "${GREEN}[1/5] Creating data directories...${NC}"
mkdir -p /data/{logs,config,models,sessions}
chown -R nano:nano /data
echo "✓ Created /data structure"

# Step 2: Copy configs to persistent storage
echo -e "${GREEN}[2/5] Setting up persistent config...${NC}"
if [ -d "$ISAAC_ROOT/config" ]; then
    mkdir -p /data/config/robot
    cp -rn "$ISAAC_ROOT/config/"* /data/config/ 2>/dev/null || true
    chown -R nano:nano /data/config
    echo "✓ Configs copied to /data/config"
fi

# Step 3: Set up headless boot
echo -e "${GREEN}[3/5] Configuring headless boot...${NC}"
systemctl set-default multi-user.target
echo "✓ Set default target to multi-user (no GUI)"

# Step 4: Install sync tools
echo -e "${GREEN}[4/5] Installing sync tools...${NC}"
apt-get update -qq
apt-get install -y -qq rsync > /dev/null 2>&1
echo "✓ Sync tools installed"

# Step 5: Install sync service
echo -e "${GREEN}[5/5] Installing sync service...${NC}"
if [ -f "$ISAAC_ROOT/config/systemd/isaac-sync.service" ]; then
    cp "$ISAAC_ROOT/config/systemd/isaac-sync.service" /etc/systemd/system/
    cp "$ISAAC_ROOT/config/systemd/isaac-sync.timer" /etc/systemd/system/
    systemctl daemon-reload
    systemctl enable isaac-sync.timer
    echo "✓ Sync service installed (edit /usr/local/bin/isaac-sync.sh to configure remote)"
fi

echo ""
echo "=========================================="
echo -e "${GREEN}Production setup complete!${NC}"
echo "=========================================="
echo ""
echo "Next steps:"
echo "  1. Configure sync: sudo nano /usr/local/bin/isaac-sync.sh"
echo "  2. Add SD fallback: sudo $ISAAC_ROOT/scripts/system/add_sd_fallback.sh"
echo "  3. Verify: make system-check"
echo ""
echo "Data structure:"
echo "  /data/logs/     - Robot logs (synced)"
echo "  /data/config/   - Calibrations and configs (synced)"
echo "  /data/models/   - VLA models"
echo "  /data/sessions/ - Session data"
