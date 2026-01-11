#!/bin/bash
# USB Microphone Setup Script
# Installs and configures USB microphone support

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
RED='\033[0;31m'
NC='\033[0m'

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}USB Microphone Setup${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

# Check if running as root
if [ "$EUID" -ne 0 ]; then
    echo -e "${RED}This script must be run with sudo${NC}"
    echo "Usage: sudo $0"
    exit 1
fi

# Install dependencies
echo -e "${GREEN}Installing audio dependencies...${NC}"
apt-get update
apt-get install -y alsa-utils pulseaudio alsa-base

# Reload ALSA modules
echo -e "${BLUE}Reloading ALSA modules...${NC}"
modprobe snd-usb-audio || true

# Ensure user is in audio group
USER_NAME=${SUDO_USER:-$USER}
if [ -z "$USER_NAME" ] || [ "$USER_NAME" = "root" ]; then
    echo -e "${YELLOW}Warning: Could not determine non-root user${NC}"
    echo -e "${YELLOW}Manually add user to audio group: sudo usermod -a -G audio <username>${NC}"
else
    if ! groups "$USER_NAME" | grep -q "\baudio\b"; then
        echo -e "${YELLOW}Adding user '$USER_NAME' to audio group...${NC}"
        usermod -a -G audio "$USER_NAME"
        echo -e "${YELLOW}Note: User may need to log out and back in for audio group changes to take effect${NC}"
    else
        echo -e "${GREEN}User '$USER_NAME' is already in audio group${NC}"
    fi
fi

echo ""
echo -e "${BLUE}Checking for USB microphones...${NC}"
if arecord -l 2>/dev/null | grep -q "USB Audio"; then
    echo -e "${GREEN}USB microphone(s) detected:${NC}"
    arecord -l | grep "USB Audio"
    echo ""

    # Test microphone
    MIC_CARD=$(arecord -l | grep "USB Audio" | head -1 | sed 's/^card \([0-9]*\):.*/\1/')
    if [ -n "$MIC_CARD" ]; then
        echo -e "${BLUE}Testing microphone on card $MIC_CARD...${NC}"

        # Try stereo first (most USB mics support this)
        if timeout 3 arecord -D plughw:$MIC_CARD,0 -d 2 -f S16_LE -r 16000 -c 2 /tmp/mic_test.wav 2>/dev/null; then
            echo -e "${GREEN}✓ Microphone test successful (stereo, 16kHz)${NC}"
            rm -f /tmp/mic_test.wav
        else
            echo -e "${YELLOW}⚠ Microphone test failed - check device configuration${NC}"
        fi
    fi
else
    echo -e "${YELLOW}No USB microphone detected${NC}"
    echo -e "${YELLOW}Connect a USB microphone and run: arecord -l${NC}"
fi

echo ""
echo -e "${GREEN}USB microphone setup complete${NC}"
echo ""
echo -e "${BLUE}Next steps:${NC}"
echo "  1. If user was added to audio group, log out and back in"
echo "  2. Verify microphone: arecord -l"
echo "  3. Test recording: arecord -D plughw:0,0 -d 5 -f S16_LE -r 16000 -c 2 test.wav"
echo "  4. Restart robot graph: ./scripts/system/manage_graph.sh restart robot"
