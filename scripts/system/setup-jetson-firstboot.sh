#!/usr/bin/env bash
# setup-jetson-firstboot.sh
# Run from the dev machine (WSL2) after flashing a fresh Jetson Orin Nano.
# Connects over SSH to complete first-boot configuration.
# When connected via USB, the Jetson uses 192.168.55.1 by default (L4T USB gadget).
#   - Sets hostname
#   - Configures WiFi (prompted) — enables LAN access, use SSH (isaac.local) thereafter
#   - Installs Docker
#   - Creates deploy user and SSH authorized keys
#   - Configures data directories on NVMe SSD
#   - Enables Docker at boot
#   - Pulls and starts the robot-control container
#
# Usage (from WSL2 / laptop):
#   bash scripts/system/setup-jetson-firstboot.sh <jetson-ip-or-hostname>
#
#   # Non-interactive (WiFi from env):
#   WIFI_SSID=MyNetwork WIFI_PASSWORD=secret bash scripts/system/setup-jetson-firstboot.sh <jetson-ip>
#
#   # Other options:
#   JETSON_HOST=192.168.1.100 JETSON_USER=nano bash scripts/system/setup-jetson-firstboot.sh

set -euo pipefail

JETSON_HOST="${1:-${JETSON_HOST:-}}"
JETSON_USER="${JETSON_USER:-nano}"
JETSON_HOSTNAME="${JETSON_HOSTNAME:-isaac}"
WIFI_SSID="${WIFI_SSID:-}"
WIFI_PASSWORD="${WIFI_PASSWORD:-}"
REGISTRY="${REGISTRY:-ghcr.io}"
IMAGE_OWNER="${IMAGE_OWNER:-}"   # GitHub org/user, e.g. "myorg"

step()  { echo -e "\n\033[36m==> $*\033[0m"; }
ok()    { echo -e "    \033[32m[OK]\033[0m $*"; }
warn()  { echo -e "    \033[33m[WARN]\033[0m $*"; }
info()  { echo -e "    \033[90m$*\033[0m"; }
fail()  { echo -e "    \033[31m[FAIL]\033[0m $*"; exit 1; }

if [[ -z "${JETSON_HOST}" ]]; then
    echo "Usage: $0 <jetson-ip-or-hostname>"
    echo ""
    echo "  Run over USB first, then connect via SSH (isaac.local) after WiFi is configured."
    echo ""
    echo "  # Example (Jetson USB default IP is 192.168.55.1):"
    echo "  $0 192.168.55.1"
    echo ""
    echo "  # Non-interactive (WiFi from env):"
    echo "  WIFI_SSID=MyNetwork WIFI_PASSWORD=secret $0 <jetson-ip>"
    echo ""
    echo "Tip: find Jetson IP with:"
    echo "  nmap -sn 192.168.1.0/24 | grep -B2 -i nvidia"
    exit 1
fi

# ─────────────────────────────────────────────────────────────────────────────
# 0. Generate or reuse deploy SSH key pair
# ─────────────────────────────────────────────────────────────────────────────
step "Preparing SSH deploy key"

DEPLOY_KEY="${HOME}/.ssh/jetson_deploy"
if [[ ! -f "${DEPLOY_KEY}" ]]; then
    ssh-keygen -t ed25519 -f "${DEPLOY_KEY}" -N "" -C "robotics-ci-deploy"
    ok "Generated deploy key: ${DEPLOY_KEY}"
else
    ok "Reusing existing deploy key: ${DEPLOY_KEY}"
fi

info "Public key:"
info "  $(cat ${DEPLOY_KEY}.pub)"

# ─────────────────────────────────────────────────────────────────────────────
# 1. Test initial SSH connection (first-boot OOBE user)
# ─────────────────────────────────────────────────────────────────────────────
step "Testing SSH connection to ${JETSON_HOST}"

ssh-keyscan -H "${JETSON_HOST}" >> ~/.ssh/known_hosts 2>/dev/null || true

if ssh -o ConnectTimeout=10 -o BatchMode=yes \
       "${JETSON_USER}@${JETSON_HOST}" "echo ok" &>/dev/null; then
    ok "SSH connected (key auth)"
else
    # Try with password prompt
    warn "Key auth failed - will attempt password auth for initial setup"
    info "Use the password you set during JetPack OOBE first-boot wizard"
fi

SSH="ssh -o ConnectTimeout=15 ${JETSON_USER}@${JETSON_HOST}"

# ─────────────────────────────────────────────────────────────────────────────
# 2. Authorize deploy key
# ─────────────────────────────────────────────────────────────────────────────
step "Installing deploy SSH key on Jetson"

ssh-copy-id -i "${DEPLOY_KEY}.pub" "${JETSON_USER}@${JETSON_HOST}"
ok "Deploy key installed"

SSH_KEY="ssh -i ${DEPLOY_KEY} -o ConnectTimeout=15 ${JETSON_USER}@${JETSON_HOST}"

# ─────────────────────────────────────────────────────────────────────────────
# 3. Configure hostname
# ─────────────────────────────────────────────────────────────────────────────
step "Setting hostname to '${JETSON_HOSTNAME}'"

${SSH_KEY} bash -s << EOF || fail "Step 3 (hostname) failed (exit $?)"
    set -e
    sudo hostnamectl set-hostname "${JETSON_HOSTNAME}"
    # Update /etc/hosts: 127.0.1.1 hostname hostname.local (for local resolution + mDNS)
    sudo sed -i "s/127.0.1.1.*/127.0.1.1\t${JETSON_HOSTNAME} ${JETSON_HOSTNAME}.local/" /etc/hosts
    echo "Hostname: \$(hostname)"
EOF
ok "Hostname set"

# ─────────────────────────────────────────────────────────────────────────────
# 3b. Configure WiFi (enables LAN access + mDNS — connect via SSH after firstboot)
# ─────────────────────────────────────────────────────────────────────────────
if [[ -z "${WIFI_SSID}" ]]; then
    echo ""
    read -p "WiFi SSID (or Enter to skip): " WIFI_SSID
    if [[ -n "${WIFI_SSID}" ]]; then
        read -s -p "WiFi password: " WIFI_PASSWORD
        echo ""
    fi
fi

if [[ -n "${WIFI_SSID}" ]]; then
    step "Configuring WiFi: ${WIFI_SSID}"

    ${SSH_KEY} bash -s << EOF || fail "Step 3b (WiFi) failed (exit $?)"
    set -e
    WIFI_SSID="${WIFI_SSID}"
    WIFI_PASSWORD="${WIFI_PASSWORD}"
    if ! command -v nmcli &>/dev/null; then
        sudo apt-get update -qq
        sudo apt-get install -y --no-install-recommends network-manager
        sudo systemctl enable NetworkManager
        sudo systemctl start NetworkManager
    fi
    WIFI_DEVICE=\$(nmcli device status | grep wifi | grep -v p2p | awk '{print \$1}' | head -1)
    if [ -z "\$WIFI_DEVICE" ]; then
        echo "No WiFi device found — skipping"
        exit 0
    fi
    sudo nmcli radio wifi on 2>/dev/null || true
    sudo nmcli device set "\$WIFI_DEVICE" managed yes 2>/dev/null || true
    CONN_NAME="WiFi-\${WIFI_SSID}"
    if nmcli connection show "\$CONN_NAME" &>/dev/null; then
        echo "WiFi connection already exists — activating"
        timeout 30 sudo nmcli connection up "\$CONN_NAME" 2>/dev/null || echo "Activation timed out or failed"
    else
        if [ -n "\$WIFI_PASSWORD" ]; then
            sudo nmcli connection add type wifi con-name "\$CONN_NAME" ifname "\$WIFI_DEVICE" ssid "\$WIFI_SSID" wifi-sec.key-mgmt wpa-psk wifi-sec.psk "\$WIFI_PASSWORD" ipv4.method auto ipv4.route-metric 200 connection.autoconnect yes
        else
            sudo nmcli connection add type wifi con-name "\$CONN_NAME" ifname "\$WIFI_DEVICE" ssid "\$WIFI_SSID" wifi-sec.key-mgmt none ipv4.method auto ipv4.route-metric 200 connection.autoconnect yes
        fi
        echo "Activating WiFi (timeout 30s)..."
        timeout 30 sudo nmcli connection up "\$CONN_NAME" || echo "Activation timed out — connection saved, may connect on reboot"
    fi
    echo "Waiting for DHCP..."
    sleep 5
    ip -4 addr show | grep -E "inet.*scope global" | head -3 || true
EOF
    ok "WiFi configured — Jetson on LAN"
else
    warn "WiFi skipped — Jetson will only be reachable via USB"
fi

# ─────────────────────────────────────────────────────────────────────────────
# 3c. Install Avahi (mDNS) for hostname.local resolution
# ─────────────────────────────────────────────────────────────────────────────
step "Configuring mDNS (Avahi) for ${JETSON_HOSTNAME}.local"

${SSH_KEY} bash -s << EOF || fail "Step 3c (mDNS) failed (exit $?)"
    set -e
    HOSTNAME="${JETSON_HOSTNAME}"
    sudo apt-get update -qq
    sudo apt-get install -y --no-install-recommends avahi-daemon avahi-utils
    # Publish workstation so hostname.local is advertised on LAN
    sudo sed -i 's/#publish-workstation=no/publish-workstation=yes/' /etc/avahi/avahi-daemon.conf
    sudo systemctl enable avahi-daemon
    sudo systemctl restart avahi-daemon
    echo "Avahi configured — \${HOSTNAME}.local will be advertised on LAN"
    avahi-resolve -n \${HOSTNAME}.local 2>/dev/null && echo "  (self-test OK)" || echo "  (self-test: run 'avahi-resolve -n \${HOSTNAME}.local' after reboot)"
EOF
ok "mDNS configured — ${JETSON_HOSTNAME}.local will resolve when on network"
info "  Reboot the Jetson for mDNS to advertise reliably on LAN (Ethernet/Wi‑Fi)"

# ─────────────────────────────────────────────────────────────────────────────
# 4. Install Docker
# ─────────────────────────────────────────────────────────────────────────────
step "Installing Docker on Jetson"

${SSH_KEY} bash -s << 'EOF' || fail "Step 4 (Docker) failed (exit $?)"
    set -e
    if command -v docker &>/dev/null; then
        echo "Docker already installed: $(docker --version)"
        exit 0
    fi

    # NVIDIA L4T ships with Docker, but may need the Compose plugin
    sudo apt-get update -qq
    sudo apt-get install -y --no-install-recommends \
        docker.io \
        docker-compose-plugin \
        ca-certificates \
        curl

    # Add user to docker group
    sudo usermod -aG docker "${USER}"

    # Enable and start
    sudo systemctl enable docker
    sudo systemctl start docker

    echo "Docker installed: $(docker --version)"
EOF
ok "Docker installed"

# ─────────────────────────────────────────────────────────────────────────────
# 5. Configure data directories on NVMe SSD
# ─────────────────────────────────────────────────────────────────────────────
step "Setting up data directories"

${SSH_KEY} bash -s << 'EOF' || fail "Step 5 (data directories) failed (exit $?)"
    set -e
    # NVMe SSD should be at /dev/nvme0n1 after flashing
    # Data partition is typically already mounted at /data by the flash config
    # If not, create the directories in the home dir as fallback

    DATA_ROOT="/data"
    if ! mountpoint -q "${DATA_ROOT}" 2>/dev/null; then
        echo "Note: /data is not a separate mount. Using home directory."
        DATA_ROOT="${HOME}/data"
    fi

    for dir in config worldgraph sessions logs; do
        mkdir -p "${DATA_ROOT}/${dir}"
    done

    echo "Data directories:"
    ls -la "${DATA_ROOT}/"
EOF
ok "Data directories created"

# ─────────────────────────────────────────────────────────────────────────────
# 6. Configure Docker to use /data for images (NVMe)
# ─────────────────────────────────────────────────────────────────────────────
step "Configuring Docker data root on NVMe"

${SSH_KEY} bash -s << 'EOF' || fail "Step 6 (Docker data root) failed (exit $?)"
    set -e
    if mountpoint -q /data 2>/dev/null; then
        sudo mkdir -p /data/docker
        cat <<DAEMON | sudo tee /etc/docker/daemon.json
{
  "data-root": "/data/docker",
  "runtimes": {
    "nvidia": {
      "path": "nvidia-container-runtime",
      "runtimeArgs": []
    }
  },
  "default-runtime": "nvidia"
}
DAEMON
        sudo systemctl restart docker
        echo "Docker data-root: /data/docker"
    else
        echo "Skipping Docker data-root move (no dedicated /data partition)"
    fi
EOF

# ─────────────────────────────────────────────────────────────────────────────
# 7. MAXN performance mode + persistent jetson_clocks
# ─────────────────────────────────────────────────────────────────────────────
step "Configuring MAXN performance mode"

${SSH_KEY} bash -s << 'EOF' || fail "Step 7 (MAXN mode) failed (exit $?)"
    set -e

    # Set MAXN (mode 0) — all cores, max GPU/memory bandwidth
    sudo nvpmodel -m 0
    echo "nvpmodel set to MAXN (mode 0)"

    # Install jetson_clocks as a systemd service so it persists across reboots
    sudo tee /etc/systemd/system/jetson-clocks.service > /dev/null << 'SERVICE'
[Unit]
Description=Maximize Jetson clocks (jetson_clocks)
After=nvpmodel.service
Wants=nvpmodel.service

[Service]
Type=oneshot
ExecStart=/usr/bin/jetson_clocks
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
SERVICE

    sudo systemctl daemon-reload
    sudo systemctl enable jetson-clocks.service
    sudo systemctl start jetson-clocks.service

    echo "jetson_clocks enabled at boot"
    sudo nvpmodel -q
EOF
ok "MAXN mode set, jetson_clocks enabled at boot"

# ─────────────────────────────────────────────────────────────────────────────
# 8. Sync repo to Jetson (needed for hardware setup scripts + pre-built .dtbo)
# ─────────────────────────────────────────────────────────────────────────────
step "Syncing repo to Jetson"

# Determine repo root (works whether running from repo or scripts/system/)
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

${SSH_KEY} "mkdir -p ~/src/jetson-orin-nano"

rsync -az --delete \
    --exclude='.git' \
    --exclude='__pycache__' \
    --exclude='*.pyc' \
    --exclude='install/' \
    --exclude='build/' \
    --exclude='log/' \
    -e "ssh -i ${DEPLOY_KEY} -o ConnectTimeout=15" \
    "${REPO_ROOT}/" \
    "${JETSON_USER}@${JETSON_HOST}:~/src/jetson-orin-nano/"

ok "Repo synced to ~/src/jetson-orin-nano on Jetson"
info "(includes pre-compiled i2c7-100khz.dtbo — no compiler needed on target)"

# ─────────────────────────────────────────────────────────────────────────────
# 9. Hardware setup — Pass 1 (groups, udev, I2C overlay → reboot)
# ─────────────────────────────────────────────────────────────────────────────
step "Running hardware setup Pass 1 (groups + I2C overlay)"

${SSH_KEY} bash -s << 'EOF' || fail "Step 9 (hardware setup Pass 1) failed (exit $?)"
    set -e
    cd ~/src/jetson-orin-nano
    chmod +x scripts/hardware/setup_hardware.sh
    chmod +x scripts/hardware/setup_i2c.sh
    chmod +x scripts/hardware/setup_gpio.sh
    # Pass 1: configures groups, udev rules, installs DT overlay, patches extlinux.conf
    # Exits cleanly with a reboot notice (does NOT call reboot itself)
    bash scripts/hardware/setup_hardware.sh full-setup
EOF
ok "Hardware setup Pass 1 complete — reboot required"

# ─────────────────────────────────────────────────────────────────────────────
# 10. Reboot Jetson
# ─────────────────────────────────────────────────────────────────────────────
step "Rebooting Jetson for I2C overlay to take effect"

${SSH_KEY} "sudo reboot" || true   # SSH will drop — that's expected
info "Waiting 45 seconds for Jetson to come back up..."
sleep 45

# Wait for SSH to become available (up to 2 minutes)
WAIT=0
until ssh -i "${DEPLOY_KEY}" -o ConnectTimeout=5 -o BatchMode=yes \
         "${JETSON_USER}@${JETSON_HOST}" "echo ok" &>/dev/null 2>&1; do
    WAIT=$((WAIT + 5))
    if [ "$WAIT" -gt 120 ]; then
        warn "Jetson not reachable after 2 minutes — continue manually:"
        warn "  ssh -i ${DEPLOY_KEY} ${JETSON_USER}@${JETSON_HOST}"
        warn "  cd ~/src/jetson-orin-nano"
        warn "  bash scripts/hardware/setup_hardware.sh full-setup"
        break
    fi
    info "  Still waiting... (${WAIT}s)"
    sleep 5
done

if [ "$WAIT" -le 120 ]; then
    ok "Jetson is back online after ${WAIT}s"
fi

# ─────────────────────────────────────────────────────────────────────────────
# 11. Hardware setup — Pass 2 (verify I2C, install drivers, full verify)
# ─────────────────────────────────────────────────────────────────────────────
step "Running hardware setup Pass 2 (driver install + verification)"

${SSH_KEY} bash -s << 'EOF' || fail "Step 11 (hardware setup Pass 2) failed (exit $?)"
    set -e
    cd ~/src/jetson-orin-nano
    # Pass 2: detects overlay is active, installs camera/GPIO/audio drivers, verifies
    bash scripts/hardware/setup_hardware.sh full-setup
EOF
ok "Hardware setup complete"

# ─────────────────────────────────────────────────────────────────────────────
# 12. Deploy initial container
# ─────────────────────────────────────────────────────────────────────────────
step "Setting up initial deployment"

${SSH_KEY} bash -s << 'EOF' || fail "Step 12 (deployment) failed (exit $?)"
    set -e
    echo "Ready for deployment via CI/CD."
    echo "Run the GitHub Actions 'Deploy to Robot' workflow targeting this host."
EOF

# ─────────────────────────────────────────────────────────────────────────────
# Summary + print deploy key for GitHub Secrets
# ─────────────────────────────────────────────────────────────────────────────
echo ""
echo "══════════════════════════════════════════════════════"
echo " First-boot config complete"
echo "══════════════════════════════════════════════════════"
echo ""
echo " Jetson hostname: ${JETSON_HOSTNAME} (${JETSON_HOST})"
echo " Deploy user:     ${JETSON_USER}"
echo " SSH key:         ${DEPLOY_KEY}"
echo ""
echo " Add these to GitHub repository secrets:"
echo "   JETSON_SSH_KEY  = contents of ${DEPLOY_KEY}"
echo "   (JETSON_HOST is passed as workflow input)"
echo ""
echo " To copy the private key to clipboard (WSL2):"
echo "   cat ${DEPLOY_KEY} | clip.exe"
echo ""
echo " Test SSH (use isaac.local when Jetson is on WiFi/LAN):"
echo "   ssh -i ${DEPLOY_KEY} ${JETSON_USER}@${JETSON_HOSTNAME}.local"
echo "   ssh -i ${DEPLOY_KEY} ${JETSON_USER}@${JETSON_HOST}   # or by IP"
echo ""
echo " Deploy via CI:"
echo "   GitHub → Actions → 'Deploy to Robot' → Run workflow"
echo "   jetson_host: ${JETSON_HOST}"
echo ""
