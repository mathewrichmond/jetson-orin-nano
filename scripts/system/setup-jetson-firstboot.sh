#!/usr/bin/env bash
# setup-jetson-firstboot.sh
# Run from the dev machine (WSL2) after flashing a fresh Jetson Orin Nano.
# Connects over SSH to complete first-boot configuration:
#   - Sets hostname
#   - Installs Docker
#   - Creates deploy user and SSH authorized keys
#   - Configures data directories on NVMe SSD
#   - Enables Docker at boot
#   - Pulls and starts the robot-control container
#
# Usage (from WSL2 / laptop):
#   bash scripts/system/setup-jetson-firstboot.sh <jetson-ip-or-hostname>
#
#   # With options:
#   JETSON_HOST=192.168.1.100 JETSON_USER=nano bash scripts/system/setup-jetson-firstboot.sh

set -euo pipefail

JETSON_HOST="${1:-${JETSON_HOST:-}}"
JETSON_USER="${JETSON_USER:-nano}"
JETSON_HOSTNAME="${JETSON_HOSTNAME:-isaac}"
REGISTRY="${REGISTRY:-ghcr.io}"
IMAGE_OWNER="${IMAGE_OWNER:-}"   # GitHub org/user, e.g. "myorg"

step()  { echo -e "\n\033[36m==> $*\033[0m"; }
ok()    { echo -e "    \033[32m[OK]\033[0m $*"; }
warn()  { echo -e "    \033[33m[WARN]\033[0m $*"; }
info()  { echo -e "    \033[90m$*\033[0m"; }
fail()  { echo -e "    \033[31m[FAIL]\033[0m $*"; exit 1; }

if [[ -z "${JETSON_HOST}" ]]; then
    echo "Usage: $0 <jetson-ip-or-hostname>"
    echo "       JETSON_HOST=192.168.1.x $0"
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

${SSH_KEY} << EOF
    sudo hostnamectl set-hostname "${JETSON_HOSTNAME}"
    sudo sed -i "s/127.0.1.1.*/127.0.1.1\t${JETSON_HOSTNAME}/" /etc/hosts
    echo "Hostname: \$(hostname)"
EOF
ok "Hostname set"

# ─────────────────────────────────────────────────────────────────────────────
# 4. Install Docker
# ─────────────────────────────────────────────────────────────────────────────
step "Installing Docker on Jetson"

${SSH_KEY} << 'EOF'
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

${SSH_KEY} << 'EOF'
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

${SSH_KEY} << 'EOF'
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
# 7. Deploy initial container
# ─────────────────────────────────────────────────────────────────────────────
step "Setting up initial deployment"

${SSH_KEY} << 'EOF'
    set -e
    mkdir -p ~/src/jetson-orin-nano
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
echo " Test SSH:"
echo "   ssh -i ${DEPLOY_KEY} ${JETSON_USER}@${JETSON_HOST}"
echo ""
echo " Deploy via CI:"
echo "   GitHub → Actions → 'Deploy to Robot' → Run workflow"
echo "   jetson_host: ${JETSON_HOST}"
echo ""
