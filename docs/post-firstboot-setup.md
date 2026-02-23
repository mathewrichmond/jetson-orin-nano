# Post First-Boot Setup: GitHub Secrets and Deploy

After `setup-jetson-firstboot.sh` completes, follow these steps to enable CI/CD deployment.

## WiFi (if not done during firstboot)

To connect the Jetson to your LAN via WiFi, re-run with credentials:

```bash
WIFI_SSID=YourNetworkName WIFI_PASSWORD=YourPassword bash scripts/system/setup-jetson-firstboot.sh <jetson-ip>
```

When connected via USB, the Jetson typically uses 192.168.55.1 (L4T default).

Or run on the Jetson directly:
```bash
ssh -i ~/.ssh/jetson_deploy nano@<jetson-ip>
sudo ./scripts/system/setup_wifi.sh   # interactive
```

## 1. Add JETSON_SSH_KEY to GitHub

1. Open your repo: **https://github.com/mathewrichmond/jetson-orin-nano** (or your fork)
2. Go to **Settings** → **Secrets and variables** → **Actions**
3. Click **New repository secret**
4. Name: `JETSON_SSH_KEY`
5. Value: Paste the **entire contents** of the deploy key (including `-----BEGIN` and `-----END` lines)

**To copy the key to clipboard (run in WSL):**
```bash
cat ~/.ssh/jetson_deploy | clip.exe
```
Then paste into the GitHub secret value field.

**Or display it to copy manually:**
```bash
cat ~/.ssh/jetson_deploy
```

## 2. Test SSH (optional)

From WSL:
```bash
ssh -i ~/.ssh/jetson_deploy nano@<jetson-ip>
```

You should see a shell on the Jetson. Type `exit` to disconnect.

**Note:** When connected via USB, the Jetson uses 192.168.55.1 by default. Once on LAN (Ethernet/Wi‑Fi), use its network IP or hostname (e.g. `isaac.local`).

## 3. Create GitHub Environments (if needed)

The deploy workflow uses `dev` and `production` environments. If they don't exist:

1. **Settings** → **Environments** → **New environment**
2. Create `dev` and `production`
3. No extra config required unless you want environment-specific secrets

## 4. Run the Deploy Workflow

1. Go to **Actions** → **Deploy to Robot**
2. Click **Run workflow**
3. Set:
   - **environment:** `dev` (or `production`)
   - **jetson_host:** Jetson IP (192.168.55.1 when on USB) or hostname (e.g. isaac.local on LAN)
4. Click **Run workflow**

The workflow will:
- Build an ARM64 Docker image on your self-hosted runner
- Push it to ghcr.io
- SSH to the Jetson and deploy the container
