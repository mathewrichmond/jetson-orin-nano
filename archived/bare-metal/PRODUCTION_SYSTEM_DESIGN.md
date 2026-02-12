# Production System Design

Design for a headless, autonomous Isaac robot system: lightweight Docker host with containerized microservices, synced storage, and no keyboard/mouse dependency.

## Design Principles

1. **Headless autonomous** – System runs fully without display, keyboard, or mouse
2. **Local-first** – All writes go to local storage first; sync when network available
3. **Synced mounts** – Logs and configs are synced, not pure NFS/mounts (avoids single-point-of-failure)
4. **Light host** – Minimal OS + Docker; application logic in containers

---

## Host: Minimal Docker Base

The Jetson runs a thin host:

| Component | Purpose |
|-----------|---------|
| JetPack L4T | Kernel, NVIDIA drivers, bootloader |
| Docker + NVIDIA Container Runtime | Container orchestration, GPU access |
| udev rules | Camera/servo device permissions |
| Network | mDNS (isaac.local), DHCP or static |
| Sync agent | Sync logs/config to remote when online |

**Excluded from host**:
- ROS 2 (runs in containers)
- Python packages (in containers)
- Application code (in containers or bind-mounted)

---

## Storage Strategy: Synced, Not Pure Mounts

For an autonomous system that may run offline, use **local-first with sync** rather than network mounts.

### Why Synced (Not NFS/Pure Mounts)

| Approach | Offline behavior | Failure mode |
|----------|------------------|---------------|
| NFS/CIFS mount | Fails if network down | Robot cannot log or load config |
| Local + sync | Works offline; syncs when online | No single dependency on network |

### Mount Layout

```
/data/
├── logs/           # Robot logs – local write, sync to remote
├── config/        # Calibrations, robot config – local write, sync to remote
├── models/        # VLA models – can be large; sync when updated
└── sessions/      # Session data – local write, sync when online
```

### Sync Strategy

- **Local**: All writes go to `/data/*` on NVMe (or SSD)
- **Sync when available**: When network is up, sync to remote (e.g., NAS, server, S3)
- **Tools**: rsync, Syncthing, rclone, or custom sync script with systemd

Example (conceptual):

```bash
# Sync script (runs periodically or when network comes up)
rsync -avz --delete /data/logs/ user@server:/backup/isaac/logs/
rsync -avz --delete /data/config/ user@server:/backup/isaac/config/
```

---

## Boot Configuration for Headless

### Boot Order

1. Kernel boots (no display output needed)
2. `multi-user.target` – no graphical login
3. Network brought up (DHCP or static)
4. Docker started
5. Robot stack started via systemd (user or system service)

### Recommended systemd Target

```ini
# Default to multi-user (no GUI)
# /etc/systemd/system/default.target -> multi-user.target
```

### Startup Sequence

1. `network-online.target` – wait for network (optional)
2. `docker.service` – Docker daemon
3. `isaac-robot.service` – Robot containers/stack

Boot should proceed without user input. Extlinux timeout can stay at 30s for dual-boot (NVMe/SD) or be reduced if desired.

---

## Containerized Microservices

### Architecture

```
┌─────────────────────────────────────────────────────┐
│ Host (minimal)                                       │
│  - Docker                                            │
│  - /data/{logs,config,models} (bind-mounted)         │
│  - Sync agent (periodic or on network-up)           │
└─────────────────────────────────────────────────────┘
         │
         │ bind mounts
         ▼
┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐
│ perception      │  │ control         │  │ logging         │
│ (cameras, etc.) │  │ (VLA, actions)  │  │ (central logs)  │
└─────────────────┘  └─────────────────┘  └─────────────────┘
         │                    │                    │
         └────────────────────┴────────────────────┘
                              │
                    ROS 2 / Zenoh / shared bus
```

### Volume Mounts (Docker)

```yaml
# Example docker-compose volumes
volumes:
  - /data/logs:/data/logs
  - /data/config:/data/config
  - /data/models:/data/models
  - /data/sessions:/data/sessions
```

Containers write to these paths; host sync agent syncs `/data` when network is available.

---

## Calibration and Config Persistence

- **Calibrations**: Stored in `/data/config/` (e.g., camera intrinsics, servo offsets)
- **Robot config**: `config/robot/*.yaml` – can be bind-mounted from `/data/config/` or a repo path
- **Persistence**: All under `/data/config/`; sync ensures backup and recovery

---

## Sync Implementation Options

### Option 1: systemd Timer + rsync

```ini
# /etc/systemd/system/isaac-sync.service
[Unit]
Description=Sync logs and config to remote
After=network-online.target

[Service]
Type=oneshot
ExecStart=/usr/local/bin/isaac-sync.sh
```

```ini
# /etc/systemd/system/isaac-sync.timer
[Unit]
Description=Periodic sync when network available
[Timer]
OnBootSec=5min
OnUnitActiveSec=1h
Persistent=true
[Install]
WantedBy=timers.target
```

### Option 2: Syncthing

- Bidirectional sync when online
- Handles conflicts; good for multi-site

### Option 3: Network Up Hook

- `systemd-networkd-wait-online` or `NetworkManager` dispatcher
- Run sync script when `network-online.target` is reached

---

## Headless Checklist

- [ ] Default runlevel: `multi-user.target` (no GUI)
- [ ] Boot proceeds without keyboard/mouse
- [ ] SSH enabled for remote access
- [ ] mDNS (avahi) for `isaac.local`
- [ ] Docker starts on boot
- [ ] Robot stack starts via systemd after Docker
- [ ] All data under `/data` with sync plan
- [ ] Extlinux: NVMe default, SD fallback (optional keypress for recovery)

---

## SSD Migration and This Design

After migrating to NVMe (see [SSD_MIGRATION.md](SSD_MIGRATION.md)):

1. **In-place**: Clone existing system; then add `/data` layout and sync
2. **Flash**: Clean install; configure `/data`, Docker, sync, and containers

For a fresh production setup, flash gives a minimal base. For preserving current config, in-place is simpler.

---

## References

- [SSD_MIGRATION.md](SSD_MIGRATION.md) – NVMe migration steps
- [DOCKER_FIRST_SETUP.md](DOCKER_FIRST_SETUP.md) – Container architecture
- [deployment/DEPLOYMENT.md](../deployment/DEPLOYMENT.md) – Multi-environment deployment
