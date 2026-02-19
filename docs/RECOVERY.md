# Recovery Procedures for Headless System

Emergency recovery procedures for headless Jetson system with network-only access.

---

## Boot Fallback (Post-Migration)

After NVMe migration, you have two boot options:

### Normal Boot (NVMe - Default)
- Power on → waits 30 seconds → boots from NVMe
- SSH available at: `ssh nano@isaac.local`

### Emergency Fallback (SD Card)
If NVMe boot fails or you lose network access:

1. **Physical access to UART/serial console** (J14 debug header)
2. **Power cycle** the Jetson
3. **During 30-second countdown**, connect serial console
4. **Press any key** to interrupt boot
5. **Select**: `Boot from SD card (fallback)`
6. System boots from original SD (with working SSH/network)

---

## Network/SSH Recovery

### If you lose SSH access after migration

**Option 1: Serial Console** (recommended for headless)
- Connect USB-to-serial adapter to J14 debug header
- Pins: GND (pin 7), RX (pin 8), TX (pin 10)
- Settings: 115200 baud, 8N1
- Power on, interrupt boot, select SD fallback

**Option 2: Boot to SD fallback**
- If you have physical access to interrupt boot
- Select SD fallback from menu
- SSH to SD system (original config)

**Option 3: Re-flash** (last resort)
- Use NVIDIA SDK Manager from host PC
- Requires recovery mode jumper

---

## Pre-Migration Safety Checklist

Before running migration:

- [x] SSH verified working: `systemctl status ssh`
- [x] Network verified: `ping -c 3 8.8.8.8`
- [x] mDNS working: `ping isaac.local` from remote machine
- [x] SSD verified: `sudo ./scripts/system/verify_ssd.sh`
- [ ] **Backup SSH keys**: `cp -r ~/.ssh ~/ssh_backup`
- [ ] **Document current IP**: `ip addr show`
- [ ] **Test SSH from remote**: `ssh nano@isaac.local` from another machine
- [ ] **Serial console available** (if possible - J14 debug header)

---

## Post-Migration Verification

After reboot (should boot to NVMe):

```bash
# SSH into system
ssh nano@isaac.local

# Verify boot device
findmnt -n -o SOURCE /
# Should show: /dev/nvme0n1p1

# Verify network
ip addr show
ping -c 3 8.8.8.8

# Run system check
make system-check

# Verify SSH still enabled for next boot
systemctl is-enabled ssh
```

---

## If Migration Fails

### Symptoms
- Cannot SSH after reboot
- System doesn't respond to `isaac.local`
- No response on network

### Recovery Steps

1. **Wait 2-3 minutes** for full boot attempt
2. **Power cycle** the Jetson (unplug/replug)
3. **Interrupt boot** during 30-second countdown:
   - If you have serial console: press key, select SD
   - If no console: wait 30 sec, it will default to NVMe again
4. **Serial console method**:
   ```
   # Connect serial (115200 baud)
   # See boot menu
   # Select: Boot from SD card (fallback)
   ```
5. **Once on SD**:
   - SSH works (original system)
   - Check logs: `sudo journalctl -b`
   - Fix issue or restore extlinux:
     ```bash
     sudo cp /boot/extlinux/extlinux.conf.pre-nvme-migration-* /boot/extlinux/extlinux.conf
     sudo reboot
     ```

---

## Serial Console Setup (Recommended for Headless)

### Hardware
- USB-to-TTL serial adapter (3.3V!)
- J14 debug header on Jetson carrier board

### Pinout
```
J14 Header (40-pin):
Pin 1:  3.3V (DO NOT CONNECT to adapter VCC)
Pin 6:  GND  ──> Adapter GND
Pin 8:  TXD  ──> Adapter RX
Pin 10: RXD  ──> Adapter TX
```

### Software
```bash
# On your laptop/workstation
screen /dev/ttyUSB0 115200
# or
minicom -D /dev/ttyUSB0 -b 115200
```

You'll see boot messages and bootloader menu.

---

## Rollback to SD-Only Boot

If you want to undo the migration:

```bash
# Boot to SD (via fallback menu)
ssh nano@isaac.local

# Restore original bootloader config
sudo cp /boot/extlinux/extlinux.conf.pre-nvme-migration-* /boot/extlinux/extlinux.conf

# Change DEFAULT back to primary (SD)
sudo nano /boot/extlinux/extlinux.conf
# Change: DEFAULT nvme
# To:     DEFAULT primary

# Reboot
sudo reboot
```

Now boots to SD by default. NVMe data remains intact.

---

## Emergency Contacts

- **Serial console**: Essential for headless recovery
- **Backup network config**: Document static IP if not using DHCP
- **Known working SSH keys**: Keep backup of `~/.ssh/authorized_keys`

---

**IMPORTANT FOR HEADLESS**: Always have a serial console adapter available for emergency recovery. It's the ultimate fallback when network access fails.

---

## Complete Hardware Documentation

For detailed hardware procedures and recovery methods, see:

### Jetson Orin Nano Hardware
- **[Hardware Overview](hardware/jetson-orin-nano/)** - Board specifications and connectors
- **[Storage Setup](hardware/jetson-orin-nano/STORAGE_SETUP.md)** - NVMe SSD and SD card installation
- **[Flashing Guide](hardware/jetson-orin-nano/FLASHING.md)** - Recovery mode and SDK Manager procedure
- **[Serial Console](hardware/jetson-orin-nano/SERIAL_CONSOLE.md)** - J14 UART setup for headless debugging
- **[Button Header](hardware/jetson-orin-nano/BUTTON_HEADER.md)** - Recovery pins, reset, and power control
- **[GPIO Reference](hardware/jetson-orin-nano/GPIO_REFERENCE.md)** - 40-pin header pinout and I2C mapping

### SparkFun Auto pHAT
- **[Hardware Overview](hardware/sparkfun-auto-phat/)** - Robotics control board documentation
