# Button Header Reference: Recovery, Reset, and Power Control

Complete reference for the Jetson Orin Nano Developer Kit Button Header (J12), including Force Recovery Mode pins required for flashing.

## Overview

The **Button Header (J12)** is a **12-pin 2x6 connector** that provides hardware control over:
- ✅ **Force Recovery Mode** (required for flashing)
- ✅ **System Reset**
- ✅ **Power On/Off control**
- ✅ **Auto Power-On configuration**
- ✅ **Sleep/Wake functions**

**Most Important**: Pins 9-10 (FC REC + GND) are required to flash the Jetson.

## Physical Location

The Button Header is located on the carrier board:
- **Near the 40-pin GPIO header**
- **Between the GPIO header and heatsink**
- Labeled "J12" or "Button Header" on PCB silkscreen
- 12 pins arranged in 2x6 configuration (6 rows, 2 columns)

## Complete Pinout

```
Button Header J12 (12-pin, 2x6):
View from top of board, pin 1 at top

Pin #  Signal          Function
═════════════════════════════════════════
 1  2  PWR BTN    N   Power Button (short to power on/off)
 3  4  GND        N   Ground for power button
 5  6  DISABLE    N   Auto Power-On Disable (short to disable)
 7  8  RST BTN    N   Reset Button (short to reset system)
 9 10  FC REC     N   Force Recovery + GND (CRITICAL FOR FLASHING)
11 12  SLP BTN    N   Sleep/Wake Button

Pin 1 is closest to 40-pin GPIO header
Pin 2 is closest to heatsink/module
```

### Detailed Pin Assignments

| Pin | Signal | Name | Function | Usage |
|-----|--------|------|----------|-------|
| **1** | PWR BTN | Power Button | Power on/off trigger | Short to GND (pin 3 or 4) to power on or initiate shutdown |
| **2** | N | - | Not connected | - |
| **3** | GND | Ground | Ground reference | Common ground for all buttons |
| **4** | N | - | Not connected | - |
| **5** | DISABLE | Auto Power Disable | Disable auto power-on | Short to pin 6 to prevent auto power-on when DC plugged in |
| **6** | N | - | Not connected | - |
| **7** | RST BTN | Reset Button | System reset | Short to pin 8 (or GND) to reset system |
| **8** | N | - | Not connected | - |
| **9** | **FC REC** | **Force Recovery** | **Recovery mode** | **Short to pin 10 (GND) to enter recovery mode** |
| **10** | **GND** | **Ground** | **Ground for recovery** | **Ground reference for recovery pin** |
| **11** | SLP BTN | Sleep Button | Sleep/wake trigger | Short to pin 12 to sleep/wake system |
| **12** | N | - | Not connected | - |

## Critical: Force Recovery Mode (Pins 9-10)

**Purpose**: Enter recovery mode for flashing JetPack with SDK Manager.

### Pin Identification

```
Button Header (view from top):
                          
  Pin 1  ● ○  Pin 2       ← Top (near GPIO header)
  Pin 3  ● ○  Pin 4
  Pin 5  ● ○  Pin 6
  Pin 7  ● ○  Pin 8
  Pin 9  ● ○  Pin 10      ← THESE PINS! (3rd from bottom)
  Pin 11 ● ○  Pin 12      ← Bottom

Pin 9 (FC REC) and Pin 10 (GND) are in the third row from bottom.
```

### Visual Identification Tips

1. **Count from top**: Pins 9-10 are the 5th row (row 9-10) from the top
2. **Count from bottom**: Pins 9-10 are the 2nd row from the bottom
3. **Look for labels**: Some boards have "FC REC" or "REC" silkscreen near pins 9-10
4. **Pin 1 marker**: Pin 1 usually has square pad or triangle marker

### How to Short Pins 9-10

**Required**: Female-to-female jumper wire, or any conductive wire/paperclip

**Method 1: Jumper Wire (Recommended)**
```
1. Take female-to-female jumper wire
2. Insert one end into pin 9 hole
3. Insert other end into pin 10 hole
4. Ensure good contact
5. Both pins are now electrically connected
```

**Method 2: Wire or Paperclip**
```
1. Take bare wire or paperclip
2. Bend to touch both pin 9 and pin 10 simultaneously
3. Hold firmly during power-on or reset
```

### Recovery Mode Procedures

#### Procedure A: From Powered-Off State (Easiest)

**Use when**: Jetson is currently powered off or you can safely power it off.

```
Steps:
1. Ensure Jetson is powered off
   - Unplug DC power adapter from Jetson
   - Wait 10 seconds

2. Prepare recovery jumper
   - Get female-to-female jumper wire
   - Locate Button Header pins 9 and 10

3. Short pins 9-10
   - Connect jumper across pins 9 (FC REC) and 10 (GND)
   - Ensure both pins are firmly connected

4. Power on while pins shorted
   - While keeping jumper connected, plug in DC power
   - System powers on automatically
   - Keep jumper connected for 5 seconds

5. Remove jumper
   - After 5 seconds, remove jumper wire
   - System is now in recovery mode

6. Connect USB-C to host PC
   - Connect USB-C cable from Jetson to Ubuntu host PC

7. Verify recovery mode
   - On host PC: lsusb | grep -i nvidia
   - Should show: ID 0955:7023 NVIDIA Corp.
```

#### Procedure B: From Powered-On State (Reset Method)

**Use when**: Jetson is currently running or stuck, and you can't easily power it off.

```
Steps:
1. Connect jumper to pins 9-10
   - Short FC REC (pin 9) to GND (pin 10)
   - Keep jumper connected throughout

2. Trigger system reset
   - While keeping pins 9-10 shorted
   - Use another jumper to briefly short pins 7-8 (Reset)
   - Touch pins 7-8 together for 1 second, then release

3. System resets into recovery mode
   - Keep pins 9-10 connected during reset
   - System will boot into recovery mode instead of normal boot

4. Remove jumper after reset
   - Wait 5 seconds after reset
   - Remove jumper from pins 9-10

5. Connect USB-C (if not already connected)

6. Verify recovery mode
   - lsusb | grep -i nvidia
   - Should show: ID 0955:7023
```

#### Procedure C: Software Recovery (Requires SSH Access)

**Use when**: You have SSH or serial console access to a running Jetson.

```bash
# SSH into Jetson
ssh user@jetson-ip

# Execute recovery reboot command
sudo reboot --force forced-recovery

# System will reboot directly into recovery mode
# No need to short pins 9-10!
```

After reboot:
1. Connect USB-C to host PC
2. Verify recovery mode: `lsusb | grep -i nvidia`

### Verifying Recovery Mode

On your Ubuntu host PC:

```bash
# Check if Jetson is in recovery mode
lsusb | grep -i nvidia

# Expected output (recovery mode):
Bus 001 Device 005: ID 0955:7023 NVIDIA Corp. APX [Jetson Orin]

# If you see this instead (NOT in recovery):
Bus 001 Device 005: ID 0955:7020 NVIDIA Corp.
# ↑ This means normal boot, not recovery mode
```

**Device IDs**:
- `0955:7023` - Recovery mode ✅ (ready to flash)
- `0955:7020` - Normal boot ❌ (not in recovery)
- Nothing - Not connected or not detected ❌

## System Reset (Pins 7-8)

**Purpose**: Hardware reset of the system (equivalent to power cycle).

### When to Use Reset

- System is frozen or unresponsive
- Need to restart without unplugging power
- Entering recovery mode from powered-on state (with pins 9-10 shorted)
- Testing boot behavior

### How to Reset

```
1. Locate pins 7 and 8 on Button Header
2. Short pins 7-8 together briefly (1-2 seconds)
   - Use jumper wire, paperclip, or screwdriver
3. Release (remove short)
4. System will reset and reboot immediately
```

**Note**: This is a hard reset. Unsaved work will be lost.

## Power Button (Pins 1-3)

**Purpose**: Soft power on/off (like pressing a physical power button).

### Power On

**Default Behavior**: Jetson powers on automatically when DC adapter is plugged in.

**Manual Power On** (if auto power-on is disabled):
```
1. Ensure DC power adapter is plugged in
2. Short pins 1 (PWR BTN) and 3 (GND) briefly
3. System powers on
```

### Power Off

```
1. While system is running
2. Short pins 1 (PWR BTN) and 3 (GND) briefly (< 1 second)
3. Initiates graceful shutdown (like "sudo shutdown")
4. Wait for system to fully power down
```

**Force Power Off** (not recommended):
```
1. Hold pins 1-3 shorted for > 10 seconds
2. System forces power off (like holding power button)
3. ⚠️ May cause filesystem corruption if not careful
```

## Auto Power-On Control (Pins 5-6)

**Purpose**: Control whether Jetson automatically powers on when DC power is connected.

### Default Behavior

By default, Jetson Orin Nano **automatically powers on** when you plug in the DC adapter.

### Disable Auto Power-On

```
To disable auto power-on:
1. Power off Jetson completely
2. Install jumper across pins 5-6 (DISABLE + NC)
3. Keep jumper installed permanently
4. Now Jetson will NOT auto power-on when DC is plugged in
5. Use pins 1-3 (power button) to manually power on
```

### Enable Auto Power-On (Re-enable Default)

```
To re-enable auto power-on:
1. Power off Jetson
2. Remove jumper from pins 5-6
3. Now Jetson will auto power-on when DC is plugged in (default)
```

**Use Cases for Disabling Auto Power-On**:
- Want manual control over power-on
- Building headless system with physical power button
- Prevent accidental boot during maintenance

## Sleep/Wake Button (Pins 11-12)

**Purpose**: Put system into sleep mode or wake from sleep.

### Entering Sleep Mode

```
1. While system is running
2. Short pins 11 (SLP BTN) and 12 briefly
3. System enters sleep/suspend mode
4. Power consumption reduced
```

### Waking from Sleep

```
1. While system is in sleep mode
2. Short pins 11-12 briefly
3. System wakes up from sleep
```

## Pin Header Connector Type

**Connector**: 2.54mm (0.1") pitch, 2x6 pin header

**Mating Connector**: Standard 0.1" female jumper wires or 2x6 socket connector

## Common Use Cases

### Flashing JetPack (Most Common)

```
1. Power off Jetson (unplug DC)
2. Short pins 9-10 (FC REC + GND) with jumper wire
3. Power on (plug in DC) while keeping 9-10 shorted
4. Remove jumper after 5 seconds
5. Connect USB-C to host PC
6. Run SDK Manager
7. Flash JetPack
```

See [FLASHING.md](FLASHING.md) for complete guide.

### Emergency Reset

```
System is frozen and SSH doesn't work:
1. Short pins 7-8 (RST BTN) for 1 second with jumper wire
2. Release
3. System reboots
```

### Graceful Shutdown

```
Want to power off without SSH:
1. Short pins 1-3 (PWR BTN) briefly (< 1 second) with jumper wire
2. System initiates shutdown
3. Wait for power LED to turn off
```

### Force Power Off (Last Resort)

```
System is completely unresponsive:
1. Hold pins 1-3 shorted for > 10 seconds with jumper wire
2. System forces power off
3. ⚠️ May cause data loss!
```

## Troubleshooting

### Recovery Mode Not Working

**Symptoms**: Jetson doesn't enter recovery mode, `lsusb` doesn't show NVIDIA device

**Check**:
1. ✅ Are you shorting the correct pins? (9 and 10, not 7 and 8)
2. ✅ Is jumper making good electrical contact?
3. ✅ Did you power on while keeping pins shorted?
4. ✅ Is USB-C cable connected and data-capable (not charge-only)?
5. ✅ Did you wait 5 seconds before removing jumper?

**Try**:
- Use different jumper wire
- Check pin identification (count from pin 1)
- Try reset method (pins 9-10 shorted, then reset via pins 7-8)
- Verify USB cable works for data transfer

### Reset Not Working

**Symptoms**: System doesn't reboot when pins 7-8 are shorted

**Solutions**:
- Short pins 7-8 for longer (2-3 seconds)
- Ensure good contact
- Try pins 1-3 (power button) held for 10+ seconds (force power off)
- Unplug DC power as last resort

### Auto Power-On Won't Disable

**Symptoms**: Jetson still powers on automatically with jumper on pins 5-6

**Solutions**:
- Ensure jumper is on correct pins (5 and 6)
- Power must be completely removed before changing jumper
- Wait 10 seconds after removing power before installing jumper
- Check jumper is making good connection

## Safety Notes

⚠️ **Ground yourself** before touching header pins (ESD protection)  
⚠️ **Don't short wrong pins** - Can damage system  
⚠️ **Remove power** before installing permanent jumpers  
⚠️ **Don't connect power** to button header pins (pins marked as "N" or GND only)  
⚠️ **Force power-off** can cause filesystem corruption - use only when necessary

## Related Documentation

- [FLASHING.md](FLASHING.md) - Complete flashing guide using recovery mode
- [SERIAL_CONSOLE.md](SERIAL_CONSOLE.md) - Debug console for monitoring boot
- [STORAGE_SETUP.md](STORAGE_SETUP.md) - Storage setup before flashing
- [RECOVERY.md](../../RECOVERY.md) - System recovery procedures

## External Resources

- **Jetson Orin Nano User Guide**: https://developer.nvidia.com/embedded/learn/jetson-orin-nano-devkit-user-guide/howto.html
- **Carrier Board Specification**: Search "Jetson Orin Nano Developer Kit Carrier Board" on NVIDIA Downloads
- **SDK Manager**: https://developer.nvidia.com/sdk-manager
