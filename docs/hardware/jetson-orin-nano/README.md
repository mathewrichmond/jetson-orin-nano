# NVIDIA Jetson Orin Nano Developer Kit Hardware Documentation

Complete hardware reference for the NVIDIA Jetson Orin Nano Developer Kit as used in the Isaac Robot project.

## Board Identification

- **Product Name**: NVIDIA Jetson Orin Nano Developer Kit
- **SKU**: 945-13766-0000-000 (8GB model)
- **Manufacturer**: NVIDIA Corporation
- **Form Factor**: Custom carrier board with 40-pin GPIO header
- **Module**: Jetson Orin Nano 8GB (260-pin SODIMM)

## Overview

The Jetson Orin Nano Developer Kit is an AI-focused single-board computer featuring:
- ARM Cortex-A78AE CPU (6-core @ 2.0 GHz)
- NVIDIA Ampere GPU (1024 CUDA cores)
- 8GB 128-bit LPDDR5 RAM
- Ubuntu 20.04/22.04 support via JetPack SDK

### Key Features for Robotics

- **40-pin GPIO Header** - Compatible with Raspberry Pi HATs and pHATs
- **M.2 NVMe Storage** - High-speed SSD support (PCIe Gen3 x4)
- **Multiple Camera Interfaces** - 2x CSI-2 MIPI camera connectors
- **USB 3.2** - High-bandwidth sensor/device connectivity
- **Gigabit Ethernet** - Reliable network communication
- **Serial Console (J14)** - UART debug access for headless systems

## Hardware Documentation

### Essential Guides

- **[STORAGE_SETUP.md](STORAGE_SETUP.md)** - NVMe SSD and SD card installation
- **[FLASHING.md](FLASHING.md)** - Recovery mode and SDK Manager flashing procedure
- **[SERIAL_CONSOLE.md](SERIAL_CONSOLE.md)** - J14 UART debug console setup
- **[BUTTON_HEADER.md](BUTTON_HEADER.md)** - Recovery, reset, and power control pins
- **[GPIO_REFERENCE.md](GPIO_REFERENCE.md)** - 40-pin GPIO header pinout and I2C mapping

### Quick Reference

| Feature | Details |
|---------|---------|
| **Storage** | M.2 2280 NVMe SSD (required), microSD card (optional fallback) |
| **Power** | 5V 4A DC barrel jack (19W typical, 25W peak) |
| **GPIO** | 40-pin header (3.3V logic, Raspberry Pi compatible) |
| **I2C** | Bus 7 on pins 3 (SDA) and 5 (SCL) |
| **Recovery Mode** | Button Header pins 9-10 (FC REC + GND) |
| **Serial Console** | J14 header (115200 baud, 8N1) |

## Key Headers and Connectors

### 40-Pin GPIO Header

Standard Raspberry Pi-compatible header with 3.3V logic levels.

**Critical for Isaac Robot**:
- **Pin 3/5**: I2C Bus 7 (SDA/SCL) - connects to SparkFun Auto pHAT
- **Pin 2/4**: 5V power output
- **Pin 1/17**: 3.3V power output

See [GPIO_REFERENCE.md](GPIO_REFERENCE.md) for complete pinout.

### Button Header (12-pin)

Control header for recovery mode, reset, and power management.

**Most Important**:
- **Pins 9-10**: Force Recovery (FC REC + GND) - required for flashing
- **Pins 7-8**: System Reset

See [BUTTON_HEADER.md](BUTTON_HEADER.md) for complete details.

### J14 Debug Header

UART serial console for headless debugging and recovery.

**Essential for Headless Systems**:
- Pin 6: GND
- Pin 8: TXD (Jetson → USB adapter RX)
- Pin 10: RXD (Jetson ← USB adapter TX)

See [SERIAL_CONSOLE.md](SERIAL_CONSOLE.md) for setup instructions.

## Storage Configuration

The Jetson Orin Nano requires storage to boot:

1. **Primary: NVMe SSD** (recommended)
   - M.2 2280 form factor
   - PCIe Gen3/Gen4 NVMe interface
   - Minimum 128GB, recommend 256GB+
   - Must be physically installed before flashing

2. **Fallback: microSD Card** (optional)
   - 32GB+ Class 10 UHS-I
   - Can be used for emergency recovery
   - Lower performance than NVMe

See [STORAGE_SETUP.md](STORAGE_SETUP.md) for installation instructions.

## Flashing and Recovery

### Quick Start

1. Install NVMe SSD ([STORAGE_SETUP.md](STORAGE_SETUP.md))
2. Put Jetson in recovery mode:
   - Power off
   - Short Button Header pins 9-10 (FC REC + GND)
   - Power on while pins shorted
3. Connect USB-C to Ubuntu host PC
4. Run SDK Manager to flash JetPack

See [FLASHING.md](FLASHING.md) for complete step-by-step guide.

### Recovery Mode Verification

```bash
# On Ubuntu host PC
lsusb | grep -i nvidia
# Should show: Bus XXX Device XXX: ID 0955:7023 NVIDIA Corp.
```

## Compatibility with Raspberry Pi Accessories

The 40-pin GPIO header is **electrically compatible** with Raspberry Pi HATs and pHATs, with one critical difference:

### I2C Bus Number Difference

| Platform | Physical Pins | Bus Number |
|----------|--------------|------------|
| Raspberry Pi | 3 (SDA), 5 (SCL) | Bus 1 |
| Jetson Orin Nano | 3 (SDA), 5 (SCL) | **Bus 7** |

**Important**: When using Raspberry Pi accessories that communicate via I2C, you must use bus 7 on Jetson instead of bus 1.

```bash
# Scan I2C bus on Jetson
sudo i2cdetect -y 7  # NOT bus 1

# On Raspberry Pi (for reference)
sudo i2cdetect -y 1
```

## Technical Specifications

### Compute Module (Jetson Orin Nano 8GB)

- **CPU**: 6-core ARM Cortex-A78AE @ 2.0 GHz
- **GPU**: 1024-core NVIDIA Ampere architecture
- **AI Performance**: 40 TOPS (INT8)
- **Memory**: 8GB 128-bit LPDDR5 @ 102.4 GB/s
- **Storage Interface**: PCIe Gen3 x4 + microSD
- **Power**: 7W - 25W (configurable power modes)

### Carrier Board

- **Dimensions**: 110mm x 100mm
- **Power Input**: DC 5V 4A (barrel jack, 5.5mm outer, 2.5mm inner, center positive)
- **Networking**: Gigabit Ethernet (RJ45)
- **USB**: 4x USB 3.2 Type-A, 1x USB 2.0 Type-C (device mode)
- **Display**: HDMI 2.1 and DisplayPort 1.4a
- **Cameras**: 2x MIPI CSI-2 (15-pin or 22-pin with adapter)

## Operating System

**Supported OS**: Ubuntu 20.04 or 22.04 with NVIDIA JetPack SDK

**JetPack Versions**:
- **JetPack 5.1.3** (recommended for production, L4T 35.5.0)
- **JetPack 6.x** (latest features, requires firmware update from 5.x first)

## Related Documentation

### Isaac Robot Project Docs

- [ARCHITECTURE.md](../../ARCHITECTURE.md) - System architecture including Jetson role
- [RECOVERY.md](../../RECOVERY.md) - System recovery procedures
- [SETUP.md](../../SETUP.md) - Initial system setup after flashing
- [SparkFun Auto pHAT Hardware](../sparkfun-auto-phat/) - Connected robotics control board

## External Resources

- **Developer Site**: https://developer.nvidia.com/embedded/jetson-orin-nano-developer-kit
- **User Guide**: https://developer.nvidia.com/embedded/learn/jetson-orin-nano-devkit-user-guide
- **Carrier Board Spec**: https://developer.nvidia.com/embedded/downloads (search "Jetson Orin Nano Carrier Board")
- **JetPack Documentation**: https://docs.nvidia.com/jetson/jetpack/
- **SDK Manager**: https://developer.nvidia.com/sdk-manager

## Support and Community

- **NVIDIA Developer Forums**: https://forums.developer.nvidia.com/c/agx-autonomous-machines/jetson-embedded-systems/
- **GitHub Issues**: For Isaac Robot project-specific issues

## License

NVIDIA Jetson hardware and software are subject to NVIDIA's licensing terms. This documentation is part of the Isaac Robot project and is provided for reference and educational purposes.
