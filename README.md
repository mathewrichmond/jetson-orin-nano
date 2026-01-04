# Isaac - Jetson Orin Nano Robot System

This repository contains the complete software stack for the Isaac robot system running on a Jetson Orin Nano. The system integrates Realsense cameras, motor controllers, and a custom Vision-Language-Action (VLA) model as the primary controller.

## 🎯 Purpose

This repository implements and manages:
- **System Setup**: Automated installation of packages, drivers, and dependencies
- **Hardware Integration**: Drivers and configuration for Realsense cameras, motor controllers, and other peripherals
- **VLA Controller**: Custom Vision-Language-Action model for robot control
- **System Monitoring**: Health checks, performance monitoring, and diagnostics
- **Logging Infrastructure**: Centralized logging with future NFS support for shared storage
- **Control Mode Switching**: Safe mode transitions and state management
- **Sub-module Controllers**: Support for Raspberry Pi and other auxiliary controllers

## 🎯 Configuration-Driven Approach

This repository uses a **configuration-driven** approach for package management:
- All packages are defined in YAML configuration files (`config/system/`)
- Installation scripts read from these configurations
- Easy to add/remove packages without editing scripts
- Single source of truth for all dependencies

See [Package Management Documentation](docs/development/PACKAGE_MANAGEMENT.md) for details.

## 📁 Repository Structure

```
jetson-orin-nano/
├── scripts/              # Setup and maintenance scripts
│   ├── system/          # System-level setup (OS, ROS, packages)
│   ├── hardware/        # Hardware-specific installation scripts
│   ├── monitoring/      # System monitoring and health check scripts
│   └── maintenance/     # Self-updating, cleaning, and recovery scripts
├── src/                 # Source code
│   ├── vla_controller/  # VLA model implementation and inference
│   ├── hardware_drivers/ # Hardware driver wrappers and interfaces
│   ├── control_modes/   # Control mode implementations
│   └── utils/           # Shared utilities and helpers
├── docs/                # Documentation
│   ├── architecture/    # System architecture and design docs
│   ├── hardware/        # Hardware setup and configuration guides
│   ├── setup/           # Setup and installation documentation
│   └── api/             # API documentation
├── config/              # Configuration files
│   ├── system/          # System-wide configurations
│   ├── hardware/        # Hardware-specific configs
│   └── control/         # Control mode configurations
├── hardware/            # Hardware setup and documentation
│   ├── realsense/       # Realsense camera setup and configs
│   ├── motor_controllers/ # Motor controller setup and configs
│   └── raspberry_pi_modules/ # Raspberry Pi sub-module configs
├── monitoring/          # Monitoring infrastructure
│   ├── system/          # System resource monitoring
│   ├── hardware/        # Hardware health monitoring
│   └── performance/     # Performance metrics and analysis
├── logging/             # Logging infrastructure
│   ├── config/          # Logging configuration files
│   └── scripts/         # Log rotation and management scripts
└── control/             # Control system
    ├── modes/           # Control mode definitions
    └── switching/       # Mode switching logic and safety checks
```

## 🚀 Quick Start

### Simple Setup Workflow

1. **Clone and setup**:
   ```bash
   git clone <repository-url>
   cd jetson-orin-nano
   ./setup.sh
   ```

   Or for non-interactive setup with auto-reboot:
   ```bash
   NON_INTERACTIVE=true sudo ./setup.sh
   ```

2. **Activate environment**:
   ```bash
   source scripts/utils/env_setup.sh
   ```

3. **Run code**:
   ```bash
   ros2 launch system_monitor system_monitor.launch.py
   ```

The setup script works identically on:
- **Native Jetson hardware**
- **Docker containers**
- **Ubuntu development machines**

See [Development Workflow](docs/development/WORKFLOW.md) for detailed workflow documentation.

### First-Time Jetson Setup

For initial Jetson system setup (after flashing):

```bash
cd ~/src/jetson-orin-nano
sudo ./scripts/system/setup_isaac.sh
sudo reboot
```

Then run the main setup:
```bash
./setup.sh
```

### Post-Setup Verification

```bash
# Verify hostname
hostname  # Should show "isaac"

# Verify ROS 2
source /opt/ros/humble/setup.bash
ros2 --help

# Check system status
./scripts/monitoring/system_health_check.sh
```

## 🔧 System Requirements

- **Hardware**: Jetson Orin Nano Developer Kit
- **OS**: Ubuntu 22.04 (JetPack 5.x)
- **ROS**: ROS 2 Humble
- **Storage**: Currently microSD (SSD support planned)
- **Network**: Dynamic IP (DHCP) with mDNS hostname resolution

## 📚 Documentation

- **[Quick Start](docs/QUICK_START.md)** - Quick reference for common tasks
- **[Setup Guide](docs/setup/SETUP.md)** - Comprehensive system setup instructions
- **[Development Workflow](docs/development/WORKFLOW.md)** - Development workflow and practices
- **[Architecture](docs/architecture/ARCHITECTURE.md)** - System architecture and design
- **[Hardware Setup](docs/hardware/HARDWARE.md)** - Hardware integration guides
- **[Deployment](docs/deployment/DEPLOYMENT.md)** - Deployment and CI/CD guides
- **[API Documentation](docs/api/API.md)** - Code API reference

## 🔌 Hardware Support

### Currently Supported
- Jetson Orin Nano base system
- ROS 2 Humble framework

### Planned Support
- Intel Realsense cameras (D435/D455)
- Motor controllers (TBD)
- Raspberry Pi sub-modules
- Additional sensors and actuators

## 🛡️ System Stability Features

This robot system includes several stability and reliability features:

- **Self-Updating**: Automated system and package updates
- **Self-Cleaning**: Log rotation, temporary file cleanup, disk space management
- **Self-Restoring**: Recovery scripts for common failure scenarios
- **Health Monitoring**: Continuous system and hardware health checks
- **Safe Mode Switching**: Graceful transitions between control modes

## 📝 Development Workflow

1. **System Setup**: Run `scripts/system/setup_isaac.sh` on first boot
2. **Hardware Setup**: Follow hardware-specific guides in `docs/hardware/`
3. **Development**: Work in `src/` directory with ROS 2 workspace
4. **Testing**: Use monitoring scripts to verify system health
5. **Deployment**: Configuration files in `config/` directory

## 🌐 Network Configuration

- **Hostname**: `isaac`
- **Network**: Dynamic IP (DHCP) via NetworkManager
- **Hostname Resolution**: mDNS (Avahi) - accessible as `isaac.local`
- **SSH**: Enabled by default

Connect from another computer:
```bash
ssh nano@isaac.local
```

## 🔐 Security Notes

- Default username: `nano`
- Change default password after first setup
- SSH keys recommended for remote access
- Firewall configuration recommended for production use

## 🚀 Deployment

The system supports multiple deployment modes:

- **Dev Sandbox**: Quick deployment of local changes (`./scripts/deployment/deploy_dev.sh`)
- **Package Installation**: Install from GitHub releases
- **Auto-Start**: Systemd services for boot-time launch
- **Dev Priority**: Dev sandbox takes precedence over installed packages

See [Deployment Guide](docs/deployment/DEPLOYMENT.md) for details.

## 📦 Future Enhancements

- [ ] NFS mount for shared logging directory
- [ ] SSD migration scripts
- [ ] Docker containerization for VLA model
- [ ] Web-based monitoring dashboard
- [ ] Automated backup and restore
- [ ] Over-the-air (OTA) update system

## 🤝 Contributing

Contributions are welcome! Please see [CONTRIBUTING.md](CONTRIBUTING.md) for guidelines. For questions or issues, contact the maintainer or open an issue.

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

The MIT License is one of the most permissive open-source licenses, allowing you to:
- Use the software commercially
- Modify the software
- Distribute the software
- Sublicense the software
- Use privately

The only requirement is that you include the original copyright notice and license text.

## 👤 Author

Mathew Richmond - mathewrichmond@gmail.com

---

**Note**: This system is designed for robot control. Always ensure safety mechanisms are in place before operating physical hardware.
