# Quick Start

## Single Setup Command

```bash
./setup.sh
```

That's it! The unified setup script handles everything.

## Non-Interactive (Auto-Yes + Auto-Reboot)

```bash
NON_INTERACTIVE=true sudo ./setup.sh
```

## What Gets Set Up

- System packages
- Python packages
- ROS 2 workspace
- Virtual environment
- Bluetooth (optional)
- WiFi (optional)
- USB-C display (optional)
- Systemd services (optional)

## After Setup

The script will:
- Check if reboot is needed
- Prompt to reboot (or auto-reboot in non-interactive mode)
- Show next steps

## More Info

See `README_SETUP.md` for detailed guide.
