# Testing USB-C Display Setup Integration

## Verify Integration

The USB-C display setup is now integrated into the unified setup script.

### Test Setup Script

```bash
# Run setup (will prompt for USB-C display setup)
./setup.sh

# Or skip to USB-C step if other steps are done
# (USB-C setup is step 7 of 8)
```

### Manual Test

```bash
# Test USB-C setup script directly
./scripts/hardware/setup_usbc_display.sh

# Check display configuration
./scripts/hardware/configure_display.sh
```

## Expected Behavior

When running `./setup.sh`:
1. Steps 1-6 run as normal
2. Step 7: Prompts "Setup USB-C display and dock support? (y/N)"
3. If yes: Runs `setup_usbc_display.sh`
4. Step 8: Prompts for systemd services

## Skip USB-C Setup

If you don't need USB-C display support:
- Answer 'N' when prompted
- Or skip by marking step complete: `echo "setup_usbc_display" >> .setup_state`

## Re-run USB-C Setup

```bash
# Remove from state file
sed -i '/setup_usbc_display/d' .setup_state

# Re-run setup (will prompt again)
./setup.sh
```
