# ZSH Setup for Cursor

## Quick Setup

1. **Install ZSH** (if not already installed):
   ```bash
   sudo apt-get install zsh
   ```

2. **Restart Cursor** - Close and reopen Cursor IDE

3. **Select ZSH in Terminal**:
   - Open terminal: Press `` Ctrl+` ``
   - Click the dropdown next to the `+` button
   - Select "zsh" from the list

## Configuration

ZSH is now configured in `.vscode/settings.json` as a terminal profile option. You can switch between bash and zsh using the terminal dropdown.

## Set ZSH as Default

To make zsh the default terminal in Cursor, edit `.vscode/settings.json`:

```json
"terminal.integrated.defaultProfile.linux": "zsh",
```

## System-Wide Default Shell

To change your system-wide default shell to zsh:

```bash
chsh -s /usr/bin/zsh
# Log out and back in for changes to take effect
```

## Verify

```bash
# Check zsh is installed
zsh --version

# Check current shell
echo $SHELL

# Test zsh
zsh
```

## ZSH Configuration (Optional)

If you want to configure zsh with oh-my-zsh or other plugins, you can do so after installation. The setup script will install zsh, but won't configure it with additional plugins.
