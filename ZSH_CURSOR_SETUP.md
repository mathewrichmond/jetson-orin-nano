# ZSH Setup for Cursor

## ✅ Configuration Complete

ZSH has been added to Cursor's terminal profiles. Here's how to use it:

## Steps to Use ZSH in Cursor

1. **Install ZSH** (if not already installed):
   ```bash
   sudo apt-get install zsh
   ```
   
   Or it will be installed automatically when you run:
   ```bash
   sudo ./setup.sh
   ```

2. **Restart Cursor**:
   - Close Cursor completely
   - Reopen Cursor
   - Or use: View → Command Palette → "Reload Window"

3. **Select ZSH in Terminal**:
   - Open terminal: Press `` Ctrl+` `` or View → Terminal
   - Click the dropdown arrow next to the `+` button in the terminal panel
   - Select **"zsh"** from the list

## Configuration Details

ZSH is configured in `.vscode/settings.json`:
```json
"terminal.integrated.profiles.linux": {
  "bash": {
    "path": "/bin/bash",
    "icon": "terminal-bash"
  },
  "zsh": {
    "path": "/usr/bin/zsh",
    "icon": "terminal"
  }
}
```

## Make ZSH Default

To make zsh the default terminal in Cursor, edit `.vscode/settings.json`:
```json
"terminal.integrated.defaultProfile.linux": "zsh",
```

## Verify

After installing zsh and restarting Cursor:
- Open terminal dropdown
- You should see both "bash" and "zsh" options
- Select "zsh" to use it

## Troubleshooting

If zsh doesn't appear:
1. Make sure zsh is installed: `which zsh`
2. Restart Cursor completely
3. Check `.vscode/settings.json` has the zsh profile
4. Try reloading the window: Ctrl+Shift+P → "Reload Window"
