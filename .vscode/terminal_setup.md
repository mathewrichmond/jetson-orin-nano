# Terminal Setup for Cursor

## ZSH Configuration

ZSH is configured as the default terminal in Cursor. To use it:

1. **Open Terminal**: Press `` Ctrl+` `` or View → Terminal
2. **Select Shell**: Click the dropdown next to the `+` button
3. **Choose ZSH**: Select "zsh" from the list

## If ZSH Doesn't Appear

1. **Install ZSH** (if not installed):
   ```bash
   sudo apt-get install zsh
   ```

2. **Restart Cursor**: Close and reopen Cursor

3. **Check Settings**: The `.vscode/settings.json` file should have:
   ```json
   "terminal.integrated.defaultProfile.linux": "zsh",
   "terminal.integrated.profiles.linux": {
     "zsh": {
       "path": "/usr/bin/zsh"
     }
   }
   ```

## Switch Default Shell

To change your default shell system-wide:
```bash
chsh -s /usr/bin/zsh
# Log out and back in
```

## Verify ZSH

```bash
echo $SHELL
zsh --version
```
