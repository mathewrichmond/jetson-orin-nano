# Cursor Rules Update - Documentation & Script Organization

**Date**: 2026-01-28  
**Purpose**: Prevent documentation proliferation and ad-hoc file creation

---

## What Changed

Updated `.cursorrules` with strict policies to prevent root-level clutter.

---

## New/Enhanced Sections

### 1. Key Principles - Added #5

**New Principle**:
```
5. Organization: Keep repo clean - NO ad-hoc docs or scripts at root level.
                 Use docs/ subdirectories and Makefile.
```

### 2. Documentation Policies - Significantly Enhanced

**Now includes**:

#### Strict Location Rules
- **ALLOWED** at root: `README.md`, `CONTRIBUTING.md`, `setup.sh`, `Makefile` only
- **FORBIDDEN** at root:
  - Ad-hoc docs (CLEANUP_*.md, FIXES*.md, SETUP_*.md, STATUS*.md)
  - Temporary docs (SUMMARY.md, NOTES.md, TODO.md, LOG.md)
  - Ad-hoc scripts (quick_*.sh, setup_*.sh, test_*.sh, run_*.sh)
  - Duplicate quick starts (QUICK_START_*.md, *_QUICK_START.md)

#### Before Creating ANY File Checklist
1. Does this file belong in `docs/` instead of root?
2. Can I update an existing doc instead?
3. Is this a temporary file that will clutter the repo?
4. Should this be a `Makefile` command instead of a script?
5. Am I creating duplicate documentation?

#### Key Documentation Files (Authoritative List)
- `docs/README.md` - Documentation index
- `docs/QUICK_START.md` - 3-step quick start
- `docs/MODULAR_QUICK_START.md` - Detailed setup
- `docs/PRE_COMMIT_DOCKER.md` - Local testing
- `docs/DEPLOYMENT.md` - Deployment guide
- `docs/TESTING.md` - Testing guide
- `docs/IMPLEMENTATION_STATUS.md` - TODOs

### 3. Script Management - NEW SECTION

**Completely new section covering**:

#### Script Locations
- `scripts/system/` - System-level setup (sudo)
- `scripts/hardware/` - Hardware installation
- `scripts/monitoring/` - Health checks
- `scripts/maintenance/` - Self-updating, cleaning
- `scripts/testing/` - Testing utilities, Docker
- `scripts/deployment/` - Deployment automation

#### Common Commands (Use Makefile)
- `make pre-commit-install`
- `make docker-build`
- `make docker-test`
- `make docker-lint`
- `make docker-unit`

#### NEVER Create at Root
- ❌ `quick_start.sh` → Use `make` commands
- ❌ `setup_precommit.sh` → Use `make pre-commit-install`
- ❌ `run_tests.sh` → Use `make docker-test`
- ❌ `build_docker.sh` → Use `make docker-build`

#### Before Creating a Script Checklist
- [ ] Should this be a `Makefile` command instead?
- [ ] Does a similar script already exist in `scripts/`?
- [ ] Which `scripts/` subdirectory does this belong in?
- [ ] Will this script be used more than once?
- [ ] Is this duplicating existing functionality?
- [ ] Did I avoid creating it at root level?

#### Examples (GOOD vs BAD)
```bash
# GOOD (organized)
make pre-commit-install
scripts/testing/docker_test.sh
scripts/system/setup_network.sh

# BAD (ad-hoc - DON'T DO THIS)
./quick_start.sh
./setup_precommit.sh
./run_tests.sh
./build_docker.sh
```

### 4. When Writing Code - Added #7

**New Reminder**:
```
7. Keep repo clean: NO ad-hoc docs or scripts at root - see Documentation
                    Policies and Script Management
```

---

## Why These Changes?

### Problem
After completing the modular architecture refactor, we accumulated:
- 7 root-level ad-hoc docs (CLEANUP_*.md, GITHUB_ACTIONS_FIXES*.md, etc.)
- 3 duplicate QUICK_START.md files in subdirectories
- 1 ad-hoc script (QUICK_START_PRECOMMIT.sh)
- Multiple temporary summary/log files

**Total clutter**: 10+ unnecessary files

### Solution
**Explicit rules** about:
1. What files are ALLOWED at root (only 4)
2. What files are FORBIDDEN at root (with examples)
3. Checklists before creating ANY file
4. Where things should go instead
5. GOOD vs BAD examples

### Approach
**Make it explicit what NOT to do**, not just what to do:
- ❌ Forbidden patterns with examples
- ✅ Correct alternatives with examples
- 📋 Checklists to follow
- 🚫 NEVER statements

---

## Impact on AI Assistants

With these rules, AI assistants (like Cursor/Claude) will:

1. **Check before creating files**
   - Is this allowed at root?
   - Should this go in `docs/` instead?
   - Can I update an existing doc?

2. **Follow checklists**
   - Documentation checklist before creating docs
   - Script checklist before creating scripts

3. **Use proper locations**
   - `docs/` subdirectories for all docs
   - `scripts/` subdirectories for all scripts
   - `Makefile` for common commands

4. **Avoid common mistakes**
   - No more CLEANUP_*.md files
   - No more QUICK_START_*.sh scripts
   - No more temporary summary files
   - No more duplicate quick starts

---

## Key Takeaways

### For AI Assistants
- **Read `.cursorrules` carefully** - It now has explicit forbidden patterns
- **Check checklists** before creating files
- **Update existing docs** instead of creating new ones
- **Use `Makefile`** instead of ad-hoc scripts

### For Humans
- Same rules apply!
- Keep root directory clean
- Use organized structure in `docs/` and `scripts/`
- Prefer `make` commands over scripts

---

## Examples

### ❌ DON'T DO THIS
```bash
# Creating ad-hoc files at root
touch GITHUB_ACTIONS_FIXES.md
touch CLEANUP_SUMMARY.md
touch quick_start.sh
touch setup_precommit.sh
```

### ✅ DO THIS INSTEAD
```bash
# Use proper locations
vim docs/development/CI.md           # CI documentation
vim docs/CLEANUP_LOG.md              # Cleanup log (if needed)
make pre-commit-install              # Use Makefile commands
vim scripts/testing/docker_test.sh   # Scripts in proper location
```

### ❌ DON'T DO THIS
```bash
# Creating duplicate documentation
touch docs/testing/QUICK_START.md
touch docs/monitoring/QUICK_START.md
touch docs/visualization/QUICK_START.md
```

### ✅ DO THIS INSTEAD
```bash
# Use main quick start, link from subdirs
cat docs/QUICK_START.md              # Single quick start
vim docs/TESTING.md                  # Add quick start section here
vim docs/monitoring/SYSTEM_MONITOR.md # Add quick start section here
```

---

## Verification

To check if repo is clean:

```bash
# Should only show essential files
ls -1 *.md *.sh 2>/dev/null
# Expected output:
# CONTRIBUTING.md
# README.md
# setup.sh

# Count docs at root level
find docs/ -maxdepth 1 -name "*.md" | wc -l
# Should be around 10-12 essential docs
```

---

## References

- `.cursorrules` - Complete rules (commit c9dff9e)
- `docs/README.md` - Documentation index
- `Makefile` - Common commands

---

**Updated**: 2026-01-28  
**Commit**: c9dff9e  
**Status**: ✅ Active and enforced
