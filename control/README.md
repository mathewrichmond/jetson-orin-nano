# Control System

This directory contains control mode definitions and mode switching logic.

## Structure

- `modes/` - Control mode definitions and implementations
- `switching/` - Mode switching logic and safety checks

## Control Modes

1. **Safe Mode**: All motors disabled, sensors active
2. **Manual Mode**: Direct user control
3. **Autonomous Mode**: VLA controller active
4. **Calibration Mode**: Hardware calibration
5. **Recovery Mode**: System recovery

## Mode Switching

Mode switching includes:
- Safety checks and validations
- Graceful transitions
- State persistence
- Emergency stop handling

## Configuration

Control mode configurations are in `config/control/`:
- Mode definitions
- Transition rules
- Safety parameters

## Status

*Control system to be implemented*

