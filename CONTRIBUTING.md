# Contributing to Isaac Robot System

## Repository Structure

Please familiarize yourself with the repository structure (see `README.md`) before making changes.

## Development Workflow

1. **Create a branch** for your changes
2. **Follow the directory structure** - place files in appropriate directories
3. **Document your changes** - update relevant documentation
4. **Test thoroughly** - especially for hardware-related changes
5. **Commit with clear messages** - describe what and why

## Code Style

- **Python**: PEP 8, type hints, docstrings
- **Bash**: Use `set -e`, check for root when needed
- **ROS 2**: Follow ROS 2 conventions
- **Documentation**: Markdown format, clear and concise

## Safety Considerations

This is a **robot system**. All changes must consider:
- Safety implications
- Error handling
- Graceful degradation
- Emergency stop functionality

## Adding New Hardware

1. Create setup script in `scripts/hardware/`
2. Add driver code in `src/hardware_drivers/`
3. Create configuration files in `config/hardware/`
4. Document in `docs/hardware/`
5. Add hardware-specific directory in `hardware/`

## Adding New Features

1. Place code in appropriate `src/` subdirectory
2. Add configuration in `config/`
3. Update documentation in `docs/`
4. Add tests if applicable
5. Update `README.md` if needed

## Documentation

- Keep documentation up to date
- Add examples for complex features
- Document assumptions and requirements
- Include troubleshooting tips

## Testing

- Test on actual hardware when possible
- Test error conditions
- Verify safety mechanisms
- Check resource usage (CPU, memory, power)

## License

By contributing to this project, you agree that your contributions will be licensed under the MIT License (see LICENSE file).

## Questions

For questions or clarifications, contact the maintainer.

