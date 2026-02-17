# SparkFun Auto pHAT Datasheets and Reference Materials

This directory contains official datasheets and schematics for the SparkFun Auto pHAT (ROB-16328) and its components.

## Files

### Board Documentation

- **SparkFun_Auto_pHAT_Schematic.pdf** - Official schematic from SparkFun
  - Source: https://cdn.sparkfun.com/assets/b/c/0/1/4/SparkFun_Auto_pHAT.pdf
  - Shows complete board design, component placement, and connections

### Component Datasheets

- **PCA9685_NXP_Datasheet.pdf** - 16-channel 12-bit PWM servo controller
  - Manufacturer: NXP Semiconductors
  - Used for: 4-channel servo control
  - Default I2C Address: 0x40 (configurable 0x40-0x47)
  - Source: https://www.nxp.com/docs/en/data-sheet/PCA9685.pdf

- **ICM-20948_Datasheet.pdf** - 9-DoF IMU (Gyroscope, Accelerometer, Magnetometer)
  - Manufacturer: InvenSense (TDK)
  - Used for: Motion sensing and orientation
  - Default I2C Address: 0x69 (configurable 0x68/0x69)
  - Source: https://invensense.tdk.com/

## Additional Resources

### SparkFun Resources

- **Product Page**: https://www.sparkfun.com/products/16328
- **GitHub Repository**: https://github.com/sparkfun/SparkFun_Auto_pHAT
  - Hardware files (Eagle)
  - Firmware for motor driver and encoder reader
  - Additional documentation
- **Hookup Guide**: https://learn.sparkfun.com/tutorials/sparkfun-auto-phat-hookup-guide

### Other Components

For datasheets of other components on the board:

- **PSoC 4245** (Motor Driver) - See SparkFun Qwiic Motor Driver documentation
- **ATtiny84** (Encoder Reader) - Available from Microchip/Atmel
- **DRV8835** (H-Bridge) - Available from Texas Instruments

## License

All SparkFun hardware is released under Creative Commons Attribution Share-Alike 4.0 License.
Component datasheets are copyright of their respective manufacturers.
