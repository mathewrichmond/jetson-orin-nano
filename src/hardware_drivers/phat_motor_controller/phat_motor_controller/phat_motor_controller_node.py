#!/usr/bin/env python3
"""
PHAT Motor Controller Node
Controls motors via GPIO and reads accelerometer via I2C
Supports various PHAT motor controller boards
"""

# Standard library
import time
from typing import Optional

# Third-party
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Header, String

# Try to import I2C library (smbus2 is preferred, fallback to smbus)
try:
    # Third-party
    from smbus2 import SMBus

    SMBUS_AVAILABLE = True
except ImportError:
    try:
        # Third-party
        import smbus

        SMBus = smbus.SMBus
        SMBUS_AVAILABLE = True
    except ImportError:
        SMBUS_AVAILABLE = False

# Try to import GPIO library
GPIO_AVAILABLE = False
GPIO = None
try:
    # Third-party
    import Jetson.GPIO as GPIO

    GPIO_AVAILABLE = True
except (ImportError, Exception):
    # Try alternative GPIO libraries or continue without GPIO
    try:
        # Third-party
        import RPi.GPIO as GPIO

        GPIO_AVAILABLE = True
    except (ImportError, Exception):
        GPIO_AVAILABLE = False
        GPIO = None


class PHATMotorControllerNode(Node):
    """ROS 2 node for PHAT motor controller board"""

    def __init__(self):
        super().__init__("phat_motor_controller_node")

        # Parameters
        self.declare_parameter("i2c_bus", 1)
        self.declare_parameter(
            "accelerometer_address", 0x69
        )  # Common addresses: 0x68 (MPU6050), 0x69 (ICM-20948/MPU6050 AD0 high), 0x6A (LSM6DS3)
        self.declare_parameter("accelerometer_type", "ICM20948")  # MPU6050, ICM20948, LSM6DS3, etc.
        self.declare_parameter("enable_accelerometer", True)
        self.declare_parameter("publish_rate", 50.0)

        # Motor control parameters (GPIO pins)
        self.declare_parameter("motor_left_pwm_pin", 18)
        self.declare_parameter("motor_left_dir_pin", 16)
        self.declare_parameter("motor_right_pwm_pin", 19)
        self.declare_parameter("motor_right_dir_pin", 20)
        self.declare_parameter("pwm_frequency", 1000)  # Hz
        self.declare_parameter("pwm_initial_duty_cycle", 0.0)  # Initial PWM duty cycle (0-100)

        # Robot kinematics parameters
        self.declare_parameter("wheelbase", 0.2)  # Distance between wheels in meters
        self.declare_parameter("max_speed", 1.0)  # Maximum speed in m/s

        # GPIO configuration
        self.declare_parameter("gpio_mode", "BOARD")  # GPIO numbering mode: "BOARD" or "BCM"

        # Topic names
        self.declare_parameter("status_topic", "/phat/status")
        self.declare_parameter("imu_topic", "/phat/imu")
        self.declare_parameter("magnetometer_topic", "/phat/magnetometer")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")

        # Frame IDs
        self.declare_parameter("imu_frame_id", "phat_imu")

        self.i2c_bus_num = self.get_parameter("i2c_bus").value
        self.accel_address = self.get_parameter("accelerometer_address").value
        self.accel_type = self.get_parameter("accelerometer_type").value
        self.enable_accel = self.get_parameter("enable_accelerometer").value
        self.publish_rate = self.get_parameter("publish_rate").value

        self.motor_left_pwm = self.get_parameter("motor_left_pwm_pin").value
        self.motor_left_dir = self.get_parameter("motor_left_dir_pin").value
        self.motor_right_pwm = self.get_parameter("motor_right_pwm_pin").value
        self.motor_right_dir = self.get_parameter("motor_right_dir_pin").value
        self.pwm_freq = self.get_parameter("pwm_frequency").value
        self.pwm_initial_duty = self.get_parameter("pwm_initial_duty_cycle").value

        self.wheelbase = self.get_parameter("wheelbase").value
        self.max_speed = self.get_parameter("max_speed").value

        gpio_mode_str = self.get_parameter("gpio_mode").value
        # Set GPIO mode value (will be used when GPIO is initialized)
        if GPIO_AVAILABLE and GPIO is not None:
            self.gpio_mode = GPIO.BOARD if gpio_mode_str == "BOARD" else GPIO.BCM
        else:
            self.gpio_mode = None  # Will be set when GPIO is available

        self.status_topic = self.get_parameter("status_topic").value
        self.imu_topic = self.get_parameter("imu_topic").value
        self.magnetometer_topic = self.get_parameter("magnetometer_topic").value
        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.imu_frame_id = self.get_parameter("imu_frame_id").value

        # I2C and GPIO connections
        self.i2c_bus: Optional[SMBus] = None
        self.accel_initialized = False
        self.gpio_initialized = False

        # Publishers
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.imu_pub = self.create_publisher(Imu, self.imu_topic, 10) if self.enable_accel else None
        self.mag_pub = self.create_publisher(MagneticField, self.magnetometer_topic, 10) if self.enable_accel else None

        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, self.cmd_vel_topic, self.cmd_vel_callback, 10
        )

        # Timer for status updates
        self.status_timer = self.create_timer(
            1.0 / self.publish_rate, self._publish_status_callback
        )

        # Initialize hardware
        self._initialize_hardware()

        self.get_logger().info("PHAT Motor Controller node started")
        self._publish_status_message("initialized", "Node initialized")

    def _initialize_hardware(self):
        """Initialize I2C and GPIO hardware"""
        # Initialize I2C for accelerometer
        if self.enable_accel:
            try:
                if not SMBUS_AVAILABLE:
                    self.get_logger().error(
                        "I2C library (smbus/smbus2) not available. Install with: pip install smbus2"
                    )
                    self._publish_status_message("error", "I2C library not available")
                    return

                self.i2c_bus = SMBus(self.i2c_bus_num)
                self.get_logger().info(f"Opened I2C bus {self.i2c_bus_num}")

                # Initialize accelerometer based on type
                if self.accel_type == "MPU6050":
                    self._init_mpu6050()
                elif self.accel_type == "ICM20948":
                    self._init_icm20948()
                elif self.accel_type == "LSM6DS3":
                    self._init_lsm6ds3()
                else:
                    self.get_logger().warn(
                        f"Unknown accelerometer type: {self.accel_type}, trying generic init"
                    )
                    self.accel_initialized = True  # Assume it works

            except Exception as e:
                self.get_logger().error(f"Failed to initialize I2C/accelerometer: {e}")
                self._publish_status_message("error", f"I2C init failed: {str(e)[:50]}")

        # Initialize GPIO for motors
        try:
            if not GPIO_AVAILABLE or GPIO is None:
                self.get_logger().warn(
                    "GPIO library not available. Motor control will be disabled."
                )
                self.get_logger().info("Install Jetson.GPIO or RPi.GPIO for motor control")
                self.gpio_initialized = False
                return

            # Set GPIO mode based on configuration
            gpio_mode_str = self.get_parameter("gpio_mode").value
            gpio_mode = GPIO.BOARD if gpio_mode_str == "BOARD" else GPIO.BCM
            GPIO.setmode(gpio_mode)  # Use configured GPIO numbering mode
            GPIO.setup(self.motor_left_pwm, GPIO.OUT)
            GPIO.setup(self.motor_left_dir, GPIO.OUT)
            GPIO.setup(self.motor_right_pwm, GPIO.OUT)
            GPIO.setup(self.motor_right_dir, GPIO.OUT)

            # Initialize PWM
            self.pwm_left = GPIO.PWM(self.motor_left_pwm, self.pwm_freq)
            self.pwm_right = GPIO.PWM(self.motor_right_pwm, self.pwm_freq)
            self.pwm_left.start(self.pwm_initial_duty)
            self.pwm_right.start(self.pwm_initial_duty)

            self.gpio_initialized = True
            self.get_logger().info("GPIO initialized for motor control")

        except Exception as e:
            self.get_logger().warn(f"Failed to initialize GPIO: {e}")
            self.get_logger().info("Motor control disabled, but node will continue")
            self.gpio_initialized = False

    def _init_mpu6050(self):
        """Initialize MPU6050 accelerometer"""
        try:
            # Wake up MPU6050 (set sleep bit to 0)
            self.i2c_bus.write_byte_data(self.accel_address, 0x6B, 0x00)
            time.sleep(0.1)
            self.accel_initialized = True
            self.get_logger().info("MPU6050 accelerometer initialized")
        except Exception as e:
            self.get_logger().warn(
                f"Failed to initialize MPU6050 at address 0x{self.accel_address:02X}: {e}"
            )
            self.get_logger().info("Accelerometer may not be connected or at different address")
            self.accel_initialized = False
            # Don't fail the node if accelerometer isn't available

    def _init_icm20948(self):
        """Initialize ICM-20948 IMU (SparkFun Auto pHAT) with accelerometer,
        gyroscope, and magnetometer"""
        try:
            # ICM-20948 uses a bank register system
            # CRITICAL: Must set bank register (0x7F) BEFORE accessing other registers

            # Step 1: Set bank to 0 FIRST
            self.i2c_bus.write_byte_data(
                self.accel_address, 0x7F, 0x00
            )  # REG_BANK_SEL = 0x7F, set to bank 0
            time.sleep(0.01)

            # Step 2: Read WHO_AM_I to verify device (should be 0xEA for ICM-20948)
            whoami = self.i2c_bus.read_byte_data(self.accel_address, 0x00)  # WHO_AM_I register
            if whoami != 0xEA:
                raise Exception(f"WHO_AM_I mismatch: expected 0xEA, got 0x{whoami:02X}")

            # Step 3: Reset device (Bank 0, Register 0x06 = PWR_MGMT_1)
            self.i2c_bus.write_byte_data(self.accel_address, 0x7F, 0x00)  # Ensure bank 0
            time.sleep(0.01)
            self.i2c_bus.write_byte_data(self.accel_address, 0x06, 0x80)  # Set DEVICE_RESET bit
            time.sleep(0.1)  # Wait for reset

            # Step 4: Wake up (clear sleep, use internal clock)
            self.i2c_bus.write_byte_data(self.accel_address, 0x7F, 0x00)  # Bank 0
            time.sleep(0.01)
            self.i2c_bus.write_byte_data(
                self.accel_address, 0x06, 0x01
            )  # Clear sleep, internal clock
            time.sleep(0.05)

            # Step 5: Configure accelerometer (Bank 2, Register 0x14 = ACCEL_CONFIG)
            self.i2c_bus.write_byte_data(self.accel_address, 0x7F, 0x20)  # Switch to bank 2
            time.sleep(0.01)
            self.i2c_bus.write_byte_data(self.accel_address, 0x14, 0x06)  # ±2g, 1kHz
            time.sleep(0.01)

            # Step 6: Configure gyroscope (Bank 2, Register 0x01 = GYRO_CONFIG_1)
            self.i2c_bus.write_byte_data(self.accel_address, 0x7F, 0x20)  # Ensure bank 2
            time.sleep(0.01)
            self.i2c_bus.write_byte_data(self.accel_address, 0x01, 0x06)  # ±250dps, 1kHz
            time.sleep(0.01)

            # Step 7: Enable accelerometer and gyroscope (Bank 0, Register 0x20 = ACCEL_CONFIG_2)
            self.i2c_bus.write_byte_data(self.accel_address, 0x7F, 0x00)  # Back to bank 0
            time.sleep(0.01)
            self.i2c_bus.write_byte_data(
                self.accel_address, 0x20, 0x07
            )  # ACCEL_CONFIG_2: enable, 1kHz
            time.sleep(0.01)

            # Step 8: Configure I2C master for magnetometer (AK09916 at address 0x0C)
            # Enable I2C master mode (Bank 0, Register 0x03 = USER_CTRL)
            self.i2c_bus.write_byte_data(self.accel_address, 0x7F, 0x00)  # Bank 0
            time.sleep(0.01)
            self.i2c_bus.write_byte_data(self.accel_address, 0x03, 0x20)  # Enable I2C master mode
            time.sleep(0.01)

            # Configure I2C master clock (Bank 3, Register 0x05 = I2C_MST_CTRL)
            self.i2c_bus.write_byte_data(self.accel_address, 0x7F, 0x60)  # Switch to bank 3
            time.sleep(0.01)
            self.i2c_bus.write_byte_data(
                self.accel_address, 0x05, 0x0D
            )  # I2C master clock = 400kHz
            time.sleep(0.01)

            # Configure I2C master delay (Bank 3, Register 0x67 = I2C_MST_DELAY_CTRL)
            self.i2c_bus.write_byte_data(self.accel_address, 0x7F, 0x60)  # Ensure bank 3
            time.sleep(0.01)
            self.i2c_bus.write_byte_data(
                self.accel_address, 0x67, 0x01
            )  # Enable delay for magnetometer
            time.sleep(0.01)

            # Reset magnetometer (AK09916) via I2C master
            # Bank 0, Register 0x36 = I2C_SLV0_ADDR, 0x37 = I2C_SLV0_REG,
            # 0x38 = I2C_SLV0_CTRL, 0x63 = I2C_SLV0_DO
            self.i2c_bus.write_byte_data(self.accel_address, 0x7F, 0x00)  # Back to bank 0
            time.sleep(0.01)

            # Reset AK09916: Write 0x01 to CNTL2 register (0x31)
            self.i2c_bus.write_byte_data(
                self.accel_address, 0x63, 0x01
            )  # I2C_SLV0_DO: data to write (reset bit)
            time.sleep(0.01)
            self.i2c_bus.write_byte_data(
                self.accel_address, 0x36, 0x0C | 0x80
            )  # SLV0: AK09916 address (0x0C) + write bit
            time.sleep(0.01)
            self.i2c_bus.write_byte_data(
                self.accel_address, 0x37, 0x31
            )  # SLV0_REG: AK09916 CNTL2 register
            time.sleep(0.01)
            self.i2c_bus.write_byte_data(
                self.accel_address, 0x38, 0x81
            )  # SLV0_CTRL: enable + 1 byte transfer
            time.sleep(0.1)  # Wait for reset

            # Clear reset: Write 0x00 to CNTL2 register
            self.i2c_bus.write_byte_data(self.accel_address, 0x63, 0x00)  # I2C_SLV0_DO: clear reset
            time.sleep(0.01)
            self.i2c_bus.write_byte_data(
                self.accel_address, 0x36, 0x0C | 0x80
            )  # SLV0: AK09916 address + write
            time.sleep(0.01)
            self.i2c_bus.write_byte_data(self.accel_address, 0x37, 0x31)  # SLV0_REG: CNTL2 register
            time.sleep(0.01)
            self.i2c_bus.write_byte_data(
                self.accel_address, 0x38, 0x81
            )  # SLV0_CTRL: enable + 1 byte transfer
            time.sleep(0.1)

            # Set magnetometer to continuous mode 2 (100Hz) via CNTL3 register (0x32)
            self.i2c_bus.write_byte_data(
                self.accel_address, 0x63, 0x02
            )  # I2C_SLV0_DO: continuous mode 2 (100Hz)
            time.sleep(0.01)
            self.i2c_bus.write_byte_data(
                self.accel_address, 0x36, 0x0C | 0x80
            )  # SLV0: AK09916 address + write
            time.sleep(0.01)
            self.i2c_bus.write_byte_data(self.accel_address, 0x37, 0x32)  # SLV0_REG: CNTL3 register
            time.sleep(0.01)
            self.i2c_bus.write_byte_data(
                self.accel_address, 0x38, 0x81
            )  # SLV0_CTRL: enable + 1 byte transfer
            time.sleep(0.1)

            # Configure I2C master to read magnetometer data (SLV0: read from AK09916)
            self.i2c_bus.write_byte_data(
                self.accel_address, 0x36, 0x0C | 0x80
            )  # SLV0: AK09916 address + read bit
            time.sleep(0.01)
            self.i2c_bus.write_byte_data(
                self.accel_address, 0x37, 0x10
            )  # SLV0_REG: Start reading from ST1 register (0x10)
            time.sleep(0.01)
            self.i2c_bus.write_byte_data(
                self.accel_address, 0x38, 0x87
            )  # SLV0_CTRL: enable + read 7 bytes (ST1 + 6 bytes mag data)
            time.sleep(0.01)

            self.accel_initialized = True
            self.mag_initialized = True
            self.get_logger().info(
                "ICM-20948 IMU initialized (SparkFun Auto pHAT) - "
                "Accelerometer, Gyroscope, and Magnetometer enabled"
            )
        except Exception as e:
            self.get_logger().warn(f"Failed to initialize ICM-20948: {e}")
            self.get_logger().info(
                "ICM-20948 may not be connected, enabled, or at different address"
            )
            self.get_logger().info("Check hardware jumpers, power, and I2C connections")
            self.accel_initialized = False
            self.mag_initialized = False

    def _init_lsm6ds3(self):
        """Initialize LSM6DS3 accelerometer"""
        try:
            # Enable accelerometer (CTRL1_XL register)
            self.i2c_bus.write_byte_data(self.accel_address, 0x10, 0x60)  # 416 Hz, ±2g
            time.sleep(0.1)
            self.accel_initialized = True
            self.get_logger().info("LSM6DS3 accelerometer initialized")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize LSM6DS3: {e}")
            self.accel_initialized = False

    def _read_mpu6050(self) -> Optional[tuple]:
        """Read accelerometer data from MPU6050"""
        if not self.accel_initialized or not self.i2c_bus:
            return None

        try:
            # Read accelerometer data (ACCEL_XOUT_H through ACCEL_ZOUT_H)
            accel_data = self.i2c_bus.read_i2c_block_data(self.accel_address, 0x3B, 6)

            # Convert to signed 16-bit values
            accel_x = accel_data[0] << 8 | accel_data[1]
            accel_y = accel_data[2] << 8 | accel_data[3]
            accel_z = accel_data[4] << 8 | accel_data[5]

            # Convert to signed integers
            if accel_x > 32767:
                accel_x -= 65536
            if accel_y > 32767:
                accel_y -= 65536
            if accel_z > 32767:
                accel_z -= 65536

            # Convert to m/s^2 (MPU6050 default: ±2g, 16384 LSB/g)
            accel_x_ms2 = accel_x / 16384.0 * 9.81
            accel_y_ms2 = accel_y / 16384.0 * 9.81
            accel_z_ms2 = accel_z / 16384.0 * 9.81

            return (accel_x_ms2, accel_y_ms2, accel_z_ms2)

        except Exception as e:
            self.get_logger().warn(f"Failed to read MPU6050: {e}")
            return None

    def _read_icm20948(self) -> Optional[dict]:
        """Read accelerometer, gyroscope, and magnetometer data from ICM-20948 (SparkFun Auto pHAT)

        Returns:
            Dictionary with keys: 'accel' (x, y, z in m/s^2), 'gyro' (x, y, z in rad/s),
            'mag' (x, y, z in Tesla), or None on error
        """
        if not self.accel_initialized or not self.i2c_bus:
            return None

        try:
            # Switch to bank 0 (sensor data is in bank 0)
            self.i2c_bus.write_byte_data(self.accel_address, 0x7F, 0x00)
            time.sleep(0.001)

            # Read accelerometer data (ACCEL_XOUT_H through ACCEL_ZOUT_H)
            # Register 0x2D-0x32 for accelerometer
            accel_data = self.i2c_bus.read_i2c_block_data(self.accel_address, 0x2D, 6)

            # Read gyroscope data (GYRO_XOUT_H through GYRO_ZOUT_H)
            # Register 0x33-0x38 for gyroscope
            gyro_data = self.i2c_bus.read_i2c_block_data(self.accel_address, 0x33, 6)

            # Convert accelerometer to signed 16-bit values (big endian)
            accel_x = accel_data[0] << 8 | accel_data[1]
            accel_y = accel_data[2] << 8 | accel_data[3]
            accel_z = accel_data[4] << 8 | accel_data[5]

            # Convert to signed integers
            if accel_x > 32767:
                accel_x -= 65536
            if accel_y > 32767:
                accel_y -= 65536
            if accel_z > 32767:
                accel_z -= 65536

            # Convert to m/s^2 (ICM-20948 default: ±2g, 16384 LSB/g)
            accel_x_ms2 = accel_x / 16384.0 * 9.81
            accel_y_ms2 = accel_y / 16384.0 * 9.81
            accel_z_ms2 = accel_z / 16384.0 * 9.81

            # Convert gyroscope to signed 16-bit values (big endian)
            gyro_x = gyro_data[0] << 8 | gyro_data[1]
            gyro_y = gyro_data[2] << 8 | gyro_data[3]
            gyro_z = gyro_data[4] << 8 | gyro_data[5]

            # Convert to signed integers
            if gyro_x > 32767:
                gyro_x -= 65536
            if gyro_y > 32767:
                gyro_y -= 65536
            if gyro_z > 32767:
                gyro_z -= 65536

            # Convert to rad/s (ICM-20948 default: ±250dps, 131 LSB/dps)
            # 1 dps = π/180 rad/s
            gyro_x_rads = (gyro_x / 131.0) * (3.14159265359 / 180.0)
            gyro_y_rads = (gyro_y / 131.0) * (3.14159265359 / 180.0)
            gyro_z_rads = (gyro_z / 131.0) * (3.14159265359 / 180.0)

            # Read magnetometer data via I2C master
            # Data is available in EXT_SLV_SENS_DATA_00-06 registers after I2C master read
            mag_data = None
            if self.mag_initialized:
                try:
                    # Trigger I2C master read by reading EXT_SLV_SENS_DATA_00
                    # The I2C master should have already read the data into these registers
                    ext_slv_data = self.i2c_bus.read_i2c_block_data(
                        self.accel_address, 0x3B, 7
                    )  # Read 7 bytes (ST1 + 6 bytes mag data)

                    # Check ST1 register (first byte) - bit 0 indicates data ready
                    if ext_slv_data[0] & 0x01:
                        # Convert magnetometer data (little endian for AK09916)
                        mag_x = (ext_slv_data[2] << 8) | ext_slv_data[1]
                        mag_y = (ext_slv_data[4] << 8) | ext_slv_data[3]
                        mag_z = (ext_slv_data[6] << 8) | ext_slv_data[5]

                        # Convert to signed integers
                        if mag_x > 32767:
                            mag_x -= 65536
                        if mag_y > 32767:
                            mag_y -= 65536
                        if mag_z > 32767:
                            mag_z -= 65536

                        # Convert to Tesla (AK09916 sensitivity: 0.15 µT/LSB = 0.15e-6 T/LSB)
                        mag_x_tesla = mag_x * 0.15e-6
                        mag_y_tesla = mag_y * 0.15e-6
                        mag_z_tesla = mag_z * 0.15e-6

                        mag_data = (mag_x_tesla, mag_y_tesla, mag_z_tesla)
                except Exception as e:
                    self.get_logger().debug(f"Failed to read magnetometer: {e}")

            return {
                "accel": (accel_x_ms2, accel_y_ms2, accel_z_ms2),
                "gyro": (gyro_x_rads, gyro_y_rads, gyro_z_rads),
                "mag": mag_data,
            }

        except Exception as e:
            self.get_logger().warn(f"Failed to read ICM-20948: {e}")
            return None

    def _read_lsm6ds3(self) -> Optional[tuple]:
        """Read accelerometer data from LSM6DS3"""
        if not self.accel_initialized or not self.i2c_bus:
            return None

        try:
            # Read accelerometer data (OUTX_L_A through OUTZ_H_A)
            accel_data = self.i2c_bus.read_i2c_block_data(self.accel_address, 0x28, 6)

            # Convert to signed 16-bit values (little endian)
            accel_x = accel_data[1] << 8 | accel_data[0]
            accel_y = accel_data[3] << 8 | accel_data[2]
            accel_z = accel_data[5] << 8 | accel_data[4]

            # Convert to signed integers
            if accel_x > 32767:
                accel_x -= 65536
            if accel_y > 32767:
                accel_y -= 65536
            if accel_z > 32767:
                accel_z -= 65536

            # Convert to m/s^2 (LSM6DS3 default: ±2g, 16384 LSB/g)
            accel_x_ms2 = accel_x / 16384.0 * 9.81
            accel_y_ms2 = accel_y / 16384.0 * 9.81
            accel_z_ms2 = accel_z / 16384.0 * 9.81

            return (accel_x_ms2, accel_y_ms2, accel_z_ms2)

        except Exception as e:
            self.get_logger().warn(f"Failed to read LSM6DS3: {e}")
            return None

    def cmd_vel_callback(self, msg: Twist):
        """Handle velocity commands"""
        if not self.gpio_initialized:
            return

        try:
            # Extract linear and angular velocities
            linear_x = msg.linear.x  # m/s
            angular_z = msg.angular.z  # rad/s

            # Convert to left/right wheel speeds (differential drive)
            # Simple model: v_left = linear_x - (angular_z * wheelbase/2)
            #               v_right = linear_x + (angular_z * wheelbase/2)
            v_left = linear_x - (angular_z * self.wheelbase / 2.0)
            v_right = linear_x + (angular_z * self.wheelbase / 2.0)

            # Convert velocity to PWM duty cycle (0-100%)
            pwm_left = max(-100, min(100, (v_left / self.max_speed) * 100))
            pwm_right = max(-100, min(100, (v_right / self.max_speed) * 100))

            # Set motor directions and speeds
            if pwm_left >= 0:
                GPIO.output(self.motor_left_dir, GPIO.LOW)
                self.pwm_left.ChangeDutyCycle(abs(pwm_left))
            else:
                GPIO.output(self.motor_left_dir, GPIO.HIGH)
                self.pwm_left.ChangeDutyCycle(abs(pwm_left))

            if pwm_right >= 0:
                GPIO.output(self.motor_right_dir, GPIO.LOW)
                self.pwm_right.ChangeDutyCycle(abs(pwm_right))
            else:
                GPIO.output(self.motor_right_dir, GPIO.HIGH)
                self.pwm_right.ChangeDutyCycle(abs(pwm_right))

        except Exception as e:
            self.get_logger().error(f"Error controlling motors: {e}")

    def _publish_status_callback(self):
        """Timer callback to publish status and IMU data"""
        # Publish IMU data if enabled
        if self.enable_accel and self.imu_pub and self.accel_initialized:
            try:
                # Read sensor data based on type
                if self.accel_type == "MPU6050":
                    accel_data = self._read_mpu6050()
                    sensor_data = (
                        {"accel": accel_data, "gyro": None, "mag": None} if accel_data else None
                    )
                elif self.accel_type == "ICM20948":
                    sensor_data = self._read_icm20948()
                elif self.accel_type == "LSM6DS3":
                    accel_data = self._read_lsm6ds3()
                    sensor_data = (
                        {"accel": accel_data, "gyro": None, "mag": None} if accel_data else None
                    )
                else:
                    sensor_data = None

                if sensor_data and sensor_data.get("accel"):
                    imu_msg = Imu()
                    imu_msg.header = Header()
                    imu_msg.header.stamp = self.get_clock().now().to_msg()
                    imu_msg.header.frame_id = self.imu_frame_id

                    # Set linear acceleration (m/s^2)
                    accel = sensor_data["accel"]
                    imu_msg.linear_acceleration.x = accel[0]
                    imu_msg.linear_acceleration.y = accel[1]
                    imu_msg.linear_acceleration.z = accel[2]

                    # Set angular velocity (rad/s) - gyroscope data
                    if sensor_data.get("gyro"):
                        gyro = sensor_data["gyro"]
                        imu_msg.angular_velocity.x = gyro[0]
                        imu_msg.angular_velocity.y = gyro[1]
                        imu_msg.angular_velocity.z = gyro[2]
                    else:
                        # Set to zero if not available
                        imu_msg.angular_velocity.x = 0.0
                        imu_msg.angular_velocity.y = 0.0
                        imu_msg.angular_velocity.z = 0.0

                    # Set covariance matrices (unknown for now)
                    imu_msg.linear_acceleration_covariance[0] = -1.0  # Unknown
                    imu_msg.angular_velocity_covariance[0] = -1.0  # Unknown
                    imu_msg.orientation_covariance[0] = -1.0  # Unknown

                    self.imu_pub.publish(imu_msg)

                    # Publish magnetometer data separately (if available)
                    if self.mag_pub and sensor_data.get("mag"):
                        mag_data = sensor_data["mag"]
                        mag_msg = MagneticField()
                        mag_msg.header = Header()
                        mag_msg.header.stamp = self.get_clock().now().to_msg()
                        mag_msg.header.frame_id = self.imu_frame_id

                        mag_msg.magnetic_field.x = mag_data[0]
                        mag_msg.magnetic_field.y = mag_data[1]
                        mag_msg.magnetic_field.z = mag_data[2]

                        # Set covariance (unknown for now)
                        mag_msg.magnetic_field_covariance[0] = -1.0  # Unknown

                        self.mag_pub.publish(mag_msg)

            except Exception as e:
                self.get_logger().warn(f"Error reading IMU sensors: {e}")

        # Publish status
        status_msg = String()
        status_parts = []
        if self.gpio_initialized:
            status_parts.append("motors:ok")
        else:
            status_parts.append("motors:disabled")

        if self.enable_accel:
            if self.accel_initialized:
                status_parts.append("accel:ok")
                if self.accel_type == "ICM20948":
                    status_parts.append("gyro:ok")
                    if self.mag_initialized:
                        status_parts.append("mag:ok")
                    else:
                        status_parts.append("mag:not_found")
            else:
                status_parts.append("accel:not_found")

        status_msg.data = "|".join(status_parts) if status_parts else "initializing"
        self.status_pub.publish(status_msg)

    def _publish_status_message(self, status: str, message: str = ""):
        """Publish a status message"""
        status_msg = String()
        status_msg.data = f"{status}: {message}" if message else status
        self.status_pub.publish(status_msg)

    def destroy_node(self):
        """Cleanup on shutdown"""
        if self.gpio_initialized:
            try:
                self.pwm_left.stop()
                self.pwm_right.stop()
                GPIO.cleanup()
            except Exception:
                pass

        if self.i2c_bus:
            try:
                self.i2c_bus.close()
            except Exception:
                pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PHATMotorControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
