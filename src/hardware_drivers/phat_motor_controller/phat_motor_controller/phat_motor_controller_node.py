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
from rclpy.qos import qos_profile_system_default
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Bool, Float32, Header, String
from std_srvs.srv import SetBool

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

        # Servo control parameters (PCA9685 via I2C)
        self.declare_parameter("enable_servos", False)
        self.declare_parameter("servo_i2c_bus", 7)
        self.declare_parameter("servo_i2c_address", 0x40)
        self.declare_parameter("servo_pwm_frequency", 50)  # Hz
        self.declare_parameter("servo_pan_channel", 0)
        self.declare_parameter("servo_tilt_channel", 1)
        self.declare_parameter("servo_pan_inverted", False)
        self.declare_parameter("servo_tilt_inverted", False)
        self.declare_parameter("servo_min_angle_deg", 0.0)
        self.declare_parameter("servo_max_angle_deg", 180.0)
        self.declare_parameter("servo_min_pulse_us", 1000.0)
        self.declare_parameter("servo_max_pulse_us", 2000.0)
        self.declare_parameter("servo_startup_pan_deg", 90.0)
        self.declare_parameter("servo_startup_tilt_deg", 90.0)
        self.declare_parameter("servo_initialize_on_start", False)

        # Servo safety parameters
        self.declare_parameter("servo_max_speed_deg_per_sec", 30.0)  # Maximum angular velocity (deg/s)
        self.declare_parameter("servo_max_accel_deg_per_sec2", 60.0)  # Maximum angular acceleration (deg/s²)
        self.declare_parameter("servo_watchdog_timeout_sec", 1.0)  # Timeout before emergency stop (seconds)
        self.declare_parameter("servo_soft_limit_margin_deg", 5.0)  # Margin before hard limits (degrees)
        self.declare_parameter("servo_emergency_stop_enabled", True)  # Enable emergency stop
        self.declare_parameter("servo_safe_mode_enabled", True)  # Start in safe mode (rate limited)
        self.declare_parameter("servo_init_speed_deg_per_sec", 10.0)  # Speed for initialization (deg/s)
        self.declare_parameter("servo_init_delay_sec", 0.1)  # Delay between init steps (seconds)

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
        self.declare_parameter("servo_pan_topic", "/phat/camera_pan")
        self.declare_parameter("servo_tilt_topic", "/phat/camera_tilt")

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

        self.enable_servos = self.get_parameter("enable_servos").value
        self.servo_bus_num = self.get_parameter("servo_i2c_bus").value
        self.servo_address = self.get_parameter("servo_i2c_address").value
        self.servo_pwm_frequency = self.get_parameter("servo_pwm_frequency").value
        self.servo_pan_channel = self.get_parameter("servo_pan_channel").value
        self.servo_tilt_channel = self.get_parameter("servo_tilt_channel").value
        self.servo_pan_inverted = self.get_parameter("servo_pan_inverted").value
        self.servo_tilt_inverted = self.get_parameter("servo_tilt_inverted").value
        self.servo_min_angle = self.get_parameter("servo_min_angle_deg").value
        self.servo_max_angle = self.get_parameter("servo_max_angle_deg").value
        self.servo_min_pulse_us = self.get_parameter("servo_min_pulse_us").value
        self.servo_max_pulse_us = self.get_parameter("servo_max_pulse_us").value
        self.servo_startup_pan = self.get_parameter("servo_startup_pan_deg").value
        self.servo_startup_tilt = self.get_parameter("servo_startup_tilt_deg").value
        self.servo_initialize_on_start = self.get_parameter(
            "servo_initialize_on_start"
        ).value

        # Servo safety parameters
        self.servo_max_speed = self.get_parameter("servo_max_speed_deg_per_sec").value
        self.servo_max_accel = self.get_parameter("servo_max_accel_deg_per_sec2").value
        self.servo_watchdog_timeout = self.get_parameter("servo_watchdog_timeout_sec").value
        self.servo_soft_limit_margin = self.get_parameter("servo_soft_limit_margin_deg").value
        self.servo_emergency_stop_enabled = self.get_parameter("servo_emergency_stop_enabled").value
        self.servo_safe_mode = self.get_parameter("servo_safe_mode_enabled").value
        self.servo_init_speed = self.get_parameter("servo_init_speed_deg_per_sec").value
        self.servo_init_delay = self.get_parameter("servo_init_delay_sec").value

        # Calculate soft limits
        self.servo_pan_soft_min = self.servo_min_angle + self.servo_soft_limit_margin
        self.servo_pan_soft_max = self.servo_max_angle - self.servo_soft_limit_margin
        self.servo_tilt_soft_min = self.servo_min_angle + self.servo_soft_limit_margin
        self.servo_tilt_soft_max = self.servo_max_angle - self.servo_soft_limit_margin

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
        self.servo_pan_topic = self.get_parameter("servo_pan_topic").value
        self.servo_tilt_topic = self.get_parameter("servo_tilt_topic").value
        self.imu_frame_id = self.get_parameter("imu_frame_id").value

        # I2C and GPIO connections
        self.i2c_bus: Optional[SMBus] = None
        self.accel_initialized = False
        self.mag_initialized = False
        self.gpio_initialized = False
        self.servo_initialized = False
        self.servo_bus: Optional[SMBus] = None
        self.servo_bus_shared = False

        # Servo safety state
        self.servo_emergency_stopped = False
        self.servo_pan_current_angle: Optional[float] = None
        self.servo_tilt_current_angle: Optional[float] = None
        self.servo_pan_target_angle: Optional[float] = None
        self.servo_tilt_target_angle: Optional[float] = None
        self.servo_pan_last_command_time: Optional[float] = None
        self.servo_tilt_last_command_time: Optional[float] = None
        self.servo_pan_last_angle: Optional[float] = None
        self.servo_tilt_last_angle: Optional[float] = None
        self.servo_pan_last_update_time: Optional[float] = None
        self.servo_tilt_last_update_time: Optional[float] = None
        self.servo_initializing = False

        # Publishers
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.imu_pub = self.create_publisher(Imu, self.imu_topic, 10) if self.enable_accel else None
        self.mag_pub = (
            self.create_publisher(MagneticField, self.magnetometer_topic, 10)
            if self.enable_accel
            else None
        )

        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, self.cmd_vel_topic, self.cmd_vel_callback, 10
        )
        self.servo_pan_sub = None
        self.servo_tilt_sub = None
        self.servo_emergency_stop_sub = None
        if self.enable_servos:
            self.servo_pan_sub = self.create_subscription(
                Float32, self.servo_pan_topic, self.servo_pan_callback, 10
            )
            self.servo_tilt_sub = self.create_subscription(
                Float32, self.servo_tilt_topic, self.servo_tilt_callback, 10
            )
            # Emergency stop topic
            self.servo_emergency_stop_sub = self.create_subscription(
                Bool, "/phat/servo_emergency_stop", self.servo_emergency_stop_callback, 10
            )
            # Emergency stop service
            self.servo_emergency_stop_srv = self.create_service(
                SetBool, "/phat/servo_emergency_stop", self.servo_emergency_stop_service
            )
            # Watchdog timer for servos
            self.servo_watchdog_timer = self.create_timer(0.1, self._servo_watchdog_callback)
            # Servo control update timer (for rate limiting)
            self.servo_control_timer = self.create_timer(0.05, self._servo_control_update)  # 20 Hz

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
            else:
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

        # Initialize servo controller (PCA9685 over I2C)
        if self.enable_servos:
            if not SMBUS_AVAILABLE:
                self.get_logger().error(
                    "I2C library (smbus/smbus2) not available. Servo control disabled."
                )
                self.servo_initialized = False
                return

            try:
                if self.i2c_bus and self.servo_bus_num == self.i2c_bus_num:
                    self.servo_bus = self.i2c_bus
                    self.servo_bus_shared = True
                else:
                    self.servo_bus = SMBus(self.servo_bus_num)
                    self.servo_bus_shared = False

                self._init_pca9685()
                self.servo_initialized = True
                self.get_logger().info(
                    "Servo controller initialized (PCA9685 on I2C)"
                )

                # Initialize current positions
                self.servo_pan_current_angle = self.servo_startup_pan
                self.servo_tilt_current_angle = self.servo_startup_tilt
                self.servo_pan_target_angle = self.servo_startup_pan
                self.servo_tilt_target_angle = self.servo_startup_tilt
                self.servo_pan_last_angle = self.servo_startup_pan
                self.servo_tilt_last_angle = self.servo_startup_tilt
                current_time = time.time()
                self.servo_pan_last_update_time = current_time
                self.servo_tilt_last_update_time = current_time

                if self.servo_initialize_on_start:
                    self.get_logger().info(
                        f"Initializing servos to safe position: pan={self.servo_startup_pan}°, "
                        f"tilt={self.servo_startup_tilt}°"
                    )
                    self._safe_servo_initialize()

            except Exception as e:
                self.get_logger().warn(f"Failed to initialize servo controller: {e}")
                self.servo_initialized = False

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
            # Set SLV0 address to AK09916 (0x0C) with read bit (0x80)
            self.i2c_bus.write_byte_data(
                self.accel_address, 0x36, 0x0C | 0x80
            )  # SLV0: AK09916 address (0x0C) + read bit (0x80)
            time.sleep(0.01)
            # Set SLV0 register to start reading from ST1 register (0x10)
            self.i2c_bus.write_byte_data(
                self.accel_address, 0x37, 0x10
            )  # SLV0_REG: Start reading from ST1 register (0x10)
            time.sleep(0.01)
            # Enable SLV0 and configure to read 7 bytes (ST1 + 6 bytes mag data)
            # Bit 7 = enable, bits 3:0 = length (7 bytes)
            self.i2c_bus.write_byte_data(
                self.accel_address, 0x38, 0x87
            )  # SLV0_CTRL: enable (0x80) + read 7 bytes (0x07)
            time.sleep(0.1)  # Give I2C master time to configure

            # Verify I2C master is enabled (Bank 0, Register 0x03 = USER_CTRL)
            self.i2c_bus.write_byte_data(self.accel_address, 0x7F, 0x00)  # Ensure bank 0
            time.sleep(0.01)
            user_ctrl = self.i2c_bus.read_byte_data(self.accel_address, 0x03)
            if not (user_ctrl & 0x20):
                self.get_logger().warn(
                    "I2C master mode not enabled, re-enabling..."
                )
                # Enable I2C master mode
                self.i2c_bus.write_byte_data(self.accel_address, 0x03, 0x20)
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
                    # Ensure we're on bank 0 (EXT_SLV_SENS_DATA registers are in bank 0)
                    self.i2c_bus.write_byte_data(self.accel_address, 0x7F, 0x00)
                    time.sleep(0.001)

                    # Verify I2C master is enabled (Bank 0, Register 0x03 = USER_CTRL)
                    user_ctrl = self.i2c_bus.read_byte_data(self.accel_address, 0x03)
                    if not (user_ctrl & 0x20):
                        # I2C master not enabled, re-enable it
                        self.i2c_bus.write_byte_data(
                            self.accel_address, 0x03, 0x20
                        )  # Enable I2C master mode
                        time.sleep(0.01)
                        # Re-configure I2C master to read magnetometer
                        self.i2c_bus.write_byte_data(
                            self.accel_address, 0x36, 0x0C | 0x80
                        )  # SLV0: AK09916 address + read bit
                        time.sleep(0.01)
                        self.i2c_bus.write_byte_data(
                            self.accel_address, 0x37, 0x10
                        )  # SLV0_REG: Start reading from ST1 register
                        time.sleep(0.01)
                        self.i2c_bus.write_byte_data(
                            self.accel_address, 0x38, 0x87
                        )  # SLV0_CTRL: enable + read 7 bytes
                        time.sleep(0.01)

                    # Trigger I2C master read by reading EXT_SLV_SENS_DATA_00
                    # Reading from this register triggers the I2C master to execute
                    ext_slv_data = self.i2c_bus.read_i2c_block_data(
                        self.accel_address, 0x3B, 7
                    )  # Read 7 bytes (ST1 + 6 bytes mag data)

                    # Check ST1 register (first byte) - bit 0 indicates data ready
                    # If data ready bit is not set, try reading again after delay
                    if not (ext_slv_data[0] & 0x01):
                        # Data not ready, wait a bit and try once more
                        time.sleep(0.02)  # Increased delay
                        self.i2c_bus.write_byte_data(self.accel_address, 0x7F, 0x00)
                        time.sleep(0.001)
                        ext_slv_data = self.i2c_bus.read_i2c_block_data(
                            self.accel_address, 0x3B, 7
                        )

                    # Check ST1 register again - bit 0 indicates data ready
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

                        # Convert to Tesla (AK09916: 0.15 µT/LSB = 0.15e-6 T/LSB)
                        mag_x_tesla = mag_x * 0.15e-6
                        mag_y_tesla = mag_y * 0.15e-6
                        mag_z_tesla = mag_z * 0.15e-6

                        mag_data = (mag_x_tesla, mag_y_tesla, mag_z_tesla)
                    else:
                        # Data ready bit not set - log for troubleshooting
                        st1_val = ext_slv_data[0]
                        self.get_logger().debug(
                            f"Magnetometer data not ready (ST1=0x{st1_val:02X})"
                        )
                except Exception as e:
                    self.get_logger().warn(f"Failed to read magnetometer: {e}")

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
                    # Debug: Log if gyro data is missing
                    if sensor_data and not sensor_data.get("gyro"):
                        self.get_logger().warn(
                            "ICM20948: No gyroscope data returned"
                        )
                    elif sensor_data and sensor_data.get("gyro"):
                        gyro = sensor_data["gyro"]
                        if all(abs(g) < 0.001 for g in gyro):
                            self.get_logger().debug(
                                "ICM20948: Gyroscope values near zero "
                                "(sensor may be stationary)"
                            )
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

        if self.enable_servos:
            if self.servo_initialized:
                servo_status = "servos:ok"
                if self.servo_emergency_stopped:
                    servo_status += "|servo_estop:active"
                if self.servo_initializing:
                    servo_status += "|servo_init:in_progress"
                if self.servo_safe_mode:
                    servo_status += "|servo_safe_mode:on"
                status_parts.append(servo_status)
            else:
                status_parts.append("servos:disabled")

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

    def _init_pca9685(self):
        """Initialize PCA9685 servo controller"""
        if not self.servo_bus:
            raise RuntimeError("Servo I2C bus not initialized")

        # MODE1 and MODE2 registers
        mode1 = self.servo_bus.read_byte_data(self.servo_address, 0x00)
        mode2 = self.servo_bus.read_byte_data(self.servo_address, 0x01)

        # Set to sleep to configure prescale
        sleep_mode = (mode1 & 0x7F) | 0x10
        self.servo_bus.write_byte_data(self.servo_address, 0x00, sleep_mode)
        time.sleep(0.005)

        prescale = int(round(25_000_000 / (4096 * self.servo_pwm_frequency)) - 1)
        prescale = max(3, min(255, prescale))
        self.servo_bus.write_byte_data(self.servo_address, 0xFE, prescale)

        # Wake up and enable auto-increment
        self.servo_bus.write_byte_data(self.servo_address, 0x00, mode1)
        time.sleep(0.005)
        self.servo_bus.write_byte_data(self.servo_address, 0x00, mode1 | 0xA1)

        # Configure output driver (totem pole)
        self.servo_bus.write_byte_data(self.servo_address, 0x01, mode2 | 0x04)

    def _set_pwm(self, channel: int, on: int, off: int):
        """Set raw PWM values for a PCA9685 channel"""
        if not self.servo_bus:
            raise RuntimeError("Servo I2C bus not initialized")
        if channel < 0 or channel > 15:
            raise ValueError("Servo channel must be between 0 and 15")

        reg_base = 0x06 + 4 * channel
        self.servo_bus.write_byte_data(self.servo_address, reg_base, on & 0xFF)
        self.servo_bus.write_byte_data(self.servo_address, reg_base + 1, (on >> 8) & 0xFF)
        self.servo_bus.write_byte_data(self.servo_address, reg_base + 2, off & 0xFF)
        self.servo_bus.write_byte_data(self.servo_address, reg_base + 3, (off >> 8) & 0xFF)

    def _angle_to_pulse_us(self, angle_deg: float, inverted: bool = False) -> float:
        """Convert angle in degrees to pulse width in microseconds"""
        angle = max(self.servo_min_angle, min(self.servo_max_angle, angle_deg))
        if inverted:
            angle = self.servo_max_angle - (angle - self.servo_min_angle)

        span_angle = self.servo_max_angle - self.servo_min_angle
        if span_angle <= 0:
            raise ValueError("Servo angle range must be positive")

        span_pulse = self.servo_max_pulse_us - self.servo_min_pulse_us
        return self.servo_min_pulse_us + (angle - self.servo_min_angle) * (span_pulse / span_angle)

    def _set_servo_angle(self, channel: int, angle_deg: float, inverted: bool = False):
        """Set servo angle in degrees for a PCA9685 channel"""
        pulse_us = self._angle_to_pulse_us(angle_deg, inverted=inverted)
        period_us = 1_000_000.0 / float(self.servo_pwm_frequency)
        duty_cycle = max(0.0, min(1.0, pulse_us / period_us))
        off_count = int(duty_cycle * 4095)
        self._set_pwm(channel, 0, off_count)

    def _check_servo_limits(self, angle_deg: float, is_pan: bool):
        """Check servo limits and return (clamped_angle, is_soft_limit, is_hard_limit)
        
        Args:
            angle_deg: Desired angle in degrees
            is_pan: True for pan servo, False for tilt servo
            
        Returns:
            Tuple of (clamped_angle, soft_limit_warning, hard_limit_hit)
        """
        soft_min = self.servo_pan_soft_min if is_pan else self.servo_tilt_soft_min
        soft_max = self.servo_pan_soft_max if is_pan else self.servo_tilt_soft_max
        hard_min = self.servo_min_angle
        hard_max = self.servo_max_angle
        
        # Check hard limits
        if angle_deg < hard_min:
            clamped = hard_min
            return clamped, False, True
        if angle_deg > hard_max:
            clamped = hard_max
            return clamped, False, True
        
        # Check soft limits
        soft_limit_warning = False
        if angle_deg < soft_min or angle_deg > soft_max:
            soft_limit_warning = True
        
        return angle_deg, soft_limit_warning, False

    def _limit_servo_velocity(self, target_angle: float, current_angle: float, 
                             last_angle: Optional[float], last_time: Optional[float],
                             is_pan: bool) -> float:
        """Apply rate limiting and acceleration limiting to servo movement
        
        Returns:
            Limited target angle
        """
        if current_angle is None:
            return target_angle
        
        current_time = time.time()
        
        # Calculate current velocity if we have history
        current_velocity = 0.0
        if last_angle is not None and last_time is not None:
            dt = current_time - last_time
            if dt > 0.001:  # Avoid division by zero
                current_velocity = abs(current_angle - last_angle) / dt
        
        # Calculate desired velocity
        angle_diff = target_angle - current_angle
        desired_velocity = abs(angle_diff) / 0.05  # Assuming 20 Hz update rate
        
        # Limit velocity
        if desired_velocity > self.servo_max_speed:
            # Scale down movement to respect max speed
            max_movement = self.servo_max_speed * 0.05
            if angle_diff > 0:
                limited_target = current_angle + max_movement
            else:
                limited_target = current_angle - max_movement
        else:
            limited_target = target_angle
        
        # Limit acceleration
        if last_angle is not None and last_time is not None:
            dt = current_time - last_time
            if dt > 0.001:
                velocity_change = abs(desired_velocity - current_velocity) / dt
                if velocity_change > self.servo_max_accel:
                    # Limit acceleration by reducing movement
                    max_accel_movement = current_velocity * dt + 0.5 * self.servo_max_accel * dt * dt
                    if abs(limited_target - current_angle) > max_accel_movement:
                        if limited_target > current_angle:
                            limited_target = current_angle + max_accel_movement
                        else:
                            limited_target = current_angle - max_accel_movement
        
        return limited_target

    def _handle_servo_command(self, channel: int, angle_deg: float, inverted: bool, is_pan: bool):
        """Handle servo command with safety checks"""
        if not self.servo_initialized:
            self.get_logger().warn("Servo command received but servos not initialized")
            return
        
        if self.servo_emergency_stopped:
            self.get_logger().warn("Servo command ignored: emergency stop active")
            return
        
        if self.servo_initializing:
            self.get_logger().warn("Servo command ignored: initialization in progress")
            return
        
        try:
            # Check limits
            clamped_angle, soft_limit, hard_limit = self._check_servo_limits(angle_deg, is_pan)
            
            if hard_limit:
                self.get_logger().error(
                    f"Servo {'pan' if is_pan else 'tilt'} hard limit hit: "
                    f"requested {angle_deg:.1f}°, clamped to {clamped_angle:.1f}°"
                )
                return
            
            if soft_limit:
                self.get_logger().warn(
                    f"Servo {'pan' if is_pan else 'tilt'} approaching limit: "
                    f"{angle_deg:.1f}° (soft limit: {self.servo_pan_soft_min if is_pan else self.servo_tilt_soft_min:.1f}° - "
                    f"{self.servo_pan_soft_max if is_pan else self.servo_tilt_soft_max:.1f}°)"
                )
            
            # Update target angle (will be rate-limited in control update)
            if is_pan:
                self.servo_pan_target_angle = clamped_angle
                self.servo_pan_last_command_time = time.time()
            else:
                self.servo_tilt_target_angle = clamped_angle
                self.servo_tilt_last_command_time = time.time()
                
        except Exception as e:
            self.get_logger().error(f"Failed to handle servo command: {e}")

    def servo_pan_callback(self, msg: Float32):
        """Handle pan servo commands (degrees)"""
        self._handle_servo_command(
            self.servo_pan_channel, msg.data, inverted=self.servo_pan_inverted, is_pan=True
        )

    def servo_tilt_callback(self, msg: Float32):
        """Handle tilt servo commands (degrees)"""
        self._handle_servo_command(
            self.servo_tilt_channel, msg.data, inverted=self.servo_tilt_inverted, is_pan=False
        )

    def servo_emergency_stop_callback(self, msg: Bool):
        """Handle emergency stop topic"""
        if msg.data:
            self._servo_emergency_stop()
        else:
            self._servo_emergency_release()

    def servo_emergency_stop_service(self, request: SetBool.Request, response: SetBool.Response):
        """Handle emergency stop service"""
        if request.data:
            self._servo_emergency_stop()
            response.message = "Emergency stop activated"
        else:
            self._servo_emergency_release()
            response.message = "Emergency stop released"
        response.success = True
        return response

    def _servo_emergency_stop(self):
        """Activate emergency stop"""
        if not self.servo_emergency_stopped:
            self.servo_emergency_stopped = True
            self.get_logger().error("SERVO EMERGENCY STOP ACTIVATED")
            # Stop all servo movement immediately
            if self.servo_pan_current_angle is not None:
                self.servo_pan_target_angle = self.servo_pan_current_angle
            if self.servo_tilt_current_angle is not None:
                self.servo_tilt_target_angle = self.servo_tilt_current_angle

    def _servo_emergency_release(self):
        """Release emergency stop"""
        if self.servo_emergency_stopped:
            self.servo_emergency_stopped = False
            self.get_logger().info("Servo emergency stop released")

    def _servo_watchdog_callback(self):
        """Watchdog timer to detect missing commands"""
        if not self.enable_servos or not self.servo_initialized:
            return
        
        if self.servo_emergency_stopped or self.servo_initializing:
            return
        
        current_time = time.time()
        
        # Check pan servo
        if self.servo_pan_last_command_time is not None:
            time_since_command = current_time - self.servo_pan_last_command_time
            if time_since_command > self.servo_watchdog_timeout:
                self.get_logger().warn(
                    f"Pan servo watchdog timeout: no command for {time_since_command:.2f}s. "
                    "Holding current position."
                )
                if self.servo_pan_current_angle is not None:
                    self.servo_pan_target_angle = self.servo_pan_current_angle
        
        # Check tilt servo
        if self.servo_tilt_last_command_time is not None:
            time_since_command = current_time - self.servo_tilt_last_command_time
            if time_since_command > self.servo_watchdog_timeout:
                self.get_logger().warn(
                    f"Tilt servo watchdog timeout: no command for {time_since_command:.2f}s. "
                    "Holding current position."
                )
                if self.servo_tilt_current_angle is not None:
                    self.servo_tilt_target_angle = self.servo_tilt_current_angle

    def _servo_control_update(self):
        """Update servo positions with rate limiting (called by timer)"""
        if not self.enable_servos or not self.servo_initialized:
            return
        
        if self.servo_emergency_stopped or self.servo_initializing:
            return
        
        current_time = time.time()
        
        # Update pan servo
        if (self.servo_pan_current_angle is not None and 
            self.servo_pan_target_angle is not None):
            
            # Apply rate limiting if in safe mode
            if self.servo_safe_mode:
                limited_target = self._limit_servo_velocity(
                    self.servo_pan_target_angle,
                    self.servo_pan_current_angle,
                    self.servo_pan_last_angle,
                    self.servo_pan_last_update_time,
                    is_pan=True
                )
            else:
                limited_target = self.servo_pan_target_angle
            
            # Move towards target
            if abs(limited_target - self.servo_pan_current_angle) > 0.1:  # 0.1° threshold
                try:
                    self._set_servo_angle(
                        self.servo_pan_channel,
                        limited_target,
                        inverted=self.servo_pan_inverted
                    )
                    self.servo_pan_last_angle = self.servo_pan_current_angle
                    self.servo_pan_current_angle = limited_target
                    self.servo_pan_last_update_time = current_time
                except Exception as e:
                    self.get_logger().error(f"Failed to update pan servo: {e}")
        
        # Update tilt servo
        if (self.servo_tilt_current_angle is not None and 
            self.servo_tilt_target_angle is not None):
            
            # Apply rate limiting if in safe mode
            if self.servo_safe_mode:
                limited_target = self._limit_servo_velocity(
                    self.servo_tilt_target_angle,
                    self.servo_tilt_current_angle,
                    self.servo_tilt_last_angle,
                    self.servo_tilt_last_update_time,
                    is_pan=False
                )
            else:
                limited_target = self.servo_tilt_target_angle
            
            # Move towards target
            if abs(limited_target - self.servo_tilt_current_angle) > 0.1:  # 0.1° threshold
                try:
                    self._set_servo_angle(
                        self.servo_tilt_channel,
                        limited_target,
                        inverted=self.servo_tilt_inverted
                    )
                    self.servo_tilt_last_angle = self.servo_tilt_current_angle
                    self.servo_tilt_current_angle = limited_target
                    self.servo_tilt_last_update_time = current_time
                except Exception as e:
                    self.get_logger().error(f"Failed to update tilt servo: {e}")

    def _safe_servo_initialize(self):
        """Safely initialize servos to startup position with slow movement"""
        if not self.servo_initialized:
            return
        
        self.servo_initializing = True
        self.get_logger().info("Starting safe servo initialization...")
        
        try:
            # Move pan servo slowly
            pan_steps = int(abs(self.servo_startup_pan - (self.servo_pan_current_angle or 90.0)) / 5.0) + 1
            pan_step_size = (self.servo_startup_pan - (self.servo_pan_current_angle or 90.0)) / pan_steps
            
            for i in range(pan_steps + 1):
                target_pan = (self.servo_pan_current_angle or 90.0) + pan_step_size * i
                self._set_servo_angle(
                    self.servo_pan_channel,
                    target_pan,
                    inverted=self.servo_pan_inverted
                )
                self.servo_pan_current_angle = target_pan
                self.servo_pan_target_angle = target_pan
                time.sleep(self.servo_init_delay)
            
            # Move tilt servo slowly
            tilt_steps = int(abs(self.servo_startup_tilt - (self.servo_tilt_current_angle or 90.0)) / 5.0) + 1
            tilt_step_size = (self.servo_startup_tilt - (self.servo_tilt_current_angle or 90.0)) / tilt_steps
            
            for i in range(tilt_steps + 1):
                target_tilt = (self.servo_tilt_current_angle or 90.0) + tilt_step_size * i
                self._set_servo_angle(
                    self.servo_tilt_channel,
                    target_tilt,
                    inverted=self.servo_tilt_inverted
                )
                self.servo_tilt_current_angle = target_tilt
                self.servo_tilt_target_angle = target_tilt
                time.sleep(self.servo_init_delay)
            
            self.get_logger().info(
                f"Servo initialization complete: pan={self.servo_pan_current_angle:.1f}°, "
                f"tilt={self.servo_tilt_current_angle:.1f}°"
            )
        except Exception as e:
            self.get_logger().error(f"Servo initialization failed: {e}")
        finally:
            self.servo_initializing = False

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
        if self.servo_bus and not self.servo_bus_shared:
            try:
                self.servo_bus.close()
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
