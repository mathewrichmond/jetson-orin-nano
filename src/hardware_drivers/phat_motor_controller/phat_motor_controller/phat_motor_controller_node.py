#!/usr/bin/env python3
"""
PHAT Motor Controller Node
Controls motors via GPIO and reads accelerometer via I2C
Supports various PHAT motor controller boards
"""

import time
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Header, String

# Try to import I2C library (smbus2 is preferred, fallback to smbus)
try:
    from smbus2 import SMBus
    SMBUS_AVAILABLE = True
except ImportError:
    try:
        import smbus
        SMBus = smbus.SMBus
        SMBUS_AVAILABLE = True
    except ImportError:
        SMBUS_AVAILABLE = False

# Try to import GPIO library
try:
    import Jetson.GPIO as GPIO
    GPIO_AVAILABLE = True
except (ImportError, Exception):
    GPIO_AVAILABLE = False


class PHATMotorControllerNode(Node):
    """ROS 2 node for PHAT motor controller board"""

    def __init__(self):
        super().__init__('phat_motor_controller_node')

        # Parameters
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('accelerometer_address', 0x68)  # Common MPU6050 address
        self.declare_parameter('accelerometer_type', 'MPU6050')  # MPU6050, LSM6DS3, etc.
        self.declare_parameter('enable_accelerometer', True)
        self.declare_parameter('publish_rate', 50.0)

        # Motor control parameters (GPIO pins)
        self.declare_parameter('motor_left_pwm_pin', 18)
        self.declare_parameter('motor_left_dir_pin', 16)
        self.declare_parameter('motor_right_pwm_pin', 19)
        self.declare_parameter('motor_right_dir_pin', 20)
        self.declare_parameter('pwm_frequency', 1000)  # Hz

        self.i2c_bus_num = self.get_parameter('i2c_bus').value
        self.accel_address = self.get_parameter('accelerometer_address').value
        self.accel_type = self.get_parameter('accelerometer_type').value
        self.enable_accel = self.get_parameter('enable_accelerometer').value
        self.publish_rate = self.get_parameter('publish_rate').value

        self.motor_left_pwm = self.get_parameter('motor_left_pwm_pin').value
        self.motor_left_dir = self.get_parameter('motor_left_dir_pin').value
        self.motor_right_pwm = self.get_parameter('motor_right_pwm_pin').value
        self.motor_right_dir = self.get_parameter('motor_right_dir_pin').value
        self.pwm_freq = self.get_parameter('pwm_frequency').value

        # I2C and GPIO connections
        self.i2c_bus: Optional[SMBus] = None
        self.accel_initialized = False
        self.gpio_initialized = False

        # Publishers
        self.status_pub = self.create_publisher(String, '/phat/status', 10)
        self.imu_pub = self.create_publisher(Imu, '/phat/imu', 10) if self.enable_accel else None

        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Timer for status updates
        self.status_timer = self.create_timer(1.0 / self.publish_rate, self._publish_status_callback)

        # Initialize hardware
        self._initialize_hardware()

        self.get_logger().info('PHAT Motor Controller node started')
        self._publish_status_message('initialized', 'Node initialized')

    def _initialize_hardware(self):
        """Initialize I2C and GPIO hardware"""
        # Initialize I2C for accelerometer
        if self.enable_accel:
            try:
                if not SMBUS_AVAILABLE:
                    self.get_logger().error('I2C library (smbus/smbus2) not available. Install with: pip install smbus2')
                    self._publish_status_message('error', 'I2C library not available')
                    return

                self.i2c_bus = SMBus(self.i2c_bus_num)
                self.get_logger().info(f'Opened I2C bus {self.i2c_bus_num}')

                # Initialize accelerometer based on type
                if self.accel_type == 'MPU6050':
                    self._init_mpu6050()
                elif self.accel_type == 'LSM6DS3':
                    self._init_lsm6ds3()
                else:
                    self.get_logger().warn(f'Unknown accelerometer type: {self.accel_type}, trying generic init')
                    self.accel_initialized = True  # Assume it works

            except Exception as e:
                self.get_logger().error(f'Failed to initialize I2C/accelerometer: {e}')
                self._publish_status_message('error', f'I2C init failed: {str(e)[:50]}')

        # Initialize GPIO for motors
        try:
            if not GPIO_AVAILABLE:
                self.get_logger().error('GPIO library not available. Jetson.GPIO should be installed.')
                self._publish_status_message('error', 'GPIO library not available')
                return

            GPIO.setmode(GPIO.BOARD)  # Use physical pin numbering
            GPIO.setup(self.motor_left_pwm, GPIO.OUT)
            GPIO.setup(self.motor_left_dir, GPIO.OUT)
            GPIO.setup(self.motor_right_pwm, GPIO.OUT)
            GPIO.setup(self.motor_right_dir, GPIO.OUT)

            # Initialize PWM
            self.pwm_left = GPIO.PWM(self.motor_left_pwm, self.pwm_freq)
            self.pwm_right = GPIO.PWM(self.motor_right_pwm, self.pwm_freq)
            self.pwm_left.start(0)
            self.pwm_right.start(0)

            self.gpio_initialized = True
            self.get_logger().info('GPIO initialized for motor control')

        except Exception as e:
            self.get_logger().error(f'Failed to initialize GPIO: {e}')
            self._publish_status_message('error', f'GPIO init failed: {str(e)[:50]}')

    def _init_mpu6050(self):
        """Initialize MPU6050 accelerometer"""
        try:
            # Wake up MPU6050 (set sleep bit to 0)
            self.i2c_bus.write_byte_data(self.accel_address, 0x6B, 0x00)
            time.sleep(0.1)
            self.accel_initialized = True
            self.get_logger().info('MPU6050 accelerometer initialized')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize MPU6050: {e}')
            self.accel_initialized = False

    def _init_lsm6ds3(self):
        """Initialize LSM6DS3 accelerometer"""
        try:
            # Enable accelerometer (CTRL1_XL register)
            self.i2c_bus.write_byte_data(self.accel_address, 0x10, 0x60)  # 416 Hz, ±2g
            time.sleep(0.1)
            self.accel_initialized = True
            self.get_logger().info('LSM6DS3 accelerometer initialized')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize LSM6DS3: {e}')
            self.accel_initialized = False

    def _read_mpu6050(self) -> Optional[tuple]:
        """Read accelerometer data from MPU6050"""
        if not self.accel_initialized or not self.i2c_bus:
            return None

        try:
            # Read accelerometer data (ACCEL_XOUT_H through ACCEL_ZOUT_H)
            accel_data = self.i2c_bus.read_i2c_block_data(self.accel_address, 0x3B, 6)

            # Convert to signed 16-bit values
            accel_x = (accel_data[0] << 8 | accel_data[1])
            accel_y = (accel_data[2] << 8 | accel_data[3])
            accel_z = (accel_data[4] << 8 | accel_data[5])

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
            self.get_logger().warn(f'Failed to read MPU6050: {e}')
            return None

    def _read_lsm6ds3(self) -> Optional[tuple]:
        """Read accelerometer data from LSM6DS3"""
        if not self.accel_initialized or not self.i2c_bus:
            return None

        try:
            # Read accelerometer data (OUTX_L_A through OUTZ_H_A)
            accel_data = self.i2c_bus.read_i2c_block_data(self.accel_address, 0x28, 6)

            # Convert to signed 16-bit values (little endian)
            accel_x = (accel_data[1] << 8 | accel_data[0])
            accel_y = (accel_data[3] << 8 | accel_data[2])
            accel_z = (accel_data[5] << 8 | accel_data[4])

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
            self.get_logger().warn(f'Failed to read LSM6DS3: {e}')
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
            wheelbase = 0.2  # meters (adjust for your robot)
            v_left = linear_x - (angular_z * wheelbase / 2.0)
            v_right = linear_x + (angular_z * wheelbase / 2.0)

            # Convert velocity to PWM duty cycle (0-100%)
            # Assuming max speed of 1 m/s maps to 100% PWM
            max_speed = 1.0  # m/s
            pwm_left = max(-100, min(100, (v_left / max_speed) * 100))
            pwm_right = max(-100, min(100, (v_right / max_speed) * 100))

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
            self.get_logger().error(f'Error controlling motors: {e}')

    def _publish_status_callback(self):
        """Timer callback to publish status and IMU data"""
        # Publish accelerometer data if enabled
        if self.enable_accel and self.imu_pub and self.accel_initialized:
            try:
                # Read accelerometer based on type
                if self.accel_type == 'MPU6050':
                    accel_data = self._read_mpu6050()
                elif self.accel_type == 'LSM6DS3':
                    accel_data = self._read_lsm6ds3()
                else:
                    accel_data = None

                if accel_data:
                    imu_msg = Imu()
                    imu_msg.header = Header()
                    imu_msg.header.stamp = self.get_clock().now().to_msg()
                    imu_msg.header.frame_id = 'phat_imu'

                    # Set linear acceleration (m/s^2)
                    imu_msg.linear_acceleration.x = accel_data[0]
                    imu_msg.linear_acceleration.y = accel_data[1]
                    imu_msg.linear_acceleration.z = accel_data[2]

                    # Note: Gyroscope data would be read similarly if available
                    # For now, we only publish accelerometer data

                    self.imu_pub.publish(imu_msg)

            except Exception as e:
                self.get_logger().warn(f'Error reading accelerometer: {e}')

        # Publish status
        status_msg = String()
        if self.gpio_initialized and (not self.enable_accel or self.accel_initialized):
            status_msg.data = 'operational'
        elif not self.gpio_initialized:
            status_msg.data = 'error: GPIO not initialized'
        else:
            status_msg.data = 'error: Accelerometer not initialized'
        self.status_pub.publish(status_msg)

    def _publish_status_message(self, status: str, message: str = ''):
        """Publish a status message"""
        status_msg = String()
        status_msg.data = f'{status}: {message}' if message else status
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


if __name__ == '__main__':
    main()
