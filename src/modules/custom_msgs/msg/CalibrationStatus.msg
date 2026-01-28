# CalibrationStatus.msg
# Calibration parameters for robot sensors and actuators

std_msgs/Header header

# IMU calibration
float32 imu_bias_x              # m/s^2
float32 imu_bias_y
float32 imu_bias_z
float32 gyro_bias_x             # rad/s
float32 gyro_bias_y
float32 gyro_bias_z

# Wheel odometry calibration
float32 wheel_diameter_left_m   # meters
float32 wheel_diameter_right_m
float32 wheelbase_m

# Camera-IMU extrinsics
geometry_msgs/Transform camera_front_to_base
geometry_msgs/Transform camera_rear_to_base

# Servo calibration
float32 servo_pan_offset_deg
float32 servo_tilt_offset_deg

# Calibration quality assessment
string calibration_quality      # "uncalibrated", "poor", "good", "excellent"
int32 sample_count             # Number of calibration samples collected
float64 last_updated           # Timestamp of last calibration update
