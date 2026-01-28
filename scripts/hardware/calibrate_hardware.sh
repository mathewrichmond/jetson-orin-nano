#!/bin/bash
# Hardware Calibration Script
# 
# This script guides through the hardware calibration process for the Isaac robot.
# It automates calibration procedures and updates the calibration.yaml file.
#
# Usage: ./scripts/hardware/calibrate_hardware.sh [--component COMPONENT]
#
# Components:
#   all       - Run all calibration procedures (default)
#   camera    - Camera intrinsics and extrinsics
#   imu       - IMU biases and alignment
#   servo     - Servo PWM-to-angle mapping
#   odometry  - Wheel odometry correction factors

set -e  # Exit on error

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
CALIBRATION_FILE="/home/nano/.config/robot/calibration.yaml"
CALIBRATION_TEMPLATE="$REPO_ROOT/config/robot/calibration.yaml.example"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Print colored message
print_msg() {
    local color=$1
    shift
    echo -e "${color}$@${NC}"
}

print_header() {
    print_msg "$BLUE" "\n========================================="
    print_msg "$BLUE" "$1"
    print_msg "$BLUE" "=========================================\n"
}

print_success() {
    print_msg "$GREEN" "✓ $1"
}

print_warning() {
    print_msg "$YELLOW" "⚠ $1"
}

print_error() {
    print_msg "$RED" "✗ $1"
}

# Check if calibration file exists, create from template if not
check_calibration_file() {
    print_header "Checking Calibration File"
    
    if [ ! -f "$CALIBRATION_FILE" ]; then
        print_warning "Calibration file not found at $CALIBRATION_FILE"
        print_msg "$NC" "Creating from template..."
        
        mkdir -p "$(dirname "$CALIBRATION_FILE")"
        cp "$CALIBRATION_TEMPLATE" "$CALIBRATION_FILE"
        
        print_success "Created calibration file from template"
    else
        print_success "Calibration file exists: $CALIBRATION_FILE"
    fi
}

# Backup calibration file
backup_calibration() {
    print_msg "$NC" "Backing up current calibration..."
    local backup_file="${CALIBRATION_FILE}.backup.$(date +%Y%m%d_%H%M%S)"
    cp "$CALIBRATION_FILE" "$backup_file"
    print_success "Backup saved: $backup_file"
}

# Camera calibration
calibrate_camera() {
    print_header "Camera Calibration"
    
    print_msg "$NC" "Camera calibration requires:"
    print_msg "$NC" "  1. Checkerboard or AprilTag calibration target"
    print_msg "$NC" "  2. RealSense cameras running"
    print_msg "$NC" "  3. ros2 camera_calibration package installed"
    print_msg "$NC" ""
    print_msg "$NC" "This is a manual process. Please follow the steps in:"
    print_msg "$NC" "  docs/hardware/MECHANICAL.md § 4.3.1"
    print_msg "$NC" ""
    
    read -p "Have you completed camera calibration? (y/n): " completed
    
    if [ "$completed" = "y" ] || [ "$completed" = "Y" ]; then
        print_success "Camera calibration marked as complete"
        print_msg "$NC" "Make sure to update calibration.yaml with the results"
    else
        print_warning "Camera calibration skipped"
    fi
}

# IMU calibration
calibrate_imu() {
    print_header "IMU Calibration"
    
    print_msg "$NC" "IMU calibration procedure:"
    print_msg "$NC" "  1. Place robot on level surface"
    print_msg "$NC" "  2. Launch calibration manager node"
    print_msg "$NC" "  3. Wait ~30 seconds for bias estimation"
    print_msg "$NC" "  4. Slowly rotate robot 360° for magnetometer calibration"
    print_msg "$NC" ""
    
    read -p "Start IMU calibration? (y/n): " start_cal
    
    if [ "$start_cal" != "y" ] && [ "$start_cal" != "Y" ]; then
        print_warning "IMU calibration skipped"
        return
    fi
    
    print_msg "$NC" "Checking if calibration manager is available..."
    
    if ! ros2 pkg list | grep -q "chassis_control"; then
        print_error "chassis_control package not found"
        print_msg "$NC" "Please build the workspace first"
        return 1
    fi
    
    print_msg "$NC" ""
    print_msg "$NC" "Launching calibration manager..."
    print_msg "$NC" "(Press Ctrl+C when calibration is complete)"
    print_msg "$NC" ""
    
    # Launch calibration manager (this will run until Ctrl+C)
    ros2 launch chassis_control calibration_manager.launch.py || true
    
    print_success "IMU calibration complete"
    print_msg "$NC" "Calibration values saved to: $CALIBRATION_FILE"
}

# Servo calibration
calibrate_servo() {
    print_header "Servo Calibration"
    
    print_msg "$NC" "⚠ SAFETY WARNING: Servo calibration involves motor movement"
    print_msg "$NC" "  - Ensure servos have clear range of motion"
    print_msg "$NC" "  - Watch for binding or obstruction"
    print_msg "$NC" "  - Keep hands clear of moving parts"
    print_msg "$NC" "  - Have emergency stop ready"
    print_msg "$NC" ""
    
    read -p "Proceed with servo calibration? (y/n): " proceed
    
    if [ "$proceed" != "y" ] && [ "$proceed" != "Y" ]; then
        print_warning "Servo calibration skipped"
        return
    fi
    
    print_msg "$NC" "Servo calibration is a manual process involving:"
    print_msg "$NC" "  1. Testing conservative PWM range (1000-2000μs)"
    print_msg "$NC" "  2. Verifying mechanical alignment at center"
    print_msg "$NC" "  3. Testing min/max positions for binding"
    print_msg "$NC" "  4. Recording actual PWM values for desired angles"
    print_msg "$NC" ""
    print_msg "$NC" "Detailed procedure in: docs/hardware/SERVO_SAFETY.md"
    print_msg "$NC" ""
    
    read -p "Have you completed servo calibration? (y/n): " completed
    
    if [ "$completed" = "y" ] || [ "$completed" = "Y" ]; then
        print_success "Servo calibration marked as complete"
        print_msg "$NC" "Update config/hardware/phat_params.yaml with PWM values"
        print_msg "$NC" "Update calibration.yaml with angle limits"
    else
        print_warning "Servo calibration skipped"
    fi
}

# Odometry calibration
calibrate_odometry() {
    print_header "Odometry Calibration"
    
    print_msg "$NC" "Odometry calibration requires:"
    print_msg "$NC" "  1. Clear floor space (≥2m straight, ≥2m diameter circle)"
    print_msg "$NC" "  2. Measuring tape or marked distance"
    print_msg "$NC" "  3. iRobot Create running"
    print_msg "$NC" ""
    
    read -p "Start odometry calibration? (y/n): " start_cal
    
    if [ "$start_cal" != "y" ] && [ "$start_cal" != "Y" ]; then
        print_warning "Odometry calibration skipped"
        return
    fi
    
    print_msg "$NC" ""
    print_msg "$YELLOW" "=== Linear Calibration ==="
    print_msg "$NC" ""
    print_msg "$NC" "1. Mark robot starting position"
    print_msg "$NC" "2. Mark a line 1 meter ahead"
    print_msg "$NC" "3. Ready to drive forward 1 meter"
    print_msg "$NC" ""
    
    read -p "Press Enter when ready to drive..."
    
    print_msg "$NC" "Publishing velocity command..."
    print_msg "$NC" "(Robot should drive forward for 5 seconds)"
    
    # Drive forward at 0.2 m/s for 5 seconds = 1 meter
    timeout 5 ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
        "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" \
        > /dev/null 2>&1 || true
    
    # Stop robot
    ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
        "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" \
        > /dev/null 2>&1
    
    print_msg "$NC" ""
    print_msg "$NC" "Measure the actual distance traveled from start mark to robot"
    read -p "Enter measured distance (meters): " actual_distance
    
    commanded_distance=1.0
    linear_scale=$(echo "scale=4; $actual_distance / $commanded_distance" | bc)
    
    print_msg "$NC" ""
    print_msg "$GREEN" "Linear scale factor: $linear_scale"
    print_msg "$NC" "(Commanded 1.0m, actual ${actual_distance}m)"
    print_msg "$NC" ""
    
    print_msg "$YELLOW" "=== Angular Calibration ==="
    print_msg "$NC" ""
    print_msg "$NC" "1. Mark robot starting orientation"
    print_msg "$NC" "2. Ready to rotate 360°"
    print_msg "$NC" ""
    
    read -p "Press Enter when ready to rotate..."
    
    print_msg "$NC" "Publishing rotation command..."
    print_msg "$NC" "(Robot should rotate for ~10 seconds)"
    
    # Rotate at 0.628 rad/s for 10 seconds = ~360° (2π radians)
    timeout 10 ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
        "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.628}}" \
        > /dev/null 2>&1 || true
    
    # Stop robot
    ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
        "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" \
        > /dev/null 2>&1
    
    print_msg "$NC" ""
    print_msg "$NC" "Measure the actual rotation from start orientation"
    read -p "Enter measured rotation (degrees, 0-360): " actual_rotation
    
    commanded_rotation=360
    angular_scale=$(echo "scale=4; $actual_rotation / $commanded_rotation" | bc)
    
    print_msg "$NC" ""
    print_msg "$GREEN" "Angular scale factor: $angular_scale"
    print_msg "$NC" "(Commanded 360°, actual ${actual_rotation}°)"
    print_msg "$NC" ""
    
    print_success "Odometry calibration complete"
    print_msg "$NC" ""
    print_msg "$NC" "Update calibration.yaml with these values:"
    print_msg "$NC" "  robot_base_calibration:"
    print_msg "$NC" "    linear_scale_factor: $linear_scale"
    print_msg "$NC" "    angular_scale_factor: $angular_scale"
}

# Main calibration routine
main() {
    local component="${1:-all}"
    
    print_header "Isaac Robot Hardware Calibration"
    
    print_msg "$NC" "This script guides through hardware calibration procedures."
    print_msg "$NC" "Calibration values will be saved to: $CALIBRATION_FILE"
    print_msg "$NC" ""
    print_msg "$NC" "Documentation: docs/hardware/MECHANICAL.md § 4"
    print_msg "$NC" ""
    
    # Check calibration file exists
    check_calibration_file
    
    # Backup current calibration
    if [ -f "$CALIBRATION_FILE" ]; then
        backup_calibration
    fi
    
    # Run calibration procedures based on component
    case "$component" in
        all)
            print_msg "$NC" "Running all calibration procedures..."
            calibrate_camera
            calibrate_imu
            calibrate_servo
            calibrate_odometry
            ;;
        camera)
            calibrate_camera
            ;;
        imu)
            calibrate_imu
            ;;
        servo)
            calibrate_servo
            ;;
        odometry)
            calibrate_odometry
            ;;
        *)
            print_error "Unknown component: $component"
            print_msg "$NC" "Usage: $0 [all|camera|imu|servo|odometry]"
            exit 1
            ;;
    esac
    
    print_header "Calibration Complete"
    
    print_msg "$NC" "Next steps:"
    print_msg "$NC" "  1. Review calibration values: cat $CALIBRATION_FILE"
    print_msg "$NC" "  2. Verify values are reasonable"
    print_msg "$NC" "  3. Update URDF model with calibrated values"
    print_msg "$NC" "  4. Test robot with calibrated parameters"
    print_msg "$NC" ""
    print_success "All calibration procedures complete!"
}

# Run main with all arguments
main "$@"
