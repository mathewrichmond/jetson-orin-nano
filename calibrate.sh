#!/bin/bash
# Calibration helper script
# Handles both fixture-based and auto-calibration routines

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

usage() {
    echo "Isaac Robot Calibration Tool"
    echo ""
    echo "Usage: $0 [command] [options]"
    echo ""
    echo "Factory Calibration (Fixture-Based, One-Time):"
    echo "  camera-intrinsics [--camera left|right] [--capture|--process]"
    echo "  stereo-extrinsics [--capture|--process|--verify]"
    echo "  camera-base-transform [--manual|--markers]"
    echo "  imu-static"
    echo ""
    echo "Field Calibration (Auto, During Operation):"
    echo "  odometry [--start|--stop]"
    echo "  camera-imu-refine"
    echo ""
    echo "Verification:"
    echo "  verify-all"
    echo "  verify-cameras"
    echo "  verify-odometry"
    echo "  verify-imu"
    echo ""
    echo "Examples:"
    echo "  $0 camera-intrinsics --camera left --capture"
    echo "  $0 odometry --start"
    echo "  $0 verify-all"
}

check_container() {
    if ! docker ps | grep -q isaac-robot-control; then
        echo -e "${YELLOW}Warning: robot-control container not running${NC}"
        echo "Start with: docker compose up -d"
        return 1
    fi
    return 0
}

exec_in_container() {
    docker exec -it isaac-robot-control "$@"
}

# ============================================================================
# Factory Calibration
# ============================================================================

cmd_camera_intrinsics() {
    local camera="${2:-left}"
    local action="${3:-capture}"
    
    check_container || return 1
    
    case "$action" in
        capture)
            echo -e "${BLUE}Capturing images for $camera camera intrinsics calibration${NC}"
            echo "Move checkerboard to various positions and orientations"
            echo "Aim for 20-30 images covering the full field of view"
            exec_in_container ros2 run camera_calibration cameracalibrator \
                --size 8x6 --square 0.025 \
                --camera /camera_$camera \
                --output /data/config/camera_${camera}_intrinsics.yaml
            ;;
        process)
            echo -e "${BLUE}Processing $camera camera calibration${NC}"
            exec_in_container python3 /workspace/scripts/calibration/process_camera_intrinsics.py \
                --camera $camera \
                --input /data/sessions/camera_intrinsics_$camera/ \
                --output /data/config/factory_calibration.yaml
            ;;
        *)
            echo "Unknown action: $action"
            echo "Use: --capture or --process"
            ;;
    esac
}

cmd_stereo_extrinsics() {
    local action="${2:-capture}"
    
    check_container || return 1
    
    case "$action" in
        capture)
            echo -e "${BLUE}Capturing images for stereo calibration${NC}"
            echo "Both cameras must see the checkerboard simultaneously"
            exec_in_container ros2 launch isaac_robot stereo_calibration_capture.launch.py
            ;;
        process)
            echo -e "${BLUE}Processing stereo calibration${NC}"
            exec_in_container python3 /workspace/scripts/calibration/process_stereo_extrinsics.py \
                --input /data/sessions/stereo_calib/ \
                --output /data/config/factory_calibration.yaml
            ;;
        verify)
            echo -e "${BLUE}Verifying stereo calibration${NC}"
            exec_in_container ros2 launch isaac_robot stereo_verify.launch.py
            ;;
        *)
            echo "Unknown action: $action"
            ;;
    esac
}

cmd_camera_base_transform() {
    local method="${2:-manual}"
    
    check_container || return 1
    
    case "$method" in
        manual)
            echo -e "${BLUE}Manual camera-to-base transform${NC}"
            echo "Measure camera position relative to robot base"
            echo ""
            read -p "X offset (meters, forward): " x
            read -p "Y offset (meters, left): " y
            read -p "Z offset (meters, up): " z
            read -p "Roll (degrees): " roll
            read -p "Pitch (degrees): " pitch
            read -p "Yaw (degrees): " yaw
            
            exec_in_container python3 /workspace/scripts/calibration/set_camera_base_transform.py \
                --x $x --y $y --z $z \
                --roll $roll --pitch $pitch --yaw $yaw \
                --output /data/config/factory_calibration.yaml
            ;;
        markers)
            echo -e "${BLUE}Marker-based camera-to-base calibration${NC}"
            echo "Ensure ArUco markers are placed at known positions"
            exec_in_container ros2 launch isaac_robot camera_base_calibration.launch.py
            ;;
        *)
            echo "Unknown method: $method"
            ;;
    esac
}

cmd_imu_static() {
    check_container || return 1
    
    echo -e "${BLUE}IMU static calibration${NC}"
    echo "Place robot in 6 orientations as prompted"
    exec_in_container python3 /workspace/scripts/calibration/imu_static_calibration.py \
        --output /data/config/factory_calibration.yaml
}

# ============================================================================
# Field Calibration
# ============================================================================

cmd_odometry() {
    local action="${2:-start}"
    
    check_container || return 1
    
    case "$action" in
        start)
            echo -e "${GREEN}Starting odometry calibration data collection${NC}"
            echo "Drive the robot with varied motion:"
            echo "  - Straight lines (3-4 meters)"
            echo "  - Pure rotation (360° in place)"
            echo "  - Circles (both directions)"
            echo "  - Figure-8 pattern"
            echo ""
            echo "Record for 2-3 minutes, then run: $0 odometry --stop"
            exec_in_container ros2 service call /calibration/start_odometry std_srvs/srv/Trigger
            ;;
        stop)
            echo -e "${GREEN}Stopping odometry calibration and processing${NC}"
            exec_in_container ros2 service call /calibration/stop_odometry std_srvs/srv/Trigger
            
            # Process calibration
            exec_in_container python3 /workspace/scripts/calibration/calibrate_odometry.py \
                --input /data/sessions/calibration_odometry_*/ \
                --output /data/config/odometry_calibration.yaml
            
            echo -e "${GREEN}Odometry calibration complete!${NC}"
            echo "Results: /data/config/odometry_calibration.yaml"
            ;;
        *)
            echo "Unknown action: $action"
            ;;
    esac
}

cmd_camera_imu_refine() {
    check_container || return 1
    
    echo -e "${BLUE}Refining camera-IMU extrinsics${NC}"
    echo "Drive robot with varied motion to excite all DOF"
    exec_in_container ros2 launch isaac_robot camera_imu_refinement.launch.py
}

# ============================================================================
# Verification
# ============================================================================

cmd_verify_all() {
    echo -e "${GREEN}Running all calibration verification checks${NC}"
    cmd_verify_cameras
    cmd_verify_odometry
    cmd_verify_imu
}

cmd_verify_cameras() {
    check_container || return 1
    echo -e "${BLUE}Verifying camera calibration${NC}"
    exec_in_container python3 /workspace/scripts/calibration/verify_camera_calibration.py
}

cmd_verify_odometry() {
    check_container || return 1
    echo -e "${BLUE}Verifying odometry calibration${NC}"
    exec_in_container python3 /workspace/scripts/calibration/verify_odometry.py
}

cmd_verify_imu() {
    check_container || return 1
    echo -e "${BLUE}Verifying IMU calibration${NC}"
    exec_in_container python3 /workspace/scripts/calibration/verify_imu.py
}

# ============================================================================
# Main
# ============================================================================

CMD="${1:-}"

if [ -z "$CMD" ]; then
    usage
    exit 0
fi

case "$CMD" in
    camera-intrinsics)
        cmd_camera_intrinsics "$@"
        ;;
    stereo-extrinsics)
        cmd_stereo_extrinsics "$@"
        ;;
    camera-base-transform)
        cmd_camera_base_transform "$@"
        ;;
    imu-static)
        cmd_imu_static "$@"
        ;;
    odometry)
        cmd_odometry "$@"
        ;;
    camera-imu-refine)
        cmd_camera_imu_refine "$@"
        ;;
    verify-all)
        cmd_verify_all
        ;;
    verify-cameras)
        cmd_verify_cameras
        ;;
    verify-odometry)
        cmd_verify_odometry
        ;;
    verify-imu)
        cmd_verify_imu
        ;;
    help|-h|--help)
        usage
        ;;
    *)
        echo "Unknown command: $CMD"
        echo ""
        usage
        exit 1
        ;;
esac
