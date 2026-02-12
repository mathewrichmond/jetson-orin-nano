"""
Jetson Perception Main - Implements PerceptionBase
Runs RealSense cameras, generates TSDF with nvblox, publishes to Zenoh
"""
import os
import sys
import logging
import signal
import time

# Add parent to path for local development
sys.path.insert(0, '/app')

try:
    from robotics_sdk import PerceptionBase
    from robotics_common import create_zenoh_session
except ImportError as e:
    logging.error(f"Failed to import robotics framework: {e}")
    logging.error("For local development, mount robotics repo:")
    logging.error("  -v ../robotics:/robotics")
    logging.error("And install: pip3 install -e /robotics")
    sys.exit(1)

# TODO: Import your RealSense and nvblox wrappers
# from realsense_driver import RealSenseDriver
# from nvblox_wrapper import NvbloxNode

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("perception")

class JetsonPerception(PerceptionBase):
    """Jetson-specific perception implementation."""
    
    def __init__(self, robot_id, config):
        super().__init__(robot_id)
        self.config = config
        self.running = True
        
        logger.info(f"Initializing perception for {robot_id}")
        
        # TODO: Initialize RealSense cameras
        # self.cameras = RealSenseDriver(config["sensors"])
        
        # TODO: Initialize nvblox for TSDF
        # self.nvblox = NvbloxNode(config["perception"]["nvblox"])
        
        logger.info("Perception initialized")
    
    def run(self):
        """Main perception loop."""
        logger.info("Starting perception loop")
        
        while self.running:
            try:
                # TODO: Get camera frames
                # rgb_left, depth_left = self.cameras.get_frames("camera_left")
                # rgb_right, depth_right = self.cameras.get_frames("camera_right")
                
                # TODO: Publish raw sensor data
                # self.publish_image(rgb_left, depth_left, camera_id="camera_left")
                # self.publish_image(rgb_right, depth_right, camera_id="camera_right")
                
                # TODO: Update TSDF with nvblox
                # tsdf_update = self.nvblox.integrate(depth_left, depth_right)
                # self.publish_tsdf(tsdf_update)
                
                # TODO: Publish pose estimate
                # pose = self.nvblox.get_pose()
                # self.publish_pose(pose)
                
                # Placeholder: sleep to avoid busy loop
                time.sleep(0.1)
                
            except Exception as e:
                logger.error(f"Perception loop error: {e}", exc_info=True)
                time.sleep(1)
    
    def shutdown(self):
        """Clean shutdown."""
        logger.info("Shutting down perception")
        self.running = False

def load_config():
    """Load robot configuration."""
    import yaml
    config_path = os.getenv("CONFIG_PATH", "/app/config/robot_config.yaml")
    
    if os.path.exists(config_path):
        with open(config_path) as f:
            return yaml.safe_load(f)
    else:
        logger.warning(f"Config not found: {config_path}, using defaults")
        return {
            "robot_id": os.getenv("ROBOT_ID", "jetson-01"),
            "sensors": {},
            "perception": {"nvblox": {"voxel_size": 0.05}}
        }

def main():
    """Main entry point."""
    logger.info("=== Jetson Perception Starting ===")
    
    # Load configuration
    config = load_config()
    robot_id = config.get("robot_id", os.getenv("ROBOT_ID", "jetson-01"))
    
    # Create perception instance
    perception = JetsonPerception(robot_id, config)
    
    # Setup signal handlers
    def signal_handler(sig, frame):
        logger.info("Received shutdown signal")
        perception.shutdown()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # Run perception loop
    try:
        perception.run()
    except Exception as e:
        logger.error(f"Fatal error: {e}", exc_info=True)
        sys.exit(1)

if __name__ == "__main__":
    main()
