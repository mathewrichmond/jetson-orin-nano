"""
Jetson Control Main - Implements ControlBase
Runs VLA model, subscribes to missions, executes actions
"""
import os
import sys
import logging
import signal

# Add parent to path for local development
sys.path.insert(0, '/app')

try:
    from robotics_sdk import ControlBase, RobotBase
    from robotics_common import create_zenoh_session
except ImportError as e:
    logging.error(f"Failed to import robotics framework: {e}")
    logging.error("For local development, mount robotics repo:")
    logging.error("  -v ../robotics:/robotics")
    logging.error("And install: pip3 install -e /robotics")
    sys.exit(1)

# TODO: Import your VLA model
# from vla_model import load_vla, run_inference

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("control")

class JetsonControl(ControlBase):
    """Jetson-specific control implementation."""
    
    def __init__(self, robot_id, config):
        super().__init__(robot_id)
        self.config = config
        
        logger.info(f"Initializing control for {robot_id}")
        
        # TODO: Load VLA model
        # model_path = os.getenv("MODEL_PATH", "/data/models/vla_latest.pth")
        # self.vla = load_vla(model_path)
        
        # Subscribe to missions from dispatch
        self.subscribe_missions(self.on_mission)
        
        logger.info("Control initialized and waiting for missions")
    
    def on_mission(self, mission):
        """Handle incoming mission from dispatch service."""
        logger.info(f"Received mission: {mission['command']}")
        mission_id = mission["mission_id"]
        
        try:
            # Update status: starting
            self.publish_mission_status(mission_id, "in_progress", 0)
            
            # TODO: VLA inference
            # action = run_inference(self.vla, mission["command"])
            
            # Placeholder: simulate action
            action = {
                "subsystem": "pan_tilt",
                "pan": 30,
                "tilt": 10
            }
            logger.info(f"Generated action: {action}")
            
            # TODO: Send subsystem commands
            if action.get("subsystem") == "pan_tilt":
                self.send_subsystem_command("pan_tilt", {
                    "pan": action.get("pan", 0),
                    "tilt": action.get("tilt", 0)
                })
            
            # Update status: success
            self.publish_mission_result(
                mission_id,
                "success",
                {"action": action}
            )
            logger.info(f"Mission {mission_id} completed successfully")
            
        except Exception as e:
            logger.error(f"Mission {mission_id} failed: {e}", exc_info=True)
            self.publish_mission_result(
                mission_id,
                "failed",
                {"error": str(e)}
            )
    
    def run(self):
        """Keep control alive, waiting for missions."""
        logger.info("Control loop started, waiting for missions...")
        signal.pause()

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
            "capabilities": ["pan_tilt_camera", "stereo_vision"],
            "sensors": {},
            "actuators": {}
        }

def main():
    """Main entry point."""
    logger.info("=== Jetson Control Starting ===")
    
    # Load configuration
    config = load_config()
    robot_id = config.get("robot_id", os.getenv("ROBOT_ID", "jetson-01"))
    
    # Register robot with dispatch service
    logger.info(f"Registering robot {robot_id} with capabilities: {config.get('capabilities', [])}")
    robot = RobotBase(robot_id)
    robot.register_capabilities(
        capabilities=config.get("capabilities", []),
        sensors=config.get("sensors", {}),
        actuators=config.get("actuators", {})
    )
    
    # Create control instance
    control = JetsonControl(robot_id, config)
    
    # Setup signal handlers
    def signal_handler(sig, frame):
        logger.info("Received shutdown signal")
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # Run control loop (waits for missions)
    try:
        control.run()
    except Exception as e:
        logger.error(f"Fatal error: {e}", exc_info=True)
        sys.exit(1)

if __name__ == "__main__":
    main()
