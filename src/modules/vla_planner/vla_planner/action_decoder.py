"""
Action Decoder
Decodes VILA model outputs to robot actions with safety constraints.
"""

import numpy as np
from typing import List, Dict, Any, Optional, Tuple
import yaml
from pathlib import Path


class ActionDecoder:
    """Decodes VLA model outputs to robot-executable actions."""
    
    def __init__(self, config_path: Optional[Path] = None):
        """Initialize action decoder with configuration.
        
        Args:
            config_path: Path to VILA config file (optional)
        """
        # Load configuration
        if config_path is None:
            workspace_root = Path(__file__).parent.parent.parent.parent.parent
            config_path = workspace_root / "config" / "models" / "vila_config.yaml"
        
        with open(config_path) as f:
            config = yaml.safe_load(f)
        
        self.action_config = config['actions']
        self.output_dim = self.action_config['output_dim']
        self.components = self.action_config['components']
        
        # Safety constraints
        self.clip_range = self.action_config['clip_range']
        self.velocity_scaling = self.action_config['velocity_scaling']
        self.max_acceleration = self.action_config['max_acceleration']
        self.emergency_stop_threshold = self.action_config['emergency_stop_threshold']
        
        # Action smoothing
        self.enable_smoothing = self.action_config.get('enable_smoothing', True)
        self.smoothing_alpha = self.action_config.get('smoothing_alpha', 0.3)
        self.last_action = np.zeros(self.output_dim, dtype=np.float32)
        
        # Build action limits
        self.action_mins = np.array([c['min'] for c in self.components], dtype=np.float32)
        self.action_maxs = np.array([c['max'] for c in self.components], dtype=np.float32)
        
        print(f"ActionDecoder initialized: {self.output_dim}D actions")
    
    def decode(self, model_output: Any, robot_state: Optional[Dict[str, Any]] = None,
               confidence: float = 1.0) -> np.ndarray:
        """Decode model output to robot actions.
        
        Args:
            model_output: Model output (tokens, logits, or direct actions)
            robot_state: Current robot state for context
            confidence: Model confidence (0-1)
        
        Returns:
            Action array of shape (output_dim,)
        """
        # Parse model output to action array
        actions = self._parse_model_output(model_output)
        
        # Emergency stop on low confidence
        if confidence < self.emergency_stop_threshold:
            print(f"Emergency stop: confidence {confidence:.3f} < threshold")
            return np.zeros(self.output_dim, dtype=np.float32)
        
        # Apply safety constraints
        actions = self._apply_safety_constraints(actions, robot_state)
        
        # Apply action smoothing
        if self.enable_smoothing:
            actions = self._apply_smoothing(actions)
        
        # Store for next iteration
        self.last_action = actions.copy()
        
        return actions
    
    def _parse_model_output(self, model_output: Any) -> np.ndarray:
        """Parse model output to action array.
        
        This handles different output formats:
        - Direct action array (already decoded)
        - Token sequences (need to decode)
        - Text descriptions (need to parse)
        
        Args:
            model_output: Model output in various formats
        
        Returns:
            Action array
        """
        # If already numpy array, return as-is
        if isinstance(model_output, np.ndarray):
            return model_output.astype(np.float32)
        
        # If list, convert to numpy
        if isinstance(model_output, (list, tuple)):
            return np.array(model_output, dtype=np.float32)
        
        # If tensor, convert to numpy
        if hasattr(model_output, 'cpu'):  # PyTorch tensor
            return model_output.cpu().numpy().astype(np.float32)
        
        # If token sequence, decode tokens to actions
        # This is model-specific and would need to be implemented based on
        # how VILA was trained (tokenized actions vs. continuous)
        if isinstance(model_output, (int, str)):
            # Placeholder: assume tokens map to discrete actions
            # Real implementation would use model's action tokenizer
            return np.zeros(self.output_dim, dtype=np.float32)
        
        # Default: return zero action (stop)
        print(f"Warning: Unknown model output type: {type(model_output)}")
        return np.zeros(self.output_dim, dtype=np.float32)
    
    def _apply_safety_constraints(self, actions: np.ndarray, 
                                  robot_state: Optional[Dict[str, Any]] = None) -> np.ndarray:
        """Apply safety constraints to actions.
        
        Args:
            actions: Raw action array
            robot_state: Current robot state
        
        Returns:
            Constrained action array
        """
        # Clip to general range first
        actions = np.clip(actions, self.clip_range[0], self.clip_range[1])
        
        # Clip to per-component limits
        actions = np.clip(actions, self.action_mins, self.action_maxs)
        
        # Apply velocity scaling (safety multiplier)
        actions[:6] *= self.velocity_scaling  # Scale all velocity components
        
        # Check acceleration limits (if we have previous action)
        if self.last_action is not None and len(self.last_action) == len(actions):
            delta = actions - self.last_action
            delta_norm = np.linalg.norm(delta[:3])  # Check linear acceleration only
            
            if delta_norm > self.max_acceleration:
                # Scale down to max acceleration
                scale = self.max_acceleration / delta_norm
                actions = self.last_action + delta * scale
        
        # Robot-state-based constraints (optional)
        if robot_state:
            # Example: reduce velocity near obstacles
            # This would require obstacle detection data
            pass
        
        return actions
    
    def _apply_smoothing(self, actions: np.ndarray) -> np.ndarray:
        """Apply exponential moving average smoothing.
        
        Args:
            actions: Current actions
        
        Returns:
            Smoothed actions
        """
        if self.last_action is None or len(self.last_action) != len(actions):
            return actions
        
        # EMA smoothing: a * new + (1-a) * old
        alpha = self.smoothing_alpha
        smoothed = alpha * actions + (1 - alpha) * self.last_action
        
        return smoothed
    
    def decode_text_to_action(self, text: str) -> np.ndarray:
        """Decode text description to action.
        
        This is useful if VILA outputs natural language descriptions.
        Example: "move forward 0.5 meters" → [0.5, 0, 0, ...]
        
        Args:
            text: Natural language action description
        
        Returns:
            Action array
        """
        actions = np.zeros(self.output_dim, dtype=np.float32)
        
        text_lower = text.lower()
        
        # Parse common action patterns
        if "forward" in text_lower or "ahead" in text_lower:
            actions[0] = 0.3  # linear_x
        elif "backward" in text_lower or "back" in text_lower:
            actions[0] = -0.3
        
        if "left" in text_lower:
            actions[5] = 0.5  # angular_yaw (turn left)
        elif "right" in text_lower:
            actions[5] = -0.5
        
        if "stop" in text_lower or "halt" in text_lower:
            actions[:] = 0.0
        
        if "open" in text_lower:
            actions[6] = 1.0  # gripper open
        elif "close" in text_lower or "grip" in text_lower:
            actions[6] = 0.0  # gripper close
        
        return actions
    
    def reset(self):
        """Reset decoder state (clear smoothing history)."""
        self.last_action = np.zeros(self.output_dim, dtype=np.float32)
    
    def get_action_description(self, actions: np.ndarray) -> str:
        """Get human-readable description of actions.
        
        Args:
            actions: Action array
        
        Returns:
            Description string
        """
        descriptions = []
        
        for component in self.components:
            idx = component['index']
            if idx < len(actions):
                value = actions[idx]
                if abs(value) > 0.01:  # Only describe non-zero actions
                    descriptions.append(
                        f"{component['name']}={value:.3f}{component['unit']}"
                    )
        
        if not descriptions:
            return "stop"
        
        return ", ".join(descriptions)


class ActionChunkDecoder(ActionDecoder):
    """Decoder for action chunking (multi-step predictions)."""
    
    def __init__(self, config_path: Optional[Path] = None, chunk_size: int = 16):
        """Initialize chunk decoder.
        
        Args:
            config_path: Path to VILA config
            chunk_size: Number of actions per chunk
        """
        super().__init__(config_path)
        self.chunk_size = chunk_size
        self.action_buffer = []
        self.buffer_index = 0
    
    def decode_chunk(self, model_output: Any, robot_state: Optional[Dict[str, Any]] = None,
                    confidence: float = 1.0) -> np.ndarray:
        """Decode chunk of actions and return first action.
        
        Args:
            model_output: Model output (should be chunk_size x output_dim)
            robot_state: Current robot state
            confidence: Model confidence
        
        Returns:
            First action from chunk
        """
        # Parse model output to action chunk
        if isinstance(model_output, np.ndarray) and len(model_output.shape) == 2:
            # Already a chunk (chunk_size, output_dim)
            action_chunk = model_output
        else:
            # Single action, treat as chunk of size 1
            action_chunk = np.array([self._parse_model_output(model_output)])
        
        # Apply constraints to each action in chunk
        constrained_chunk = np.array([
            self._apply_safety_constraints(action, robot_state)
            for action in action_chunk
        ])
        
        # Update buffer
        self.action_buffer = constrained_chunk.tolist()
        self.buffer_index = 0
        
        # Return first action
        return self.get_next_action()
    
    def get_next_action(self) -> np.ndarray:
        """Get next action from buffer.
        
        Returns:
            Next action, or zero action if buffer empty
        """
        if not self.action_buffer or self.buffer_index >= len(self.action_buffer):
            return np.zeros(self.output_dim, dtype=np.float32)
        
        action = np.array(self.action_buffer[self.buffer_index], dtype=np.float32)
        self.buffer_index += 1
        
        # Apply smoothing
        if self.enable_smoothing:
            action = self._apply_smoothing(action)
        
        self.last_action = action.copy()
        return action
    
    def has_actions_remaining(self) -> bool:
        """Check if buffer has more actions."""
        return self.buffer_index < len(self.action_buffer)
