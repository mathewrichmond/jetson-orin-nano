"""
VLA Data Logger
Logs model inputs, outputs, and metadata for training data collection.
"""

import os
import json
import time
from pathlib import Path
from typing import Dict, Any, Optional, List
from datetime import datetime
import threading
from collections import deque
import numpy as np


class DataLogger:
    """Logs VLA inference data to session directory."""
    
    def __init__(self, session_id: Optional[str] = None, buffer_size: int = 100, 
                 max_file_size_mb: int = 100):
        """Initialize data logger.
        
        Args:
            session_id: Session ID (reads from SESSION_ID env var if None)
            buffer_size: Number of entries to buffer before writing
            max_file_size_mb: Maximum size per log file before rotating
        """
        # Get session ID
        self.session_id = session_id or os.getenv('SESSION_ID')
        if not self.session_id:
            raise ValueError("SESSION_ID not set. Cannot initialize data logger.")
        
        # Find session directory
        self.session_dir = self._find_session_dir()
        if not self.session_dir:
            raise ValueError(f"Session {self.session_id} not found")
        
        # Setup log file path
        self.log_dir = self.session_dir / "raw"
        self.log_dir.mkdir(parents=True, exist_ok=True)
        
        self.log_file_prefix = f"vla_inference_{self.session_id}"
        self.log_file_index = 0
        self.current_log_file = self._get_log_file_path()
        
        # Buffer for async I/O
        self.buffer_size = buffer_size
        self.buffer: deque = deque(maxlen=buffer_size)
        self.max_file_size = max_file_size_mb * 1024 * 1024  # Convert to bytes
        
        # Threading for async writes
        self.lock = threading.Lock()
        self.write_thread = None
        self.stop_flag = threading.Event()
        self.enabled = True
        
        # Stats
        self.frame_count = 0
        self.total_size = 0
        
        print(f"DataLogger initialized for session {self.session_id}")
        print(f"  Log directory: {self.log_dir}")
        print(f"  Log file: {self.current_log_file.name}")
    
    def _find_session_dir(self) -> Optional[Path]:
        """Find session directory by ID."""
        # Check common locations
        search_paths = [
            Path("/mnt/nfs/robot_data"),
            Path("/home/nano/data")
        ]
        
        for root in search_paths:
            if not root.exists():
                continue
            
            for session_dir in root.glob(f"session_*_{self.session_id}"):
                if session_dir.is_dir():
                    return session_dir
        
        return None
    
    def _get_log_file_path(self) -> Path:
        """Get path for current log file."""
        filename = f"{self.log_file_prefix}_{self.log_file_index:03d}.jsonl"
        return self.log_dir / filename
    
    def _rotate_log_file(self):
        """Rotate to a new log file."""
        # Flush current buffer
        self._flush_buffer()
        
        # Increment index and create new file path
        self.log_file_index += 1
        self.current_log_file = self._get_log_file_path()
        
        print(f"Rotated to new log file: {self.current_log_file.name}")
    
    def _check_file_rotation(self):
        """Check if log file needs rotation based on size."""
        if self.current_log_file.exists():
            file_size = self.current_log_file.stat().st_size
            if file_size >= self.max_file_size:
                self._rotate_log_file()
    
    def log_inference(self, 
                     timestamp: float,
                     frame_id: int,
                     camera_frames: Optional[Dict[str, Any]] = None,
                     audio_features: Optional[np.ndarray] = None,
                     robot_state: Optional[Dict[str, Any]] = None,
                     transcription: Optional[str] = None,
                     vla_output: Optional[Dict[str, Any]] = None,
                     executed_action: Optional[List[float]] = None,
                     metadata: Optional[Dict[str, Any]] = None):
        """Log a single VLA inference.
        
        Args:
            timestamp: Inference timestamp
            frame_id: Frame ID
            camera_frames: Dict with camera frame references (paths or bag references)
            audio_features: Audio features array
            robot_state: Robot state dict (position, velocity, etc.)
            transcription: Voice command transcription
            vla_output: VLA model output (tokens, confidence, decoded action)
            executed_action: Actually executed action
            metadata: Additional metadata (model version, inference time, etc.)
        """
        if not self.enabled:
            return
        
        # Build log entry
        entry = {
            'timestamp': timestamp,
            'session_id': self.session_id,
            'frame_id': frame_id,
            'inputs': {},
            'vla_output': vla_output or {},
            'executed_action': executed_action,
            'metadata': metadata or {}
        }
        
        # Add inputs (store references to avoid large JSON)
        if camera_frames:
            entry['inputs']['camera_frames'] = camera_frames
        
        if audio_features is not None:
            # Store summary, not full array
            entry['inputs']['audio_features'] = {
                'shape': list(audio_features.shape),
                'dtype': str(audio_features.dtype),
                'mean': float(np.mean(audio_features)),
                'std': float(np.std(audio_features))
            }
        
        if robot_state:
            entry['inputs']['robot_state'] = robot_state
        
        if transcription:
            entry['inputs']['transcription'] = transcription
        
        # Add to buffer
        with self.lock:
            self.buffer.append(entry)
            self.frame_count += 1
            
            # Flush if buffer is full
            if len(self.buffer) >= self.buffer_size:
                self._flush_buffer()
    
    def _flush_buffer(self):
        """Write buffer to disk (called with lock held)."""
        if not self.buffer:
            return
        
        # Check if rotation needed
        self._check_file_rotation()
        
        # Write buffered entries
        with open(self.current_log_file, 'a') as f:
            while self.buffer:
                entry = self.buffer.popleft()
                line = json.dumps(entry) + '\n'
                f.write(line)
                self.total_size += len(line)
    
    def flush(self):
        """Manually flush buffer to disk."""
        with self.lock:
            self._flush_buffer()
    
    def close(self):
        """Close logger and flush remaining data."""
        self.enabled = False
        
        # Final flush
        self.flush()
        
        print(f"DataLogger closed")
        print(f"  Total frames logged: {self.frame_count}")
        print(f"  Total size: {self.total_size / 1024 / 1024:.2f} MB")
    
    def get_stats(self) -> Dict[str, Any]:
        """Get logging statistics."""
        return {
            'session_id': self.session_id,
            'frame_count': self.frame_count,
            'total_size_bytes': self.total_size,
            'buffer_size': len(self.buffer),
            'current_log_file': str(self.current_log_file)
        }
    
    def __enter__(self):
        """Context manager entry."""
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.close()
        return False


class MockDataLogger:
    """Mock data logger for testing without active session."""
    
    def __init__(self, *args, **kwargs):
        print("MockDataLogger: No active session, logging disabled")
    
    def log_inference(self, *args, **kwargs):
        pass
    
    def flush(self):
        pass
    
    def close(self):
        pass
    
    def get_stats(self):
        return {'enabled': False}
    
    def __enter__(self):
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        return False


def get_data_logger(session_id: Optional[str] = None, 
                   enable_logging: bool = True) -> DataLogger:
    """Get data logger instance.
    
    Args:
        session_id: Session ID (reads from env if None)
        enable_logging: If False, returns mock logger
    
    Returns:
        DataLogger or MockDataLogger instance
    """
    if not enable_logging:
        return MockDataLogger()
    
    try:
        return DataLogger(session_id=session_id)
    except (ValueError, FileNotFoundError) as e:
        print(f"Failed to initialize DataLogger: {e}")
        print("Using MockDataLogger instead")
        return MockDataLogger()
