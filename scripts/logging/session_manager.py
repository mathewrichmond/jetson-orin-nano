#!/usr/bin/env python3
"""
Session Manager
Manages creation, tracking, and querying of data collection sessions.
"""

import os
import json
import uuid
import yaml
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Any
import subprocess
import shutil


class SessionManager:
    """Manages robot data collection sessions."""
    
    def __init__(self, config_path: Optional[str] = None):
        """Initialize session manager with configuration.
        
        Args:
            config_path: Path to session_structure.yaml config file
        """
        if config_path is None:
            # Default to workspace config
            workspace_root = Path(__file__).parent.parent.parent
            config_path = workspace_root / "config" / "logging" / "session_structure.yaml"
        
        with open(config_path, 'r') as f:
            self.config = yaml.safe_load(f)['session_structure']
        
        # Determine storage root (NFS or local fallback)
        self.root = Path(self.config['root'])
        self.local_fallback = Path(self.config['local_fallback'])
        
        if not self._check_nfs_available():
            print(f"NFS mount {self.root} not available, using local fallback: {self.local_fallback}")
            self.storage_root = self.local_fallback
        else:
            self.storage_root = self.root
        
        # Ensure storage root exists
        self.storage_root.mkdir(parents=True, exist_ok=True)
    
    def _check_nfs_available(self) -> bool:
        """Check if NFS mount point is available."""
        if not self.root.exists():
            return False
        
        # Check if it's actually mounted (not just an empty directory)
        try:
            result = subprocess.run(
                ['mountpoint', '-q', str(self.root)],
                capture_output=True
            )
            return result.returncode == 0
        except FileNotFoundError:
            # mountpoint command not available, check if directory is writable
            return os.access(self.root, os.W_OK)
    
    def create_session(self, environment: str = "robot", 
                      metadata: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        """Create a new session with directory structure.
        
        Args:
            environment: "robot" or "sim"
            metadata: Additional metadata fields
        
        Returns:
            Session info dictionary with session_id, path, and manifest
        """
        # Generate session ID and timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        session_id = uuid.uuid4().hex[:8]
        
        # Create session directory name
        session_name = self.config['session_naming'].format(
            timestamp=timestamp,
            session_id=session_id
        )
        
        session_path = self.storage_root / session_name
        
        # Create directory structure
        for dir_key, dir_name in self.config['directories'].items():
            (session_path / dir_name).mkdir(parents=True, exist_ok=True)
        
        # Create session manifest
        manifest = {
            'session_id': session_id,
            'session_name': session_name,
            'environment': environment,
            'start_time': datetime.now().isoformat(),
            'end_time': None,
            'duration_sec': None,
            'status': 'recording',
            'storage_location': str(session_path),
            'robot_config': self._get_robot_config(),
            'model_version': self._get_model_version(),
            'git_commit': self._get_git_commit(),
            'total_size_bytes': 0,
            'bag_count': 0,
        }
        
        # Add user-provided metadata
        if metadata:
            manifest.update(metadata)
        
        # Write manifest
        manifest_path = session_path / "metadata" / "session_manifest.json"
        with open(manifest_path, 'w') as f:
            json.dump(manifest, f, indent=2)
        
        # Create lock file to indicate active session
        lock_path = session_path / ".session_lock"
        lock_path.write_text(str(os.getpid()))
        
        print(f"Created session: {session_name}")
        print(f"  Session ID: {session_id}")
        print(f"  Path: {session_path}")
        print(f"  Environment: {environment}")
        
        return {
            'session_id': session_id,
            'session_name': session_name,
            'session_path': str(session_path),
            'manifest': manifest
        }
    
    def finalize_session(self, session_id: str, status: str = "completed"):
        """Finalize a session by updating manifest and removing lock.
        
        Args:
            session_id: Session ID to finalize
            status: Final status ("completed", "failed", etc.)
        """
        session_path = self.find_session(session_id)
        if not session_path:
            raise ValueError(f"Session {session_id} not found")
        
        # Read existing manifest
        manifest_path = session_path / "metadata" / "session_manifest.json"
        with open(manifest_path, 'r') as f:
            manifest = json.load(f)
        
        # Update manifest
        end_time = datetime.now()
        start_time = datetime.fromisoformat(manifest['start_time'])
        duration = (end_time - start_time).total_seconds()
        
        manifest['end_time'] = end_time.isoformat()
        manifest['duration_sec'] = duration
        manifest['status'] = status
        
        # Calculate total size
        total_size = sum(
            f.stat().st_size 
            for f in session_path.rglob('*') 
            if f.is_file()
        )
        manifest['total_size_bytes'] = total_size
        
        # Count bag files
        bag_count = len(list((session_path / "raw").glob("*.bag")))
        manifest['bag_count'] = bag_count
        
        # Write updated manifest
        with open(manifest_path, 'w') as f:
            json.dump(manifest, f, indent=2)
        
        # Remove lock file
        lock_path = session_path / ".session_lock"
        if lock_path.exists():
            lock_path.unlink()
        
        print(f"Finalized session: {session_id}")
        print(f"  Status: {status}")
        print(f"  Duration: {duration:.1f}s ({duration/60:.1f}m)")
        print(f"  Size: {total_size / 1e9:.2f} GB")
        print(f"  Bags: {bag_count}")
    
    def find_session(self, session_id: str) -> Optional[Path]:
        """Find session path by ID.
        
        Args:
            session_id: Session ID to find
        
        Returns:
            Path to session directory or None if not found
        """
        # Search for session with matching ID in name
        for session_dir in self.storage_root.glob(f"session_*_{session_id}"):
            if session_dir.is_dir():
                return session_dir
        return None
    
    def list_sessions(self, environment: Optional[str] = None,
                     status: Optional[str] = None,
                     limit: Optional[int] = None) -> List[Dict[str, Any]]:
        """List sessions with optional filtering.
        
        Args:
            environment: Filter by "robot" or "sim"
            status: Filter by status
            limit: Maximum number of sessions to return
        
        Returns:
            List of session manifest dictionaries
        """
        sessions = []
        
        # Iterate through all session directories
        for session_dir in sorted(self.storage_root.glob("session_*"), reverse=True):
            if not session_dir.is_dir():
                continue
            
            manifest_path = session_dir / "metadata" / "session_manifest.json"
            if not manifest_path.exists():
                continue
            
            with open(manifest_path, 'r') as f:
                manifest = json.load(f)
            
            # Apply filters
            if environment and manifest.get('environment') != environment:
                continue
            if status and manifest.get('status') != status:
                continue
            
            sessions.append(manifest)
            
            if limit and len(sessions) >= limit:
                break
        
        return sessions
    
    def get_active_session(self) -> Optional[Dict[str, Any]]:
        """Get currently active (recording) session if any.
        
        Returns:
            Session manifest or None
        """
        for session_dir in self.storage_root.glob("session_*"):
            lock_path = session_dir / ".session_lock"
            if lock_path.exists():
                manifest_path = session_dir / "metadata" / "session_manifest.json"
                if manifest_path.exists():
                    with open(manifest_path, 'r') as f:
                        return json.load(f)
        return None
    
    def update_session_status(self, session_id: str, status: str):
        """Update session status in manifest.
        
        Args:
            session_id: Session ID
            status: New status value
        """
        session_path = self.find_session(session_id)
        if not session_path:
            raise ValueError(f"Session {session_id} not found")
        
        manifest_path = session_path / "metadata" / "session_manifest.json"
        with open(manifest_path, 'r') as f:
            manifest = json.load(f)
        
        manifest['status'] = status
        
        with open(manifest_path, 'w') as f:
            json.dump(manifest, f, indent=2)
    
    def _get_robot_config(self) -> Dict[str, Any]:
        """Get current robot configuration."""
        # TODO: Read from actual robot config files
        return {
            'platform': 'jetson_orin_nano',
            'camera_count': 2,
            'imu': True,
            'microphone': True
        }
    
    def _get_model_version(self) -> Optional[str]:
        """Get current VLA model version/checksum."""
        # TODO: Read from model deployment manifest
        model_manifest = Path("/opt/models/vila/model_manifest.json")
        if model_manifest.exists():
            with open(model_manifest, 'r') as f:
                data = json.load(f)
                return data.get('sha256', 'unknown')
        return None
    
    def _get_git_commit(self) -> Optional[str]:
        """Get current git commit hash."""
        try:
            result = subprocess.run(
                ['git', 'rev-parse', 'HEAD'],
                cwd=Path(__file__).parent.parent.parent,
                capture_output=True,
                text=True,
                timeout=2
            )
            if result.returncode == 0:
                return result.stdout.strip()
        except Exception:
            pass
        return None


def main():
    """CLI interface for session manager."""
    import argparse
    
    parser = argparse.ArgumentParser(description='Manage data collection sessions')
    subparsers = parser.add_subparsers(dest='command', help='Command to execute')
    
    # Create session
    create_parser = subparsers.add_parser('create', help='Create new session')
    create_parser.add_argument('--env', choices=['robot', 'sim'], default='robot',
                              help='Environment type')
    
    # Finalize session
    finalize_parser = subparsers.add_parser('finalize', help='Finalize session')
    finalize_parser.add_argument('session_id', help='Session ID')
    finalize_parser.add_argument('--status', default='completed',
                                help='Final status')
    
    # List sessions
    list_parser = subparsers.add_parser('list', help='List sessions')
    list_parser.add_argument('--env', choices=['robot', 'sim'],
                            help='Filter by environment')
    list_parser.add_argument('--status', help='Filter by status')
    list_parser.add_argument('--limit', type=int, help='Limit number of results')
    
    # Get active session
    subparsers.add_parser('active', help='Show active session')
    
    args = parser.parse_args()
    
    manager = SessionManager()
    
    if args.command == 'create':
        result = manager.create_session(environment=args.env)
        print(f"\nSession ID: {result['session_id']}")
        print(f"Export with: export SESSION_ID={result['session_id']}")
    
    elif args.command == 'finalize':
        manager.finalize_session(args.session_id, status=args.status)
    
    elif args.command == 'list':
        sessions = manager.list_sessions(
            environment=args.env,
            status=args.status,
            limit=args.limit
        )
        print(f"\nFound {len(sessions)} sessions:\n")
        for session in sessions:
            print(f"  {session['session_id']}: {session['session_name']}")
            print(f"    Environment: {session['environment']}")
            print(f"    Status: {session['status']}")
            print(f"    Start: {session['start_time']}")
            if session.get('duration_sec'):
                print(f"    Duration: {session['duration_sec']/60:.1f}m")
            print()
    
    elif args.command == 'active':
        session = manager.get_active_session()
        if session:
            print(f"\nActive session: {session['session_id']}")
            print(f"  Name: {session['session_name']}")
            print(f"  Environment: {session['environment']}")
            print(f"  Start: {session['start_time']}")
        else:
            print("\nNo active session")
    
    else:
        parser.print_help()


if __name__ == '__main__':
    main()
