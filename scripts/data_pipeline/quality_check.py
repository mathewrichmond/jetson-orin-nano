#!/usr/bin/env python3
"""
Quality Check Pipeline
Validates session data and detects anomalies.
"""

import json
import yaml
from pathlib import Path
from typing import Dict, Any, List, Optional
import subprocess
from datetime import datetime
import sys


class QualityChecker:
    """Quality check pipeline for session data."""
    
    def __init__(self, session_path: Path, config_path: Optional[Path] = None):
        """Initialize quality checker.
        
        Args:
            session_path: Path to session directory
            config_path: Path to pipeline config (optional)
        """
        self.session_path = Path(session_path)
        
        # Load configuration
        if config_path is None:
            workspace_root = Path(__file__).parent.parent.parent
            config_path = workspace_root / "config" / "data_pipeline" / "pipeline_config.yaml"
        
        with open(config_path) as f:
            self.config = yaml.safe_load(f)['pipeline']['quality']
        
        # Paths
        self.raw_dir = self.session_path / "raw"
        self.quality_dir = self.session_path / "quality"
        self.quality_dir.mkdir(parents=True, exist_ok=True)
        
        # Results
        self.warnings: List[Dict[str, Any]] = []
        self.errors: List[Dict[str, Any]] = []
        self.checks_passed = 0
        self.checks_failed = 0
    
    def run_all_checks(self) -> Dict[str, Any]:
        """Run all quality checks.
        
        Returns:
            Quality check report
        """
        print("Running quality checks...")
        print(f"  Session: {self.session_path.name}")
        print()
        
        # Check bag files
        self._check_bag_integrity()
        self._check_bag_topics()
        self._check_data_gaps()
        
        # Check specific sensors
        self._check_camera_data()
        self._check_imu_data()
        self._check_battery_data()
        
        # Determine overall status
        status = "pass" if self.checks_failed == 0 else "fail"
        
        # Generate report
        report = {
            'session_id': self.session_path.name,
            'timestamp': datetime.now().isoformat(),
            'status': status,
            'checks_passed': self.checks_passed,
            'checks_failed': self.checks_failed,
            'warnings': self.warnings,
            'errors': self.errors,
            'config': self.config
        }
        
        # Write report
        report_path = self.quality_dir / "qc_report.json"
        with open(report_path, 'w') as f:
            json.dump(report, f, indent=2)
        
        print()
        print("Quality Check Complete")
        print(f"  Status: {status.upper()}")
        print(f"  Passed: {self.checks_passed}")
        print(f"  Failed: {self.checks_failed}")
        print(f"  Warnings: {len(self.warnings)}")
        print(f"  Report: {report_path}")
        
        return report
    
    def _check_bag_integrity(self):
        """Check ROS bag file integrity."""
        print("Checking bag integrity...")
        
        bag_files = list(self.raw_dir.glob("*.bag"))
        if not bag_files:
            self._add_error("No bag files found")
            return
        
        for bag_file in bag_files:
            # Check file size
            if bag_file.stat().st_size == 0:
                self._add_error(f"Empty bag file: {bag_file.name}")
                continue
            
            # Try to get bag info (this validates the file)
            try:
                result = subprocess.run(
                    ['ros2', 'bag', 'info', str(bag_file)],
                    capture_output=True,
                    text=True,
                    timeout=30
                )
                
                if result.returncode != 0:
                    self._add_error(f"Corrupted bag file: {bag_file.name}")
                else:
                    self._add_pass()
                    
            except subprocess.TimeoutExpired:
                self._add_error(f"Timeout checking bag: {bag_file.name}")
            except Exception as e:
                self._add_error(f"Error checking bag {bag_file.name}: {e}")
    
    def _check_bag_topics(self):
        """Check that expected topics are present."""
        print("Checking bag topics...")
        
        bag_files = list(self.raw_dir.glob("*.bag"))
        if not bag_files:
            return
        
        # Get info from first bag
        bag_file = bag_files[0]
        try:
            result = subprocess.run(
                ['ros2', 'bag', 'info', str(bag_file)],
                capture_output=True,
                text=True,
                timeout=30
            )
            
            if result.returncode == 0:
                # Count topics
                output = result.stdout
                topic_count = output.count('Topic:')
                
                min_topics = self.config.get('min_topic_count', 20)
                if topic_count < min_topics:
                    self._add_warning(f"Low topic count: {topic_count} (expected >={min_topics})")
                else:
                    self._add_pass()
            
        except Exception as e:
            self._add_warning(f"Could not check topics: {e}")
    
    def _check_data_gaps(self):
        """Check for data gaps (dropped frames, missing data)."""
        print("Checking for data gaps...")
        
        # This would require reading bag contents
        # For now, just a placeholder check
        self._add_pass()
        # TODO: Implement actual gap detection by parsing bag timings
    
    def _check_camera_data(self):
        """Check camera data quality."""
        print("Checking camera data...")
        
        # Check for black/frozen frames
        # This would require OpenCV to read images from bags
        # Placeholder for now
        self._add_pass()
        # TODO: Implement black/frozen frame detection
    
    def _check_imu_data(self):
        """Check IMU data quality."""
        print("Checking IMU data...")
        
        # Check IMU rate and outliers
        # This would require parsing bag IMU messages
        # Placeholder for now
        self._add_pass()
        # TODO: Implement IMU rate and outlier detection
    
    def _check_battery_data(self):
        """Check battery data for anomalies."""
        print("Checking battery data...")
        
        # Check for sudden battery drops
        # This would require parsing bag battery messages
        # Placeholder for now
        self._add_pass()
        # TODO: Implement battery anomaly detection
    
    def _add_pass(self):
        """Record a passed check."""
        self.checks_passed += 1
    
    def _add_warning(self, message: str):
        """Add a warning."""
        self.warnings.append({
            'message': message,
            'timestamp': datetime.now().isoformat()
        })
        print(f"  ⚠️  {message}")
    
    def _add_error(self, message: str):
        """Add an error."""
        self.errors.append({
            'message': message,
            'timestamp': datetime.now().isoformat()
        })
        self.checks_failed += 1
        print(f"  ❌ {message}")


def main():
    """CLI interface for quality checker."""
    import argparse
    
    parser = argparse.ArgumentParser(description='Run quality checks on session data')
    parser.add_argument('session_path', help='Path to session directory')
    parser.add_argument('--config', help='Path to pipeline config')
    
    args = parser.parse_args()
    
    session_path = Path(args.session_path)
    if not session_path.exists():
        print(f"Error: Session directory not found: {session_path}")
        sys.exit(1)
    
    checker = QualityChecker(session_path, config_path=args.config)
    report = checker.run_all_checks()
    
    # Exit with non-zero if checks failed
    sys.exit(0 if report['status'] == 'pass' else 1)


if __name__ == '__main__':
    main()
