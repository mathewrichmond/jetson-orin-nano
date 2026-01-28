#!/usr/bin/env python3
"""
Session Processing Orchestrator
Runs the complete data processing pipeline: QC → Parquet → Artifacts
"""

import json
from pathlib import Path
from typing import Dict, Any
from datetime import datetime
import sys
import subprocess


class SessionProcessor:
    """Orchestrates session data processing pipeline."""
    
    def __init__(self, session_path: Path):
        """Initialize processor.
        
        Args:
            session_path: Path to session directory
        """
        self.session_path = Path(session_path)
        self.metadata_dir = self.session_path / "metadata"
        self.script_dir = Path(__file__).parent
        
        # Check if processing is already in progress
        self.lock_file = self.session_path / ".processing_lock"
        if self.lock_file.exists():
            raise RuntimeError(f"Processing already in progress for {session_path.name}")
        
        # Create lock file
        self.lock_file.write_text(str(subprocess.os.getpid()))
        
        print(f"Session Processor initialized")
        print(f"  Session: {self.session_path.name}")
    
    def process(self, resume: bool = False) -> Dict[str, Any]:
        """Run complete processing pipeline.
        
        Args:
            resume: If True, skip completed steps
        
        Returns:
            Processing report
        """
        start_time = datetime.now()
        results = {
            'session_id': self.session_path.name,
            'start_time': start_time.isoformat(),
            'steps': {}
        }
        
        print("\n" + "=" * 70)
        print("  SESSION PROCESSING PIPELINE")
        print("=" * 70)
        print()
        
        try:
            # Step 1: Quality Check
            if not resume or not self._step_completed('quality_check'):
                print("STEP 1: Quality Check")
                print("-" * 70)
                results['steps']['quality_check'] = self._run_quality_check()
                self._mark_step_completed('quality_check')
            else:
                print("STEP 1: Quality Check [SKIPPED - already completed]")
                results['steps']['quality_check'] = {'status': 'skipped'}
            print()
            
            # Step 2: Bag to Parquet Conversion
            if not resume or not self._step_completed('bag_to_parquet'):
                print("STEP 2: Bag to Parquet Conversion")
                print("-" * 70)
                results['steps']['bag_to_parquet'] = self._run_bag_to_parquet()
                self._mark_step_completed('bag_to_parquet')
            else:
                print("STEP 2: Bag to Parquet [SKIPPED - already completed]")
                results['steps']['bag_to_parquet'] = {'status': 'skipped'}
            print()
            
            # Step 3: Artifact Generation
            if not resume or not self._step_completed('generate_artifacts'):
                print("STEP 3: Artifact Generation")
                print("-" * 70)
                results['steps']['generate_artifacts'] = self._run_generate_artifacts()
                self._mark_step_completed('generate_artifacts')
            else:
                print("STEP 3: Generate Artifacts [SKIPPED - already completed]")
                results['steps']['generate_artifacts'] = {'status': 'skipped'}
            print()
            
            # All steps completed
            end_time = datetime.now()
            duration = (end_time - start_time).total_seconds()
            
            results['end_time'] = end_time.isoformat()
            results['duration_sec'] = duration
            results['status'] = 'completed'
            
            print("=" * 70)
            print("  PROCESSING COMPLETE")
            print("=" * 70)
            print(f"  Duration: {duration:.1f}s ({duration/60:.1f}m)")
            print(f"  Session: {self.session_path.name}")
            print()
            
        except Exception as e:
            results['status'] = 'failed'
            results['error'] = str(e)
            print()
            print("=" * 70)
            print("  PROCESSING FAILED")
            print("=" * 70)
            print(f"  Error: {e}")
            print()
            raise
        
        finally:
            # Write processing manifest
            manifest_path = self.metadata_dir / "processing_manifest.json"
            with open(manifest_path, 'w') as f:
                json.dump(results, f, indent=2)
            
            # Remove lock file
            if self.lock_file.exists():
                self.lock_file.unlink()
            
            # Update session status
            self._update_session_status(results['status'])
        
        return results
    
    def _run_quality_check(self) -> Dict[str, Any]:
        """Run quality check step."""
        script = self.script_dir / "quality_check.py"
        
        result = subprocess.run(
            [sys.executable, str(script), str(self.session_path)],
            capture_output=True,
            text=True
        )
        
        if result.returncode == 0:
            return {'status': 'pass', 'returncode': 0}
        else:
            return {'status': 'fail', 'returncode': result.returncode, 'error': result.stderr}
    
    def _run_bag_to_parquet(self) -> Dict[str, Any]:
        """Run bag to Parquet conversion step."""
        script = self.script_dir / "bag_to_parquet.py"
        
        result = subprocess.run(
            [sys.executable, str(script), str(self.session_path)],
            capture_output=True,
            text=True
        )
        
        if result.returncode == 0:
            return {'status': 'completed', 'returncode': 0}
        else:
            return {'status': 'failed', 'returncode': result.returncode, 'error': result.stderr}
    
    def _run_generate_artifacts(self) -> Dict[str, Any]:
        """Run artifact generation step."""
        script = self.script_dir / "generate_artifacts.py"
        
        result = subprocess.run(
            [sys.executable, str(script), str(self.session_path)],
            capture_output=True,
            text=True
        )
        
        if result.returncode == 0:
            return {'status': 'completed', 'returncode': 0}
        else:
            return {'status': 'failed', 'returncode': result.returncode, 'error': result.stderr}
    
    def _step_completed(self, step_name: str) -> bool:
        """Check if a pipeline step was already completed."""
        manifest_path = self.metadata_dir / "processing_manifest.json"
        if not manifest_path.exists():
            return False
        
        with open(manifest_path) as f:
            manifest = json.load(f)
        
        step_result = manifest.get('steps', {}).get(step_name, {})
        return step_result.get('status') in ['pass', 'completed']
    
    def _mark_step_completed(self, step_name: str):
        """Mark a pipeline step as completed."""
        # This is tracked in the processing manifest
        pass
    
    def _update_session_status(self, processing_status: str):
        """Update session manifest with processing status."""
        manifest_path = self.metadata_dir / "session_manifest.json"
        if not manifest_path.exists():
            return
        
        with open(manifest_path) as f:
            manifest = json.load(f)
        
        if processing_status == 'completed':
            manifest['status'] = 'completed'
        elif processing_status == 'failed':
            manifest['status'] = 'processing_failed'
        else:
            manifest['status'] = 'processing'
        
        with open(manifest_path, 'w') as f:
            json.dump(manifest, f, indent=2)


def main():
    """CLI interface for session processor."""
    import argparse
    
    parser = argparse.ArgumentParser(description='Process session data through pipeline')
    parser.add_argument('session_path', help='Path to session directory')
    parser.add_argument('--resume', action='store_true',
                       help='Resume processing, skip completed steps')
    
    args = parser.parse_args()
    
    session_path = Path(args.session_path)
    if not session_path.exists():
        print(f"Error: Session directory not found: {session_path}")
        sys.exit(1)
    
    try:
        processor = SessionProcessor(session_path)
        report = processor.process(resume=args.resume)
        sys.exit(0 if report['status'] == 'completed' else 1)
    except RuntimeError as e:
        print(f"Error: {e}")
        sys.exit(1)


if __name__ == '__main__':
    main()
