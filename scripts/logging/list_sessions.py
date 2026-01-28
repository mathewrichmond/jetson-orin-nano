#!/usr/bin/env python3
"""
List and query data collection sessions.
"""

import argparse
from datetime import datetime
from pathlib import Path
from session_manager import SessionManager
from tabulate import tabulate


def format_size(bytes_val):
    """Format bytes to human-readable size."""
    for unit in ['B', 'KB', 'MB', 'GB', 'TB']:
        if bytes_val < 1024.0:
            return f"{bytes_val:.1f}{unit}"
        bytes_val /= 1024.0
    return f"{bytes_val:.1f}PB"


def format_duration(seconds):
    """Format seconds to human-readable duration."""
    if seconds is None:
        return "N/A"
    if seconds < 60:
        return f"{seconds:.0f}s"
    elif seconds < 3600:
        return f"{seconds/60:.1f}m"
    else:
        return f"{seconds/3600:.1f}h"


def main():
    parser = argparse.ArgumentParser(description='List and query data collection sessions')
    parser.add_argument('--env', choices=['robot', 'sim'], help='Filter by environment')
    parser.add_argument('--status', help='Filter by status (recording, processing, completed, failed)')
    parser.add_argument('--date', help='Filter by date (YYYY-MM-DD)')
    parser.add_argument('--limit', type=int, default=20, help='Maximum number of sessions (default: 20)')
    parser.add_argument('--failed-qc', action='store_true', help='Show only sessions that failed QC')
    parser.add_argument('--format', choices=['table', 'json', 'csv'], default='table',
                       help='Output format')
    
    args = parser.parse_args()
    
    manager = SessionManager()
    sessions = manager.list_sessions(
        environment=args.env,
        status=args.status,
        limit=args.limit
    )
    
    # Additional filtering
    if args.date:
        date_filter = args.date
        sessions = [s for s in sessions if s['start_time'].startswith(date_filter)]
    
    if args.failed_qc:
        # Check for QC failures in session data
        filtered = []
        for session in sessions:
            session_path = manager.find_session(session['session_id'])
            qc_report = session_path / "quality" / "qc_report.json"
            if qc_report.exists():
                import json
                with open(qc_report) as f:
                    qc_data = json.load(f)
                    if qc_data.get('status') == 'fail':
                        filtered.append(session)
        sessions = filtered
    
    if not sessions:
        print("No sessions found matching criteria.")
        return
    
    # Format output
    if args.format == 'json':
        import json
        print(json.dumps(sessions, indent=2))
    
    elif args.format == 'csv':
        import csv
        import sys
        writer = csv.DictWriter(sys.stdout, fieldnames=[
            'session_id', 'environment', 'status', 'start_time', 
            'duration_sec', 'total_size_bytes', 'bag_count'
        ])
        writer.writeheader()
        writer.writerows(sessions)
    
    else:  # table format
        table_data = []
        for session in sessions:
            table_data.append([
                session['session_id'],
                session['session_name'][:30] + '...' if len(session['session_name']) > 30 else session['session_name'],
                session['environment'],
                session['status'],
                session['start_time'].split('T')[0],  # Just date
                format_duration(session.get('duration_sec')),
                format_size(session.get('total_size_bytes', 0)),
                session.get('bag_count', 0)
            ])
        
        headers = ['ID', 'Name', 'Env', 'Status', 'Date', 'Duration', 'Size', 'Bags']
        print(f"\nFound {len(sessions)} sessions:\n")
        print(tabulate(table_data, headers=headers, tablefmt='grid'))
        
        # Summary statistics
        total_size = sum(s.get('total_size_bytes', 0) for s in sessions)
        total_duration = sum(s.get('duration_sec', 0) or 0 for s in sessions)
        print(f"\nTotal: {format_size(total_size)}, {format_duration(total_duration)}")


if __name__ == '__main__':
    main()
