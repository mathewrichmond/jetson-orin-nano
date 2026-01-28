#!/bin/bash
# Quick script to monitor GitHub Actions without installing gh CLI
# Usage: ./github_action_monitor.sh

REPO="mathewrichmond/jetson-orin-nano"

echo "GitHub Actions Status for $REPO"
echo "========================================"
echo ""
echo "Latest workflow runs:"
echo ""

# Fetch latest runs (requires no authentication for public repos)
curl -s "https://api.github.com/repos/$REPO/actions/runs?per_page=5" | \
  python3 -c "
import json, sys
try:
    data = json.load(sys.stdin)
    for run in data.get('workflow_runs', [])[:5]:
        status = run['status']
        conclusion = run.get('conclusion', 'pending')
        icon = '✅' if conclusion == 'success' else '❌' if conclusion == 'failure' else '⏳'
        print(f\"{icon} {run['name']}: {status}/{conclusion}\")
        print(f\"   Branch: {run['head_branch']}, Commit: {run['head_sha'][:7]}\")
        print(f\"   Started: {run['created_at']}\")
        print(f\"   URL: {run['html_url']}\")
        print()
except:
    print('Error parsing JSON. Check your internet connection.')
"

echo ""
echo "View all runs: https://github.com/$REPO/actions"
