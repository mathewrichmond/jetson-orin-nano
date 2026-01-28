#!/bin/bash
# Deploy VILA model to production location

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MODEL_NAME="${1:-VILA1.5-3b}"
MODEL_SOURCE="${2:-huggingface}"  # huggingface, local, url

PRODUCTION_PATH="/opt/models/vila"
MODEL_PATH="$PRODUCTION_PATH/$MODEL_NAME"

echo "═══════════════════════════════════════════════════════════════"
echo "  MODEL DEPLOYMENT"
echo "═══════════════════════════════════════════════════════════════"
echo "  Model: $MODEL_NAME"
echo "  Source: $MODEL_SOURCE"
echo "  Destination: $MODEL_PATH"
echo ""

# Create production directory
sudo mkdir -p "$PRODUCTION_PATH"
sudo chown -R $USER:$USER "$PRODUCTION_PATH"

# Deploy based on source
case $MODEL_SOURCE in
    huggingface)
        echo "Downloading from HuggingFace..."
        python3 - <<EOF
from transformers import AutoModel, AutoProcessor
import os

model_name = "Efficient-Large-Model/$MODEL_NAME"
cache_dir = os.path.expanduser("~/.cache/huggingface")

print(f"Downloading {model_name} to cache...")
processor = AutoProcessor.from_pretrained(model_name, cache_dir=cache_dir)
model = AutoModel.from_pretrained(model_name, cache_dir=cache_dir)

print(f"Model downloaded to cache: {cache_dir}")
EOF
        
        # Symlink cache to production path
        CACHE_PATH="$HOME/.cache/huggingface/hub/models--Efficient-Large-Model--$MODEL_NAME"
        if [ -d "$CACHE_PATH" ]; then
            ln -sf "$CACHE_PATH" "$MODEL_PATH"
            echo "  ✓ Linked cache to production path"
        else
            echo "  ERROR: Cache not found at $CACHE_PATH"
            exit 1
        fi
        ;;
    
    local)
        echo "Copying from local path..."
        LOCAL_PATH="${3:-$HOME/models/$MODEL_NAME}"
        if [ ! -d "$LOCAL_PATH" ]; then
            echo "  ERROR: Local path not found: $LOCAL_PATH"
            exit 1
        fi
        cp -r "$LOCAL_PATH" "$MODEL_PATH"
        echo "  ✓ Copied from $LOCAL_PATH"
        ;;
    
    url)
        echo "Downloading from URL..."
        MODEL_URL="$3"
        if [ -z "$MODEL_URL" ]; then
            echo "  ERROR: URL not provided"
            exit 1
        fi
        wget -O "/tmp/$MODEL_NAME.tar.gz" "$MODEL_URL"
        tar -xzf "/tmp/$MODEL_NAME.tar.gz" -C "$PRODUCTION_PATH"
        rm "/tmp/$MODEL_NAME.tar.gz"
        echo "  ✓ Downloaded and extracted"
        ;;
    
    *)
        echo "  ERROR: Unknown source: $MODEL_SOURCE"
        echo "  Valid sources: huggingface, local, url"
        exit 1
        ;;
esac

# Create model manifest
echo "Creating model manifest..."
MODEL_MANIFEST="$PRODUCTION_PATH/model_manifest.json"
python3 - <<EOF
import json
import hashlib
import os
from datetime import datetime

def get_dir_hash(path):
    """Get hash of directory contents"""
    hash_md5 = hashlib.md5()
    for root, dirs, files in os.walk(path):
        for file in sorted(files):
            filepath = os.path.join(root, file)
            with open(filepath, 'rb') as f:
                for chunk in iter(lambda: f.read(4096), b""):
                    hash_md5.update(chunk)
    return hash_md5.hexdigest()

manifest = {
    'model_name': '$MODEL_NAME',
    'deployed_at': datetime.now().isoformat(),
    'source': '$MODEL_SOURCE',
    'path': '$MODEL_PATH',
    'sha256': get_dir_hash('$MODEL_PATH') if os.path.exists('$MODEL_PATH') else 'unknown',
    'deployed_by': os.getenv('USER', 'unknown')
}

with open('$MODEL_MANIFEST', 'w') as f:
    json.dump(manifest, f, indent=2)

print(f"Manifest: {manifest['sha256'][:16]}...")
EOF

echo ""
echo "✓ Model deployment complete"
echo "  Path: $MODEL_PATH"
echo "  Manifest: $MODEL_MANIFEST"
echo ""
echo "To use this model, update config/models/vila_config.yaml:"
echo "  model.path: \"$MODEL_PATH\""
echo "═══════════════════════════════════════════════════════════════"
