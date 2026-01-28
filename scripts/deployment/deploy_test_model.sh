#!/bin/bash
# Deploy model to test location (separate from production)

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MODEL_NAME="${1:-VILA1.5-3b}"
MODEL_SOURCE="${2:-huggingface}"

TEST_PATH="/home/nano/models/test"
MODEL_PATH="$TEST_PATH/$MODEL_NAME"

echo "═══════════════════════════════════════════════════════════════"
echo "  TEST MODEL DEPLOYMENT"
echo "═══════════════════════════════════════════════════════════════"
echo "  Model: $MODEL_NAME"
echo "  Source: $MODEL_SOURCE"
echo "  Destination: $MODEL_PATH (test location)"
echo ""

# Create test directory
mkdir -p "$TEST_PATH"

# Deploy (same logic as production but different path)
case $MODEL_SOURCE in
    huggingface)
        echo "Using cached HuggingFace model..."
        CACHE_PATH="$HOME/.cache/huggingface/hub/models--Efficient-Large-Model--$MODEL_NAME"
        if [ -d "$CACHE_PATH" ]; then
            ln -sf "$CACHE_PATH" "$MODEL_PATH"
            echo "  ✓ Linked cache to test path"
        else
            echo "  Model not in cache, downloading..."
            python3 - <<EOF
from transformers import AutoModel, AutoProcessor

model_name = "Efficient-Large-Model/$MODEL_NAME"
print(f"Downloading {model_name}...")
processor = AutoProcessor.from_pretrained(model_name)
model = AutoModel.from_pretrained(model_name)
print("Download complete")
EOF
            ln -sf "$CACHE_PATH" "$MODEL_PATH"
        fi
        ;;
    
    local)
        LOCAL_PATH="${3:-$HOME/models/$MODEL_NAME}"
        if [ ! -d "$LOCAL_PATH" ]; then
            echo "  ERROR: Local path not found: $LOCAL_PATH"
            exit 1
        fi
        cp -r "$LOCAL_PATH" "$MODEL_PATH"
        echo "  ✓ Copied from $LOCAL_PATH"
        ;;
    
    *)
        echo "  ERROR: Unknown source: $MODEL_SOURCE"
        exit 1
        ;;
esac

echo ""
echo "✓ Test model deployment complete"
echo "  Path: $MODEL_PATH"
echo ""
echo "To use this test model, update VLA node parameters:"
echo "  model_path: \"$MODEL_PATH\""
echo "  model_type: \"vila\""
echo "═══════════════════════════════════════════════════════════════"
