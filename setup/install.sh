#!/bin/bash

# togikaidrive Setup Script
# Usage: bash setup/install.sh
# Detects platform, installs dependencies, and generates config.py

set -e

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PROJECT_DIR=$(echo "$PROJECT_DIR" | tr -d '\r')

echo "=== togikaidrive setup ==="

# Detect platform
if [ -f /proc/device-tree/model ]; then
    MODEL=$(tr -d '\0' < /proc/device-tree/model)
else
    MODEL="unknown"
fi

if echo "$MODEL" | grep -qi "jetson"; then
    PLATFORM="jetson"
elif echo "$MODEL" | grep -qi "raspberry"; then
    PLATFORM="rpi"
else
    PLATFORM="rpi"
    echo "Warning: Unknown platform ($MODEL), defaulting to rpi"
fi

echo "Platform: $PLATFORM"
echo "Project dir: $PROJECT_DIR"

# Install apt dependencies
if [ "$PLATFORM" = "rpi" ]; then
    sudo apt-get update
    sudo apt-get install -y libcap-dev
fi

# Install pip dependencies
pip install -r "$PROJECT_DIR/setup/requirements-${PLATFORM}.txt"

# Generate config.py from config_default.py
if [ ! -f "$PROJECT_DIR/config.py" ]; then
    cp "$PROJECT_DIR/config_default.py" "$PROJECT_DIR/config.py"
    echo "config_default.py から config.py を作成しました"
else
    echo "config.py は既に存在します（スキップ）"
fi

echo "=== setup complete ==="
