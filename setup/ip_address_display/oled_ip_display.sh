#!/bin/bash

# OLED IP Address Display Script
# Works on both Jetson and Raspberry Pi platforms

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Change to the script directory
cd "$SCRIPT_DIR"

# Activate the virtual environment
# Try project-local venv first (RPi), then home directory venv (Jetson)
PROJECT_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
if [ -f "$PROJECT_DIR/venv/bin/activate" ]; then
    source "$PROJECT_DIR/venv/bin/activate"
elif [ -f "$HOME/venv/bin/activate" ]; then
    source "$HOME/venv/bin/activate"
else
    echo "Error: virtual environment not found" >&2
    exit 1
fi

# Run the Python script using the virtual environment's Python
python3 "$SCRIPT_DIR/oled_ip_display.py"