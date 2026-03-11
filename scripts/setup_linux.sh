#!/bin/bash
# Setup script for Linux: prepares PlatformIO and Python venv for DOPE project
# Run this from the repo root

set -e

SCRIPT_DIR="$(dirname "$(realpath "$0")")"
REPO_ROOT="$(realpath "$SCRIPT_DIR/../..")"
cd "$REPO_ROOT"

# Create Python venv if missing
echo "Checking for Python venv..."
if [[ ! -d ".venv" ]]; then
    echo "Creating Python venv..."
    python3 -m venv .venv
fi

echo "Activating venv and upgrading pip..."
if [[ -f ".venv/bin/activate" ]]; then
    source .venv/bin/activate
elif [[ -f ".venv/Scripts/activate" ]]; then
    source .venv/Scripts/activate
else
    echo "Could not find venv activation script (.venv/bin/activate or .venv/Scripts/activate)." >&2
    exit 1
fi
python -m pip install --upgrade pip

# Install PlatformIO if missing
echo "Checking for PlatformIO..."
if ! python -m platformio --version >/dev/null 2>&1; then
    echo "Installing PlatformIO..."
    python -m pip install -U platformio
else
    echo "PlatformIO already installed."
fi

# Optionally install other requirements here
# echo "Installing other requirements..."
# python -m pip install -r requirements.txt

# Deactivate venv
deactivate

echo "Setup complete. You can now run ./scripts/run_native_gui.sh"
