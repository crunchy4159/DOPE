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

# Check for GLFW development libraries (system dependency)
echo "Checking for GLFW development libraries..."
if ! pkg-config --exists glfw3 2>/dev/null; then
    echo ""
    echo "GLFW3 development libraries not found." >&2
    echo "Install with one of:" >&2
    echo "  Ubuntu/Debian:  sudo apt install libglfw3-dev" >&2
    echo "  Fedora/RHEL:    sudo dnf install glfw-devel" >&2
    echo "  Arch:           sudo pacman -S glfw" >&2
    echo ""
    echo "Then rerun: ./scripts/setup_linux.sh" >&2
    exit 1
fi
echo "GLFW3 found."

# Optionally install other requirements here
# echo "Installing other requirements..."
# python -m pip install -r requirements.txt

# Deactivate venv
deactivate

echo "Setup complete. You can now run ./scripts/run_native_gui.sh"
