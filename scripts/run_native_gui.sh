#!/bin/bash
# Script to build and launch the native GUI for Linux
# Mirrors Windows setup logic

set -e

# Move to repo root
SCRIPT_DIR="$(dirname "$(realpath "$0")")"
REPO_ROOT="$(realpath "$SCRIPT_DIR/../..")"
cd "$REPO_ROOT"

# PlatformIO runner detection
RUNNER_TYPE=""
RUNNER_PATH=""
VENV_PYTHON=".venv/bin/python"

test_platformio_module() {
	"$1" -m platformio --version >/dev/null 2>&1
	return $?
}

if [[ -x "$VENV_PYTHON" ]] && test_platformio_module "$VENV_PYTHON"; then
	RUNNER_TYPE="venv-python"
	RUNNER_PATH="$VENV_PYTHON"
elif command -v pio >/dev/null; then
	RUNNER_TYPE="pio"
	RUNNER_PATH="pio"
elif command -v platformio >/dev/null; then
	RUNNER_TYPE="platformio"
	RUNNER_PATH="platformio"
elif command -v py >/dev/null && test_platformio_module "py"; then
	RUNNER_TYPE="py-module"
	RUNNER_PATH="py"
elif command -v python >/dev/null && test_platformio_module "python"; then
	RUNNER_TYPE="python-module"
	RUNNER_PATH="python"
fi

if [[ -z "$RUNNER_TYPE" ]]; then
	echo "PlatformIO was not found on PATH or as a Python module." >&2
	echo "" >&2
	echo "Install options:" >&2
	echo "  1) py -m pip install -U platformio" >&2
	echo "  2) pip install -U platformio" >&2
	echo "" >&2
	echo "Then rerun: ./tools/native_gui/run_native_gui.sh" >&2
	exit 1
fi

echo "Using PlatformIO runner: $RUNNER_TYPE ($RUNNER_PATH)"

# Build unless --no-build is passed
NO_BUILD=0
for arg in "$@"; do
	if [[ "$arg" == "--no-build" ]]; then
		NO_BUILD=1
	fi
done

if [[ $NO_BUILD -eq 0 ]]; then
	echo "Building native_gui..."
	case "$RUNNER_TYPE" in
		venv-python|py-module|python-module)
			"$RUNNER_PATH" -m platformio run -e native_gui
			;;
		pio)
			pio run -e native_gui
			;;
		platformio)
			platformio run -e native_gui
			;;
		*)
			echo "No PlatformIO runner available." >&2
			exit 1
			;;
	esac
	CODE=$?
	if [[ $CODE -ne 0 ]]; then
		echo "Build failed with exit code $CODE" >&2
		exit $CODE
	fi
fi

EXE=".pio/build/native_gui/program"
if [[ ! -x "$EXE" ]]; then
	echo "GUI executable not found at: $EXE" >&2
	echo "Try building first: pio run -e native_gui" >&2
	exit 1
fi

echo "Launching GUI..."
"$EXE"
