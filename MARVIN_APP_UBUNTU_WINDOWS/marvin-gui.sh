#!/bin/bash
# Wrapper script for marvin-gui that ensures correct working directory

# Get the directory where this script (or its symlink target) resides
SCRIPT_DIR="$(dirname "$(readlink -f "$0")")"

# The executable lives in the repo root (one level up)
REPO_ROOT="$(dirname "$SCRIPT_DIR")"

# Change to the repo root so the app finds its config files
cd "$REPO_ROOT"

# Run the application
exec "$REPO_ROOT/MARVIN_APP-ubuntu2404-english" "$@"
