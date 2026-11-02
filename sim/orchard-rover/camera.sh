#!/usr/bin/env bash
set -euo pipefail
ROVER_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec python3 "$ROVER_ROOT/tools/camera_command.py" "$@"
