#!/usr/bin/env bash
set -euo pipefail
ROVER_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
TREE_MODELS_PATH="${LYCHEE_TREE_MODELS_PATH:-$ROVER_ROOT/../../external/lychee-tree/models}"
if [[ ! -f "$TREE_MODELS_PATH/lychee_tree/model.sdf" ]]; then
  echo "找不到荔枝树模型。请将 LYCHEE_TREE_MODELS_PATH 设为包含 lychee_tree 的 models 目录。" >&2
  exit 1
fi
export GZ_SIM_RESOURCE_PATH="$ROVER_ROOT/models:$TREE_MODELS_PATH${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
mkdir -p "$ROVER_ROOT/logs"
exec gz sim -r "$ROVER_ROOT/worlds/orchard.sdf" --gui-config "$ROVER_ROOT/gui/rover.config" "$@"
