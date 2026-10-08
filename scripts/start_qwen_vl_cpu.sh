#!/usr/bin/env bash
set -euo pipefail

VISION_HOME="${SURA_VLM_HOME:-${SURA_WS_META:-$(pwd)}/.sura/vision}"
MODEL_PATH="$VISION_HOME/models/Qwen3VL-2B-Instruct-Q4_K_M.gguf"
PROJECTOR_PATH="$VISION_HOME/models/mmproj-Qwen3VL-2B-Instruct-Q8_0.gguf"
VLM_PORT="${SURA_VLM_PORT:-8080}"

if [[ -x "$VISION_HOME/bin/llama-server" && -f "$MODEL_PATH" && -f "$PROJECTOR_PATH" ]]; then
  exec "$VISION_HOME/bin/llama-server" \
    --model "$MODEL_PATH" --mmproj "$PROJECTOR_PATH" \
    --alias Qwen3-VL-2B-Instruct --host 127.0.0.1 --port "$VLM_PORT" \
    -ngl 0 --threads 6 --ctx-size 4096
fi

if ! command -v llama-server >/dev/null 2>&1; then
  echo "Qwen VL is not installed in $VISION_HOME and llama-server is not in PATH." >&2
  exit 1
fi

# When using a system llama-server, start it before the mission. The first
# start downloads the official model into the llama.cpp cache.
exec llama-server \
  -hf Qwen/Qwen3-VL-2B-Instruct-GGUF:Q4_K_M \
  --alias Qwen3-VL-2B-Instruct --host 127.0.0.1 --port "$VLM_PORT" \
  -ngl 0 --threads 6 --ctx-size 4096
