#!/usr/bin/env bash
# Execute with a MuJoCo interpreter without importing a ROS Python overlay.
set -euo pipefail
arm_workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
unset PYTHONHOME PYTHONPATH LD_LIBRARY_PATH QT_PLUGIN_PATH QT_QPA_PLATFORM_PLUGIN_PATH
if [[ -n "${MUJOCO_PYTHON:-}" ]]; then
    arm_candidates=("$MUJOCO_PYTHON")
else
    arm_candidates=("$arm_workspace/.venv/bin/python" "$HOME/openpi/.venv/bin/python" /usr/bin/python3)
fi
for arm_python in "${arm_candidates[@]}"; do
    if [[ -x "$arm_python" ]] && "$arm_python" -c 'import mujoco, numpy' >/dev/null 2>&1; then
        exec "$arm_python" "$@"
    fi
done
echo "No compatible MuJoCo Python found. Set MUJOCO_PYTHON=/path/to/venv/bin/python." >&2
echo "Dependencies: $arm_workspace/requirements-mujoco.txt" >&2
exit 2
