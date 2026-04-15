#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

TMP_DIR="$(mktemp -d /tmp/glim_localization_run_offline_test.XXXXXX)"
trap 'rm -rf "${TMP_DIR}"' EXIT

FAKE_BIN="${TMP_DIR}/bin"
mkdir -p "${FAKE_BIN}"

cat > "${FAKE_BIN}/ros2" <<'EOF'
#!/usr/bin/env bash
set -euo pipefail

if [[ "${1:-}" == "run" && "${2:-}" == "glim_ros" && "${3:-}" == "glim_rosbag" ]]; then
  exit 0
fi

if [[ "${1:-}" == "pkg" && "${2:-}" == "prefix" && "${3:-}" == "glim_localization" ]]; then
  exit 1
fi

echo "unexpected ros2 invocation: $*" >&2
exit 1
EOF
chmod +x "${FAKE_BIN}/ros2"

export PATH="${FAKE_BIN}:$PATH"
export GLIM_LOCALIZATION_MATCHING_METHOD="gpu_vgicp"
export GLIM_LOCALIZATION_CONFIG_DIR="${REPO_ROOT}/config"

bash "${REPO_ROOT}/tools/run_offline_localization.sh" \
  "/tmp/fake_input_bag" \
  "/tmp/fake_map" \
  0 0 0 0 0 0 \
  "${TMP_DIR}/traj.txt"

echo "test_run_offline_localization_runtime passed"
