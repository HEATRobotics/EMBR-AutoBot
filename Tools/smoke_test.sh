#!/usr/bin/env bash
# Usage: tools/smoke_test.sh [seconds]   (run from anywhere, after building)
set -u
TIMEOUT=${1:-15}
# Lives in tools/; finds the ROS 2 workspace relative to itself. Override with ROS_WS=/path
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="${ROS_WS:-$SCRIPT_DIR/../embr_phys/ros2_ws}"
cd "$WS" || { echo "Workspace not found: $WS"; exit 1; }
set +u   # ROS setup files reference unset vars (AMENT_TRACE_SETUP_FILES)
source install/setup.bash
set -u
FAILED=()

mapfile -t LAUNCHES < <(find src -name "*.launch.py" -o -name "*.launch.xml" | sort)

for lf in "${LAUNCHES[@]}"; do
  echo "=== $lf ==="
  log=$(mktemp)
  timeout --signal=INT "$TIMEOUT" ros2 launch "$lf" >"$log" 2>&1 &
  pid=$!
  sleep $((TIMEOUT - 5))

  echo "-- nodes --";  ros2 node list
  echo "-- topics --"; ros2 topic list
  wait $pid

  # Only judge output before our SIGINT; shutdown noise (exit -2, KeyboardInterrupt) isn't a failure
  pre=$(sed '/user interrupted with ctrl-c\|signal_handler(SIGINT/q' "$log")
  if grep -qE "process has died|Traceback|\[ERROR\]|exit code -?[1-9]" <<<"$pre"; then
    echo "FAIL: $lf"; grep -E "process has died|Traceback|\[ERROR\]" <<<"$pre" | head -5
    FAILED+=("$lf")
  else
    echo "OK: $lf"
  fi
  rm -f "$log"
done

echo
if [ ${#FAILED[@]} -eq 0 ]; then
  echo "All ${#LAUNCHES[@]} launch files passed."
else
  echo "${#FAILED[@]} failed:"; printf '  %s\n' "${FAILED[@]}"; exit 1
fi