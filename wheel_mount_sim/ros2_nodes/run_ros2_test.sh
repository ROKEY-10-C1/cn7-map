#!/usr/bin/env bash
# Phase-2 end-to-end test: Isaac Sim robot controller + fake AMR, the sequencer node, and a /Mount + /Parking log.
#   ./wheel_mount_sim/ros2_nodes/run_ros2_test.sh OUT_DIR [extra cell_mount_ros2.py args]
# Isaac and the sequencer stop when the controller finishes (all four wheels reported or --t-max).
set -u
HERE="$(cd "$(dirname "$0")/.." && pwd)"
OUT="${1:-$HERE/output/cell/ros2_test}"; shift || true
mkdir -p "$OUT"
JAZZY="source /opt/ros/jazzy/setup.bash"

bash -c "$JAZZY; exec ros2 topic echo /Mount std_msgs/msg/String" > "$OUT/mount_topic.log" 2>&1 &
ECHO1=$!
bash -c "$JAZZY; exec ros2 topic echo /Parking std_msgs/msg/String" > "$OUT/parking_topic.log" 2>&1 &
ECHO2=$!
bash -c "$JAZZY; exec python3 $HERE/ros2_nodes/mount_sequencer.py" > "$OUT/sequencer.log" 2>&1 &
SEQ=$!
"$HERE/cell_mount_ros2.sh" --no-video --out "$OUT" "$@" > "$OUT/controller.log" 2>&1
RC=$?
sleep 1
kill $SEQ $ECHO1 $ECHO2 2>/dev/null
wait 2>/dev/null
echo "controller exit $RC"
grep -E "^RESULT|^ERROR" "$OUT/controller.log"
echo "--- /Parking"; grep "data:" "$OUT/parking_topic.log"
echo "--- /Mount";   grep "data:" "$OUT/mount_topic.log"
