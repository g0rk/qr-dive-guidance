#!/usr/bin/env bash
# Reproduce the decode-threshold table in one command.
#
#     bash sim/tools/measure_alt.sh [/path/to/PX4-Autopilot]
#
# Brings up the MEASUREMENT world (qr_measure.sdf: six static cameras aimed at
# the pad, no aircraft), bridges their topics into ROS 2, runs measure_alt.py,
# and tears everything down again.
#
# No PX4, no MAVSDK, no flight: the cameras do not move, so the measurement is
# a property of the render and the perception code alone. That is the point --
# a dive gives one altitude per run and scatters; this gives every altitude at
# once and repeats.
set -u

PX4_DIR="${1:-$HOME/PX4-Autopilot}"
SIM_DIR="$(cd "$(dirname "$0")/../.." && pwd)"
WORLD=qr_measure
WORLD_FILE="$SIM_DIR/sim/worlds/$WORLD.sdf"

[ -f "$WORLD_FILE" ] || {
  echo "measurement world missing: $WORLD_FILE"
  echo "generate it with:"
  echo "  python3 sim/tools/build_world.py --measure \\"
  echo "      $PX4_DIR/Tools/simulation/gz/worlds/default.sdf \\"
  echo "      sim/worlds/$WORLD.sdf"
  exit 1
}

# The camera topics are read out of the world, so this script never carries a
# second copy of the altitude list either.
TOPICS=$(python3 - "$WORLD_FILE" <<'PY'
import sys, xml.etree.ElementTree as ET
w = ET.parse(sys.argv[1]).getroot().find("world")
out = []
for m in w.findall("model"):
    for s in m.iter("sensor"):
        if s.get("type") == "camera":
            t = s.find("topic")
            if t is not None and t.text.startswith("cam"):
                out.append(t.text)
print(" ".join(out))
PY
)
[ -n "$TOPICS" ] || { echo "no cam* cameras in $WORLD_FILE"; exit 1; }
echo "cameras: $TOPICS"

cleanup() {
  for pat in "g[z] sim" "parameter_bridg[e]"; do
    for p in $(pgrep -f "$pat"); do kill -9 "$p" 2>/dev/null; done
  done
}
trap cleanup EXIT
cleanup
sleep 3

export GZ_SIM_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH:-}"
source "$PX4_DIR/build/px4_sitl_default/rootfs/gz_env.sh"

echo "=== 1) gz server (headless) ==="
nohup gz sim -r -s -v1 "$PX4_GZ_WORLDS/$WORLD.sdf" > /tmp/gz_measure.log 2>&1 &
for i in $(seq 1 60); do
  gz service -l 2>/dev/null | grep -q "/world/$WORLD/create" && break
  sleep 0.5
done
echo "  ready"

echo "=== 2) camera bridges ==="
# ⚠️ `set +u` around the ROS setup: setup.bash reads AMENT_TRACE_SETUP_FILES
#    without a default, which is an error under `set -u` and would abort this
#    script silently.
set +u
source /opt/ros/humble/setup.bash
set -u
# An array, not a string: each topic spec is one argument, and the '[' in the
# gz bridge syntax must reach ros2 untouched by word splitting or globbing.
ARGS=()
for t in $TOPICS; do
  ARGS+=("/$t@sensor_msgs/msg/Image[gz.msgs.Image")
done
nohup ros2 run ros_gz_bridge parameter_bridge "${ARGS[@]}" > /tmp/bridge_measure.log 2>&1 &
sleep 8
echo "  bridged: $TOPICS"

echo "=== 3) measure ==="
cd "$SIM_DIR" || exit 1
python3 sim/tools/measure_alt.py
RC=$?
echo
echo "annotated frames: /tmp/alt/"
exit $RC
