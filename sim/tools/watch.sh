#!/usr/bin/env bash
# One command to WATCH the simulation.
#
#     bash sim/tools/watch.sh
#
# Two windows open (straight onto the Windows desktop, thanks to WSLg):
#   1. the gz sim GUI  - the 3D world, the aircraft, the QR pad
#   2. the perception window - what the aircraft's CAMERA sees, with the
#      target-area box and the QR quadrilateral drawn on it
#      (because config.QR_SHOW_WINDOW is True)
#
# ⚠️ NOT FOR MEASUREMENT. The cost of GUI rendering slows the lockstep down,
#    so the timing numbers come out wrong. For margin measurements use the
#    measurement scripts in sim/tools/ (no GUI).

set -u
PX4_DIR="$HOME/PX4-Autopilot"
SIM_DIR="$(cd "$(dirname "$0")/../.." && pwd)"
WORLD=qr_target

echo "=== 0) cleaning up old processes ==="
for p in $(pgrep -f "bin/p[x]4"); do kill -9 "$p" 2>/dev/null; done
for p in $(pgrep -f "g[z] sim"); do kill -9 "$p" 2>/dev/null; done
for p in $(pgrep -f "python3 mai[n].py"); do kill "$p" 2>/dev/null; done
for p in $(pgrep -f "ros_camer[a]"); do kill "$p" 2>/dev/null; done
for p in $(pgrep -f "parameter_bridg[e]"); do kill "$p" 2>/dev/null; done
sleep 5

export GZ_SIM_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH:-}"
source "$PX4_DIR/build/px4_sitl_default/rootfs/gz_env.sh"
export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA   # use the dGPU, not the iGPU, under WSL2

echo "=== 1) gz server ==="
nohup gz sim -r -s "$PX4_GZ_WORLDS/$WORLD.sdf" > /tmp/gz_sim.log 2>&1 &
for i in $(seq 1 40); do
  gz service -l 2>/dev/null | grep -q "/world/$WORLD/create" && break
  sleep 0.5
done
echo "  ready"

echo "=== 2) gz GUI (3D window) ==="
nohup gz sim -g > /tmp/gz_gui.log 2>&1 &
sleep 6

echo "=== 3) PX4 ==="
export PX4_GZ_STANDALONE=1
export PX4_SIM_MODEL=gz_rc_cessna_cam
export PX4_SYS_AUTOSTART=4009
export PX4_GZ_WORLD="$WORLD"
cd "$PX4_DIR/build/px4_sitl_default/src/modules/simulation/gz_bridge" || exit 1
nohup "$PX4_DIR/build/px4_sitl_default/bin/px4" > /tmp/px4_sitl.log 2>&1 &
sleep 10

echo "=== 4) camera bridge ==="
# ⚠️ `set +u` around the ROS setup, and it is NOT cosmetic. ROS 2's
#    setup.bash reads AMENT_TRACE_SETUP_FILES without a default, which is an
#    unbound-variable error under the `set -u` at the top of this file -- and
#    a failing `source` aborts the whole script. This file therefore used to
#    die right here, silently: gz and PX4 were left running, but the camera
#    bridge, the mission and the perception window never started at all.
set +u
source /opt/ros/humble/setup.bash
set -u
nohup ros2 run ros_gz_bridge parameter_bridge \
      "/camera@sensor_msgs/msg/Image[gz.msgs.Image" > /tmp/bridge.log 2>&1 &
sleep 6

echo "=== 5) EKF convergence (60 s) ==="
echo "    (the aircraft WILL NOT ARM before this: 'height estimate not stable')"
sleep 60

echo "=== 6) mission + perception window ==="
cd "$SIM_DIR" || exit 1
nohup python3 main.py > /tmp/mission.log 2>&1 &
for i in $(seq 1 40); do
  grep -qa "Entered IDLE state" /tmp/mission.log 2>/dev/null && break
  sleep 0.5
done
nohup python3 ros_camera.py > /tmp/roscam.log 2>&1 &
sleep 4

echo
echo "======================================================================"
echo "  READY. Send the commands to start the mission:"
echo "     python3 sim/tools/command.py takeoff"
echo "     ... once it reaches HOLD ..."
echo "     python3 sim/tools/command.py align"
echo
echo "  To also record the camera feed to MP4:"
echo "     python3 sim/tools/flight_video.py 240"
echo
echo "  Logs: /tmp/mission.log  /tmp/px4_sitl.log  /tmp/gz_sim.log"
echo "======================================================================"
