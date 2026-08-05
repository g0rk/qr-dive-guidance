#!/usr/bin/env bash
# Simulasyonu GOZLE IZLEMEK icin tek komut.
#
#     bash sim/tools/izle.sh
#
# Iki pencere acilir (WSLg sayesinde dogrudan Windows masaustunde):
#   1. gz sim GUI  - 3 boyutlu dunya, ucak, QR pad
#   2. algi penceresi - ucagin KAMERASINDAN gordugu, AV kutusu ve QR
#      dortgeni cizili hali (config.QR_SHOW_WINDOW=True oldugu icin)
#
# ⚠️ OLCUM ICIN DEGIL. GUI render maliyeti lockstep'i yavaslattigi icin
#    zamanlama sayilari bozulur. Marj olcumu icin sim/tools/ olcum
#    betiklerini kullan (GUI'siz).

set -u
PX4_DIR="$HOME/PX4-Autopilot"
SIM_DIR="$(cd "$(dirname "$0")/../.." && pwd)"
WORLD=qr_target

echo "=== 0) eski surecler temizleniyor ==="
for p in $(pgrep -f "bin/p[x]4"); do kill -9 "$p" 2>/dev/null; done
for p in $(pgrep -f "g[z] sim"); do kill -9 "$p" 2>/dev/null; done
for p in $(pgrep -f "python3 mai[n].py"); do kill "$p" 2>/dev/null; done
for p in $(pgrep -f "ros_camer[a]"); do kill "$p" 2>/dev/null; done
for p in $(pgrep -f "parameter_bridg[e]"); do kill "$p" 2>/dev/null; done
sleep 5

export GZ_SIM_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH:-}"
source "$PX4_DIR/build/px4_sitl_default/rootfs/gz_env.sh"
export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA   # WSL2'de iGPU yerine RTX

echo "=== 1) gz sunucusu ==="
nohup gz sim -r -s "$PX4_GZ_WORLDS/$WORLD.sdf" > /tmp/gz_sim.log 2>&1 &
for i in $(seq 1 40); do
  gz service -l 2>/dev/null | grep -q "/world/$WORLD/create" && break
  sleep 0.5
done
echo "  hazir"

echo "=== 2) gz GUI (3B pencere) ==="
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

echo "=== 4) kamera koprusu ==="
source /opt/ros/humble/setup.bash
nohup ros2 run ros_gz_bridge parameter_bridge \
      "/camera@sensor_msgs/msg/Image[gz.msgs.Image" > /tmp/bridge.log 2>&1 &
sleep 6

echo "=== 5) EKF yakinsamasi (60 sn) ==="
echo "    (bu sure gecmeden ucak ARM OLMAZ: 'height estimate not stable')"
sleep 60

echo "=== 6) gorev + algi penceresi ==="
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
echo "  HAZIR. Simdi gorevi baslatmak icin komutlari gonder:"
echo "     python3 sim/tools/komut.py takeoff"
echo "     ... HOLD'a ulasinca ..."
echo "     python3 sim/tools/komut.py align"
echo
echo "  Kamera goruntusunu ayrica MP4'e almak istersen:"
echo "     python3 sim/tools/ucus_videosu.py 240"
echo
echo "  Loglar: /tmp/mission.log  /tmp/px4_sitl.log  /tmp/gz_sim.log"
echo "======================================================================"
