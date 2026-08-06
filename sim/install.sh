#!/usr/bin/env bash
# Install the simulation assets into a PX4-Autopilot tree.
#
#     ./install.sh /path/to/PX4-Autopilot
#
# What gets installed:
#   models/nose_cam       - the nose camera (REAL HARDWARE parameters)
#   models/rc_cessna_cam  - rc_cessna + nose_cam (patterned on x500_mono_cam)
#   models/qr_pad         - the QR pad, rulebook p.17 (2x2 m + 45 degree plates)
#   worlds/qr_target.sdf  - the default world + qr_pad @ (500, 0)
#   airframes/4009_...    - PX4 airframe (selects the world and the model)
set -e
PX4="${1:-$HOME/PX4-Autopilot}"
[ -d "$PX4/Tools/simulation/gz" ] || { echo "PX4 tree not found: $PX4"; exit 1; }
HERE="$(cd "$(dirname "$0")" && pwd)"

cp -r "$HERE/models/"*  "$PX4/Tools/simulation/gz/models/"
cp    "$HERE/worlds/"*  "$PX4/Tools/simulation/gz/worlds/"
cp    "$HERE/airframes/"* "$PX4/ROMFS/px4fmu_common/init.d-posix/airframes/"
chmod +x "$PX4/ROMFS/px4fmu_common/init.d-posix/airframes/4009_gz_rc_cessna_cam"

CM="$PX4/ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt"
grep -q '4009_gz_rc_cessna_cam' "$CM" || \
  sed -i 's/\t4003_gz_rc_cessna/\t4003_gz_rc_cessna\n\t4009_gz_rc_cessna_cam/' "$CM"

echo "installed. To build:  cd $PX4 && make px4_sitl gz_rc_cessna_cam"
