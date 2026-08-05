#!/usr/bin/env bash
# Simulasyon varliklarini bir PX4-Autopilot agacina kurar.
#
#     ./install.sh /yol/PX4-Autopilot
#
# Kurulanlar:
#   models/nose_cam    - burun kamerasi (GERCEK DONANIM parametreleri)
#   models/rc_cessna_cam  - rc_cessna + nose_cam (x500_mono_cam kalibi)
#   models/qr_pad         - QR pad, sartname s.17 (2x2 m + 45 derece plakalar)
#   worlds/qr_target.sdf - default dunya + qr_pad @ (500, 0)
#   airframes/4009_...    - PX4 airframe (dunya + model secimi)
set -e
PX4="${1:-$HOME/PX4-Autopilot}"
[ -d "$PX4/Tools/simulation/gz" ] || { echo "PX4 agaci bulunamadi: $PX4"; exit 1; }
HERE="$(cd "$(dirname "$0")" && pwd)"

cp -r "$HERE/models/"*  "$PX4/Tools/simulation/gz/models/"
cp    "$HERE/worlds/"*  "$PX4/Tools/simulation/gz/worlds/"
cp    "$HERE/airframes/"* "$PX4/ROMFS/px4fmu_common/init.d-posix/airframes/"
chmod +x "$PX4/ROMFS/px4fmu_common/init.d-posix/airframes/4009_gz_rc_cessna_cam"

CM="$PX4/ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt"
grep -q '4009_gz_rc_cessna_cam' "$CM" || \
  sed -i 's/\t4003_gz_rc_cessna/\t4003_gz_rc_cessna\n\t4009_gz_rc_cessna_cam/' "$CM"

echo "kuruldu. Derleme:  cd $PX4 && make px4_sitl gz_rc_cessna_cam"
