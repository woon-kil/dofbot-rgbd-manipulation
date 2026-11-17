#!/usr/bin/env bash
# G01 DaBai 취득(Jetson). 로봇 명령 없음. 드라이버 기동 → 안정화 대기 → 프로브 → 종료·확인.
# 사용: JETSON_HOST=user@host bash scripts/g01_dabai.sh <설정이름: default|registered>
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
H="${JETSON_HOST:?JETSON_HOST 없음}"; CFG="${1:?default|registered}"
SSH=(ssh -o BatchMode=yes -o ConnectTimeout=8 "${H}")
OUT="${ROOT}/evidence/G01/raw/dabai/${CFG}"; mkdir -p "${OUT}"
case "${CFG}" in default) EXTRA="";; registered) EXTRA="depth_registration:=true";; *) echo "설정 이름 오류"; exit 2;; esac
SRC='source /opt/ros/humble/setup.bash; source ~/dofbot_pro_ws/install/setup.bash'
stop_driver() { "${SSH[@]}" 'kill -INT $(cat /tmp/g01_dabai.pid 2>/dev/null) 2>/dev/null; sleep 2; pkill -INT -f "[_]_ns:=/dabai" 2>/dev/null; sleep 2; pgrep -fa "[_]_ns:=/dabai" || echo "드라이버 없음"' | tee -a "${OUT}/driver-stop.log"; }
trap stop_driver EXIT
"${SSH[@]}" 'pgrep -f "[_]_ns:=/dabai" >/dev/null && { echo "이미 드라이버가 떠 있다"; exit 3; } || true'
scp -q -o BatchMode=yes "${ROOT}/scripts/g01_dabai_probe.py" "${H}:/tmp/g01_dabai_probe.py"
"${SSH[@]}" "${SRC}; nohup ros2 launch orbbec_camera dabai_dcw2.launch.py camera_name:=dabai enable_point_cloud:=false enable_colored_point_cloud:=false enable_ir:=false ${EXTRA} > /tmp/g01_dabai_drv.log 2>&1 & echo \$! > /tmp/g01_dabai.pid; sleep 12; pgrep -f '[_]_ns:=/dabai' >/dev/null && echo 드라이버시작"
"${SSH[@]}" "${SRC}; python3 /tmp/g01_dabai_probe.py /tmp/g01_dabai_view_${CFG} 15 dabai > /dev/null; echo probe_exit=\$?"
for f in color.png depth.png probe.json; do scp -q -o BatchMode=yes "${H}:/tmp/g01_dabai_view_${CFG}/${f}" "${OUT}/"; done
"${SSH[@]}" 'cat /tmp/g01_dabai_drv.log' > "${OUT}/driver.log" 2>&1 || true
"${SSH[@]}" 'timedatectl show -p NTPSynchronized -p TimeUSec' > "${OUT}/clock.txt" 2>&1 || true
echo "저장: evidence/G01/raw/dabai/${CFG}"
