#!/usr/bin/env bash
# DaBai 컬러 노출 스윕(로봇 명령 없음). 드라이버 기동 → 노출값마다 프레임 저장 → 종료·확인. 사용: bash scripts/g01_dabai_expsweep.sh 값1,값2,... [이름]
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
H="${JETSON_HOST:-$(grep '^LG_JETSON_HOST=' "${HOME}/projects/learned-grasp-poc/.env" | cut -d= -f2- | tr -d '"')}"
VALS="${1:?노출값 목록(쉼표)}"; NAME="${2:-sweep}"
SSH=(ssh -o BatchMode=yes -o ConnectTimeout=8 "${H}")
OUT="${ROOT}/evidence/G01/raw/dabai_exp/${NAME}"; mkdir -p "${OUT}"
SRC='source /opt/ros/humble/setup.bash; source ~/dofbot_pro_ws/install/setup.bash'
stop_driver() { "${SSH[@]}" 'kill -INT $(cat /tmp/g01_dabai.pid 2>/dev/null) 2>/dev/null; sleep 2; pkill -INT -f "[_]_ns:=/dabai" 2>/dev/null; sleep 2; pgrep -fa "[_]_ns:=/dabai" >/dev/null && echo "드라이버 남음" || echo "드라이버 없음"' 2>&1 | tail -1; }
trap stop_driver EXIT
"${SSH[@]}" 'pgrep -f "[_]_ns:=/dabai" >/dev/null && { echo "이미 드라이버가 떠 있다"; exit 3; } || true'
scp -q -o BatchMode=yes "${ROOT}/scripts/g01_dabai_probe.py" "${ROOT}/scripts/g01_dabai_expsweep.py" "${H}:/tmp/"
"${SSH[@]}" "${SRC}; nohup ros2 launch orbbec_camera dabai_dcw2.launch.py camera_name:=dabai enable_point_cloud:=false enable_colored_point_cloud:=false enable_ir:=false > /tmp/g01_dabai_drv.log 2>&1 & echo \$! > /tmp/g01_dabai.pid; sleep 12; pgrep -f '[_]_ns:=/dabai' >/dev/null && echo 드라이버시작"
"${SSH[@]}" "${SRC}; rm -rf /tmp/g01_expsweep; python3 /tmp/g01_dabai_expsweep.py /tmp/g01_expsweep ${VALS} dabai"
"${SSH[@]}" 'ls /tmp/g01_expsweep' | while read f; do scp -q -o BatchMode=yes "${H}:/tmp/g01_expsweep/${f}" "${OUT}/"; done
echo "저장: evidence/G01/raw/dabai_exp/${NAME}"
