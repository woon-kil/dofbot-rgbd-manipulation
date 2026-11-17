#!/usr/bin/env bash
# P-A: 상시 드라이버에서 사진 1장(요청 이후 노출된 프레임)을 받아 Mac에서 id 7 검출까지 한다. 로봇 명령 없음.
# 사용: bash scripts/pa_snap_mac.sh <이름>     (이름: 영문·숫자·_·-)   전제: pa_driver.sh start 후, p620 ~/dofbot-g01-work/pa_snap.py·g01_gemini_probe.py 있음
# 전송: ssh·scp는 같은 ControlMaster 연결을 재사용하고(첫 호출이 연결을 만든다) 파일 3개는 scp 1회, 크기·요청 id 대조와 검출은 python 1회다(측정 전 설계: 들어올림 러너도 같은 전송 경로를 써야 측정값이 유효하다).
# 단계 시각은 Mac 시계(python3 time.time())로 data/g04/pa/<이름>.mac_timing.txt에 남긴다(계측 실패는 흐름을 바꾸지 않는다, || true). M1 끝점 = m_detect_done(검출 결과).
# 결과: data/g04/pa/<이름>.{color.png,depth_mm.npy,meta.json,detect.json,req_id.txt,mac_timing.txt}. 실패하면 부분 결과를 남기지 않는다(옛 파일 없음).
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
NAME="${1:?이름 필요}"; [[ "${NAME}" =~ ^[A-Za-z0-9_-]+$ ]] || { echo "이름은 영문·숫자·_·- 만 가능: ${NAME}"; exit 2; }
PY="${P1_CHECK_PY:-${ROOT}/.venv/bin/python}"
SO=(-o BatchMode=yes -o ConnectTimeout=8 -o ControlMaster=auto -o "ControlPath=/tmp/pa-ssh-%C" -o ControlPersist=120)
D="${ROOT}/data/g04/pa"; mkdir -p "${D}"; MT="${D}/${NAME}.mac_timing.txt"
[ ! -e "${D}/${NAME}.color.png" ] || { echo "이미 있음: ${NAME} (다른 이름을 쓴다)"; exit 2; }
: > "${MT}"
M() { echo "$1 $(python3 -c 'import time;print("%.3f"%time.time())' 2>/dev/null)" >> "${MT}" 2>/dev/null || true; }
M m_start
REQ="${NAME}-$(python3 -c 'import time;print(int(time.time()*1000))')"; echo "${REQ}" > "${D}/${NAME}.req_id.txt"
fail() { rm -f "${D}/${NAME}.color.png" "${D}/${NAME}.depth_mm.npy" "${D}/${NAME}.meta.json" "${D}/${NAME}.detect.json"; echo "실패: $1"; exit "${2:-4}"; }
M m_ssh_snap_start
ssh "${SO[@]}" p620 "docker exec dofbot-g01-pa bash -lc 'set +u; source /opt/ros/humble/setup.bash; source /opt/cad-less-install/setup.bash; set -u; cd /work && python3 pa_snap.py /work/pa/${NAME} ${REQ} 5'" > /dev/null || fail "p620 snap 실행(드라이버가 꺼졌거나 프레임 없음)" 3
M m_ssh_snap_done
TMP="${D}/.${NAME}.tmp"; rm -rf "${TMP}"; mkdir -p "${TMP}"
scp -q "${SO[@]}" "p620:dofbot-g01-work/pa/${NAME}/color.png" "p620:dofbot-g01-work/pa/${NAME}/depth_mm.npy" "p620:dofbot-g01-work/pa/${NAME}/meta.json" "${TMP}/" || { rm -rf "${TMP}"; fail "scp"; }
M m_scp_done
for f in color.png depth_mm.npy meta.json; do mv "${TMP}/${f}" "${D}/${NAME}.${f}"; done; rmdir "${TMP}"
"${PY}" -I "${ROOT}/scripts/pa_detect.py" "${D}/${NAME}.color.png" "${D}/${NAME}.meta.json" "${REQ}" > "${D}/${NAME}.detect.json" || fail "사진 확인·요청 id 대조·검출"
M m_detect_done
echo "OK ${NAME} $(cat "${D}/${NAME}.detect.json")"
