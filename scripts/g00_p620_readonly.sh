#!/usr/bin/env bash
# G00 p620(공유 서버) 읽기 전용 조회. 컨테이너 기동·정지·변경 없음. Gemini 영상 캡처와 펌웨어 조회는 G01.
# 사용: bash scripts/g00_p620_readonly.sh   (ssh 별칭 p620 사용)
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="${ROOT}/evidence/G00/raw"; mkdir -p "${OUT}"; LOG="${OUT}/p620-inventory.log"
SSH=(ssh -o BatchMode=yes -o ConnectTimeout=8 p620)
: > "${LOG}"; echo "# p620 $(date -u +%FT%TZ)" >> "${LOG}"
for cmd in 'uname -srm; grep -E "^(NAME|VERSION)=" /etc/os-release' \
           'lsusb | grep -iE "2bc5|orbbec" || true' \
           'docker ps --format "{{.Image}} | {{.Status}} | {{.Names}}" 2>&1 | head -20 || true' \
           'docker images --format "{{.Repository}}:{{.Tag}} {{.Size}}" 2>&1 | head -20 || true' \
           'ls /dev/video* 2>&1 | head -5 || true'; do
  { echo "\$ ${cmd}"; "${SSH[@]}" "${cmd}" 2>&1 || echo "[exit=$?]"; echo; } | tee -a "${LOG}"
done
echo "로그: evidence/G00/raw/p620-inventory.log"
