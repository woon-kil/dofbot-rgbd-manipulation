#!/usr/bin/env bash
# S4-2a 시작 자세 준비: ① J1을 정면(90)으로 복구 ② J3를 93으로 재중심(접기 높이 게이트 70 mm 확보, J3 시작 92 이상 필요).
# 이유: 깊은 자세 높이가 J3 시작값에 좌우된다(J3 91 -> 68.6 mm 실패, 92 -> 71.2, 93 -> 73.9). 각 단계는 쓰기 1회(8000 ms), 안전 확인 1회, 이동 직전 소리.
# 사용: bash scripts/g01_5b4_prepare_s42.sh [--check]
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
if [ "${1:-}" = "--check" ]; then bash scripts/g01_5b4_recenter_run.sh --check; exit 0; fi
last_ended() { python3 -c "import json,glob;f=sorted(glob.glob('evidence/G01/raw/g01_5b4_recenter_result-real-*.json'))[-1];j=json.load(open(f));print(j['ended'])"; }
bash scripts/g01_5b4_recenter_run.sh --joint 1 --target 90 --real
[ "$(last_ended)" = "completed" ] || { echo "J1 복구가 completed가 아니다($(last_ended)). J3 단계로 넘어가지 않는다."; exit 1; }
echo; echo "== 다음: J3 재중심(93) =="; sleep 3
bash scripts/g01_5b4_recenter_run.sh --joint 3 --target 93 --real
[ "$(last_ended)" = "completed" ] || { echo "J3 재중심이 completed가 아니다($(last_ended))."; exit 1; }
echo "끝. 다음: bash scripts/g01_5b4_s42_run.sh --set 2a  (dry_run 먼저). 계획 게이트가 실패하면 재중심을 반복하지 말고 현재 회독부터 확인한다."
