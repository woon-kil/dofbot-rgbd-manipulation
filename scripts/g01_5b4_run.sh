#!/usr/bin/env bash
# G01.5b-4 S1/S2 단일 피치 관절 시험 실행 래퍼. 사람이 입회해 직접 실행한다(무인 실행 금지).
# 사용: bash scripts/g01_5b4_run.sh --joint 4|3|2 --delta 10|20 --dir minus|plus --check   (사전 검증만)
#       bash scripts/g01_5b4_run.sh --joint 4 --delta 10 --dir minus              (dry_run: 읽기만, 쓰기 0회)
#       bash scripts/g01_5b4_run.sh --joint 4 --delta 10 --dir minus --real       (실제 쓰기. 관절·폭·방향별 확인 문구)
# 권장 순서: J4 → J3 → J2 (팔 끝에 가까운 관절부터). 관절마다 별도 실행·별도 확인.
# 대화형이므로 터미널(TTY)에서만 실행된다. TTY가 아니면 대화형 단계 전에 중단한다.
# 사전 검증: 로컬 파일 존재, ssh 도달, Jetson에 Arm_Lib 있음, 팔 제어·카메라 구독 프로세스 없음.
# 결과: evidence/G01/raw/g01_5b4_result-<모드>-<시각>.json (원본 파일명이 매번 달라 멱등, 덮어쓰기 없음)
# 비상: 이상 징후 시 손부터 빼고 Ctrl-C(이후 명령 없음). 전원 차단은 담당자가 판단.
# 이 시험은 --joint로 정한 서보 하나(4=J4, 3=J3, 2=J2, 중력 하중이 걸리는 피치 관절)만 쓴다. 관절마다 별도 실행·별도 확인 문구. 서보 1~6 읽기 1회(b-3-0)를 먼저 수행한다.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
H="${JETSON_HOST:-$(grep '^LG_JETSON_HOST=' "${HOME}/projects/learned-grasp-poc/.env" | cut -d= -f2- | tr -d '"')}"
MODE="dry_run"; ARGS=""; CHECK=0; JOINT=""; DELTA=""; DIR=""
while [ $# -gt 0 ]; do
  case "$1" in
    --real) MODE="real"; ARGS="--real";;
    --check) CHECK=1;;
    --joint) JOINT="${2:-}"; shift;;
    --delta) DELTA="${2:-}"; shift;;
    --dir) DIR="${2:-}"; shift;;
    *) echo "알 수 없는 인자: $1"; exit 2;;
  esac; shift
done
case "${JOINT}" in 4|3|2) ;; *) echo "--joint 4|3|2 가 필요하다"; exit 2;; esac
case "${DELTA}" in 10|20) ;; *) echo "--delta 10|20 이 필요하다"; exit 2;; esac
case "${DIR}" in minus|plus) ;; *) echo "--dir minus|plus 가 필요하다"; exit 2;; esac
TS="$(date +%y%m%d-%H%M%S)"; OUT="${ROOT}/evidence/G01/raw/g01_5b4_result-J${JOINT}-d${DELTA}-${DIR}-${MODE}-${TS}.json"; mkdir -p "${ROOT}/evidence/G01/raw"
for f in scripts/g01_5b4_joint_test.py scripts/gripper_adapter.py; do [ -f "${ROOT}/${f}" ] || { echo "파일 없음: ${f}"; exit 2; }; done
cmp -s "${ROOT}/src/control/gripper_adapter.py" "${ROOT}/scripts/gripper_adapter.py" || { echo "scripts/gripper_adapter.py가 src/control/gripper_adapter.py와 다르다. 복사 후 재실행"; exit 2; }
SSH=(ssh -o BatchMode=yes -o ConnectTimeout=8 "${H}")
"${SSH[@]}" 'python3 -c "import Arm_Lib" 2>/dev/null && echo Arm_Lib_ok' | grep -q Arm_Lib_ok || { echo "Jetson에서 Arm_Lib import 확인 실패(읽기 전용 확인, 생성자 미호출)"; exit 3; }
"${SSH[@]}" 'ps -eo args | grep -Ei "color_grasp|color_detect|arm_driver|dofbot_pro_(color|moveit|driver)" | grep -v grep && exit 9 || exit 0' || { echo "팔 제어 관련 프로세스가 실행 중이다. 중단"; exit 4; }
if [ "${CHECK}" = 1 ]; then echo "사전 검증 통과(대화·쓰기 없음)"; exit 0; fi
[ -t 0 ] && [ -t 1 ] || { echo "대화형 터미널(TTY)이 아니다. 사람이 입회한 별도 터미널에서 실행하라. 명령은 보내지 않았다."; exit 5; }
echo "J${JOINT} Δ${DELTA} ${DIR}, 모드: ${MODE}. 대화형 확인이 시작된다. 사람이 입회한 상태에서만 계속하라."
RD=dofbot-g01_5b4   # 모터 제어 코드는 공용 /tmp가 아니라 전용 폴더(권한 700)에서 실행한다
"${SSH[@]}" "mkdir -p ~/${RD} && chmod 700 ~/${RD} && rm -f ~/${RD}/result.json"   # 이전 실행 기록이 섞이지 않게 먼저 지운다
scp -q -o BatchMode=yes "${ROOT}/scripts/g01_5b4_joint_test.py" "${ROOT}/scripts/gripper_adapter.py" "${H}:${RD}/"
# 터미널 벨(\a)은 설정에 따라 소리가 안 날 수 있어, 이동 직전 안내 줄("[이동 N/4]")을 보면 맥에서 직접 소리를 낸다.
beep_watch() { while IFS= read -r l; do case "$l" in *"[이동 "*) afplay /System/Library/Sounds/Glass.aiff >/dev/null 2>&1 &;; esac; done; }
ssh -t -o ConnectTimeout=8 "${H}" "cd ~/${RD} && python3 g01_5b4_joint_test.py --joint ${JOINT} --delta ${DELTA} --dir ${DIR} ${ARGS} --run-id ${TS} --out ~/${RD}/result.json" | tee >(beep_watch) || echo "(시험 프로세스가 비정상 종료. 기록이 있으면 가져온다)"
scp -q -o BatchMode=yes "${H}:${RD}/result.json" "${OUT}" || { echo "기록 파일을 가져오지 못했다. Jetson의 ~/${RD}/result.json 확인 필요"; exit 6; }
python3 -c "import json,sys;j=json.load(open(sys.argv[1]));assert j['run_id']==sys.argv[2] and j['mode']==sys.argv[3], '기록의 run_id/mode가 이번 실행과 다르다';print('기록 확인: mode=',j['mode'],'ended=',j['ended'],'steps=',len(j['steps']),'returned_to_start=',j.get('returned_to_start'))" "${OUT}" "${TS}" "${MODE}"
echo "저장: ${OUT}"
echo "비상 절차: 시험은 마지막에 시작값 S로 복귀한다. 복귀 없이 끝났거나 관절에서 소리·발열·처짐이 있으면 손부터 빼고, 낙하 받침 위치를 확인한 뒤 담당자가 전원을 차단한다."
echo "이 스크립트를 다시 실행하면 그 시점의 위치를 새 S로 읽으므로 원래 위치로 돌아가지 않는다. 원래 S는 기록(JSON)의 targets[0]이다."
