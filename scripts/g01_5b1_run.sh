#!/usr/bin/env bash
# G01.5b-1 손목 회전(J5) 소각도 시험 실행 래퍼. 사람이 입회해 직접 실행한다(무인 실행 금지).
# 사용: bash scripts/g01_5b1_run.sh --check        (사전 검증만, 대화·쓰기 없음)
#       bash scripts/g01_5b1_run.sh                (인자 없음 = dry_run: 실제 읽기만, 쓰기 0회, 도착 검사 생략)
#       bash scripts/g01_5b1_run.sh --real         (실제 쓰기. 확인 문구를 직접 입력해야 한다)
# 대화형이므로 터미널(TTY)에서만 실행된다. TTY가 아니면 대화형 단계 전에 중단한다.
# 사전 검증: 로컬 파일 존재, ssh 도달, Jetson에 Arm_Lib 있음, 팔 제어·카메라 구독 프로세스 없음.
# 결과: evidence/G01/raw/g01_5b1_result-<모드>-<시각>.json (원본 파일명이 매번 달라 멱등, 덮어쓰기 없음)
# 비상: 이상 징후 시 손부터 빼고 Ctrl-C(이후 명령 없음). 전원 차단은 담당자가 판단.
# 이 시험은 서보 5(J5)만 쓴다. 서보 1~6 읽기 1회(b-1-0)를 먼저 수행한다. 어댑터는 서보 id 5·6 외를 거부한다.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
H="${JETSON_HOST:-$(grep '^LG_JETSON_HOST=' "${HOME}/projects/learned-grasp-poc/.env" | cut -d= -f2- | tr -d '"')}"
MODE="dry_run"; ARGS=""; CHECK=0
[ "${1:-}" = "--real" ] && { MODE="real"; ARGS="--real"; }
[ "${1:-}" = "--check" ] && CHECK=1
TS="$(date +%y%m%d-%H%M%S)"; OUT="${ROOT}/evidence/G01/raw/g01_5b1_result-${MODE}-${TS}.json"; mkdir -p "${ROOT}/evidence/G01/raw"
for f in scripts/g01_5b1_joint_test.py scripts/gripper_adapter.py; do [ -f "${ROOT}/${f}" ] || { echo "파일 없음: ${f}"; exit 2; }; done
cmp -s "${ROOT}/src/control/gripper_adapter.py" "${ROOT}/scripts/gripper_adapter.py" || { echo "scripts/gripper_adapter.py가 src/control/gripper_adapter.py와 다르다. 복사 후 재실행"; exit 2; }
SSH=(ssh -o BatchMode=yes -o ConnectTimeout=8 "${H}")
"${SSH[@]}" 'python3 -c "import Arm_Lib" 2>/dev/null && echo Arm_Lib_ok' | grep -q Arm_Lib_ok || { echo "Jetson에서 Arm_Lib import 확인 실패(읽기 전용 확인, 생성자 미호출)"; exit 3; }
"${SSH[@]}" 'ps -eo args | grep -Ei "color_grasp|color_detect|arm_driver|dofbot_pro_(color|moveit|driver)" | grep -v grep && exit 9 || exit 0' || { echo "팔 제어 관련 프로세스가 실행 중이다. 중단"; exit 4; }
if [ "${CHECK}" = 1 ]; then echo "사전 검증 통과(대화·쓰기 없음)"; exit 0; fi
[ -t 0 ] && [ -t 1 ] || { echo "대화형 터미널(TTY)이 아니다. 사람이 입회한 별도 터미널에서 실행하라. 명령은 보내지 않았다."; exit 5; }
echo "모드: ${MODE}. 대화형 확인이 시작된다. 사람이 입회한 상태에서만 계속하라."
RD=dofbot-g01_5b1   # 모터 제어 코드는 공용 /tmp가 아니라 전용 폴더(권한 700)에서 실행한다
"${SSH[@]}" "mkdir -p ~/${RD} && chmod 700 ~/${RD} && rm -f ~/${RD}/result.json"   # 이전 실행 기록이 섞이지 않게 먼저 지운다
scp -q -o BatchMode=yes "${ROOT}/scripts/g01_5b1_joint_test.py" "${ROOT}/scripts/gripper_adapter.py" "${H}:${RD}/"
ssh -t -o ConnectTimeout=8 "${H}" "cd ~/${RD} && python3 g01_5b1_joint_test.py ${ARGS} --run-id ${TS} --out ~/${RD}/result.json" || echo "(시험 프로세스가 비정상 종료. 기록이 있으면 가져온다)"
scp -q -o BatchMode=yes "${H}:${RD}/result.json" "${OUT}" || { echo "기록 파일을 가져오지 못했다. Jetson의 ~/${RD}/result.json 확인 필요"; exit 6; }
python3 -c "import json,sys;j=json.load(open(sys.argv[1]));assert j['run_id']==sys.argv[2] and j['mode']==sys.argv[3], '기록의 run_id/mode가 이번 실행과 다르다';print('기록 확인: mode=',j['mode'],'ended=',j['ended'],'steps=',len(j['steps']),'returned_to_start=',j.get('returned_to_start'))" "${OUT}" "${TS}" "${MODE}"
echo "저장: ${OUT}"
echo "비상 절차: 시험은 마지막에 시작값 S로 복귀한다. 복귀 없이 끝났거나 손목에서 소리·발열이 있으면 손부터 빼고, 낙하 받침 위치를 확인한 뒤 담당자가 전원을 차단한다."
echo "이 스크립트를 다시 실행하면 그 시점의 위치를 새 S로 읽으므로 원래 위치로 돌아가지 않는다. 원래 S는 기록(JSON)의 targets[0]이다."
