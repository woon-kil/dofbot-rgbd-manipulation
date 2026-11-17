#!/usr/bin/env bash
# G07 P-B 배치 사진용 J6 열기·대기·닫기 실행 래퍼. 사람이 입회해 직접 실행한다(무인 실행 금지).
# 승인 요청 문서: docs/gates/G07-PB-J6-연속-누름-시험-승인-요청-문서-초안.md §8-4 (사용자 승인 2026-11-17: J6만·쓰기 8·누름 없음, --check·dry run 통과 시 --real 1회)
# 사용: bash scripts/g01_pb_layout_run.sh --check      (사전 검증만, 대화·쓰기 없음)
#       bash scripts/g01_pb_layout_run.sh               (인자 없음 = dry_run: 실제 읽기만, 쓰기 0회)
#       bash scripts/g01_pb_layout_run.sh --real        (실제 쓰기. 확인 문구를 직접 입력해야 한다)
# 사전 검증: 로컬 파일 존재, 스냅샷 sha 일치, ssh 도달, Arm_Lib 있음, 팔 제어 프로세스 없음.
# 결과: evidence/G01/raw/g01_pb_layout_result-<모드>-<시각>.json (원본 파일명이 매번 달라 멱등, 덮어쓰기 없음)
# 비상: 이상 징후 시 손부터 빼고 Ctrl-C(이후 명령 없음). 전원 차단은 담당자가 판단. 이 스크립트를 다시 실행하면 그 시점의 위치를 새 S로 읽는다(S가 176~179 오므림도 99~101 열림도 아니면 시작하지 않는다).
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
H="${JETSON_HOST:-$(grep '^LG_JETSON_HOST=' "${HOME}/projects/learned-grasp-poc/.env" | cut -d= -f2- | tr -d '"')}"
MODE="dry_run"; ARGS=""; CHECK=0
[ "${1:-}" = "--real" ] && { MODE="real"; ARGS="--real"; }
[ "${1:-}" = "--check" ] && CHECK=1
TS="$(date +%y%m%d-%H%M%S)"; OUT="${ROOT}/evidence/G01/raw/g01_pb_layout_result-${MODE}-${TS}.json"; mkdir -p "${ROOT}/evidence/G01/raw"
for f in scripts/g01_pb_layout_photo.py scripts/g01_pb_press_test.py scripts/g01_p2i_gripper_test.py scripts/g01_p2h_gripper_test.py scripts/g01_p2g_gripper_test.py scripts/g01_5a_gripper_test.py scripts/gripper_adapter.py scripts/press_budget.py; do [ -f "${ROOT}/${f}" ] || { echo "파일 없음: ${f}"; exit 2; }; done
cmp -s "${ROOT}/src/control/gripper_adapter.py" "${ROOT}/scripts/gripper_adapter.py" || { echo "scripts/gripper_adapter.py가 src/control/gripper_adapter.py와 다르다. 복사 후 재실행"; exit 2; }
cmp -s "${ROOT}/src/planning/press_budget.py" "${ROOT}/scripts/press_budget.py" || { echo "scripts/press_budget.py가 src/planning/press_budget.py와 다르다. 복사 후 재실행"; exit 2; }
SNAP="${ROOT}/evidence/scripts_snapshot/261117-pbl1"
if [ -f "${SNAP}/MANIFEST.sha256" ]; then   # 스냅샷에 적힌 실행 파일의 sha가 지금 scripts/와 같은지 확인한다
  while read -r h n; do [ "$(shasum -a 256 "${ROOT}/scripts/${n}" | cut -d' ' -f1)" = "${h}" ] || { echo "스냅샷과 다른 파일: ${n}. 중단(승인 뒤 파일이 바뀌었는지 확인)"; exit 7; }; done < "${SNAP}/MANIFEST.sha256"
else echo "스냅샷 MANIFEST가 없다. 중단"; exit 7; fi
SSH=(ssh -o BatchMode=yes -o ConnectTimeout=8 "${H}")
"${SSH[@]}" 'python3 -c "import Arm_Lib" 2>/dev/null && echo Arm_Lib_ok' | grep -q Arm_Lib_ok || { echo "Jetson에서 Arm_Lib import 확인 실패(읽기 전용 확인, 생성자 미호출)"; exit 3; }
"${SSH[@]}" 'ps -eo args | grep -Ei "color_grasp|color_detect|arm_driver|dofbot_pro_(color|moveit|driver)" | grep -v grep && exit 9 || exit 0' || { echo "팔 제어 관련 프로세스가 실행 중이다. 중단"; exit 4; }
if [ "${CHECK}" = 1 ]; then echo "사전 검증 통과(대화·쓰기 없음)"; exit 0; fi
[ -t 0 ] && [ -t 1 ] || { echo "대화형 터미널(TTY)이 아니다. 사람이 입회한 별도 터미널에서 실행하라. 명령은 보내지 않았다."; exit 5; }
echo "P-B 배치 사진(J6 열기·대기·닫기), 모드: ${MODE}. 대화형 확인이 시작된다. 사람이 입회한 상태에서만 계속하라."
RD=dofbot-g01_pbl   # 모터 제어 코드는 공용 /tmp가 아니라 전용 폴더(권한 700)에서 실행한다
"${SSH[@]}" "mkdir -p ~/${RD} && chmod 700 ~/${RD} && rm -f ~/${RD}/result.json"   # 이전 실행 기록이 섞이지 않게 먼저 지운다
scp -q -o BatchMode=yes "${ROOT}/scripts/g01_pb_layout_photo.py" "${ROOT}/scripts/g01_pb_press_test.py" "${ROOT}/scripts/g01_p2i_gripper_test.py" "${ROOT}/scripts/g01_p2h_gripper_test.py" "${ROOT}/scripts/g01_p2g_gripper_test.py" "${ROOT}/scripts/g01_5a_gripper_test.py" "${ROOT}/scripts/gripper_adapter.py" "${ROOT}/scripts/press_budget.py" "${H}:${RD}/"
ssh -t -o ConnectTimeout=8 "${H}" "cd ~/${RD} && python3 g01_pb_layout_photo.py ${ARGS} --run-id ${TS} --out ~/${RD}/result.json" || echo "(시험 프로세스가 비정상 종료. 기록이 있으면 가져온다)"
scp -q -o BatchMode=yes "${H}:${RD}/result.json" "${OUT}" || { echo "기록 파일을 가져오지 못했다. Jetson의 ~/${RD}/result.json 확인 필요"; exit 6; }
python3 -c "import json,sys;j=json.load(open(sys.argv[1]));assert j['run_id']==sys.argv[2] and j['mode']==sys.argv[3], '기록의 run_id/mode가 이번 실행과 다르다';print('기록 확인: mode=',j['mode'],'ended=',j['ended'],'writes_total=',j.get('writes_total'),'gate_close=',j.get('gate_close'),'phases=',[p['phase'] for p in j.get('phases',[])])" "${OUT}" "${TS}" "${MODE}"
echo "저장: ${OUT}"
echo "비상 절차: 이 시험은 복귀·재전송 경로가 없다. 게이트 N·Ctrl-C·끊김이면 열린 채로(100) 끝난다(열린 쪽이 끼임에 안전). 소리·발열이 있으면 손부터 빼고 담당자가 전원을 차단한다."
echo "ended=left_open이면 그리퍼가 열려 있다. 닫으려면 사진·홀더·손을 치운 뒤 이 스크립트를 다시 실행한다(S=100이면 열기를 생략하고 닫기만 한다)."
