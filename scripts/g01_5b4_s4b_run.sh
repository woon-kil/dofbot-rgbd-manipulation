#!/usr/bin/env bash
# G01.5b-4 S4-1b 실행 래퍼: S3 순차 접기(J3 후 J4)로 가장 깊은 자세(J3≈12, J4≈12)까지 간 뒤 DaBai **노출 스윕**(고정 목록 15,25,35,50,70,100)을 촬영하고 같은 계단으로 복귀한다. 사람이 입회해 직접 실행한다(무인 실행 금지).
# 사용: bash scripts/g01_5b4_s4b_run.sh --check | (dry_run: 읽기만, 촬영 없음) | --real
# --real 이면 DaBai 드라이버를 먼저 띄우고(약 12초) 끝나면 종료한다. 촬영 결과: evidence/G01/raw/s4b/<시각>/ (exp_<값>.png, expsweep.json, 로그)
# 대화형이므로 터미널(TTY)에서만 실행된다. 이동 직전 안내 줄("[이동 N/16]")을 보면 맥에서 afplay로 소리를 낸다.
# 비상: 이상 징후 시 손부터 빼고 Ctrl-C(3초 대기 중인 다음 이동만 취소, 이미 보낸 이동은 서보가 끝까지 간다). SDK에 정지 함수는 없다. 전원 차단은 담당자가 판단.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
H="${JETSON_HOST:-$(grep '^LG_JETSON_HOST=' "${HOME}/projects/learned-grasp-poc/.env" | cut -d= -f2- | tr -d '"')}"
MODE="dry_run"; ARGS=""; CHECK=0; GATE=""
while [ $# -gt 0 ]; do
  case "$1" in
    --real) MODE="real"; ARGS="--real";;
    --check) CHECK=1;;
    *) echo "알 수 없는 인자: $1"; exit 2;;
  esac; shift
done
TS="$(date +%y%m%d-%H%M%S)"; KIND=s4b; OUT="${ROOT}/evidence/G01/raw/g01_5b4_${KIND}_result-${MODE}-${TS}.json"; mkdir -p "${ROOT}/evidence/G01/raw"
for f in scripts/g01_5b4_s3seq_test.py scripts/g01_5b4_joint_test.py scripts/gripper_adapter.py; do [ -f "${ROOT}/${f}" ] || { echo "파일 없음: ${f}"; exit 2; }; done
cmp -s "${ROOT}/src/control/gripper_adapter.py" "${ROOT}/scripts/gripper_adapter.py" || { echo "scripts/gripper_adapter.py가 src/control/gripper_adapter.py와 다르다. 복사 후 재실행"; exit 2; }
SSH=(ssh -o BatchMode=yes -o ConnectTimeout=8 "${H}")
"${SSH[@]}" 'python3 -c "import Arm_Lib" 2>/dev/null && echo Arm_Lib_ok' | grep -q Arm_Lib_ok || { echo "Jetson에서 Arm_Lib import 확인 실패(읽기 전용 확인, 생성자 미호출)"; exit 3; }
"${SSH[@]}" 'ps -eo args | grep -Ei "color_grasp|color_detect|arm_driver|dofbot_pro_(color|moveit|driver)" | grep -v grep && exit 9 || exit 0' || { echo "팔 제어 관련 프로세스가 실행 중이다. 중단"; exit 4; }
if [ "${CHECK}" = 1 ]; then echo "사전 검증 통과(대화·쓰기 없음)"; exit 0; fi
[ -t 0 ] && [ -t 1 ] || { echo "대화형 터미널(TTY)이 아니다. 사람이 입회한 별도 터미널에서 실행하라. 명령은 보내지 않았다."; exit 5; }
echo "S4-1b(S3 순차 + DaBai 노출 스윕), 모드: ${MODE}. 대화형 확인이 시작된다. 사람이 입회한 상태에서만 계속하라."
RD=dofbot-g01_5b4   # 모터 제어 코드는 공용 /tmp가 아니라 전용 폴더(권한 700)에서 실행한다
"${SSH[@]}" "mkdir -p ~/${RD} && chmod 700 ~/${RD} && rm -f ~/${RD}/result.json"   # 이전 실행 기록이 섞이지 않게 먼저 지운다
scp -q -o BatchMode=yes "${ROOT}/scripts/g01_dabai_probe.py" "${ROOT}/scripts/g01_dabai_expsweep.py" "${ROOT}/scripts/g01_5b4_s3seq_test.py" "${ROOT}/scripts/g01_5b4_joint_test.py" "${ROOT}/scripts/gripper_adapter.py" "${H}:${RD}/"
SRC='source /opt/ros/humble/setup.bash; source ~/dofbot_pro_ws/install/setup.bash'
stop_driver() { "${SSH[@]}" 'kill -INT $(cat /tmp/g01_dabai.pid 2>/dev/null) 2>/dev/null; sleep 2; pkill -INT -f "[_]_ns:=/dabai" 2>/dev/null; sleep 2; pgrep -fa "[_]_ns:=/dabai" >/dev/null && echo "DaBai 드라이버 남음" || echo "DaBai 드라이버 없음"' 2>&1 | tail -1; }
CAP=""
if [ "${MODE}" = real ]; then
  "${SSH[@]}" 'pgrep -f "[_]_ns:=/dabai" >/dev/null && { echo "이미 DaBai 드라이버가 떠 있다"; exit 3; } || true' || { echo "DaBai 드라이버가 이미 실행 중이다. 중단"; exit 3; }
  trap stop_driver EXIT
  "${SSH[@]}" "${SRC}; nohup ros2 launch orbbec_camera dabai_dcw2.launch.py camera_name:=dabai enable_point_cloud:=false enable_colored_point_cloud:=false enable_ir:=false > /tmp/g01_dabai_drv.log 2>&1 & echo \$! > /tmp/g01_dabai.pid; sleep 12; pgrep -f '[_]_ns:=/dabai' >/dev/null && echo DaBai드라이버시작" | grep -q DaBai드라이버시작 || { echo "DaBai 드라이버 시작 실패. 중단(팔은 움직이지 않았다)"; exit 8; }
  echo "DaBai 드라이버 시작 확인"
  CAP="--capture --sweep 15,25,35,50,70,100"
fi
# 터미널 벨(\a)은 설정에 따라 소리가 안 날 수 있어, 이동 직전 안내 줄("[이동 N/8]")을 보면 맥에서 직접 소리를 낸다.
beep_watch() { while IFS= read -r l; do case "$l" in *"[이동 "*) afplay /System/Library/Sounds/Glass.aiff >/dev/null 2>&1 &;; esac; done; }
ssh -t -o ConnectTimeout=8 "${H}" "cd ~/${RD} && python3 g01_5b4_s3seq_test.py ${ARGS} ${GATE} ${CAP} --run-id ${TS} --out ~/${RD}/result.json" | tee >(beep_watch) || echo "(시험 프로세스가 비정상 종료. 기록이 있으면 가져온다)"
if [ "${MODE}" = real ]; then
  SOUT="${ROOT}/evidence/G01/raw/s4b/${TS}"; mkdir -p "${SOUT}"
  "${SSH[@]}" 'ls /tmp/g01_s4_view 2>/dev/null' | while read f; do scp -q -o BatchMode=yes "${H}:/tmp/g01_s4_view/${f}" "${SOUT}/" || echo "촬영 파일 복사 실패: ${f}"; done
  "${SSH[@]}" 'cat /tmp/g01_s4_sweep.log 2>/dev/null' > "${SOUT}/sweep.log" || true
  "${SSH[@]}" 'cat /tmp/g01_dabai_drv.log' > "${SOUT}/driver.log" 2>&1 || true
  echo "촬영 저장: ${SOUT}"
fi
scp -q -o BatchMode=yes "${H}:${RD}/result.json" "${OUT}" || { echo "기록 파일을 가져오지 못했다. Jetson의 ~/${RD}/result.json 확인 필요"; exit 6; }
python3 -c "import json,sys;j=json.load(open(sys.argv[1]));assert j['run_id']==sys.argv[2] and j['mode']==sys.argv[3], '기록의 run_id/mode가 이번 실행과 다르다';print('기록 확인: mode=',j['mode'],'ended=',j['ended'],'기록 항목=',len(j['steps']),'returned_to_start=',j.get('returned_to_start'))" "${OUT}" "${TS}" "${MODE}"
echo "저장: ${OUT}"
echo "비상 절차: 시험은 마지막에 시작값 S로 복귀한다. 복귀 없이 끝났거나 관절에서 소리·발열·처짐이 있으면 손부터 빼고, 낙하 받침 위치를 확인한 뒤 담당자가 전원을 차단한다."
echo "이 스크립트를 다시 실행하면 그 시점의 위치를 새 S로 읽으므로 원래 위치로 돌아가지 않는다. 원래 시작 자세는 기록(JSON)의 state0(J1~J6 시작 회독)에 있다."
