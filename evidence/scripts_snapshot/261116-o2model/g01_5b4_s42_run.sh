#!/usr/bin/env bash
# G01.5b-4 S4-2 실행 래퍼: 실제 시작 회독으로 Mac에서 이동 계획을 만들고(FK 게이트 통과 필수) 그 계획대로 한 번에 한 관절씩 접기·파일럿 자세·복귀를 실행하며 자세마다 DaBai 노출 스윕을 촬영한다.
# 사용: bash scripts/g01_5b4_s42_run.sh --set 2a|2b|3|4|5 --check | --set 2a (dry_run: 읽기만) | --set 2a --real
# 대화형이므로 터미널(TTY)에서만 실행된다. 이동 직전 안내 줄("[이동 N/M]")을 보면 맥에서 afplay로 소리를 낸다. 계획 게이트가 실패하면 팔은 움직이지 않는다.
# 비상: 이상 징후 시 손부터 빼고 Ctrl-C(대기 중인 다음 이동만 취소, 이미 보낸 이동은 서보가 끝까지 간다). SDK에 정지 함수는 없다. 전원 차단은 담당자가 판단.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
H="${JETSON_HOST:-$(grep '^LG_JETSON_HOST=' "${HOME}/projects/learned-grasp-poc/.env" | cut -d= -f2- | tr -d '"')}"
MODE="dry_run"; ARGS=""; CHECK=0; SET=""
while [ $# -gt 0 ]; do
  case "$1" in
    --real) MODE="real"; ARGS="--real";;
    --check) CHECK=1;;
    --set) SET="${2:-}"; shift;;
    *) echo "알 수 없는 인자: $1"; exit 2;;
  esac; shift
done
TS="$(date +%y%m%d-%H%M%S)"; case "${SET}" in 2a|2b|3|4|5|6|7|8|9|10|11|12|14|15|16|17|18|19|20|21|22) ;; *) echo "--set 2a|2b|3|4|5|6|7|8|9|10|11|12|14|15|16|17|18|19|20|21|22 가 필요하다"; exit 2;; esac; KIND="s42${SET}"; OUT="${ROOT}/evidence/G01/raw/g01_5b4_${KIND}_result-${MODE}-${TS}.json"; mkdir -p "${ROOT}/evidence/G01/raw"
for f in scripts/g01_5b4_s42_test.py scripts/g01_s42_plan.py scripts/g01_s4_2_path.py scripts/g01_s3_path.py scripts/g01_pose_generator.py scripts/g01_5b4_joint_test.py scripts/gripper_adapter.py; do [ -f "${ROOT}/${f}" ] || { echo "파일 없음: ${f}"; exit 2; }; done
cmp -s "${ROOT}/src/control/gripper_adapter.py" "${ROOT}/scripts/gripper_adapter.py" || { echo "scripts/gripper_adapter.py가 src/control/gripper_adapter.py와 다르다. 복사 후 재실행"; exit 2; }
SSH=(ssh -o BatchMode=yes -o ConnectTimeout=8 "${H}")
"${SSH[@]}" 'python3 -c "import Arm_Lib" 2>/dev/null && echo Arm_Lib_ok' | grep -q Arm_Lib_ok || { echo "Jetson에서 Arm_Lib import 확인 실패(읽기 전용 확인, 생성자 미호출)"; exit 3; }
"${SSH[@]}" 'ps -eo args | grep -Ei "color_grasp|color_detect|arm_driver|dofbot_pro_(color|moveit|driver)" | grep -v grep && exit 9 || exit 0' || { echo "팔 제어 관련 프로세스가 실행 중이다. 중단"; exit 4; }
if [ "${CHECK}" = 1 ]; then echo "사전 검증 통과(대화·쓰기 없음)"; exit 0; fi
[ -t 0 ] && [ -t 1 ] || { echo "대화형 터미널(TTY)이 아니다. 사람이 입회한 별도 터미널에서 실행하라. 명령은 보내지 않았다."; exit 5; }
echo "S4-2${SET}(파일럿 자세 취득), 모드: ${MODE}. 대화형 확인이 시작된다. 사람이 입회한 상태에서만 계속하라."
RD=dofbot-g01_5b4   # 모터 제어 코드는 공용 /tmp가 아니라 전용 폴더(권한 700)에서 실행한다
"${SSH[@]}" "mkdir -p ~/${RD} && chmod 700 ~/${RD} && rm -f ~/${RD}/result.json"   # 이전 실행 기록이 섞이지 않게 먼저 지운다
# (계획은 아래에서 만든 뒤 함께 복사)
scp -q -o BatchMode=yes "${ROOT}/scripts/g01_dabai_probe.py" "${ROOT}/scripts/g01_dabai_expsweep.py" "${ROOT}/scripts/g01_5b4_s42_test.py" "${ROOT}/scripts/g01_5b4_joint_test.py" "${ROOT}/scripts/gripper_adapter.py" "${H}:${RD}/"
START=$("${SSH[@]}" 'python3 -c "
from Arm_Lib import Arm_Device
a=Arm_Device(); print(\",\".join(str(a.Arm_serial_servo_read(i)) for i in range(1,7)))"' | tail -1)
echo "현재 회독(J1~J6): ${START}"
case "${START}" in *None*|"") echo "회독에 None이 있다. 중단(팔은 움직이지 않았다)"; exit 8;; esac
PLAN="${ROOT}/evidence/G01/s4_2_plan_${SET}_${TS}.json"
RN=""; if [ "${SET}" = 16 ] || [ "${SET}" = 17 ] || [ "${SET}" = 18 ] || [ "${SET}" = 19 ] || [ "${SET}" = 22 ]; then RN="--return-nominal"; fi   # 복귀 목표를 K1 명목값으로 고정(복귀 래칫 차단, docs/*복귀래칫-재발-이슈보고서.md). 다른 세트는 기존 동작 유지
python3 "${ROOT}/scripts/g01_s42_plan.py" --set "${SET}" --start "${START}" ${RN} --out "${PLAN}" || { echo "이동 계획 게이트 실패. 중단(팔은 움직이지 않았다)"; exit 8; }
if [ "${SET}" = 12 ] || [ "${SET}" = 20 ] || [ "${SET}" = 21 ] || [ "${SET}" = 22 ] || [ "${SET}" = 14 ] || [ "${SET}" = 15 ] || [ "${SET}" = 16 ] || [ "${SET}" = 17 ] || [ "${SET}" = 18 ] || [ "${SET}" = 19 ]; then   # P1: 실행 시점 회독으로 만든 계획 전체(정상+복귀 상태별 경로)를 블록·종이·베이스 충돌 모델로 다시 검증한다(scipy 필요: venv 파이썬)
  if [ "${SET}" = 20 ] || [ "${SET}" = 21 ] || [ "${SET}" = 22 ]; then export P1_GATE_MM=10; fi
  if [ "${SET}" = 22 ]; then export P1_DZ_EXTRA_MM=45; fi   # D1-A: 실제 손끝이 FK보다 최대 45 mm 낮은 변형을 충돌 검증에 추가(D0 실측 26~36 mm 위로 여유)   # P1-b2 (b) 세트 20(C만): 충돌 여유 게이트 10 mm(사용자 승인 B1, 2026-11-16). 다른 세트는 기존 값(기본 20 mm)
  VPY="${P1_CHECK_PY:-${HOME}/projects/simple-vision-tech-poc/.venv/bin/python}"
  "${VPY}" -I "${ROOT}/src/planning/p1_plan_check.py" "${PLAN}" "${PLAN%.json}_check.json" || { echo "P1 블록·종이·베이스 여유 검증 실패(또는 실행 오류). 중단(팔은 움직이지 않았다)"; exit 8; }
fi
scp -q -o BatchMode=yes "${PLAN}" "${H}:${RD}/plan.json"
SRC='source /opt/ros/humble/setup.bash; source ~/dofbot_pro_ws/install/setup.bash'
stop_driver() { "${SSH[@]}" 'kill -INT $(cat /tmp/g01_dabai.pid 2>/dev/null) 2>/dev/null; sleep 2; pkill -INT -f "[_]_ns:=/dabai" 2>/dev/null; sleep 2; pgrep -fa "[_]_ns:=/dabai" >/dev/null && echo "DaBai 드라이버 남음" || echo "DaBai 드라이버 없음"' 2>&1 | tail -1; }
if [ "${MODE}" = real ]; then "${SSH[@]}" 'rm -rf /tmp/g01_s42_view'; fi   # 이전 실행의 촬영 결과가 섞이지 않게 폴더 전체를 지운다(카메라를 안 쓰는 세트 16·17에서도 결과 복사에 옛 폴더가 섞이지 않도록 DaBai 블록 밖에서 지운다)
CAP=""
if [ "${MODE}" = real ] && [ "${SET}" != 16 ] && [ "${SET}" != 17 ] && [ "${SET}" != 18 ] && [ "${SET}" != 19 ] && [ "${SET}" != 21 ] && [ "${SET}" != 22 ]; then   # 세트 16·17·18(서보 응답)은 카메라를 쓰지 않아 DaBai 드라이버를 띄우지 않는다
  "${SSH[@]}" 'pgrep -f "[_]_ns:=/dabai" >/dev/null && { echo "이미 DaBai 드라이버가 떠 있다"; exit 3; } || true' || { echo "DaBai 드라이버가 이미 실행 중이다. 중단"; exit 3; }
  trap stop_driver EXIT
  "${SSH[@]}" "${SRC}; nohup ros2 launch orbbec_camera dabai_dcw2.launch.py camera_name:=dabai enable_point_cloud:=false enable_colored_point_cloud:=false enable_ir:=false > /tmp/g01_dabai_drv.log 2>&1 & echo \$! > /tmp/g01_dabai.pid; sleep 12; pgrep -f '[_]_ns:=/dabai' >/dev/null && echo DaBai드라이버시작" | grep -q DaBai드라이버시작 || { echo "DaBai 드라이버 시작 실패. 중단(팔은 움직이지 않았다)"; exit 8; }
  echo "DaBai 드라이버 시작 확인"
  CAP=""
fi
# 터미널 벨(\a)은 설정에 따라 소리가 안 날 수 있어, 이동 직전 안내 줄("[이동 N/8]")을 보면 맥에서 직접 소리를 낸다.
rmdir /tmp/g01_d1_photo.lock 2>/dev/null || true   # 이전 실행이 죽으며 남긴 사진 잠금 제거(없으면 무시)
gemini_photo_bg() { local pid="$1"; ( while ! mkdir /tmp/g01_d1_photo.lock 2>/dev/null; do sleep 2; done; LG="${ROOT}/evidence/G06/d1_photo_${TS}_${pid}.log"; echo "photo_start_epoch=$(date +%s)" > "${LG}"; bash "${ROOT}/scripts/p1_m3_gemini_photo.sh" "d1_${TS}_${pid}" >> "${LG}" 2>&1 || true; echo "photo_end_epoch=$(date +%s)" >> "${LG}"; rmdir /tmp/g01_d1_photo.lock; afplay /System/Library/Sounds/Ping.aiff >/dev/null 2>&1 ) >/dev/null 2>&1 & }   # D1-A: 측정 대기가 시작되면 Gemini 사진 1장(약 40초, 팔 명령 없음). 실패해도 시험은 계속(눈 읽기가 기준)
beep_watch() { while IFS= read -r l; do case "$l" in *"[이동 "*) afplay /System/Library/Sounds/Glass.aiff >/dev/null 2>&1 &;; *"[측정 대기]"*) afplay /System/Library/Sounds/Glass.aiff >/dev/null 2>&1 & if [ "${SET}" = 22 ]; then PH=$(echo "$l" | sed -n 's/.*자세 \([0-9][0-9]*\)에서.*/\1/p'); [ -n "${PH}" ] && gemini_photo_bg "${PH}"; fi;; esac; done; }
ssh -t -o ConnectTimeout=8 "${H}" "cd ~/${RD} && python3 g01_5b4_s42_test.py --plan plan.json ${ARGS} --run-id ${TS} --out ~/${RD}/result.json" | tee >(beep_watch) || echo "(시험 프로세스가 비정상 종료. 기록이 있으면 가져온다)"
if [ "${MODE}" = real ]; then
  SOUT="${ROOT}/evidence/G01/raw/s42/${TS}"; mkdir -p "${SOUT}"
  "${SSH[@]}" 'cd /tmp && tar cf - g01_s42_view 2>/dev/null' | tar xf - -C "${SOUT}" 2>/dev/null || echo "촬영 파일 복사 실패(없을 수 있음)"
  "${SSH[@]}" 'cat /tmp/g01_dabai_drv.log' > "${SOUT}/driver.log" 2>&1 || true
  cp "${PLAN}" "${SOUT}/plan.json"
  echo "촬영 저장: ${SOUT}"
fi
scp -q -o BatchMode=yes "${H}:${RD}/result.json" "${OUT}" || { echo "기록 파일을 가져오지 못했다. Jetson의 ~/${RD}/result.json 확인 필요"; exit 6; }
python3 -c "import json,sys;j=json.load(open(sys.argv[1]));assert j['run_id']==sys.argv[2] and j['mode']==sys.argv[3], '기록의 run_id/mode가 이번 실행과 다르다';print('기록 확인: mode=',j['mode'],'ended=',j['ended'],'이동 기록=',len(j['steps']),'자세=',len(j.get('poses',[])),'returned_to_start=',j.get('returned_to_start'))" "${OUT}" "${TS}" "${MODE}"
echo "저장: ${OUT}"
PADTXT="낙하 받침 위치를 확인한 뒤"; ([ "${SET}" = 20 ] || [ "${SET}" = 21 ] || [ "${SET}" = 22 ]) && PADTXT="팔 주변(넘어질 수 있는 범위, 낙하 받침 없음)을 확인한 뒤"   # 세트 20은 사용자 결정으로 받침 없음
echo "비상 절차: 시험은 마지막에 시작값 S로 복귀한다. 복귀 없이 끝났거나 관절에서 소리·발열·처짐이 있으면 손부터 빼고, ${PADTXT} 담당자가 전원을 차단한다."
echo "이 스크립트를 다시 실행하면 그 시점의 위치를 새 S로 읽으므로 원래 위치로 돌아가지 않는다. 원래 시작 자세는 기록(JSON)의 state0(J1~J6 시작 회독)에 있다."
