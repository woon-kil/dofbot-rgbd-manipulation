#!/usr/bin/env bash
# G01 들어올림(세트 33) 실행 래퍼. 세트 31 래퍼(g01_p2k_run.sh)는 건드리지 않고(case 25|28|29|31이 33을 거부한다) 같은 안전 단계를 이 파일이 따로 갖는다. 어느 단계가 어디서 왔는지는 docs/261118-1038-들어올림-러너-구현-플랜.md §3-2 분기표.
# 사용: bash scripts/g01_lift_run.sh --plan-check --start J1,J2,J3,J4,J5,J6   (Mac만: 계획 생성 + 모든 간격 검증, ssh·사진·질문 없음)
#       bash scripts/g01_lift_run.sh --check | (인자 없음: dry_run, 읽기만) | --real   (대화형 TTY, 사람 입회, --real은 별도 승인 뒤에만)
# 순서(--real): 계획 생성 → ★미확정 상수 거부(scp·사진·질문·P-A 기동보다 앞) → 계획 게이트·간격 검증(세트 31과 같은 명령) → 배포 → 질문 → Gemini 기준 사진·마커 관문 → P-A 기동·기준 스냅 → 러너(사진 다리 연결) → 복귀 뒤 P-A 스냅 → P-A 정지
# 비상: 이상 징후 시 손부터 빼고 Ctrl-C(대기 중인 다음 이동만 취소, 이미 보낸 이동은 서보가 끝까지 간다). 전원 차단은 담당자가 판단.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MODE="dry_run"; ARGS=""; CHECK=0; PLANCHECK=0; START_ARG=""
while [ $# -gt 0 ]; do
  case "$1" in
    --real) MODE="real"; ARGS="--real";;
    --check) CHECK=1;;
    --plan-check) PLANCHECK=1;;
    --start) START_ARG="${2:-}"; shift;;
    *) echo "알 수 없는 인자: $1"; exit 2;;
  esac; shift
done
[ "${PLANCHECK}" = 1 ] && [ "${MODE}" = real ] && { echo "--plan-check와 --real은 함께 쓸 수 없다"; exit 2; }
[ "${PLANCHECK}" = 1 ] && [ -z "${START_ARG}" ] && { echo "--plan-check는 --start J1,J2,J3,J4,J5,J6이 필요하다(Jetson을 읽지 않는다)"; exit 2; }
SET=33; BASE=31   # 세트 33 = 세트 31 계획 + 들어올림 후처리. 간격 검증은 세트 31과 같은 명령·같은 상수를 쓴다
TS="$(date +%y%m%d-%H%M%S)"; KIND="s42${SET}"; OUT="${ROOT}/evidence/G01/raw/g01_5b4_${KIND}_result-${MODE}-${TS}.json"
for f in scripts/g01_p2k_test.py scripts/g01_5a_gripper_test.py scripts/g01_s42_plan_p2k.py scripts/g01_s4_2_path.py scripts/g01_s3_path.py scripts/g01_pose_generator.py scripts/g01_5b4_joint_test.py scripts/gripper_adapter.py scripts/g01_lift_params.py scripts/g01_lift_logic.py scripts/g01_lift_hook.py scripts/g01_s42_plan_lift.py scripts/g01_lift_bridge.py scripts/g01_lift_classify.py scripts/g01_lift_r8.py scripts/g01_lift_marker_ref.py scripts/pa_driver.sh scripts/pa_snap_mac.sh scripts/pa_snap.py scripts/pa_detect.py; do [ -f "${ROOT}/${f}" ] || { echo "파일 없음: ${f}"; exit 2; }; done
cmp -s "${ROOT}/src/control/gripper_adapter.py" "${ROOT}/scripts/gripper_adapter.py" || { echo "scripts/gripper_adapter.py가 src/control/gripper_adapter.py와 다르다. 복사 후 재실행"; exit 2; }
VPY="${P1_CHECK_PY:-${ROOT}/.venv/bin/python}"
if [ "${PLANCHECK}" = 0 ]; then
  H="${JETSON_HOST:-$(grep '^LG_JETSON_HOST=' "${HOME}/projects/learned-grasp-poc/.env" | cut -d= -f2- | tr -d '"')}"
  SSH=(ssh -o BatchMode=yes -o ConnectTimeout=8 "${H}")
  "${SSH[@]}" 'python3 -c "import Arm_Lib" 2>/dev/null && echo Arm_Lib_ok' | grep -q Arm_Lib_ok || { echo "Jetson에서 Arm_Lib import 확인 실패(읽기 전용 확인, 생성자 미호출)"; exit 3; }
  "${SSH[@]}" 'ps -eo args | grep -Ei "color_grasp|color_detect|arm_driver|dofbot_pro_(color|moveit|driver)" | grep -v grep && exit 9 || exit 0' || { echo "팔 제어 관련 프로세스가 실행 중이다. 중단"; exit 4; }
  if [ "${CHECK}" = 1 ]; then echo "사전 검증 통과(대화·쓰기 없음)"; exit 0; fi
  START=$("${SSH[@]}" 'python3 -c "
from Arm_Lib import Arm_Device
a=Arm_Device(); print(\",\".join(str(a.Arm_serial_servo_read(i)) for i in range(1,7)))"' | tail -1)
else START="${START_ARG}"; fi
echo "현재 회독(J1~J6): ${START}"
case "${START}" in *None*|"") echo "회독에 None이 있다. 중단(팔은 움직이지 않았다)"; exit 8;; esac
mkdir -p "${ROOT}/evidence/G01/raw"
PLAN31="${ROOT}/evidence/G01/s4_2_plan_${BASE}_${TS}.json"; PLAN="${ROOT}/evidence/G01/s4_2_plan_${SET}_${TS}.json"
python3 "${ROOT}/scripts/g01_s42_plan_p2k.py" --set "${BASE}" --start "${START}" --return-nominal --out "${PLAN31}" || { echo "이동 계획 게이트 실패. 중단(팔은 움직이지 않았다)"; exit 8; }
"${VPY}" -I "${ROOT}/scripts/g01_s42_plan_lift.py" "${PLAN31}" "${PLAN}" || { echo "세트 33(들어올림) 계획 게이트 실패. 중단(팔은 움직이지 않았다)"; exit 8; }
# ★ 미확정 상수(UNSET)가 하나라도 있으면 --real 거부: scp·사진·질문·P-A 기동보다 앞(러너가 거부하면 결과 파일이 없어 엉뚱한 exit 6이 나온다)
UNSETMSG=$("${VPY}" -I -c "import sys,json; sys.path.insert(0,'${ROOT}/scripts'); import g01_lift_params as P; p=P.params_for_plan(json.load(open('${PLAN}',encoding='utf-8'))); print(', '.join(P.unset_names(p)) or '없음'); P.check_real_allowed('${MODE}', p)") || { echo "미확정 상수(UNSET)가 남아 --real을 거부한다: ${UNSETMSG}. 팔은 움직이지 않았다(사진·질문·P-A 기동 없음)."; exit 8; }
echo "미확정 상수(UNSET): ${UNSETMSG}$([ "${MODE}" = real ] || echo ' (--real이면 거부됨)')"
if [ "${MODE}" = real ]; then REFINFO=$("${VPY}" -I "${ROOT}/scripts/g01_lift_marker_ref.py" info) || { echo "${REFINFO}"; echo "세트 33 마커 기준이 재등록되어 있지 않다. scp·사진·질문·P-A 기동 없이 중단(팔은 움직이지 않았다)."; exit 7; }; echo "마커 기준: ${REFINFO}"; fi   # 파일 없음·복수는 NG(옛 기준·Run 1 위치로 되돌아가지 않는다)
# --- 간격 검증: 세트 31 래퍼(g01_p2k_run.sh)와 같은 명령·같은 상수(tests/test_lift_wrapper.py가 문자열 일치를 확인한다) ---
PLAN30="${ROOT}/evidence/G01/s4_2_plan_30_${TS}.json"
TSET=32   # 세트 31(Run 4): J1 86 접근 자세 3101의 전이 점검은 세트 32
python3 "${ROOT}/scripts/g01_s42_plan_p2k.py" --set "${TSET}" --start "${START}" --return-nominal --out "${PLAN30}" || { echo "이동 구간 계획 게이트 실패. 중단(팔은 움직이지 않았다)"; exit 8; }
P1_GATE_MM=10 P1_DZ_EXTRA_MM=35.8 "${VPY}" -I "${ROOT}/src/planning/p1_plan_check.py" "${PLAN30}" "${PLAN30%.json}_check.json" || { echo "이동 구간(K1→50 mm, 기본 캡) 여유 검증 실패. 중단(팔은 움직이지 않았다)."; exit 8; }
BXS="7.4 13.4"   # 블록 월드 x = 캡 중점 월드 x + d_x: J1 86.4 → 10.42 ± 3 = 7.4·13.4(PA.caps_world)
for BX in ${BXS}; do   # 팔 전체(모든 링크 껍질 + 손가락 캡슐, 열린 손가락 중심 간격 30.9 mm, 비관 변형 포함) vs 블록: 블록 월드 x = 7.3 + d_x(프로젝트 x와 부호 반대), 최악 d_x −3·+3
  P1_CAP_OPEN_ONLY_MM=30.9 P1_BLOCK_X_MM=${BX} P1_GATE_MM=2 "${VPY}" -I "${ROOT}/src/planning/p1_plan_check.py" "${PLAN}" "${PLAN%.json}_contact_bx${BX}.json" || { echo "접촉 구간 팔 전체 검증 실패(블록 월드 x +${BX} mm, 게이트-접촉 2 mm). 중단(팔은 움직이지 않았다)."; exit 8; }
done
RUN2_GATE_MM=2.0 "${VPY}" -I "${ROOT}/evidence/G06/run2_contact_clearance_261117.py" "${PLAN}" | tee "${PLAN%.json}_contact.txt" || { echo "접촉 구간(열린 손가락+블록, 게이트-접촉 2 mm) 검증 실패. 중단(팔은 움직이지 않았다)."; exit 8; }
if [ "${PLANCHECK}" = 1 ]; then echo "계획·간격 검증 통과(Mac만, 계획 ${PLAN})"; exit 0; fi
[ -t 0 ] && [ -t 1 ] || { echo "대화형 터미널(TTY)이 아니다. 사람이 입회한 별도 터미널에서 실행하라. 명령은 보내지 않았다."; exit 5; }
echo "S4-2${SET}(들어올림 시험), 모드: ${MODE}. 대화형 확인이 시작된다. 사람이 입회한 상태에서만 계속하라."
RD=dofbot-g01_5b4   # 모터 제어 코드는 공용 /tmp가 아니라 전용 폴더(권한 700)에서 실행한다
"${SSH[@]}" "mkdir -p ~/${RD} && chmod 700 ~/${RD} && rm -f ~/${RD}/result.json ~/${RD}/lift_photo_ref.json ~/${RD}/lift_photo_A.json ~/${RD}/lift_photo_B.json ~/${RD}/lift_photo_C.json"   # 이전 실행 기록·사진 결과가 섞이지 않게 먼저 지운다(정확한 이름만, 와일드카드 없음)
scp -q -o BatchMode=yes "${ROOT}/scripts/g01_dabai_probe.py" "${ROOT}/scripts/g01_dabai_expsweep.py" "${ROOT}/scripts/g01_p2k_test.py" "${ROOT}/scripts/g01_5a_gripper_test.py" "${ROOT}/scripts/g01_5b4_joint_test.py" "${ROOT}/scripts/gripper_adapter.py" "${ROOT}/scripts/g01_lift_params.py" "${ROOT}/scripts/g01_lift_logic.py" "${ROOT}/scripts/g01_lift_hook.py" "${H}:${RD}/"
scp -q -o BatchMode=yes "${PLAN}" "${H}:${RD}/plan.json"
PA_MAXSEC=1800; PA_STARTED=0
pa_ssh() { local act="$1"; shift; ssh -o BatchMode=yes -o ConnectTimeout=8 p620 "ACTION=${act} $* bash -s" < "${ROOT}/scripts/pa_driver.sh"; }
pa_stop() { [ "${PA_STARTED}" = 1 ] && { pa_ssh stop || echo "(P-A 드라이버 정지 실패: p620에서 ACTION=stop 확인)"; PA_STARTED=0; }; true; }
trap pa_stop EXIT
beep_watch() { while IFS= read -r l; do case "$l" in *"[이동 "*) afplay /System/Library/Sounds/Glass.aiff >/dev/null 2>&1 &;; esac; done; }   # 세트 33은 36 s Gemini 사진(D1)을 쓰지 않는다(사진은 P-A 스냅)
if [ "${MODE}" = real ]; then
QTXT="기준 사진 전 확인: 노란 블록(마커 id 7)을 **마커 기준 재등록 사진 이후 만지지 않았고**(${REFINFO}) 지정 자리 그대로이며, 눈금자·클램프·거치대는 없고 종이·보드 위에 그 밖의 물체가 없으며, 사람·손이 화면 밖입니까? (y 이외는 중단) [y/N] "
read -r -p "${QTXT}" _ok; [ "${_ok}" = y ] || { echo "중단(팔은 움직이지 않았다)"; exit 7; }
echo "[기준 사진] 팔 K1·눈금자 없음·사람 없음 상태로 Gemini 기준 사진 1장을 찍습니다(약 40초). 화면 밖에 계세요."
bash "${ROOT}/scripts/p1_m3_gemini_photo.sh" "p2kref_${TS}" && [ -s "${ROOT}/data/g04/p2kref_${TS}.png" ] || { echo "기준 사진 실패(또는 파일 없음): 시험을 시작하지 않는다. 팔은 움직이지 않았다."; exit 7; }
"${VPY}" -I -c "import sys,cv2; sys.path.insert(0,'${ROOT}/scripts'); import g01_e0g_analyze as E, g01_lift_marker_ref as LMR; m=E.detect(cv2.imread('${ROOT}/data/g04/p2kref_${TS}.png')).get(7); assert m is not None, '마커 id 7 미검출'
RX, RY = LMR.load()[:2]   # 세트 33: 재등록 1회 한정 기준 파일만(없거나 2개 이상이면 오류, Run 1 위치로 되돌아가지 않는다)
d=((m['center'][0]-RX)**2+(m['center'][1]-RY)**2)**0.5; print('기준 사진 마커', [round(x,1) for x in m['center']], '기준 (%.2f, %.2f)과 차 %.2f px (관문 <= 1.0)' % (RX, RY, d)); assert d <= 1.0, '마커 관문 미달: 블록 또는 카메라가 재등록 이후 움직였다'" || { echo "마커 관문 실패: 블록·카메라 위치가 재등록 기준과 다르거나 기준 파일이 없다. 시작하지 않는다(팔은 움직이지 않았다)."; exit 7; }
# --- P-A 기동(같은 카메라를 쓰므로 Gemini 36 s 사진 뒤에) + 기준 스냅(S4' 기준, 같은 P-A 기동) ---
echo "[P-A 기동] p620 상시 드라이버를 켭니다(약 12초 예열, 최대 ${PA_MAXSEC} s). 화면 밖에 계세요."
PA_STARTED=1; pa_ssh start WARMUP=12 MAXSEC=${PA_MAXSEC} || { echo "P-A 드라이버 기동 실패. 중단(팔은 움직이지 않았다)"; exit 8; }
bash "${ROOT}/scripts/pa_snap_mac.sh" "lift_${TS}_ref" || { echo "P-A 기준 스냅 실패. 중단(팔은 움직이지 않았다)"; exit 7; }
REF="${ROOT}/data/g04/pa/lift_${TS}_ref.lift_photo_ref.json"
"${VPY}" -I -c "import sys,json,re,subprocess,time; sys.path.insert(0,'${ROOT}/scripts'); import g01_lift_bridge as B
r=subprocess.run(['ssh','-o','BatchMode=yes','-o','ConnectTimeout=8','p620','ACTION=status bash -s'],stdin=open('${ROOT}/scripts/pa_driver.sh'),capture_output=True,text=True,timeout=20)
m=re.search(r'started_epoch\s+([0-9.]+)',r.stdout); assert m, 'P-A 기동 시각을 읽지 못했다'
det=json.load(open('${ROOT}/data/g04/pa/lift_${TS}_ref.detect.json',encoding='utf-8')); assert det.get('found'), 'P-A 기준 스냅에서 마커 미검출'
import hashlib; sha=hashlib.sha256(open('${ROOT}/data/g04/pa/lift_${TS}_ref.color.png','rb').read()).hexdigest()
json.dump(B.make_ref('${TS}', det, float(m.group(1)), ${PA_MAXSEC}, time.time(), sha), open('${REF}','w',encoding='utf-8'), ensure_ascii=False)" || { echo "P-A 기준 결과 생성 실패. 중단(팔은 움직이지 않았다)"; exit 7; }
# 기준 결과를 Jetson에 원자적으로 둔다(러너 기동 직전: 러너는 이 파일의 elapsed_at_ref_s에 자기 경과를 더해 P-A 경과를 판단한다)
scp -q -o BatchMode=yes "${REF}" "${H}:${RD}/.lift_photo_ref.json.tmp" && "${SSH[@]}" "mv ~/${RD}/.lift_photo_ref.json.tmp ~/${RD}/lift_photo_ref.json" || { echo "기준 결과 전달 실패. 중단(팔은 움직이지 않았다)"; exit 7; }
fi
BLOG="${ROOT}/evidence/G01/raw/lift_bridge_${TS}.log"; RLOG="${ROOT}/evidence/G01/raw/lift_runner_${TS}.log"   # 러너 출력은 매번 파일로도 남긴다(래퍼 경유 dry run에서 result.json이 사라진 원인을 숨기지 않기 위해)
RUNNER="cd ~/${RD} && python3 g01_p2k_test.py --plan plan.json ${ARGS} --run-id ${TS} --out ~/${RD}/result.json"
if [ "${MODE}" = real ]; then
  ssh -t -o ConnectTimeout=8 "${H}" "${RUNNER}" | tee >(beep_watch) >("${VPY}" -I "${ROOT}/scripts/g01_lift_bridge.py" --run-id "${TS}" --ref "${REF}" --plan "${PLAN}" --host "${H}" --rd "${RD}" --name-prefix "lift_${TS}" --log "${BLOG}") "${RLOG}" || echo "(시험 프로세스가 비정상 종료. 기록이 있으면 가져온다)"
  for _ in $(seq 1 20); do grep -q '"results"' "${BLOG}" 2>/dev/null && break; sleep 0.5; done   # 다리가 EOF를 받아 결과를 적을 때까지 잠시 기다린다
  echo "[복귀 뒤 사진] 팔이 K1로 돌아왔습니다. 화면 밖에 계세요. P-A 스냅 1장(기록용, P4)."
  bash "${ROOT}/scripts/pa_snap_mac.sh" "lift_${TS}_post" || echo "(복귀 뒤 P-A 스냅 실패: 기록 없음)"
  pa_stop
else
  ssh -t -o ConnectTimeout=8 "${H}" "${RUNNER}" | tee >(beep_watch) "${RLOG}" || echo "(시험 프로세스가 비정상 종료. 기록이 있으면 가져온다)"   # dry_run: 쓰기 0, 사진·P-A 없음
fi
scp -q -o BatchMode=yes "${H}:${RD}/result.json" "${OUT}" || { echo "기록 파일을 가져오지 못했다. Jetson의 ~/${RD}/result.json 확인 필요"; echo "--- 진단(읽기 전용): Jetson 작업 폴더와 러너 프로세스 ---"; "${SSH[@]}" "date; ls -la --time-style=full-iso ~/${RD} | head -20; pgrep -fa '[g]01_p2k_test' || echo '러너 프로세스 없음'" || true; exit 6; }
python3 -c "import json,sys;j=json.load(open(sys.argv[1]));assert j['run_id']==sys.argv[2] and j['mode']==sys.argv[3], '기록의 run_id/mode가 이번 실행과 다르다';print('기록 확인: mode=',j['mode'],'ended=',j['ended'],'이동 기록=',len(j['steps']),'returned_to_start=',j.get('returned_to_start'),'lift=',(j.get('lift') or {}).get('ended_reason'))" "${OUT}" "${TS}" "${MODE}"
echo "저장: ${OUT}"
"${VPY}" -I "${ROOT}/scripts/g01_lift_classify.py" "${OUT}" || echo "(분류 실패: 결과 JSON을 직접 확인)"
[ "${MODE}" = real ] && { "${VPY}" -I "${ROOT}/scripts/g01_lift_r8.py" "${OUT}" --ts "${TS}" --log "${BLOG}" || echo "(R8 사후 계산 실패: 결과 JSON·이미지로 직접 계산)"; }   # R8: E1·E2·E3·Q3(사전 등록 규칙, 판정 기록 수는 네 기준이 모두 합격/불합격인 기록)   # 시도 분류(접촉 어댑터 쓰기 로그 기준, 판정 기록 수는 R8 뒤)
echo "비상 절차: 시험은 마지막에 시작값 S로 복귀한다. 복귀 없이 끝났거나 관절에서 소리·발열·처짐이 있으면 손부터 빼고, 팔 주변(넘어질 수 있는 범위, 낙하 받침 없음)을 확인한 뒤 담당자가 전원을 차단한다."
echo "이 스크립트를 다시 실행하면 그 시점의 위치를 새 S로 읽으므로 원래 위치로 돌아가지 않는다. 원래 시작 자세는 기록(JSON)의 state0(J1~J6 시작 회독)에 있다."
