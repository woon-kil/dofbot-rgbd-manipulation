#!/usr/bin/env bash
# G01 들어올림 사진 경로 dry run(로봇 명령 없음, 카메라 p620 + Jetson 스텁). 계획 §3 step 4(전달 경로 D 측정)의 변형: 운반 줄은 래퍼(g01_lift_run.sh)와 같은 줄을 쓰고 원격 명령만 스텁(g01_lift_dryrun_stub.py)으로 바꿨다(tests/test_lift_dryrun.py가 문자열 일치를 확인).
# 사용: bash scripts/g01_lift_dryrun.sh --plan <세트 33 계획 JSON>   (같은 세션에서 먼저 `bash scripts/g01_lift_run.sh`(--real 없음)를 실행해 생기는 evidence/G01/s4_2_plan_33_<TS>.json을 쓴다. 계획 생성·간격 게이트는 여기서 다시 하지 않는다)
# 전제: 사람이 화면 밖, 팔은 K1에서 명령 없음(이 스크립트는 팔을 움직이지 않는다), P-A 정지(컨테이너 없음). 약 10분(차가운 시험 3회 포함). 판독 규칙은 g01_lift_dryrun_report.py 머리말과 플랜 §3-2에 사전 등록.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PLAN=""; while [ $# -gt 0 ]; do case "$1" in --plan) PLAN="${2:-}"; shift;; *) echo "알 수 없는 인자: $1"; exit 2;; esac; shift; done
[ -n "${PLAN}" ] && [ -f "${PLAN}" ] || { echo "--plan <세트 33 계획 JSON>이 필요하다(g01_lift_run.sh --real 없이 먼저 실행)"; exit 2; }
PLAN="$(cd "$(dirname "${PLAN}")" && pwd)/$(basename "${PLAN}")"   # 브리지는 절대 경로로 받는다
TS="$(date +%y%m%d-%H%M%S)"; VPY="${P1_CHECK_PY:-${ROOT}/.venv/bin/python}"; RD=dofbot-g01_5b4
H="${JETSON_HOST:-$(grep '^LG_JETSON_HOST=' "${HOME}/projects/learned-grasp-poc/.env" | cut -d= -f2- | tr -d '"')}"; SSH=(ssh -o BatchMode=yes -o ConnectTimeout=8 "${H}")
for f in scripts/g01_lift_params.py scripts/g01_lift_logic.py scripts/g01_lift_hook.py scripts/g01_lift_bridge.py scripts/g01_lift_dryrun_stub.py scripts/g01_lift_dryrun_report.py scripts/g01_s42_plan_lift.py scripts/g01_lift_marker_ref.py scripts/pa_driver.sh scripts/pa_snap_mac.sh scripts/pa_snap.py scripts/pa_detect.py; do [ -f "${ROOT}/${f}" ] || { echo "파일 없음: ${f}"; exit 2; }; done
[ -t 0 ] && [ -t 1 ] || { echo "대화형 터미널(TTY)이 아니다. 사람이 입회한 별도 터미널에서 실행하라."; exit 5; }
"${VPY}" -I "${ROOT}/scripts/g01_lift_marker_ref.py" info >/dev/null || { echo "세트 33 마커 기준이 재등록되어 있지 않다(정렬 보고 경로가 NG 보고만 낸다). 중단"; exit 7; }
read -r -p "dry run 전 확인: 사람이 화면 밖에 있고 팔은 K1에서 명령이 없으며 P-A가 꺼져 있습니까? (y 이외는 중단) [y/N] " _ok; [ "${_ok}" = y ] || { echo "중단(아무것도 시작하지 않았다)"; exit 7; }
"${SSH[@]}" "mkdir -p ~/${RD} && chmod 700 ~/${RD} && rm -f ~/${RD}/lift_photo_ref.json ~/${RD}/lift_photo_A.json ~/${RD}/lift_photo_B.json ~/${RD}/lift_photo_C.json ~/${RD}/dryrun_stub_result.json"
# ControlMaster 3개(pa-ssh→p620, lift-ssh→p620, lift-ssh→Jetson)를 모두 정리해 차가운 시험이 정말 차갑게 한다(해당 ControlPath를 줘야 -O exit가 마스터를 찾는다). 정리 뒤 -O check 결과를 기록한다
for spec in "/tmp/pa-ssh-%C p620" "/tmp/lift-ssh-%C p620" "/tmp/lift-ssh-%C ${H}"; do set -- ${spec}; ssh -o ControlPath="$1" -O exit "$2" >/dev/null 2>&1 || true; if ssh -o ControlPath="$1" -O check "$2" >/dev/null 2>&1; then echo "마스터 정리 실패: $1 $2"; exit 8; else echo "마스터 없음 확인: $1 $2"; fi; done
PA_MAXSEC=1800; PA_STARTED=0
pa_ssh() { local act="$1"; shift; ssh -o BatchMode=yes -o ConnectTimeout=8 p620 "ACTION=${act} $* bash -s" < "${ROOT}/scripts/pa_driver.sh"; }
pa_stop() { [ "${PA_STARTED}" = 1 ] && { pa_ssh stop || echo "(P-A 드라이버 정지 실패: p620에서 ACTION=stop 확인)"; PA_STARTED=0; }; true; }
trap pa_stop EXIT
beep_watch() { while IFS= read -r l; do case "$l" in *"[이동 "*) afplay /System/Library/Sounds/Glass.aiff >/dev/null 2>&1 &;; esac; done; }   # 세트 33은 36 s Gemini 사진(D1)을 쓰지 않는다(사진은 P-A 스냅)
ssh -o BatchMode=yes -o ConnectTimeout=8 p620 "ACTION=status bash -s" < "${ROOT}/scripts/pa_driver.sh" | grep -q "컨테이너 없음" || { echo "P-A가 이미 켜져 있거나 상태를 알 수 없다(컨테이너 없음이어야 한다). 중단"; exit 8; }
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
scp -q -o BatchMode=yes "${ROOT}/scripts/g01_lift_params.py" "${ROOT}/scripts/g01_lift_logic.py" "${ROOT}/scripts/g01_lift_hook.py" "${ROOT}/scripts/g01_lift_dryrun_stub.py" "${H}:${RD}/" || { echo "스텁 전달 실패. 중단"; exit 7; }
BLOG="${ROOT}/evidence/G01/raw/lift_dryrun_bridge_${TS}.log"; RLOG="${ROOT}/evidence/G01/raw/lift_runner_${TS}.log"
STUBCMD="cd ~/${RD} && python3 g01_lift_dryrun_stub.py --run-id ${TS} --out ~/${RD}/dryrun_stub_result.json"
ssh -t -o ConnectTimeout=8 "${H}" "${STUBCMD}" | tee >(beep_watch) >("${VPY}" -I "${ROOT}/scripts/g01_lift_bridge.py" --run-id "${TS}" --ref "${REF}" --plan "${PLAN}" --host "${H}" --rd "${RD}" --name-prefix "lift_${TS}" --log "${BLOG}") "${RLOG}" || echo "(시험 프로세스가 비정상 종료. 기록이 있으면 가져온다)"
for _ in $(seq 1 20); do grep -q '"results"' "${BLOG}" 2>/dev/null && break; sleep 0.5; done   # 다리가 EOF를 받아 결과를 적을 때까지 잠시 기다린다

SOUT="${ROOT}/evidence/G01/raw/lift_dryrun_stub_${TS}.json"
scp -q -o BatchMode=yes "${H}:${RD}/dryrun_stub_result.json" "${SOUT}" || { echo "스텁 결과를 가져오지 못했다(Jetson ~/${RD}/dryrun_stub_result.json 확인)"; exit 6; }
pa_stop
"${VPY}" -I "${ROOT}/scripts/g01_lift_dryrun_report.py" "${SOUT}" --log "${BLOG}" --out "${ROOT}/evidence/G01/raw/lift_dryrun_report_${TS}.json"
echo "스텁 결과: ${SOUT}"
echo "참고: 이 dry run의 스텁은 접촉 분기에서만 import되던 g01_lift_hook의 Jetson 첫 import를 확인한다(러너 dry run은 접촉 분기 앞에서 끝난다)."
