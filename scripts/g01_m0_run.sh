#!/usr/bin/env bash
# M0 읽기 전용 정지 측정 래퍼(플랜 M0, 사용자 입회). 쓰기 명령은 보내지 않는다(도구의 대리 객체가 쓰기를 예외로 막는다). 팔은 정지 상태여야 한다.
# 사용: bash scripts/g01_m0_run.sh --check | --fake | --sanity | --n N
#   --check: Jetson 사전 검증만(Arm_Lib import, 팔 제어 프로세스 없음). 대화·읽기 없음
#   --fake: 이 Mac에서 가짜 팔로 도구 동작만 확인(Jetson 접속 없음)
#   --sanity: 읽기 전용 1 unit(조건마다 3회)로 호출당 시간·총 시간을 실측한다. 이 실행의 None은 표에 넣지 않는다
#   --n N: 본 측정(N은 sanity 뒤 플랜에 사전 등록한 값). 측정은 1세트만 한다(재측정으로 고르지 않음)
# 비상: Ctrl-C 또는 터미널 끊김이면 도구가 그때까지의 기록을 저장하고 끝난다. 팔에 소리·발열·이상이 보이면 손부터 빼고 전원을 차단한다.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
H="${JETSON_HOST:-$(grep '^LG_JETSON_HOST=' "${HOME}/projects/learned-grasp-poc/.env" | cut -d= -f2- | tr -d '"')}"
MODE=""; N=""
while [ $# -gt 0 ]; do
  case "$1" in
    --check) MODE="check";;
    --fake) MODE="fake";;
    --sanity) MODE="sanity";;
    --n) MODE="measure"; N="${2:-}"; shift;;
    *) echo "알 수 없는 인자: $1"; exit 2;;
  esac; shift
done
[ -n "${MODE}" ] || { echo "인자 필요: --check | --fake | --sanity | --n N"; exit 2; }
if [ "${MODE}" = "measure" ]; then case "${N}" in ''|*[!0-9]*) echo "--n은 양의 정수(플랜에 사전 등록한 값)여야 한다"; exit 2;; esac; [ "${N}" -ge 1 ] || { echo "--n은 1 이상"; exit 2; }; fi
TOOL="${ROOT}/scripts/g01_m0_readonly.py"; [ -f "${TOOL}" ] || { echo "파일 없음: scripts/g01_m0_readonly.py"; exit 2; }
if [ "${MODE}" = "sanity" ] || [ "${MODE}" = "measure" ]; then
  git -C "${ROOT}" diff --quiet HEAD -- scripts/g01_m0_readonly.py scripts/g01_m0_run.sh || { echo "도구·래퍼가 커밋과 다르다(측정은 커밋된 코드로만). 커밋 뒤 재실행"; exit 2; }
  git -C "${ROOT}" ls-files --error-unmatch scripts/g01_m0_readonly.py scripts/g01_m0_run.sh >/dev/null 2>&1 || { echo "도구·래퍼가 추적되지 않는다(커밋 필요)"; exit 2; }
  [ -t 0 ] && [ -t 1 ] || { echo "대화형 터미널(TTY)이 아니다. 사람이 입회한 별도 터미널에서 실행하라. 읽기를 보내지 않았다."; exit 5; }
fi
if [ "${MODE}" = "measure" ]; then
  ls "${ROOT}"/evidence/G01/raw/g01_m0_readonly_n*_*.json >/dev/null 2>&1 && { echo "본 측정 기록이 이미 있다(측정은 1세트만, 재측정으로 고르지 않음). 중단"; exit 7; }
fi
GITHEAD="$(git -C "${ROOT}" rev-parse HEAD)"
PY="${ROOT}/.venv/bin/python"; [ -x "${PY}" ] || PY="python3"
TS="$(date +%y%m%d-%H%M%S)"; mkdir -p "${ROOT}/evidence/G01/raw"
if [ "${MODE}" = "fake" ]; then
  OUT="$(mktemp -t m0_fake.XXXXXX).json"
  "${PY}" -I "${TOOL}" --fake --n 2 --out "${OUT}"; echo "가짜 팔 확인 완료(Jetson 접속 없음): ${OUT}"; exit 0
fi
SSH=(ssh -o BatchMode=yes -o ConnectTimeout=8 "${H}")
"${SSH[@]}" 'python3 -c "import Arm_Lib" 2>/dev/null && echo Arm_Lib_ok' | grep -q Arm_Lib_ok || { echo "Jetson에서 Arm_Lib import 확인 실패(생성자 미호출)"; exit 3; }
"${SSH[@]}" 'ps -eo args | grep -Ei "color_grasp|color_detect|arm_driver|dofbot_pro_(color|moveit|driver)" | grep -v grep && exit 9 || exit 0' || { echo "팔 제어 관련 프로세스가 실행 중이다. 중단"; exit 4; }
if [ "${MODE}" = "check" ]; then echo "사전 검증 통과(읽기·쓰기 없음)"; exit 0; fi
RD=dofbot-g01_m0   # 전용 폴더(권한 700), 공용 /tmp 아님
"${SSH[@]}" "mkdir -p ~/${RD} && chmod 700 ~/${RD} && rm -f ~/${RD}/m0_result.json"
scp -q -o BatchMode=yes "${TOOL}" "${H}:${RD}/"
if [ "${MODE}" = "sanity" ]; then ARGS="--sanity"; KIND="sanity"; else ARGS="--n ${N}"; KIND="n${N}"; fi
OUT="${ROOT}/evidence/G01/raw/g01_m0_readonly_${KIND}_${TS}.json"
echo "M0 읽기 전용 측정(${KIND}). 쓰기 0, 팔 정지 상태여야 한다. 시작합니다."
ssh -t -o ConnectTimeout=8 "${H}" "cd ~/${RD} && python3 g01_m0_readonly.py ${ARGS} --out ~/${RD}/m0_result.json"
scp -q -o BatchMode=yes "${H}:${RD}/m0_result.json" "${OUT}" || { echo "기록 파일을 가져오지 못했다. Jetson의 ~/${RD}/m0_result.json 확인 필요"; exit 6; }
python3 -c "import json,sys;j=json.load(open(sys.argv[1]));j['git_head']=sys.argv[2];json.dump(j,open(sys.argv[1],'w'),ensure_ascii=False,indent=1);print('코드 커밋:',sys.argv[2][:10]);print('기록 확인: mode=',j['mode'],'sanity=',j['sanity'],'n=',j['n_per_condition'],'소요 s=',j['elapsed_s'],'중지=',j['stopped'])" "${OUT}"
echo "저장: ${OUT}"
echo "비상 롤백: 이 측정은 쓰기가 없어 되돌릴 상태가 없다. 같은 이름의 기록을 지우지 말고(증거) 다음 실행은 새 이름으로 저장된다."
