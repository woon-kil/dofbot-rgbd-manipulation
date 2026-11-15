#!/usr/bin/env bash
# 출발 전 공통 점검(읽기 전용): ① 팔 제어 프로세스 없음 ② 회독 J1~J6를 기준 K1과 비교 ③ Jetson Arm_Lib 소스 sha256을 읽은 소스와 비교 ④ 원시 서보 값 표본(J1~J4 각 20회) 기록
#   ⑤ (--image) Gemini 사진 장면 확인 ⑥ (--manifest) 스크립트 sha256 확인 ⑦ (--manifest-src) 충돌 검증 코드·자세 파일·URDF·메시 sha256 확인. 게이트(②③⑤⑥) 중 하나라도 실패하면 종료 코드 1 — 팔을 움직이지 않는다. ④는 기록만(이동 판단에 쓰지 않음).
# 사용: bash scripts/p1_preflight.sh [--image 사진.png] [--manifest evidence/scripts_snapshot/<날짜>/MANIFEST.sha256] [--manifest-src …/MANIFEST-src.sha256] [--readback J1,..,J6]
#   --readback을 주면 Jetson에 접속하지 않는다(오프라인 시험용, ①③④ 생략). 없으면 Jetson에서 서보 읽기만 한다(쓰기·구동 없음).
#   기준 사진·마커 px: P1-0 확인값(data/g04/p1_block_check6, 마커 id 7 중심 702.8,273.0).
# 비상 롤백: 이 스크립트는 팔·카메라에 쓰지 않는다. 만들어진 기록(evidence/G06/p1_preflight_*.json)만 지우면 된다.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VPY="${P1_CHECK_PY:-${HOME}/projects/simple-vision-tech-poc/.venv/bin/python}"
REF="${ROOT}/data/g04/p1_block_check6_261115.png"; MREF="702.8,273.0"
IMG=""; MAN=""; MSRC=""; RB=""; ARGS=()
while [ $# -gt 0 ]; do case "$1" in --image) IMG="${2:-}"; shift;; --manifest) MAN="${2:-}"; shift;; --manifest-src) MSRC="${2:-}"; shift;; --readback) RB="${2:-}"; shift;; *) echo "알 수 없는 인자: $1"; exit 2;; esac; shift; done
TMP="$(mktemp -d)"; trap 'rm -rf "${TMP}"' EXIT
if [ -z "${RB}" ]; then
  H="${JETSON_HOST:-$(grep '^LG_JETSON_HOST=' "${HOME}/projects/learned-grasp-poc/.env" | cut -d= -f2- | tr -d '"')}"
  SSH=(ssh -o BatchMode=yes -o ConnectTimeout=8 "${H}")
  # ① 다른 팔 제어 프로세스가 있으면 읽지 않는다(보드는 읽기 직후의 쓰기를 버린다: 그쪽 쓰기를 해칠 수 있음)
  "${SSH[@]}" 'ps -eo args | grep -Ei "color_grasp|color_detect|arm_driver|dofbot_pro_(color|moveit|driver)" | grep -v grep && exit 9 || exit 0' || { echo "팔 제어 관련 프로세스가 실행 중이다. 읽지 않고 중단(종료 4)"; exit 4; }
  OUTJ="$("${SSH[@]}" python3 - <<'PY'
import base64, hashlib, inspect, json, textwrap, time
from Arm_Lib import Arm_Device
a = Arm_Device()
print("RB " + ",".join(str(a.Arm_serial_servo_read(i)) for i in range(1, 7)))
time.sleep(0.3)
out = {}
for j in (1, 2, 3, 4):
    ints, raws = [], []
    for _ in range(20):
        ints.append(a.Arm_serial_servo_read(j)); time.sleep(0.05)
        a.bus.write_byte_data(a.addr, j + 0x30, 0); time.sleep(0.003); w = a.bus.read_word_data(a.addr, j + 0x30)
        raws.append(None if w == 0 else ((w >> 8 & 0xff) | (w << 8 & 0xff00))); time.sleep(0.05)
    out[str(j)] = {"int": ints, "raw": raws}
print("RAW " + json.dumps(out))
try:   # 설치 형태(zip egg 등)에 따라 실패할 수 있다: 실패해도 위 회독·표본은 이미 출력했다
    src = inspect.getsource(Arm_Device.Arm_serial_servo_read)
    norm = "\n".join(l.rstrip() for l in textwrap.dedent(src).strip().splitlines())
    print("FUNC " + hashlib.sha256(norm.encode()).hexdigest()[:16])
    print("FUNCSRC " + base64.b64encode(norm.encode()).decode())
    try:
        f = inspect.getsourcefile(Arm_Device); print("FILE " + hashlib.sha256(open(f, "rb").read()).hexdigest() + " " + f)
    except Exception as e:
        print("FILEERR " + repr(e)[:150])
except Exception as e:
    print("SHAERR " + repr(e)[:200])
PY
)" || { echo "Jetson 읽기 실패(종료 3). 팔은 움직이지 않았다"; exit 3; }
  RB="$(echo "${OUTJ}" | sed -n 's/^RB //p' | tail -1)"; FN="$(echo "${OUTJ}" | sed -n 's/^FUNC //p' | tail -1)"; FILEL="$(echo "${OUTJ}" | sed -n 's/^FILE //p' | tail -1)"; SERR="$(echo "${OUTJ}" | sed -n 's/^SHAERR //p' | tail -1)"; echo "${OUTJ}" | sed -n 's/^RAW //p' | tail -1 > "${TMP}/raw.json"
  [ -n "${RB}" ] && [ -s "${TMP}/raw.json" ] || { echo "Jetson 출력 해석 실패(종료 3): ${OUTJ:0:200}"; exit 3; }
  echo "현재 회독(J1~J6): ${RB}"
  FSRC="$(echo "${OUTJ}" | sed -n 's/^FUNCSRC //p' | tail -1)"   # 설치된 함수 소스를 남겨 둔다(불일치 때 추가 왕복 없이 diff)
  if [ -n "${FSRC}" ]; then JF="${ROOT}/data/refs/jetson-Arm_serial_servo_read-$(date +%y%m%d-%H%M%S).py"; echo "${FSRC}" | base64 -d > "${JF}" 2>/dev/null && echo "설치본 함수 소스 저장: ${JF}"; fi
  if [ -n "${FN}" ]; then ARGS+=(--armlib-func-hash "${FN}"); else ARGS+=(--armlib-error "${SERR:-Jetson이 함수 해시를 출력하지 않았다}"); fi
  [ -n "${FILEL}" ] && ARGS+=(--armlib-file-sha "${FILEL%% *}")
  ARGS+=(--raw-samples "${TMP}/raw.json")
fi
ARGS+=(--readback "${RB}" --scripts-dir "${ROOT}/scripts")
[ -n "${IMG}" ] && ARGS+=(--image "${IMG}" --ref-image "${REF}" --marker-ref "${MREF}")
[ -n "${MAN}" ] && ARGS+=(--manifest "${ROOT}/${MAN#${ROOT}/}")
[ -n "${MSRC}" ] && ARGS+=(--manifest-src "${ROOT}/${MSRC#${ROOT}/}" --root "${ROOT}")
OUT="${ROOT}/evidence/G06/p1_preflight_$(date +%y%m%d-%H%M%S).json"; ARGS+=(--out "${OUT}")
"${VPY}" -I "${ROOT}/src/planning/p1_preflight.py" "${ARGS[@]}" && echo "기록: ${OUT}" || { echo "점검 실패. 팔은 움직이지 않았다. 기록: ${OUT}"; echo "롤백: 필요 없음(기록 파일만 남음: rm ${OUT})"; exit 1; }
echo "롤백: 필요 없음(읽기 전용). 기록을 지우려면: rm ${OUT}"
