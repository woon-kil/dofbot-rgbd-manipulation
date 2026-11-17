#!/usr/bin/env bash
# [P-A 0단계 측정용 복사본] p1_m3_gemini_photo.sh에 단계 타임스탬프(Mac 시계, python3 time.time())와 p620 타임스탬프 파일 가져오기·보고만 추가했다. 촬영 동작은 원본과 같다(로봇 명령 없음).
# 사용: bash scripts/pa0_timing_photo.sh [이름]   (기본 이름: pa0_<YYMMDD-HHMMSS>)  결과: data/g04/<이름>.png + .mac/.host/.container_timing.txt + 보고 표
# Gemini(위쪽 카메라) 사진 1장을 p620 docker로 받아 data/g04/<이름>.png로 가져온다. 팔·로봇 명령 없음(카메라 드라이버만 약 40초). 출발 전 점검(p1_preflight.sh)의 --image 입력용.
# 사용: bash scripts/p1_m3_gemini_photo.sh [이름]    (기본 이름: m3_<YYMMDD-HHMMSS>)
# 전제: p620 전원 ON, Gemini USB 연결, 다른 컨테이너가 Gemini를 쓰지 않음(스크립트가 검사해 실패하면 종료).
# 롤백: ssh p620 'rm -rf ~/dofbot-g01-work/<이름>' (컨테이너는 스크립트가 종료 때 스스로 제거한다)
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
NAME="${1:-pa0_$(date +%y%m%d-%H%M%S)}"
[[ "${NAME}" =~ ^[A-Za-z0-9_-]+$ ]] || { echo "이름은 영문·숫자·_·- 만 가능: ${NAME}"; exit 2; }
CHK="${ROOT}/scripts/pa0_g01_gemini_check_timed.sh"; [ -f "${CHK}" ] || { echo "scripts/pa0_g01_gemini_check_timed.sh 없음"; exit 2; }
OUT="${ROOT}/data/g04/${NAME}.png"; [ ! -e "${OUT}" ] || { echo "이미 있음: ${OUT} (다른 이름을 쓴다)"; exit 2; }
mkdir -p "${ROOT}/data/g04"; MAC_T="${ROOT}/data/g04/${NAME}.mac_timing.txt"; : > "${MAC_T}"
M() { echo "$1 $(python3 -c 'import time;print("%.3f"%time.time())')" >> "${MAC_T}"; }
M m_start
echo "[1/3] p620 연결 확인"; M m_p620_check_start; ssh -o BatchMode=yes -o ConnectTimeout=8 p620 'test -f ~/dofbot-g01-work/g01_gemini_probe.py && echo p620_ok' | grep -q p620_ok || { echo "p620 접속 또는 프로브 파일 확인 실패(전원 확인)"; exit 3; }
M m_p620_check_done
echo "[2/3] Gemini 촬영(약 40초)"; M m_capture_start; ssh -o BatchMode=yes p620 "CFG_NAME=${NAME} bash -s" < "${CHK}" | tail -5
M m_capture_done
echo "[3/3] 사진 가져오기"
scp -q -o BatchMode=yes "p620:dofbot-g01-work/${NAME}/color.png" "${OUT}" || { echo "사진을 가져오지 못했다. p620의 ~/dofbot-g01-work/${NAME}/ 확인"; exit 4; }
M m_scp_done
"${P1_CHECK_PY:-${HOME}/projects/simple-vision-tech-poc/.venv/bin/python}" -I -c "import cv2,sys; im=cv2.imread(sys.argv[1]); assert im is not None and im.shape==(720,1280,3), '사진 크기가 720x1280이 아니다'; print('사진 확인', im.shape)" "${OUT}"
M m_verify_done
# 시계 오프셋 참고값(Mac 시계 앞뒤 사이에 p620 epoch 1회)과 NTP 동기 여부, p620 타임스탬프 파일 가져오기(읽기 전용)
M m_clk_a; ssh -o BatchMode=yes p620 'date +%s.%N; timedatectl show -p NTPSynchronized' > "${ROOT}/data/g04/${NAME}.p620_clock.txt" 2>&1 || true; M m_clk_b
scp -q -o BatchMode=yes "p620:dofbot-g01-work/${NAME}.host_timing.txt" "${ROOT}/data/g04/${NAME}.host_timing.txt" || echo "(호스트 타임스탬프 파일 못 가져옴)"
scp -q -o BatchMode=yes "p620:dofbot-g01-work/${NAME}/timing.txt" "${ROOT}/data/g04/${NAME}.container_timing.txt" || echo "(컨테이너 타임스탬프 파일 못 가져옴)"
python3 "${ROOT}/scripts/pa0_timing_report.py" "${ROOT}/data/g04/${NAME}" || echo "(보고 표 실패)"
echo "저장: ${OUT}"
echo "롤백(필요 시): rm ${OUT}; ssh p620 'rm -rf ~/dofbot-g01-work/${NAME}'"
echo "다음: bash scripts/p1_preflight.sh --image ${OUT} --manifest evidence/scripts_snapshot/261115-bigm3/MANIFEST.sha256"
