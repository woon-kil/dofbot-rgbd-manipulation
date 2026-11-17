#!/usr/bin/env bash
# 근거리 M3(세트 15) 실행 전 점검 묶음(팔에 쓰기 없음): ① 래퍼 --check ② Gemini 사진 1장 ③ 출발 전 공통 점검(회독·Arm_Lib 함수·원시 표본·사진·스크립트 sha·코드/자세 sha·마커 투영 가시성)
# 사용: bash scripts/p1_near_precheck.sh        (dry run은 터미널 대화가 필요해 따로: bash scripts/g01_5b4_s42_run.sh --set 15; 실행 직전 순서: 이 점검 → dry run → --real)
# 순서가 중요하다: 프로세스 확인(①)이 Jetson 읽기(③)보다 먼저여야 한다. 하나라도 실패하면 즉시 종료하고 이후 단계는 실행하지 않는다.
# 비상 롤백: 팔 쓰기가 없어 복구할 것이 없다. 만들어진 사진만 지운다(종료 때 출력). 증거 기록은 와일드카드로 지우지 않는다.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"; cd "${ROOT}"
SNAP="evidence/scripts_snapshot/261115-nearm3"; NAME="near_$(date +%y%m%d-%H%M%S)"
rollback() { echo "롤백(팔 복구 아님, 만든 사진만): rm -f data/g04/${NAME}.png; ssh p620 'rm -rf ~/dofbot-g01-work/${NAME}'  (점검 기록 evidence/G06/p1_preflight_*.json은 증거라 지우지 않는다)"; }
trap 'c=$?; [ $c -eq 0 ] || echo "점검 중단(종료 $c): 팔은 움직이지 않았다."; rollback' EXIT
echo "== 1/3 래퍼 --check"; bash scripts/g01_5b4_s42_run.sh --set 15 --check
echo "== 2/3 Gemini 사진(${NAME})"; bash scripts/p1_m3_gemini_photo.sh "${NAME}"
echo "== 3/3 출발 전 공통 점검"; bash scripts/p1_preflight.sh --image "data/g04/${NAME}.png" --manifest "${SNAP}/MANIFEST.sha256" --manifest-src "${SNAP}/MANIFEST-src.sha256" --poses-file evidence/G06/p1_near_m3_poses_s42.json
echo "전체 통과. 다음(터미널에서 직접): bash scripts/g01_5b4_s42_run.sh --set 15"
