#!/usr/bin/env bash
# 읽기 뒤 쓰기 간격 스윕: J1 +-4도, 지연 1.0/0.3/0.1/0.03/0초를 쓰기마다 번갈아(교차) 조건당 8회 = 40회. 한 프로세스, 질문은 시작 1회.
# 사용: bash scripts/g01_5b4_predelay_sweep.sh [--check]
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
if [ "${1:-}" = "--check" ]; then bash scripts/g01_5b4_wiggle_run.sh --check --joint 1 --n 8 --pre-delays 1.0,0.3,0.1,0.03,0; exit 0; fi
bash scripts/g01_5b4_wiggle_run.sh --joint 1 --n 8 --pre-delays 1.0,0.3,0.1,0.03,0 --real
echo "비상: 시험은 마지막에 시작값으로 복귀한다. 이상 시 손부터 빼고 낙하 받침 확인 뒤 전원을 차단한다."
