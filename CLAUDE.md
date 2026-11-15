# CLAUDE.md — dofbot-rgbd-manipulation (프로젝트 규칙, 전역 `~/.claude/CLAUDE.md`와 함께 적용)

## 방향 점검 자동화 (2026-11-15, 사용자 요청)
- `git commit` 직후 훅(`.claude/settings.json`)과 세션 시작 때 `tools/direction_check.py --hook`이 돈다. 마지막 점검 이후 **커밋 8개 / 신규 실기기 실행 기록 3개 / 2일** 중 하나가 쌓이면 `[방향 점검 알림]`이 나온다(기준은 환경변수 `DC_COMMITS`·`DC_REAL`·`DC_DAYS`로 조정).
- **알림이 보이면** 하던 작업을 정리한 뒤 `docs/direction-checks/CHECKLIST.md`로 점검하고, `docs/YYMMDD-HHMM-방향-점검-보고서.md`를 쓰고, `python3 tools/direction_check.py --mark-reviewed <보고서>`로 기준점을 갱신한 뒤 사용자에게 요약 보고한다. 사용자가 보류하면 기준점은 갱신하지 않는다.
- 수동 확인: `python3 tools/direction_check.py --status`. 상태 파일: `docs/direction-checks/state.json`.
- 점검은 읽기·기록 작업이며 하드웨어 명령을 보내지 않는다. 정적 검증·테스트는 venv(`~/projects/simple-vision-tech-poc/.venv/bin/python`)로 실행한다.
