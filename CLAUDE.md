# CLAUDE.md — dofbot-rgbd-manipulation (프로젝트 규칙, 전역 `~/.claude/CLAUDE.md`와 함께 적용)

## 방향 점검 자동화 (2026-11-15, 사용자 요청)
- `git commit` 직후 훅(`.claude/settings.json`)과 세션 시작 때 `tools/direction_check.py --hook`이 돈다. 마지막 점검 이후 **커밋 8개 / 신규 실기기 실행 기록 3개 / 2일** 중 하나가 쌓이면 `[방향 점검 알림]`이 나온다(기준은 환경변수 `DC_COMMITS`·`DC_REAL`·`DC_DAYS`로 조정).
- **알림이 보이면** 하던 작업을 정리한 뒤 `docs/direction-checks/CHECKLIST.md`로 점검하고, `docs/YYMMDD-HHMM-방향-점검-보고서.md`를 쓰고, `python3 tools/direction_check.py --mark-reviewed <보고서>`로 기준점을 갱신한 뒤 사용자에게 요약 보고한다. 사용자가 보류하면 기준점은 갱신하지 않는다.
- 수동 확인: `python3 tools/direction_check.py --status`. 상태 파일: `docs/direction-checks/state.json`.
- 점검은 읽기·기록 작업이며 하드웨어 명령을 보내지 않는다. 정적 검증·테스트는 프로젝트 가상환경(`.venv/bin/python`, `requirements.txt`로 numpy 2.5.4·opencv-python-headless 4.14.0.94·scipy 1.18.1 고정, 2026-11-16)으로 실행한다: `.venv/bin/python -I -m unittest discover -s tests`. (생성: `python3.12 -m venv .venv && .venv/bin/pip install -r requirements.txt`. 로컬 `scripts/` 래퍼의 기본 파이썬은 아직 옛 경로 `~/projects/simple-vision-tech-poc/.venv`이며 환경변수 `P1_CHECK_PY`로 바꿀 수 있다.)

## 다음 작업 제시 전 큰틀 점검 (2026-11-15, 사용자 요청 — 강제)
**다음 작업·선택지·권장안을 사용자에게 제시하기 전에 반드시 먼저 이 작업이 전체 프로젝트의 큰틀에서 제대로 된 방향인지 점검하고, 점검 결과를 제안에 함께 적는다.** 점검 없이 제시하지 않는다. (훅으로 강제하지 않는다: 키워드 추정 오탐과 형식 채우기 위험이 이득보다 크다고 판단해 2026-11-15 Stop 훅을 제거했다. 빠뜨리면 사용자가 지적한다.)
1. **근거를 읽는다(추측 금지):** `PROJECT-STATUS.md`(현재 Gate·열린 결정), `docs/gates/roadmap.md`(Gate 정의), `docs/gates/열린-결정-정리표.md`, 최신 `docs/*방향-점검-보고서.md`. 필요하면 `python3 tools/direction_check.py --status`.
2. **세 가지를 답한다:** ① 이 작업이 최종 목표(블록 P&P 30회 실패 0건)의 어느 Gate·남은 요소(접촉·파지·임계값 동결·G03·G05·G08 등)를 전진시키는가, 아니면 하위 문제(정밀도·모델 보정)인가. ② 지름길이 있는가(더 싼 측정·기존 증거·이미 안전한 영역으로 같은 질문을 답할 수 있는가). ③ 미검증 가정·로드맵 예외·새 안전 임계값을 늘리는가.
3. **제안에 한 줄 이상 포함한다:** `큰틀 점검: <Gate/목표 연결> · <하위문제 여부> · <새로 늘어나는 가정·예외> · <대안/지름길>` 형식. 점검에서 방향이 어긋나 보이면 작업을 제시하는 대신 그 사실과 정정안을 먼저 말한다.
4. **생략 가능:** 사용자가 이미 정한 작업을 그대로 수행할 때, 한 줄 문서 수정 같은 사소한 후속. 단 새 작업 제안·선택지 나열·권장안에는 항상 적용한다.
5. 점검에서 알게 된 큰틀 이탈 신호가 `tools/direction_check.py` 기준(커밋·실행·경과일)과 겹치면 방향 점검 보고서 절차(위 절)로 넘긴다.

## 사후 분석 표시 규칙 (2026-11-16, 사용자 요청 — 강제)
**사전 등록하지 않은 방법으로 기존 촬영·측정 자료를 다시 분석한 결과(사후 검출기·사후 문턱·사후 ROI 등)는 반드시 "참고용, 결정 근거 아님"으로 표시한다.** 그런 결과로 판정·승인 요청·안전 한도·합격선·모델 확정을 하지 않는다. 판정은 결과를 보기 전에 규칙(검출기 스크립트·ROI·문턱·합격 기준)을 먼저 등록하고 새로 촬영·측정한 자료로만 한다. 사전 등록 규칙이 실패(제외·미검출)하면 규칙을 고치지 말고 그 결과를 "판정 불가"로 기록한다. 상세: `docs/gates/G07-서보-최소유효단계-표와-D3-설계원칙-초안.md` §3 D3-3, 검출기 `evidence/G06/gemini_fingertip_detect_r15.py`(`--post-hoc`).

## 플랜 제시 전 advisor 검토 (2026-11-16, 사용자 요청 — 원칙)
**플랜·승인 요청 문서·시험 설계·선택지 권장안을 사용자에게 제시하기 전에 항상 `advisor`를 호출해 검토를 통과시킨다.** 검토에서 나온 차단 항목은 고친 뒤 제시하고, 제시문에 "advisor 검토 반영" 사실과 고친 내용을 적는다(검토 결과와 고친 문서를 함께 보여 준 뒤 승인을 요청). `.sh` 실행 안내 직전 advisor(전역 규칙)와는 별개로 추가 적용한다. 이 프로젝트에서는 codex 자문 게이트를 advisor로 대체한다. 생략: 사용자가 이미 정한 작업을 그대로 수행할 때, 한 줄 문서 수정.

## 인용한 실기기 기록은 추적 복사본으로 (2026-11-16, 사용자 승인)
`evidence/**/raw/`는 추적하지 않는다. 문서가 근거로 인용하는 raw JSON은 `python3 tools/verify_records.py --add <raw 경로>`로 `evidence/records/`에 복사하고 sha256을 `MANIFEST.tsv`에 남긴 뒤 커밋한다. 검증은 `python3 tools/verify_records.py`. 사진·영상·G00 원문 로그는 복사하지 않는다. 상세: `evidence/records/README.md`.

