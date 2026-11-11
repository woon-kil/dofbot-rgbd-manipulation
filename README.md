# DOFBOT RGB-D Manipulation — Dual Camera v2

**상태: DESIGN_DRAFT_STATIC_CHECKS_ONLY / 실물 검증 NOT_RUN.** (정적 검증이 확인하는 것은 파일 존재, 예시 설정 안전 플래그, 단위 테스트뿐이며 설계의 타당성이 검증된 것은 아니다. 진단: `docs/261112-0748-설계진단보고서.md`)

DOFBOT Pro + 말단 DaBai RGB-D (eye-in-hand) + **고정 Orbbec Gemini 335L (eye-to-hand)** 기반 Pick & Place. Gemini 335L은 필수 시스템 구성요소이며, 실제 시야/깊이/베이스 정합 검증 전에는 위치추정 근거로 쓰지 않는다. 기존 고정 카메라를 이동시키지 않는다.

## 실행 우선순위
1. `docs/gates/roadmap.md` G00~G01: 두 카메라 취득/시야 (읽기 전용, 이동 명령 금지). G01.5: 보정 이동 안전 승인. G02: 관절 회독/FK 검증.
2. G03~G05: Gemini base 외부 보정, DaBai hand-eye 보정, 두 좌표계 교차 검증 (승인된 보정 자세 목록에 한한 저속 이동, ADR-005 결정 1-B).
3. G06: 5축 도달성/충돌 dry-run. G07: 운영자 승인 후 단일 시나리오 저속 P&P. G08: 듀얼 카메라 검증/실패 복구. (정의는 `docs/gates/roadmap.md`가 정본)

> **결정:** 팔 자세는 보정용 저속 이동으로 확보한다(ADR-005 결정 1-B). 이동은 G01.5 안전 승인 전에는 금지이며 안전 상한 수치는 아직 없다. Gemini 베이스 기준 방식은 순서대로 시험해 처음 합격한 방식을 채택한다(결정 2). **미결:** G06 이전 충돌 검사 범위(`docs/adr/ADR-005-pose-and-datum.md`, PROPOSED).

## 필수 문서
- `docs/architecture/system-design.md`: 통합 아키텍처 및 변환 그래프
- `docs/architecture/data-contracts.md`: 스키마·타임스탬프·프레임
- `docs/architecture/calibration-protocol.md`: 보정/holdout 및 오류 격리
- `docs/gates/roadmap.md`: Gate와 증거
- `docs/validation/validation-report.json`: 재현 가능한 정적 검증
- `docs/agent-work-order.md`: Cursor Agent 첫 실행 의뢰서

## 실행
`python3 scripts/validate_design.py` (하드웨어 명령 없음). `python3 -m unittest discover -s tests -v`.

**중요:** `configs/project.example.json`은 안전한 예시 설정이다. 보정 결과나 로봇 실기기 동작을 의미하지 않는다. 실물 테스트는 안전 확보 및 승인 후 별도 진행.
