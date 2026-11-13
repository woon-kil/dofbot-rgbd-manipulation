# Cursor Agent 작업의뢰서 — G00/G01만

목적: Gemini 335L + DaBai + DOFBOT Pro의 **읽기 전용 인벤토리 및 데이터 취득 검증**. 로봇 이동 명령 금지.

1. `README.md`, ADR-004, Gate roadmap, calibration-protocol을 먼저 읽고 현재 설정이 `dry_run=true`, `hardware_enabled=false`, `calibration_motion_approved=false`인지 확인.
2. Gemini 기존 고정 위치에서 RGB/정합 Depth/CameraInfo/프레임 ID/타임스탬프/유효 픽셀/작업영역 시야를 캡처. 이전 프로젝트 B-00 자료를 참고할 수 있으나 새 프로젝트 조건에서 다시 증거 수집.
3. DaBai 정확한 모델·장착 프레임·RGB-D 지원/SDK·카메라 내부 파라미터 확인. DOFBOT 관절/그리퍼/URDF/FK/SDK 회독 mapping 조사. 제어 명령은 호출하지 말 것.
4. `evidence/G00/`, `evidence/G01/`에 원본과 manifest, PASS/BLOCKED 근거 작성. 캘리브레이션이나 실행은 시작하지 말 것.
5. 외부 문서/실기기 값 충돌 시 실기기 read-only evidence를 우선하고 모호함은 UNVERIFIED로 유지.

완료 보고: 변경 파일, 장치 확인 결과, RGB-D 정합 여부, Gemini 시야, 차단 이슈, 다음 Gate 진행 가능 여부.

## 보강 (진단 2026-11-12, `docs/261112-0748-설계진단보고서.md`)
- **결정 반영:** 팔 자세는 보정용 저속 이동으로 확보하기로 했다(ADR-005 결정 1-B). 그러나 **이 의뢰서(G00/G01)는 여전히 읽기 전용**이다. G01.5 안전 승인 전에는 이동 명령을 보내지 말고 DaBai 취득은 현재 팔 자세 그대로 한다. 토크 해제 상태 회독 가능 여부는 **명령 없이 SDK 문서·함수 목록으로만** 조사한다.
- **manifest 최소 필드:** 장치 ID(모델·시리얼·펌웨어), SDK/드라이버 버전, 호스트 OS, 해상도, 깊이 스트림 해상도, (Gemini) 시차 탐색 폭, 정합 모드, 캡처 시각, 파일명, sha256.
- **PASS 임계값은 아직 동결되지 않았다.** 수치를 임의로 정하지 말고 측정값만 기록하며 PASS/BLOCKED는 UNVERIFIED로 둔다.
- 참고 자료(이전 프로젝트 B-00 등)는 프로젝트 밖 경로를 근거로 쓰지 말고 `data/refs/`에 복사하고 `data/refs/README.md`에 원본·복사일을 적는다.
- **G01.5 입력용 조사(문서·소스 읽기만, 명령 호출 금지):** SDK 이동 명령의 파라미터와 단위(속도인지 이동 시간인지), 관절 한계의 출처, 토크 해제·정지 API. 참고 소스 사본은 이전 프로젝트 `data/refs/yahboom/src/`에 있으며 읽기 전용 grep으로 확인하고 인용하는 파일만 이 프로젝트 `data/refs/`로 복사한다.
