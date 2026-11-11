# PROJECT-STATUS — DOFBOT RGB-D Manipulation (Dual Camera v2)

프로젝트 진행 상황의 단일 진실의 원천(SSOT). 마지막 갱신: 2026-11-12.

**한 줄 상태:** 설계 초안 + 정적 검증 단계. 실기기 검증은 한 건도 수행하지 않았다(`NOT_RUN`). 코드는 SE(3) 변환 계약뿐이다.

## 품질 지표
| 지표 | 값 | 출처 |
|---|---|---|
| 정적 검증 | `PASS_STATIC_ONLY` (파일 존재 + 예시 설정 안전 플래그 + 단위 테스트) | `python3 scripts/validate_design.py` |
| 단위 테스트 | 15개 통과 | `python3 -m unittest discover -s tests` |
| 실기기 Gate | G00~G08 전부 미수행 | `docs/gates/roadmap.md` |
| 안전 기본값 | `dry_run=true`, `hardware_enabled=false`, `calibration_motion_approved=false` | `configs/project.example.json` |
| 외부 검토 | 진단보고서·플랜·ADR-005 변경은 advisor 검토. codex 검토는 한도 소진으로 미수행 | `docs/261112-0851-세션이슈종합보고서.md` |

## 열린 결정과 차단 사항
| 항목 | 상태 | 비고 |
|---|---|---|
| ADR-005 결정 1: 팔 자세 확보 | **결정: 1-B 보정용 저속 이동** | 안전 상한 수치(속도·가속도 등) 미정 |
| ADR-005 결정 2: Gemini 베이스 기준 | **결정: 2-A→2-B→2-C 순서 시험, 처음 합격한 방식 채택** | 합격 임계값 미동결 |
| ADR-005 결정 3: G06 이전 충돌 검사 범위 | **미결** | 옵션 3-A/3-B/3-C, 사용자 결정 대기 |
| 오차 예산·합격 임계값 | **미결** | 그리퍼 개구폭·물체 크기 필요 |
| DaBai 모델 | UNVERIFIED | 이전 프로젝트 자료상 DCW2 후보 |
| DOFBOT 관절 수 | UNVERIFIED | 판매 자료상 5축 + 그리퍼 (작업 가정) |

## 체크리스트

### 설계·문서
- [x] 시스템 설계, 데이터 계약, 보정 프로토콜, ADR-001~004, 로드맵, 위험 등록부 (v2)
- [x] 설계 진단보고서 (`docs/261112-0748-설계진단보고서.md`)
- [x] 진단 반영: 문서 불일치 정정, G01 증거 항목 확장, 위험 R13~R20, 작업의뢰서 보강
- [x] ADR-005 작성 및 결정 1, 2 반영
- [x] G01.5 "보정 이동 안전 승인" Gate 신설
- [ ] ADR-005 결정 3 (충돌 검사 범위)
- [ ] 오차 예산표 작성, 합격 임계값 동결 (`docs/validation/acceptance-matrix.md`)
- [ ] 증거 manifest 스키마 파일 (`evidence/`)
- [ ] 실행 호스트·SDK 호환성 표 (R18)

### 코드·테스트
- [x] `src/transforms/se3.py`: 강체 검증(직교성·det), `inverse`
- [x] 비자명 회전·역변환·반사 거부 테스트
- [x] `scripts/validate_design.py`: 필수 파일 + README 링크 검사
- [ ] camera / perception / calibration / kinematics / planning / control / verification 구현 (모두 빈 패키지)
- [ ] 의존성 고정(Python 버전, SDK 버전)

### 실기기 Gate (모두 미수행)
- [ ] G00 장치 인벤토리 (DOFBOT SKU/서보/FK/SDK, DaBai 사양, Gemini FW/SDK)
- [ ] G01 두 RGB-D 취득, Gemini 시야 (깊이 스트림 해상도·시차 탐색 폭·정합 모드·유효율 기록)
- [ ] G01.5 보정 이동 안전 승인
- [ ] G02 회독↔FK/URDF 검증
- [ ] G03 Gemini 베이스 보정
- [ ] G04 DaBai hand-eye 보정
- [ ] G05 정지 검체 교차 검증
- [ ] G06 도달성·충돌 dry-run
- [ ] G07 저속 P&P
- [ ] G08 듀얼 검증

### 저장소
- [x] GitHub public 저장소 생성: https://github.com/woon-kil/dofbot-rgbd-manipulation
- [x] `.gitignore` (`data/`, 증거 원본 이미지)
- [ ] LICENSE (소유자 결정 대기)

## 다음 작업 후보
1. G00 읽기 전용 인벤토리 수행(작업의뢰서 `docs/agent-work-order.md`). SDK 이동 명령의 파라미터·단위, 관절 한계 출처, 토크 해제·정지 API 조사 포함.
2. G00 결과로 G01.5 안전 상한 수치를 정한다.
3. 오차 예산 → 합격 임계값 동결 → 결정 3.

## 세션 이력
| 날짜 | 요약 | 문서 |
|---|---|---|
| 2026-11-12 | 설계 진단보고서 작성·검증, 진단 반영(코드·문서), ADR-005 결정 1·2, GitHub public 저장소 생성 | `docs/261112-0748-설계진단보고서.md`, `docs/261112-0811-진단반영플랜.md`, `docs/261112-0851-세션이슈종합보고서.md` |
