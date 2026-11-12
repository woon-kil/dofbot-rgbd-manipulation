# PROJECT-STATUS — DOFBOT RGB-D Manipulation (Dual Camera v2)

프로젝트 진행 상황의 단일 진실의 원천(SSOT). 마지막 갱신: 2026-11-12 (G00 읽기 전용 인벤토리 완료).

**한 줄 상태:** 설계 초안 + 정적 검증 단계. G00 읽기 전용 인벤토리를 수행해 판정 `PARTIAL_READONLY`(G00 PASS 아님, 관절 회독·FK는 G02 이월). 로봇 이동·쓰기 호출은 0건이다. 코드는 SE(3) 변환 계약뿐이다.

## 품질 지표
| 지표 | 값 | 출처 |
|---|---|---|
| 정적 검증 | `PASS_STATIC_ONLY` (파일 존재 + 예시 설정 안전 플래그 + 단위 테스트) | `python3 scripts/validate_design.py` |
| 단위 테스트 | 15개 통과 | `python3 -m unittest discover -s tests` |
| 실기기 Gate | G00 `PARTIAL_READONLY`, G01~G08 미수행 | `docs/261112-0908-G00-인벤토리-결과보고서.md` |
| 안전 기본값 | `dry_run=true`, `hardware_enabled=false`, `calibration_motion_approved=false` | `configs/project.example.json` |
| 외부 검토 | 진단보고서·플랜·ADR-005 변경은 advisor 검토. codex 검토는 한도 소진으로 미수행 | `docs/261112-0851-세션이슈종합보고서.md`, `docs/261112-0902-G00-인벤토리-실행플랜.md`, `docs/261112-0908-G00-인벤토리-결과보고서.md` |

## 열린 결정과 차단 사항
| 항목 | 상태 | 비고 |
|---|---|---|
| ADR-005 결정 1: 팔 자세 확보 | **결정: 1-B 보정용 저속 이동** | 상한은 이전 값(90°, 60 ms/°) 이내에서 G01.5a 소각도 실측 후 G01.5b에서 동결. J3·J4 ≥ 5°, 전원 차단 담당 상주, 낙하 받침 |
| ADR-005 결정 2: Gemini 베이스 기준 | **결정: 2-A→2-B→2-C 순서 시험, 처음 합격한 방식 채택** | 합격 임계값 미동결 |
| ADR-005 결정 3: G06 이전 충돌 검사 범위 | **결정: 3-A 간이 검사** | 3-B는 G02 후 G06 착수 전 재결정 |
| ADR-005 결정 4: 임계값 동결 방식 | **결정: 물리 여유 역산 + 파일럿, G03 시험 전 동결. P&P 30회 실패 0건** | 열린 손 간격 캘리퍼 재측정 필요(사용자 협조) |
| 합격 임계값 숫자 | **미동결** | 위 절차 진행 후 |
| DaBai 모델 | **확인: DaBai DCW2** (USB 제품명·드라이버, 펌웨어 RD1017) | G00 |
| DOFBOT 관절 수 | 5축 + 그리퍼: SDK(서보 id 1~6)와 URDF 일치, 물리 동작 UNVERIFIED(G02) | G00 |
| DaBai 장착 링크 | URDF상 `Arm4_Link` 고정 자식(손목 회전 Arm5 아래가 아님). 실물 장착은 UNVERIFIED | G00 |
| Gemini 펌웨어·깊이 모드 | UNVERIFIED (G01) | G00 |

## 체크리스트

### 설계·문서
- [x] 시스템 설계, 데이터 계약, 보정 프로토콜, ADR-001~004, 로드맵, 위험 등록부 (v2)
- [x] 설계 진단보고서 (`docs/261112-0748-설계진단보고서.md`)
- [x] 진단 반영: 문서 불일치 정정, G01 증거 항목 확장, 위험 R13~R20, 작업의뢰서 보강
- [x] ADR-005 작성 및 결정 1, 2 반영
- [x] G01.5 "보정 이동 안전 승인" Gate 신설
- [x] ADR-005 결정 3, 4 반영 (3-A, 임계값 동결 절차), G01.5 승인 문서 양식
- [ ] 열린 손 간격 캘리퍼 재측정(사용자 협조, 오차 예산 선행 조건)
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
- [~] G00 장치 인벤토리: **읽기 전용 범위 완료 (PARTIAL_READONLY)**. 이월: 관절 회독 길이·서보별 움직임·FK 검증·토크 해제 회독·`time` 단위 확정(G02), Gemini 펌웨어(G01)
- [ ] G01 두 RGB-D 취득, Gemini 시야 (깊이 스트림 해상도·시차 탐색 폭·정합 모드·유효율 기록)
- [ ] G01.5a 소각도 시험 승인, G01.5b 보정 이동 안전 승인
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
- [x] LICENSE: MIT (저작권자 표기 `woon-kil`, 변경 가능)

## 다음 작업 후보
1. G01: 두 카메라 취득·Gemini 시야(펌웨어·깊이 스트림 해상도·시차 탐색 폭·정합 모드·작업영역 유효율 기록). Jetson 디스크 여유 8.6 GB 주의.
2. G01.5a/b: 승인 문서(`docs/gates/G01.5-승인-문서-양식.md`)를 채우고 소각도 실측으로 상한을 동결한다.
3. 열린 손 간격 재측정 → 오차 예산 → 파일럿 → G03 시험 전 임계값 동결.

## 세션 이력
| 날짜 | 요약 | 문서 |
|---|---|---|
| 2026-11-12 | 설계 진단보고서 작성·검증, 진단 반영(코드·문서), ADR-005 결정 1·2, GitHub public 저장소 생성, G00 읽기 전용 인벤토리 | `docs/261112-0748-설계진단보고서.md`, `docs/261112-0811-진단반영플랜.md`, `docs/261112-0851-세션이슈종합보고서.md`, `docs/261112-0902-G00-인벤토리-실행플랜.md`, `docs/261112-0908-G00-인벤토리-결과보고서.md` |
