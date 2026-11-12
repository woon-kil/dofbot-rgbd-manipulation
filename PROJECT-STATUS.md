# PROJECT-STATUS — DOFBOT RGB-D Manipulation (Dual Camera v2)

프로젝트 진행 상황의 단일 진실의 원천(SSOT). 마지막 갱신: 2026-11-12 (G00 읽기 전용 인벤토리, G01 취득 완료, ADR-006 p620 활용).

**한 줄 상태:** 설계 초안 + 정적 검증 단계. G00 `PARTIAL_READONLY`(관절 회독·FK는 G02 이월), G01 `PROVISIONAL`(두 카메라 취득 완료, 작업영역 마스크 v1 사용자 확인·동결). 로봇 이동·쓰기 호출은 0건이다. 코드는 SE(3) 변환 계약뿐이다.

## 품질 지표
| 지표 | 값 | 출처 |
|---|---|---|
| 정적 검증 | `PASS_STATIC_ONLY` (파일 존재 + 예시 설정 안전 플래그 + 단위 테스트) | `python3 scripts/validate_design.py` |
| 단위 테스트 | 15개 통과 | `python3 -m unittest discover -s tests` |
| 실기기 Gate | G00 `PARTIAL_READONLY`, G01 `PROVISIONAL`, G01.5~G08 미수행 | `docs/261112-0908-G00-인벤토리-결과보고서.md`, `docs/261112-1059-G01-취득-결과보고서.md` |
| 안전 기본값 | `dry_run=true`, `hardware_enabled=false`, `calibration_motion_approved=false` | `configs/project.example.json` |
| 외부 검토 | 진단보고서·플랜·ADR-005 변경은 advisor 검토. codex 검토는 한도 소진으로 미수행 | `docs/261112-0851-세션이슈종합보고서.md`, `docs/261112-0902-G00-인벤토리-실행플랜.md`, `docs/261112-0908-G00-인벤토리-결과보고서.md`, `docs/261112-0940-G01-취득-실행플랜.md`, `docs/261112-1059-G01-취득-결과보고서.md` |

## 열린 결정과 차단 사항
| 항목 | 상태 | 비고 |
|---|---|---|
| ADR-005 결정 1: 팔 자세 확보 | **결정: 1-B 보정용 저속 이동** | 상한은 이전 값(90°, 60 ms/°) 이내에서 G01.5a 소각도 실측 후 G01.5b에서 동결. J3·J4 ≥ 5°, 전원 차단 담당 상주, 낙하 받침 |
| ADR-005 결정 2: Gemini 베이스 기준 | **결정: 2-A→2-B→2-C 순서 시험, 처음 합격한 방식 채택** | 합격 임계값 미동결 |
| ADR-005 결정 3: G06 이전 충돌 검사 범위 | **결정: 3-A 간이 검사** | 3-B는 G02 후 G06 착수 전 재결정 |
| ADR-005 결정 4: 임계값 동결 방식 | **결정: 물리 여유 역산 + 파일럿, G03 시험 전 동결. P&P 30회 실패 0건** | 열린 손 간격 캘리퍼 재측정 필요(사용자 협조) |
| 합격 임계값 숫자 | **미동결** | 오차 예산 초안(PROPOSED)에서 ψ 가정 선택 대기 |
| DaBai 모델 | **확인: DaBai DCW2** (USB 제품명·드라이버, 펌웨어 RD1017) | G00 |
| DOFBOT 관절 수 | 5축 + 그리퍼: SDK(서보 id 1~6)와 URDF 일치, 물리 동작 UNVERIFIED(G02) | G00 |
| DaBai 장착 링크 | URDF상 `Arm4_Link` 고정 자식(손목 회전 Arm5 아래가 아님). 실물 장착은 UNVERIFIED | G00 |
| Gemini 펌웨어·SDK | **확인: 펌웨어 1.4.60, SDK 2.10.6, ISP 20241125** | G01 |
| Gemini 시차 탐색 폭 | UNVERIFIED (launch 파라미터 기본 -1, 로그에 값 없음) | G01 |
| RGB-D 정합 | 두 카메라 모두 정합 설정에서 해상도·K·프레임 ID 일치 확인. 위치 정확도는 미검증 | G01 |
| DaBai 작업영역 시야 | 현재 자세(직립 park)에서는 벽·천장(약 1.26 m)만 본다. 근접 시야는 팔 이동 후 확인 | G01 |
| 작업영역 마스크 v1 ↔ 도달 영역 | **관계 미상**(사용자: 최대 도달 영역인지 모름). 도달 포락선 산정 필요 (R24) | 사용자 보고 |
| p620 활용 | **결정: 허용 + 전용 docker 컨테이너 격리** (ADR-006). GPU 사용 범위는 미결정 | 사용자 지시 |

## 체크리스트

### 설계·문서
- [x] 시스템 설계, 데이터 계약, 보정 프로토콜, ADR-001~004, 로드맵, 위험 등록부 (v2)
- [x] 설계 진단보고서 (`docs/261112-0748-설계진단보고서.md`)
- [x] 진단 반영: 문서 불일치 정정, G01 증거 항목 확장, 위험 R13~R20, 작업의뢰서 보강
- [x] ADR-005 작성 및 결정 1, 2 반영
- [x] G01.5 "보정 이동 안전 승인" Gate 신설
- [x] ADR-005 결정 3, 4 반영 (3-A, 임계값 동결 절차), G01.5 승인 문서 양식
- [~] 열린 손 간격 재측정: **48.5 mm 수령(사용자 보고, 최대 개방 아님 → 하한값)**. 명령 각도 대응·측정면·3회 판독·사진 미확인, 대응표는 G01.5a 범위에 포함 (`evidence/G01/open_hand_gap_measurement.json`)
- [ ] 오차 예산 정렬 가정(ψ) 선택 — 초안 `docs/261112-1103-오차예산-초안.md` (권장 B: ψ ≤ 5°, 합성 σ ≤ 2.24 mm. G는 하한값 48.5 mm)
- [x] Gemini 작업영역 마스크 v1 사용자 확인·동결 (`evidence/G01/workspace_mask_v1.json`)
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
- [x] ADR-006 p620 활용·docker 격리 반영 (`docs/adr/ADR-006-p620-docker.md`)
- [~] G00 장치 인벤토리: **읽기 전용 범위 완료 (PARTIAL_READONLY)**. 이월: 관절 회독 길이·서보별 움직임·FK 검증·토크 해제 회독·`time` 단위 확정(G02), Gemini 펌웨어(G01)
- [ ] G01 두 RGB-D 취득, Gemini 시야 (깊이 스트림 해상도·시차 탐색 폭·정합 모드·유효율 기록)
- [ ] G01.5a 소각도 시험 승인, G01.5b 보정 이동 안전 승인
- [ ] 도달 포락선 산정(URDF·SDK 관절 한계 기반, 실물 이동 없이) — 마스크 v1과의 포함 관계 확인
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
1. 사용자 협조 1건(열린 손 간격 재측정)과 PASS 임계값 동결 후 G01을 확정한다. Jetson 디스크 여유 8.6 GB 주의.
2. G01.5a/b: 승인 문서(`docs/gates/G01.5-승인-문서-양식.md`)를 채우고 소각도 실측으로 상한을 동결한다.
3. 열린 손 간격 재측정 → 오차 예산 → 파일럿 → G03 시험 전 임계값 동결.

## 세션 이력
| 날짜 | 요약 | 문서 |
|---|---|---|
| 2026-11-12 | 설계 진단보고서 작성·검증, 진단 반영(코드·문서), ADR-005 결정 1·2, GitHub public 저장소 생성, G00 읽기 전용 인벤토리, G01 두 카메라 취득, ADR-006 | `docs/261112-0748-설계진단보고서.md`, `docs/261112-0811-진단반영플랜.md`, `docs/261112-0851-세션이슈종합보고서.md`, `docs/261112-0902-G00-인벤토리-실행플랜.md`, `docs/261112-0908-G00-인벤토리-결과보고서.md`, `docs/261112-0940-G01-취득-실행플랜.md`, `docs/261112-1059-G01-취득-결과보고서.md` |
