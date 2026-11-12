# Gate Roadmap v2 — STRICT_LINEAR (G03/G04 순서만 ADR-005 결정 2에 따른 예외)

| Gate | 목표 | PASS 증거 | FAIL/BLOCKED |
|---|---|---|---|
| G00 | DOFBOT SKU/서보/FK/SDK, DaBai 사양, Gemini FW/SDK 기준선 | 장치 인벤토리·SDK read-only log | 관절/카메라 모델 불명 |
| G01 | 두 RGB-D 취득 및 Gemini 기존 위치 시야 확인 | 각 RGB/depth/K/정합/시간/작업영역 마스크·깊이 유효율. **각 카메라의 깊이 스트림 해상도·(Gemini) 시차 탐색 폭·정합 모드·작업영역 내 유효율을 설정값으로 함께 기록** (데이터시트상 335L 최소 깊이는 이 설정에 의존) | `BLOCKED_GLOBAL_FOV`, 미정합 |
| G01.5a | 소각도 시험 승인 (ADR-005 결정 1-B) | 단일 관절·단일 자세·최대 쓰기 횟수가 적힌 승인 문서, 전원 차단 담당·낙하 받침 확인. 증거: `time` 단위와 도착 응답 실측 | `BLOCKED_CAL_MOTION_SAFETY` |
| G01.5b | 보정 이동 안전 승인 | 관절 한계표, G01.5a 실측으로 동결한 상한(이전 값 90°·60 ms/° 이내), J3·J4 ≥ 5° 자세 목록(이전 프로젝트의 미검증 후보값), 정지 수단·낙하 받침, 간이 충돌 검사(3-A), 운영자·전원 차단 담당 절차, 승인 문서(`docs/gates/G01.5-승인-문서-양식.md`) | `BLOCKED_CAL_MOTION_SAFETY` |
| G02 | DOFBOT SDK 회독↔FK/URDF 검증 (G01.5 PASS 전: 현재 자세 읽기만, 이후: 승인된 자세 목록에 한해 저속 이동) | 자세 샘플, joint mapping, FK 오차 보고 | `BLOCKED_FK` |
| G03 | Gemini eye-to-hand 보정 | 베이스 기준점 대응, 독립 holdout 3D 잔차 | `BLOCKED_BASE_DATUM` |
| G04 | DaBai eye-in-hand 보정 | 다양성 있는 pose pairs, 보정 및 holdout | `BLOCKED_HAND_EYE` |
| G05 | 정지 검체 교차 검증 | 두 베이스 좌표·잔차 분포, outlier/가림 사례 | `BLOCKED_DISAGREEMENT` |
| G06 | 5축 도달성·충돌·작업대 제약 dry-run | IK 잔차/관절한계/충돌·거부 사례 | `BLOCKED_PLANNING` |
| G07 | 운영자 승인 후 단일 시나리오 저속 P&P | 명령/회독/전후 RGB-D, 성공/실패 로그 | `BLOCKED_EXECUTION` |
| G08 | Gemini 전역+DaBai 근접 검증/실패 복구 | 동일 조건 반복 시험, 단독 대비 개선 지표 | `BLOCKED_DUAL_VALIDATION` |

**각 Gate:** 선행 Gate PASS 없이는 다음 Gate의 실제 명령 금지. G00·G01·G06에는 이동 명령 없음. G01.5a 승인 범위 안의 소각도 시험, G01.5b PASS 후에만 G02~G05에서 승인된 보정 자세 목록에 한한 저속 이동 허용(ADR-005 결정 1-B, `execution.calibration_motion_approved`). G07 이전에 안전계획·전원차단 절차·관절 제한·작업대/자기충돌 모델·운영자 확인 필수. 재시험은 실패 근거를 보존하고 새로운 evidence 버전 생성.

**실험:** 첫 물체는 단순한 무광 블록 1개, 고정 조명, 고정 배치 영역. 반복은 30회 실패 0건을 기준으로 한다(ADR-005 결정 4). 아직 성공한 결과가 아니다.


**결정(2026-11-12):** 팔 자세 확보는 1-B 보정용 저속 이동(ADR-005 결정 1). **결정 2:** Gemini 베이스 기준 방식은 2-A→2-B→2-C 순서로 시험해 처음 합격한 방식을 채택(시험 규칙은 ADR-005). 2-A 시험 때만 G04가 G03보다 먼저이므로 "STRICT_LINEAR"는 이 경우 예외다. **결정 3:** G06 이전 충돌 검사는 3-A 간이 검사, 3-B는 G02 후 G06 착수 전 재결정. **결정 4:** 임계값은 물리 여유 역산 + 파일럿 점검 후 G03 시험 시작 전 동결. **결정 미결은 없으나 수치(소각도 크기, 이동 상한, PASS 임계값)는 미동결이다.** PASS 임계값(깊이 유효율, FK 오차, 3D 잔차)은 위 절차로 아직 동결되지 않았다(`docs/validation/acceptance-matrix.md`).

**도달 영역(진단 2026-11-12, 사용자 보고):** G01에서 확정한 Gemini 작업영역 마스크 v1은 "작업 영역"이며 로봇팔 도달 가능 영역의 최대치인지는 알 수 없다(R24). G06 도달성 dry-run 전에 URDF·SDK 관절 한계로 도달 포락선을 산정하고, G03 이후 Gemini 영상에 투영해 시야 포함 여부를 확인한다.

**접근 기울기·시험 영역(ADR-007, 2026-11-12):** 파지 접근은 수직 아래 기준 ≤15°, 시험 영역은 마스크 v1 ∩ 이론 도달 반경 약 203 mm 안쪽이다. 안쪽 여유 폭은 G02·G03 후 정한다. G06 dry-run은 이 한도를 입력으로 쓴다.
