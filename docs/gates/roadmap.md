# Gate Roadmap v2 — STRICT_LINEAR (G03/G04 순서만 ADR-005 결정 2에 따른 예외)

| Gate | 목표 | PASS 증거 | FAIL/BLOCKED |
|---|---|---|---|
| G00 | DOFBOT SKU/서보/FK/SDK, DaBai 사양, Gemini FW/SDK 기준선 | 장치 인벤토리·SDK read-only log | 관절/카메라 모델 불명 |
| G01 | 두 RGB-D 취득 및 Gemini 기존 위치 시야 확인 | 각 RGB/depth/K/정합/시간/작업영역 마스크·깊이 유효율. **각 카메라의 깊이 스트림 해상도·(Gemini) 시차 탐색 폭·정합 모드·작업영역 내 유효율을 설정값으로 함께 기록** (데이터시트상 335L 최소 깊이는 이 설정에 의존) | `BLOCKED_GLOBAL_FOV`, 미정합 |
| G01.5 | 보정 이동 안전 승인 (ADR-005 결정 1-B) | 관절 한계표, 속도·가속도 상한, 정지 수단, 간섭 검토, 운영자 절차, 자세 목록 검토 기록 (상한 수치 미정) | `BLOCKED_CAL_MOTION_SAFETY` |
| G02 | DOFBOT SDK 회독↔FK/URDF 검증 (G01.5 PASS 전: 현재 자세 읽기만, 이후: 승인된 자세 목록에 한해 저속 이동) | 자세 샘플, joint mapping, FK 오차 보고 | `BLOCKED_FK` |
| G03 | Gemini eye-to-hand 보정 | 베이스 기준점 대응, 독립 holdout 3D 잔차 | `BLOCKED_BASE_DATUM` |
| G04 | DaBai eye-in-hand 보정 | 다양성 있는 pose pairs, 보정 및 holdout | `BLOCKED_HAND_EYE` |
| G05 | 정지 검체 교차 검증 | 두 베이스 좌표·잔차 분포, outlier/가림 사례 | `BLOCKED_DISAGREEMENT` |
| G06 | 5축 도달성·충돌·작업대 제약 dry-run | IK 잔차/관절한계/충돌·거부 사례 | `BLOCKED_PLANNING` |
| G07 | 운영자 승인 후 단일 시나리오 저속 P&P | 명령/회독/전후 RGB-D, 성공/실패 로그 | `BLOCKED_EXECUTION` |
| G08 | Gemini 전역+DaBai 근접 검증/실패 복구 | 동일 조건 반복 시험, 단독 대비 개선 지표 | `BLOCKED_DUAL_VALIDATION` |

**각 Gate:** 선행 Gate PASS 없이는 다음 Gate의 실제 명령 금지. G00·G01·G06에는 이동 명령 없음. G01.5 PASS 후에만 G02~G05에서 승인된 보정 자세 목록에 한한 저속 이동 허용(ADR-005 결정 1-B, `execution.calibration_motion_approved`). G07 이전에 안전계획·전원차단 절차·관절 제한·작업대/자기충돌 모델·운영자 확인 필수. 재시험은 실패 근거를 보존하고 새로운 evidence 버전 생성.

**실험:** 첫 물체는 단순한 무광 블록 1개, 고정 조명, 고정 배치 영역. 반복 20회는 제안 실험 설계이지 이미 성공한 결과가 아님.


**결정(2026-11-12):** 팔 자세 확보는 1-B 보정용 저속 이동(ADR-005 결정 1). **결정 2:** Gemini 베이스 기준 방식은 2-A→2-B→2-C 순서로 시험해 처음 합격한 방식을 채택(시험 규칙은 ADR-005). 2-A 시험 때만 G04가 G03보다 먼저이므로 "STRICT_LINEAR"는 이 경우 예외다. **미결:** G06 이전 충돌 검사 범위(결정 3), 합격 임계값 동결. PASS 임계값(깊이 유효율, FK 오차, 3D 잔차)은 아직 동결되지 않았다(`docs/validation/acceptance-matrix.md`).
