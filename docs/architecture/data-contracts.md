# 데이터 계약 v2

| 타입 | 필수 필드 | 거부 조건 |
|---|---|---|
| `RgbdFrame` | camera_id(`gemini_335l`/`dabai`), frame_id, rgb, depth, K, depth_unit_m, aligned_to_rgb, capture_time_ns, clock_domain, sequence, calibration_id | 미정합, 무효 K, 시각 누락, stale frame |
| `RobotState` | joint_names, joint_positions_rad, capture_time_ns, clock_domain, sdk_readback, fk_model_id | FK 불명, 관절 순서 불일치, 시각 보간 불가 |
| `ObjectObservation` | object_id, camera_id, xyz_m, frame_id, depth_valid_fraction, timestamp, uncertainty, mask_id | invalid depth, ambiguous object |
| `Calibration` | transform_parent, transform_child, matrix4x4, calibration_id, dataset_id, holdout_metrics, approved | 미승인, 프레임 역전, 비직교 회전 |
| `GraspCandidate` | object_id, tcp_pose_base, approach_vector, jaw_width, ik_residual, collision_status(`SIMPLE_CHECKED`/`FULL_CHECKED`/`UNVERIFIED`), evidence_ids | 5축 IK 불가, 충돌 `UNVERIFIED` |
| `ExecutionRecord` | plan_id, approvals, safety_interlocks, command, readback, result, before_after_ids | 승인 없음, stale calibration |

- `camera_id`와 `frame_id`를 혼동하지 않는다. Gemini RGB frame과 Depth optical frame을 무조건 같다고 가정하지 않는다.
- 동일 시계가 아니면 장치별 타임스탬프를 직접 빼지 않는다. 정지 장면 비교 또는 시계 오프셋/지연 검증이 필요.
- 유효 Depth 마스크와 미터 단위 환산, 이미지와 K의 동일 해상도 보장.
- 모든 calibration은 `frame_id`, parent/child, 단위, 수집 시각, 입력 해시와 함께 immutable evidence 저장.
- 실행 임계값은 `docs/validation/acceptance-matrix.md`에서 사전 동결; 현재 수치 일부는 제안값이며 실측 PASS 아님.
