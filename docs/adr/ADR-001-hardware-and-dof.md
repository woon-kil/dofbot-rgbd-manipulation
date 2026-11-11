# ADR-001 하드웨어와 자유도 계약
**결정:** 독립 구동 관절 수는 확정하지 않는다. Yahboom 공식 소개는 6-DOF라고 명시하지만 실제 팔 관절·그리퍼 서보 수와 URDF/SDK 관절 정의는 별도 대조해야 한다.

## 필수 확인
- 모델/SKU, 제어보드, 펌웨어, DaBai 정확한 모델, RGB/Depth resolution, depth scale, alignment 지원 여부
- `get_joint_state` 실제 반환 길이, 물리 서보별 움직임, gripper 구동축, URDF movable joints, FK의 EE 링크
- 5축이면 제한된 orientation manifold에서만 목표 자세 허용; 6축이어도 reachability 검증 필수
- 사용자가 기존 프로젝트에서 사용한 `Arm4_Link` FK를 새 장치의 실제 tool frame으로 가정하지 말 것

**Gate:** 현장 서보/URDF/SDK 대조표가 있어야 G00 PASS.

## 작업 가정 (2026-11-12, 미검증)
판매 자료(2차 출처)는 DOFBOT Pro를 6-DOF, 즉 팔 서보 5개 + 그리퍼 서보 1개로 소개한다. 이를 **작업 가정(5축 팔 + 그리퍼)**으로만 사용하며 G00 실측 전까지 UNVERIFIED다. 문서 곳곳의 "5축" 표기는 이 가정에 기대며, G00 결과로 갱신한다.
