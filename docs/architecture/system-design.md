# 듀얼 RGB-D 시스템 아키텍처 v2

## 고정된 시스템 역할
- **Gemini 335L (eye-to-hand)**: 작업대 전체 탐색, 물체 후보, 로봇 접근 전 scene/depth 검증, 배치 후 결과 확인. 카메라 고정 위치 유지. 작업 공간이 안 보이면 `BLOCKED_GLOBAL_FOV`로 기록; 카메라 이동 금지.
- **DaBai (eye-in-hand)**: 접근 전 근접 재관측, 파지 후보 보정. 정확한 모델·Depth 제공·스트림 규격은 G00에서 실기기 확인. 움직임 중 촬영은 q(t) 및 시각 일치 필요.
- **Robot**: DOFBOT Pro 관절 수/실제 서보 채널·FK·그리퍼를 분리 확인. 팔 5축 가설은 미확정이며 6D 임의 자세는 보장하지 않음.

## 데이터 흐름
Gemini RGB-D → 정합/품질검사 → 객체 위치 `^G p` → `^B T_G` → 베이스 후보 → 5축 IK 가능성 검사 → DaBai RGB-D 재관측 → `^B T_E(q_t) ^E T_D ^D p` → 독립 두 측정 비교 → 계획/충돌검사 → 사용자 승인 → 저속 명령 → Gemini 사후 검증.

## 변환 그래프 (컬럼 벡터, `^A T_B`는 B좌표를 A좌표로)
- `^B p = ^B T_G · ^G p` (Gemini: 고정 외부 보정)
- `^B p = ^B T_E(q_t) · ^E T_D · ^D p` (DaBai: eye-in-hand)
- `^B T_G`와 `^E T_D`는 별도 추정·별도 검증·버전 관리. G와 D는 **각 RGB 정합 Depth의 optical frame**; 실제 frame ID는 측정 후 확정.
- ROS optical convention: +x 우측 +y 아래 +z 전방. SI m/rad. 이미지 픽셀→3D는 보정된 K와 유효 depth 사용. RGB-depth 정합 없는 혼합 금지.
- 두 카메라의 관측은 동일 물체·동일 상태·시간 정렬이 확인될 때만 비교. 서로 다른 시각의 이동 물체를 융합하지 않는다.

## 제어 정책
- Gemini가 전역 목표를 만들고 DaBai가 근접 목표를 확인하는 **순차적 계층형 사용**이 기본. 두 카메라 간 하드웨어 동기화 가정 없음.
- 두 카메라의 3D 목표가 불일치하거나 가림/Depth 무효/관절 상태 지연 시 `BLOCKED_DISAGREEMENT` 및 명령 금지. 자동 평균/가중 융합 금지.
- 기본 dry-run, 실제 SDK command adapter 없음. 시야/IK/충돌/작업대 평면/전원 차단 절차 확인 전 하드웨어 활성화 금지. 보정을 위한 이동은 G01.5 안전 승인 후 승인된 자세 목록에 한해서만 허용한다(ADR-005 결정 1-B 및 플래그 조합표).
- 사람 근접 작업, 무작위 물체 6D, 고속 작업, 온라인 학습 제어는 범위 밖.

## 선택 및 실패 시 정책
Gemini 시야 미확보: 듀얼 카메라 Gate FAIL, DaBai 단독 별도 실험만 가능(듀얼 PASS 주장 금지). DaBai 불안정: Gemini 단독 dry-run은 가능하나 듀얼 P&P 승인 불가. 카메라 보정 실패 시 2D 픽셀 비례 매핑으로 우회하지 않는다.

## 기술
초기: Python/OpenCV/NumPy, 저장/재생 검증. Gemini 기존 OrbbecSDK_ROS2 기록 재사용 가능하되 드라이버 버전과 프레임 계약 재확인. ROS2 tf2/MoveIt2는 검증된 URDF/SDK 제어 연결 후 선택. AI grasp/VLA는 확장 단계.

## 실행 토폴로지와 p620 활용 (ADR-006)
Jetson에는 DOFBOT SDK와 DaBai 드라이버, p620에는 Gemini 335L 취득(전용 docker 컨테이너)이 있고, Mac은 작업·증거 저장을 맡는다. p620은 다른 프로젝트와 공유하는 GPU 서버이므로 **이 프로젝트 전용 docker 컨테이너·작업 폴더 안에서만** 사용하며 타 프로젝트 컨테이너·호스트 설정은 건드리지 않는다. GPU 사용(인지·IK 등)은 해당 단계에서 근거와 권장안을 제시한 뒤 결정한다. 두 호스트의 시계는 별개이므로 장치 간 타임스탬프를 직접 비교하지 않는다. 상세 규칙: `docs/adr/ADR-006-p620-docker.md`.
