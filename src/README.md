# 모듈 구현 계약
하위 폴더는 구현 경계용 빈 패키지입니다. 장치 SDK/API를 확인하기 전 가짜 제어 API를 작성하지 않습니다.

- camera: capture/replay
- perception: segmentation/depth ROI
- calibration: hand-eye dataset and solver
- transforms: timestamped SE(3)
- kinematics: FK/IK and joint limits
- planning: collision scene and trajectory
- control: dry-run-first hardware adapter
- verification: independent before/after observation
