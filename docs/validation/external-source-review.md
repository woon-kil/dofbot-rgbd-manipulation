# 외부 공식자료 기반 설계 검토 (2026-11-12)

| 출처 | 확인한 사실 | 설계 반영 | 한계 |
|---|---|---|---|
| https://www.orbbec.com/products/stereo-vision-camera/gemini-335l/ | Gemini 335L은 RGB/Depth/Point Cloud, Orbbec SDK, 최적 Depth 0.25~6m 지원 | Gemini 전역 RGB-D adapter | 제품 사양은 현장 정확도 보장 아님 |
| https://github.com/orbbec/OrbbecSDK_ROS2/blob/v2-main/README.MD | Gemini 335L의 ROS2 SDK 지원 | 기존 캡처 재사용 검토 | 실제 설치 버전/토픽/정합은 G01 검증 |
| https://moveit.picknik.ai/main/doc/examples/hand_eye_calibration/hand_eye_calibration_tutorial.html | eye-to-hand 및 eye-in-hand 보정 구분, CameraInfo·정확한 프레임 필요 | 보정 분리 및 holdout | DOFBOT 전용 보정 정확도는 제공하지 않음 |

미검증: DaBai 정확한 카메라 모델/드라이버/Depth API, DOFBOT SKU·관절 수·SDK 회독/URDF, Gemini 고정 시야에서 DOFBOT 작업대가 보이는지, 실제 보정 성능. 문서상 근거를 실기기 PASS로 오인하지 않는다.
