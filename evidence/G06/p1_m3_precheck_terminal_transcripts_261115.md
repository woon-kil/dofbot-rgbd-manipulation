# 큰 이동 M3 실행 전 점검: 사용자 실행 터미널 출력 전사 (2026-11-15)

**이 파일은 원본 기록이 아니다.** 사용자가 실기기에서 실행한 `bash scripts/p1_m3_precheck.sh`의 터미널 출력을 사용자가 대화에 붙여 준 그대로 옮긴 **전사본**이다. 원래 기록 파일 `evidence/G06/p1_preflight_261115-105756.json`(실패)과 `p1_preflight_261115-110915.json`(통과)은 세션 A가 시험 뒷정리에서 와일드카드로 **잘못 삭제**했다(추적되지 않던 파일이라 복구 불가). 삭제 사실과 경위는 `docs/handoff/A-결과.md`에 적었다. 남아 있는 원본 증거: Jetson 설치본 함수 소스 `data/refs/jetson-Arm_serial_servo_read-261115-105756.py`·`-110915.py`(gitignore), dry run 기록 `evidence/G01/raw/g01_5b4_s4214_result-dry_run-261115-105808.json`.

## 1차 실행 (10:57, 게이트 변경 전: `scene` 실패)
```
== 1/3 래퍼 --check
사전 검증 통과(대화·쓰기 없음)
== 2/3 Gemini 사진(m3_261115-105709)
(촬영 성공: color/depth 약 30 Hz, 컨테이너 정리 0개 남음) 사진 확인 (720, 1280, 3)
== 3/3 출발 전 공통 점검
현재 회독(J1~J6): 89,91,92,91,90,179
설치본 함수 소스 저장: data/refs/jetson-Arm_serial_servo_read-261115-105756.py
[통과] readback_vs_K1 [(1, 89.0, 0.0), (2, 91.0, 0.0), (3, 92.0, 1.0), (4, 91.0, 0.0), (5, 90.0, 0.0), (6, 179.0, 0.0)]
[실패] scene {'camera_shift_px': 2.463, 'camera_shift_tol_px': 0.5, 'phase_response': 0.894, 'marker_center_px': [700.25, 272.25], 'marker_ref_px': [702.8, 273.0], 'marker_diff_px': 2.66, 'marker_tol_px': 4.0, 'camera_still_ok': False, 'marker_ok': True}
[통과] scripts_sha256 {'n': 16, 'mismatch': [], 'manifest_sha256': '852602059d4048a7ed4cfaa331dc137319a0c0eeb3432adcb2a69ffafce90e8a'}
[통과] arm_lib_version {'reference_func_hash': '69d43556568ea13e', ..., 'status': 'match', 'installed_func_hash': '69d43556568ea13e', 'note': None}
[기록(정상)] raw_samples [(1, 0.033, 1.0), (2, 0.04, 1.0), (3, 0.035, 1.0), (4, 0.041, 1.0)]
출발 전 공통 점검: 실패(팔 이동 없이 중단)
점검 중단(종료 1): 팔은 움직이지 않았다.
```

## 2차 실행 (11:09, 게이트 변경 뒤: 통과)
```
== 1/3 래퍼 --check
사전 검증 통과(대화·쓰기 없음)
== 2/3 Gemini 사진(m3_261115-110829)
(촬영 성공: color/depth 약 30 Hz, 컨테이너 정리 0개 남음) 사진 확인 (720, 1280, 3)
== 3/3 출발 전 공통 점검
현재 회독(J1~J6): 89,92,92,91,90,179
설치본 함수 소스 저장: data/refs/jetson-Arm_serial_servo_read-261115-110915.py
[통과] readback_vs_K1 [(1, 89.0, 0.0), (2, 92.0, 1.0), (3, 92.0, 1.0), (4, 91.0, 0.0), (5, 90.0, 0.0), (6, 179.0, 0.0)]
[통과] scene {'block_move_in_scene_px': 0.36, 'marker_tol_px': 4.0, 'camera_shift_px': 2.467, 'camera_shift_xy_px': [-2.39, -0.61], 'camera_shift_ref_px': 0.5, 'phase_response': 0.888, 'response_min': 0.5, 'marker_center_px': [700.75, 272.5], 'marker_ref_px': [702.8, 273.0], 'marker_move_px': [-2.05, -0.5], 'camera_still_ok': False, 'marker_ok': True, 'response_ok': True}
[통과] scripts_sha256 {'n': 16, 'mismatch': [], 'manifest_sha256': '852602059d4048a7ed4cfaa331dc137319a0c0eeb3432adcb2a69ffafce90e8a'}
[통과] arm_lib_version {'reference_func_hash': '69d43556568ea13e', ..., 'status': 'match', 'installed_func_hash': '69d43556568ea13e', 'note': None}
[기록(정상)] raw_samples [(1, 0.029, 1.0), (2, 0.041, 1.0), (3, 0.039, 1.0), (4, 0.04, 1.0)]
출발 전 공통 점검: 통과
전체 통과. 다음(터미널에서 직접): bash scripts/g01_5b4_s42_run.sh --set 14
```
(위 두 실행의 `scripts_sha256`은 당시 MANIFEST `85260205…ce90e8a`다. 이후 `--manifest-src` 점검을 추가하면서 MANIFEST가 `42cb0307…da774`로 바뀌었다.)
