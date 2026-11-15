# 실행 스크립트 스냅샷 (재현용)

`scripts/`는 `.gitignore` 대상(사용자 지시 2026-11-12)이라 git에 없다. 이 폴더는 **정책을 바꾸지 않고** P1 `--real`(2026-11-15 02:25)과 dry run에 쓴 실행 경로의 파일을 같은 날 복사해 추적 가능하게 둔 사본이다. 원본 경로·이름은 같고(`scripts/<파일>`), `MANIFEST.sha256`이 각 파일의 sha256이다(`shasum -a 256 -c MANIFEST.sha256`).
- `261115/`: `g01_s42_plan.py`(P1 세트 12·P1-b2 세트 13 포함), `g01_5b4_s42_run.sh`(래퍼, P1 충돌 검증 연결), `g01_5b4_s42_test.py`(러너, P1 안전 질문), `gripper_adapter.py`(`src/control/gripper_adapter.py`와 동일), `g01_s4_2_path.py`, `g01_s3_path.py`, `g01_pose_generator.py`, `g01_5b4_joint_test.py`, `g01_dabai_probe.py`, `g01_dabai_expsweep.py`, `g01_gemini_remote.sh`, `validate_design.py`.
- 이후 스크립트를 바꾼 실행이 있으면 새 날짜 폴더에 다시 복사한다. 시크릿·호스트 주소는 포함하지 않는다(래퍼는 `.env`의 `LG_JETSON_HOST` 변수명만 참조).
- 정책 결정(`scripts/` 추적 해제/부분 추적)은 사용자 몫이다: `docs/gates/열린-결정-정리표.md` #9.
