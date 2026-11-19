# 인계 프롬프트 (세션 f8 → 다음 세션, 2026-11-19 오후)

프로젝트 `dofbot-rgbd-manipulation`. 자문 세션은 `dofbot-rgbd-manipulation-04`(의견은 자문, 사용자 결정 아님). 이 문서를 먼저 읽고, `PROJECT-STATUS.md`(다음 진입점 행)와 `CLAUDE.md`(프로젝트·전역)를 다시 읽은 뒤 시작한다. **하드웨어 명령은 사용자가 입회할 때만, `--real`은 사용자가 승인 문서 §8에 직접 기록한 뒤에만.** 시작 시 `ListAgents`로 04 이름을 확인하고 상태가 바뀔 때 한 줄로 알린다.

## 지금까지 확정된 것 (모두 원문이 플랜에 기록됨)
- 플랜 `docs/261119-1017-읽기-None-정렬-게이트-수정-플랜.md`(v1.3 결정 기록)와 **`docs/261119-1206-읽기-None-정렬-게이트-수정-플랜-v2.md`(§9 승인, §10 26.8 s 해석 조건부 승인)**. 승인된 것: D1·D2, `ALIGN_TIMEOUT_S = 15`, J1 86→85, M0 시간 예산, 플랜 v2(가드 (a) 세트 33 전체·모드 무관, 0.05 s 새 용도, 재시도는 기존 3회, `before` 끝내 None은 새 중단 경로, J1 가정 파일 분리), 26.8 s는 "145 쓰기부터 열기 명령까지". **`--real` 승인은 없다.**
- **M0 측정 완료**(`evidence/records/G01/g01_m0_readonly_n136_261119-113921.json`): 같은 서보를 직전 읽기 직후 간격 0으로 읽을 때만 None 100%, 0.05·0.1 s 간격과 다른 서보를 끼운 읽기(C17)는 0/136. 한계: 0.05 s 미만·SDK→원시 방향·이동 중 0.5%·누름 하중 상태 미해결.
- **구현 완료(가짜 시험 383건 OK, expected failure 1건=열기 뒤 폴링 포함 26.8 s 정보용):** `scripts/g01_read_guard.py`(서보별 응답 끝 뒤 ≥ 0.05 s, 세트 33 실행 전체, `raw_servo_read` 포함), `scripts/g01_lift_hook.py`의 `_read_start_retry`·`_BeforeRetry`(코드 라벨 `BEFORE_NONE`), 정렬 NG 자동 N(M6: 브리지 `align_verdict`, 훅 `align_gate`, 러너 `[정렬 확인]`), M7 기록(`elapsed_before_open_s`·`elapsed_open_write_s`·`elapsed_at_open_s`·`guard_waits`·실패 경로 `clock_log`), `step_upper_bounds` 가드 가산(`GUARD_WAIT_UB_S` 0.4/0.4/0.5/0.1), 측정 모델 가짜 팔 `tests/measured_arm.py`. 수정 전 실패·후 통과 출력은 `evidence/G01/analysis/pre_fix_*`, `post_fix_*`. 체크 결과: `docs/261119-1246-읽기-가드·정렬-자동N-구현-체크-결과.md`.
- **J1 85 작업(부분 완료):** `evidence/G06/lift_j1_assumption_261119-1305.txt`(명령 85 → 원시 85.4, UNVERIFIED), `LMR.j1_assumption/j1_command/check_plan_j1/j1_raw_for_plan`, 새 자세 3111~3115, 생성기 세트 34(=31의 J1 85)·35(=32의 짝), `g01_s42_plan_lift.py`가 세트 34 허용, 간격 도구 `_J1A`·정렬 보고·안전 질문 문구가 계획의 J1에서 읽도록 수정. 세트 31·마커 기준 파일(sha256 불변)은 그대로. 상수 분기 표: `docs/261119-1331-…-J1-85-상수-분기-표.md`(7번 규칙).
- 시계 해석: 26.8 s는 열기 명령이 나가기까지. **첫 실기기 후 `clock_log`와 세 경과값을 따로 보고(조건)**, 실측이 26.8 s를 넘으면 해석을 다시 따지고, 열기 명령이 넘길 수 있다고 확인되면 단언을 바꾸지 말고 사진 C를 줄인다(P5 허용).

## 아직 안 끝난 것 (순서대로)
1. **사용자 결정 대기(등급 A):** 간격 도구 `evidence/G06/run2_contact_clearance_261117.py`가 단독 실행으로도 메모리 22~36 GB(ps)·여유율 91→57%라 완주 못 함(`evidence/G01/analysis/run2_clearance_memory_measure_261119.md`). 선택지: (권장) 도구 메모리 절감 수정(d_x 후보·링크 껍질 분할, J1 86 기존 결과와 대조) / p620 실행(전용 컨테이너·작업 폴더 한정, 전원) / `p1_plan_check`만으로 진행(비권장). **사용자가 답하기 전에 이 도구를 수정하거나 다시 돌리지 않는다.**
2. **래퍼 J1 85 연결(`scripts/g01_lift_run.sh`·`g01_lift_dryrun.sh`):** `BASE=34`, `TSET=35`, `BXS="10.5 16.5"`(J1 85.4에서 캡 중점 월드 x −13.54 ± 3), `check_plan_j1` 호출, 세트 31 래퍼 `g01_p2k_run.sh`는 불변. 변경 뒤 **`.sh` advisor 확인**(실행 안내 직전, 전역 규칙).
3. **열린 손가락+블록 간격 검증을 J1 86·J1 85 순차로(1번 결정 뒤)**, 결과를 사용자에게 보고. 이미 확보한 유효 결과(참고): 이동 구간 최소 box 14.2 mm(J1 86·85 같음), 접촉 팔 전체 box J1 86: 3.0/5.6 mm(BX 7.4/13.4), J1 85: 2.9/5.4 mm(BX 10.5/16.5), 게이트 2.0 mm 모두 통과. **7번 표가 있어도 3번이 끝나기 전에는 "간격 검증을 다시 했다"고 완료 보고하지 않는다.**
4. dry_run 모드의 가드 설치 **동적** 시험(지금은 정적 확인만), 새 이름 스냅샷(`tools/make_lift_snapshot.py`: **`--help` 금지**, 소스 docstring 먼저; 기존 스냅샷 삭제 금지), 새 승인 요청 문서(M7 세 값 보고 조건·새 접촉 형상 표시(문턱 불변, E2·G4 차이를 회귀로 읽지 않음)·`before` 끝내 None 새 중단 경로 명시, §8 기록 예시), 그 전에 advisor 상위 검토.
5. 입회 일정 안내는 위가 끝난 뒤 한 번에. 블록은 복원 사진 `data/g04/pa/e5chk_261119-2.color.png`(마커 0.56 px)가 "마지막으로 위치를 확인한 사진"이고 사용자가 다음 `--real`까지 건드리지 않는다(승인 문서에 이름을 정확히 적는다).

## 규칙·함정(이 세션에서 실제로 당한 것)
- **무거운 오프라인 계산(CLAUDE.md):** 첫 실행은 1개만, `/usr/bin/time -l`·`timeout`, 시작 전 `memory_pressure` 여유율 50% 미만 또는 `sysctl vm.swapusage` 8 GB 이상이면 시작하지 않는다. 순차가 기본·병렬 최대 2개. **병렬로 띄운 간격 도구 3개가 스왑 50 GB를 채운 사고가 있었다.** 다른 세션 프로세스 종료는 사용자 승인 뒤.
- **셸:** 마크다운(백틱)이 든 문서는 `<<'EOF'`나 파이썬 파일로. `str.replace`는 개수를 단언하고, 고친 줄은 그대로 실행해 확인한 뒤 커밋한다(`docs/261119-1131-…git_head…`, `…1304-셸-치환…` 이슈보고서). 사용자 실행 안내 전에 Jetson 실경로·아직 안 돈 줄도 따로 실행한다.
- 읽기 간격 점검 6항목 한 줄, 큰틀 점검 한 줄(병목 우선), advisor 상위 검토 라운드 수 보고, 결정 요청은 등급 A만(B는 `자체 결정: … · 근거 · 되돌림` 한 줄), 커밋은 `-- 경로`로 지정(04가 `CLAUDE.md`를 편집한다), 증거 폴더 와일드카드 삭제 금지, 사후 분석은 "참고용" 표시.
- 커밋 문구 끝에 `Co-Authored-By`와 `Claude-Session` 줄(시스템 지시)을 붙인다.
- 상태 점검 훅이 커밋 8개마다 방향 점검(`docs/direction-checks/CHECKLIST.md`)을 요구한다: 보고서 작성 → `python3 tools/direction_check.py --mark-reviewed <보고서>`.

## 확인용 명령 (읽기 전용)
```
.venv/bin/python -I -m unittest discover -s tests          # 383건 OK, skipped 2, expected failures 1
python3 tools/verify_records.py                              # 항목 91, 문제 0
git log --oneline -12                                        # 최근 구현·정정 커밋
```
