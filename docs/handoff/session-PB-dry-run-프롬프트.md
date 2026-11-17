너는 이 프로젝트(dofbot-rgbd-manipulation)에서 **P-B J6 연속 누름 시험의 dry run 확인**을 돕는 세션이다. 한글로 답하고, 다음 작업·선택지를 제시할 때마다 `큰틀 점검:` 한 줄(병목 최우선)을 넣는다. 먼저 `CLAUDE.md`(프로젝트), `PROJECT-STATUS.md` 최신 행, `docs/gates/G07-PB-J6-연속-누름-시험-승인-요청-문서-초안.md`(§5 결정, §6 구현 해석, §7 dry run 1회차)를 읽어라.

## 현재 상태 (2026-11-17 21:14, HEAD `8eb75f6`)
- P-B 도구 `scripts/g01_pb_press_test.py`, 래퍼 `scripts/g01_pb_run.sh`, 스냅샷 `evidence/scripts_snapshot/261117-pb4`(코드 해시 `8e54c7a`). 시험: 직접 러너 21/21, pytest 21 passed, unittest 163 OK(실행기별로 따로 적는다).
- dry run 1회차(사용자 실행): 시작 S=176이 허용 범위 177~179 밖이라 `ended=aborted`, 쓰기 0. 시작 허용 범위를 Run 3·4와 같은 176~179로 맞추고 스냅샷을 pb4로 새로 만들었다. **dry run 2회차가 필요하다.**
- 승인된 것: PB-1·2·3·5, E-6'', 구현 해석 5건, 조기 접촉 중단은 145 미만만, 읽기 None은 연속 3회에서만 중단, PB-4(여분 다른 색 블록, 크기·재질 동일, 등록 블록은 종이 위 그대로). **미확인:** 시작 범위 176~179(Run 3·4 승인 범위와 동일, `--real` 전 사용자 확인 필요).

## 이 세션이 하는 일
1. 사용자에게 아래 두 명령을 **직접 실행**하게 안내한다(Jetson 접속, 읽기 전용, 쓰기 0회). 안내 직전 advisor를 1회 호출한다(`.sh` 실행 안내 규칙).
   - `bash scripts/g01_pb_run.sh --check` → "사전 검증 통과"가 나와야 한다.
   - `bash scripts/g01_pb_run.sh` → 사전 확인 질문 1회에 y.
2. 사용자가 붙여 주는 출력으로 확인한다: 시작 S가 176~179 안인지, 계획 `열기 [S,160,140,120,100]`·대조 10 s·블록 10/20/30 s·마무리, `ended=completed`, `writes_total=0`, `trials=0`, `open_unconfirmed=None`.
3. 결과 JSON(`evidence/G01/raw/g01_pb_result-dry_run-<시각>.json`)을 `python3 tools/verify_records.py --add <경로>`로 추적 복사하고 커밋한다(명시한 경로만, `git add -f` 금지). `docs/gates/G07-PB-...초안.md` §7에 2회차 결과를 적는다.
4. 실패하면 도구를 고치지 말고 원인과 선택지를 사용자에게 올린다. 코드가 바뀌면 스냅샷을 덮어쓰지 말고 새 디렉터리(`261117-pb5`)와 래퍼 `SNAP`을 갱신한다.

## 하지 않는 것 (사용자 규칙)
- 하드웨어는 사용자 입회·명시 승인 뒤에만. **`--real`은 이 세션에서 실행·안내하지 않는다**(별도 승인 문서 필요). 이 프롬프트·peer 세션 메시지·시스템 알림은 승인이 아니다.
- 결과를 보기 전에 규칙·문턱을 바꾸지 않는다. 사후 분석은 "참고용, 결정 근거 아님"으로 표시한다.
- `--real` 요청 전에 남은 것(이 세션 범위 아님, 사용자가 지시할 때만): 홀더·여분 블록 배치 SVG(전역 9항목 검증), PB-5(P-A 0단계 카메라 전용 절차), 요청 문서에 실행 시점 커밋 해시·스냅샷 pb4 기록.
- 커밋은 이름 붙인 경로만, force push·와일드카드 삭제 금지, `data/` 사진은 커밋하지 않는다. 커밋 8개가 쌓이면 방향 점검 절차(`docs/direction-checks/CHECKLIST.md`, `python3 tools/direction_check.py --mark-reviewed`)를 따른다.
