# 진행 지도 갱신 절차

진행 지도(Claude Artifact "DOFBOT 프로젝트 진행 지도", 비공개)의 데이터를 최신으로 맞추는 방법이다. 하드웨어 명령은 없다.

## 무엇이 자동이고 무엇이 사람 몫인가
| 항목 | 방식 |
|---|---|
| 열린 결정 건수(확정·부분·미확인) | **자동 계산.** `tools/progress_map.py`가 `docs/gates/열린-결정-정리표.md`의 확인 칸 `[x]` `[~]` `[ ]`를 센다 |
| 관문·접촉 파일럿 단계·서보 시험·남은 것의 상태 | **사람이 고침.** `docs/progress-map/status.json`. 문장을 기계가 해석하면 틀린 해석이 지도에 퍼지므로 일부러 자동화하지 않았다 |

## 상태를 바꿀 때
1. `docs/progress-map/status.json`에서 해당 행의 `state`, `state_ko`, `note`, `source`를 고치고 `as_of`를 갱신한다.
   - `state` 값: `done`(완료) `cond`(조건부·잠정) `partial`(일부·진행) `draft`(초안) `wait`(승인 대기) `none`(미수행·제안)
   - 근거 문서를 `source`에 적는다. 근거 없이 상태를 올리지 않는다.
2. `python3 tools/progress_map.py --check`로 구조와 결정 건수를 점검한다(잘못된 `state`, 빠진 열, 중복 `id`는 오류로 멈춘다).
3. Claude에게 "진행 지도 갱신해 줘"라고 요청한다. Claude는 `python3 tools/progress_map.py`로 `data/progress-map/*.json`을 만들고, 아티팩트에 데이터 파일을 다시 올려 `datasets/*`의 파일 주소와 `updated`를 바꾼다.

## 알아 둘 점
- 이 지도는 정적 사본이라 위 절차를 거치지 않으면 저절로 바뀌지 않는다. 기준 시각이 페이지에 적혀 있다.
- 아티팩트는 비공개이고 올린 데이터 파일은 그것을 여는 모든 사람이 읽을 수 있다. 저장소 문서에서 옮긴 내용만 담는다.
- 2단계(선택): 낡는 일이 잦으면 `git commit` 뒤 훅(`tools/direction_check.py --hook`과 같은 방식)으로 "지도 갱신 필요"를 알리게 할 수 있다. 훅은 설정 파일 변경이라 사용자 승인 뒤에 붙인다.
- 상태 파일의 소유: 관문 상태를 바꾸는 세션이 같이 고친다(지금은 세션 A).

## 검증 근거
```
$ python3 tools/progress_map.py --check
as_of(status.json): 2026-11-15 21:41
관문 10, 접촉 파일럿 9, 서보 5, 남은 것 5
열린 결정 12건: 확정 2, 부분 1, 미확인 9
```
- 잘못된 입력 시험(복사본 사용): `state` 값 오류, 열 누락, `id` 중복이 모두 오류 메시지와 함께 멈췄다.
- 출력 데이터 4종(gates, pilot, servo, remaining)은 지금 올려 둔 데이터와 내용이 같고, 결정 건수 4개도 같다(12, 2, 1, 9).
