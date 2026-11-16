#!/usr/bin/env python3
"""S4-2 실행 계획 생성(Mac, 오프라인, URDF FK 모델). 실제 시작 회독으로 이동 목록(접기 계단 + 파일럿 자세 + 복귀 경로)을 만들고 같은 FK로 경로 최저 높이를 계산해 기준 미만이면 거부한다(종료 코드 1).
사용: python3 scripts/g01_s42_plan.py --set 2a|2b --start J1,J2,J3,J4,J5,J6 --out plan.json
모델 값=UNVERIFIED. 쓰기는 한 번에 한 관절, 이동 순서는 자세마다 클리어런스가 가장 큰 순열. 접기·펴기 계단은 S3 순차와 같은 값(이미 실측 검증)이라 높이 기준에서 기준 자세 근처 구간은 BASE_MIN을 쓴다."""
import argparse, itertools, json, sys
import numpy as np
src = open("scripts/g01_s4_2_path.py", encoding="utf-8").read().split("d = json.load(")[0]
ns = {"__file__": "scripts/g01_s4_2_path.py"}; _argv = list(sys.argv); exec(compile(src, "s4_2_path_head", "exec"), ns); sys.argv = _argv   # 아래 스크립트가 sys.argv를 바꾼다
chain_z, seg_min, best_order = ns["chain_z"], ns["seg_min"], ns["best_order"]
SETS = {"2a": [0, 7, 18, 2, 5, 11, 15], "2b": [14, 4, 9, 8, 3], "3": [7, 10, 6, 16, 12, 19, 13, 1, 17, 5], "4": [7, 100, 101, 102, 103, 104, 105, 106, 107, 5], "5": [7, 200, 201, 202, 203, 204, 205, 206, 207, 5], "6": [7, 3, 0, 5], "7": [7, 0, 14, 0, 14, 0, 14, 0, 14, 0, 14, 0, 5], "8": [14, 3], "9": [0, 5], "10": [14, 0, 5], "11": [300, 301, 302, 303, 304], "12": [1201, 1202, 1203], "13": [1301, 1302], "14": [1401, 1402, 1401, 1403, 1401, 1404], "15": [1501, 1502, 1501, 1503, 1501, 1504], "16": [1601], "17": [1701], "18": [1801], "19": [1901], "20": [1301], "21": [2101, 2102, 2103, 2104], "22": [2202, 2203, 2204, 2205]}   # 집합 17 = T1-A 서보 응답 모델 보정·홀드아웃(카메라 없음): 기준 B(J2)·C(J3), 교차 자세 홀드아웃 포함(docs/gates/G07-소각응답T1-승인-요청-문서.md)   # 집합 16 = 소각 응답 시험(카메라 없음): 기준 자세 1601에서 J2·J3 단일 관절 소각 탐침(위·아래 1~4°), 이동마다 원시 값 기록   # 집합 15 = 근거리 M3(S에서 5·10·15 mm급 도착, S 3회 방문). 집합 14는 가시성 버그로 무효   # 집합 14 = 큰 이동 M3: 공통 출발 S에서 50·70·100 mm급 도착 E로 이동·S 복귀 반복(S 3회 방문)   # 집합 10 = S4-10 항 b 파일럿: 자세 14·0·5(피치 3종)에서 손끝 수평 위치(x, y) 사용자 실측   # 집합 9 = S4-9: 집게 방향(J2+J3+J4 합 115·125, 피치 약 25°·35°)이 다른 자세 0·5에서 사용자 손끝 높이 실측(TCP–집게 끝 대 FK 원점 분리)   # 집합 8 = 항 b 간격 실측: 자세 14·3에서 정지 약 34초(사용자가 옆 사진+자로 손끝 높이를 잰다), 복귀 J2 보정(S−1) 검증 겸용   # 집합 7 = 폐루프 파일럿 항 c: 자세 0과 14를 번갈아 재도달(0은 6회, 14는 5회), 방문마다 3프레임 촬영   # 집합 6 = 폐루프 파일럿 항 a: 거리 210/177/196/227 mm 자세에서 정지 10회 반복 촬영   # 집합 5 = 점검 7 + G04 확증 8개 + 점검 5   # 집합 4 = 점검 7 + G04 학습 8개 + 점검 5   # "3" = 점검 7 + G02 holdout 8자세 + 점검 5 (순서: 이동 합 최소 160°, 2026-11-12)
EPS = 0.05   # 부동소수점 경계 허용(mm)
BRACKET = {7, 5}   # G02 집합 3의 처음·끝에 넣는 파일럿 자세: 보드가 움직였는지 점검(판정량 계산에서 제외)
POSE_MIN = {"2a": 75.0, "2b": 60.0, "3": 60.0, "4": 60.0, "5": 60.0, "6": 60.0, "7": 60.0, "8": 60.0, "9": 60.0, "10": 60.0, "11": 60.0, "12": 60.0, "13": 30.0, "14": 60.0, "15": 60.0, "16": 60.0, "17": 60.0, "18": 60.0, "19": 60.0, "20": 44.0, "21": 33.0, "22": 44.0}      # 자세 자체와 자세 사이 전이의 최저 높이 기준(mm)
BASE_MIN = 70.0                          # 기준 깊은 자세(S3·S4-1b에서 실측 검증, 모델 약 71 mm) 연결 구간 기준(mm)
STAIR_TIMES = [8000, 1500, 1500, 3000]   # 접기 계단 시간(S3 순차와 같음)
UNFOLD_TIMES = [3000, 3000, 1500, 1500]  # 복귀 시 펴기 계단(S3 순차 단계 5~8: 3000,3000,1500,1500)
POSE_TIME = {1: 1500, 2: 1500, 3: 1500, 4: 1500}   # 속도 상향(사용자 2026-11-13): 동결 하한(최소 1500 ms, 도당 60 ms) 안에서 최소 시간으로. 이전에는 J1·J2를 3000 ms로 했다(S4-2a~S4-5에서 재전송 0·드리프트 0 확인)
MS_PER_DEG = 60
FIRST_MS = 8000   # 동결 상한: 각 서보의 첫 이동은 8000 ms(J1·J2도 이번 실행의 첫 이동에 적용)
RET_TIME = 3000
K1_NOMINAL = [89, 91, 91, 91]   # K1 기립(오므림) 명목 J1~J4. --return-nominal이면 복귀 목표를 읽은 시작값이 아니라 이 값으로 고정한다(2026-11-15: 읽은 값을 명령으로 되먹이는 래칫 차단, docs/*복귀래칫-재발-이슈보고서.md)
J2_RET_OFFSET = 1   # 복귀 J2 목표 = 시작값 − 1(위에서 접근할 때만, A′): 서보가 목표보다 +1° 높게 멈추는 쏠림 사전 보정(취득 7회 + 재중심 1회 기록, docs/gates/복귀경로-J2누적-근본수정-제안.md). 다음 취득 한 번으로 끝 J2 − 시작값을 확인한다


def z_of(j): return chain_z(list(j) + [90])[0]


def order_moves(cur, tgt):
    """cur→tgt 사이 한 관절씩 이동하는 순서 중 최저 높이가 가장 큰 순서(순열). 반환: (최저 높이, [(joint(1~4), from, to)])"""
    lo, order = best_order(list(cur) + [90], list(tgt) + [90]); c = list(cur); moves = []
    for j in order:
        if c[j] == tgt[j]: continue
        n = list(c); n[j] = tgt[j]; moves.append((j + 1, c[j], tgt[j])); c = n
    return lo, moves


RET_J1_LAST = False


def return_route(state, start):
    """상태(J1~J4) → 시작값. 단계 1: 현재 상태에서 J1→시작값, J2→시작값, J3→최대 20° 위로, J4→최대 20° 위로(각 관절 한 번씩) 4개 이동의 순열 중 최저 높이가 가장 큰 순서를 고른다.
    단계 2: 남은 J3·J4를 더 접힌 쪽부터 20°씩 펴서 시작값으로(S3 순차 펴기 계단과 같은 방식). 반환: (단계 1 최저 높이, [(joint, to, time)])"""
    s = list(state); best = None
    first = []
    for j in range(4):
        tgt = (start[j] - (J2_RET_OFFSET if (j == 1 and s[1] > start[1]) else 0)) if j < 2 else min(start[j], s[j] + 20)   # A′: J2가 시작값보다 높은 상태(위에서 접근)에서만 S−1, 아래에서 접근하면 S(취득 8건: 위 +1, 아래 0)
        if tgt != s[j]: first.append((j, tgt))
    for order in itertools.permutations(first):
        if RET_J1_LAST and any(j == 0 for j, _ in first) and order[-1][0] != 0: continue   # 세트 21(D0): J1 옆 이동 자세에서 낮은 채로 J1을 먼저 돌리면 블록 위를 쓸므로 J1은 마지막(먼저 올린다)
        c = list(s); l = 1e9; mv = []
        for j, tgt in order:
            n = list(c); n[j] = tgt; l = min(l, seg_min(c + [90], n + [90], j)); mv.append((j + 1, tgt, RET_TIME)); c = n
        if best is None or l > best[0]: best = (l, mv, c)
    if best is None: best = (1e9, [], list(s))
    lo, route, c = best[0], list(best[1]), list(best[2]); k = 0
    while c[2] < start[2] or c[3] < start[3]:
        j = 2 if (c[2] < start[2] and (c[3] >= start[3] or c[2] <= c[3])) else 3
        nxt = min(start[j], c[j] + 20); route.append((j + 1, nxt, UNFOLD_TIMES[min(k, 3)])); c[j] = nxt; k += 1
    return lo, route


def route_min(state, route):
    """복귀 경로 전체(J1·J2 정렬, J3·J4 펴기 계단 포함)의 최저 높이."""
    c = list(state); lo = 1e9
    for j, tgt, _ in route:
        n = list(c); n[j - 1] = tgt; lo = min(lo, seg_min(c + [90], n + [90], j - 1)); c = n
    return lo


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--set", choices=list(SETS), required=True); ap.add_argument("--start", required=True); ap.add_argument("--out", required=True); ap.add_argument("--return-nominal", action="store_true", help="복귀 목표를 K1 명목값(89,91,91,91)으로 고정(기본: 읽은 시작값)"); a = ap.parse_args()
    s6 = [int(x) for x in a.start.split(",")]; assert len(s6) == 6
    global RET_J1_LAST; RET_J1_LAST = (a.set in ("21", "22"))
    start = s6[:4]; ret_target = list(K1_NOMINAL) if a.return_nominal else start   # 복귀 목표(기본은 읽은 시작값)
    pose_db = json.load(open({"4": "evidence/G02/s4_4_poses.json", "5": "evidence/G02/s4_5_poses.json", "11": "evidence/G02/s4_11_poses.json", "12": "evidence/G06/p1_poses_s42.json", "13": "evidence/G06/p1_b2_poses_s42.json", "20": "evidence/G06/p1_b2_poses_s42.json", "21": "evidence/G06/d0_poses_s42.json", "22": "evidence/G06/d1_poses_s42.json", "14": "evidence/G06/p1_bigm3_poses_s42.json", "15": "evidence/G06/p1_near_m3_poses_s42.json", "16": "evidence/G06/p1_step_poses_s42.json", "17": "evidence/G06/p1_t1_poses_s42.json", "18": "evidence/G06/p1_t1b_poses_s42.json", "19": "evidence/G06/p1_t1c_poses_s42.json"}.get(a.set, "evidence/G01/b4_pose_candidates_y230_72x60_j34min10_clear60.json")))   # 집합 4 = G04 추가 학습 자세
    P = {p["id"]: p for p in pose_db["poses"]}
    want = "holdout" if a.set == "3" else ({"4": "g04_train", "5": "g04_confirm", "11": "j2sweep", "12": "p1", "13": "p1b2", "20": "p1b2", "21": "d0", "22": "d1", "14": "bigm3", "15": "nearm3", "16": "stepresp", "17": "t1a", "18": "t1b", "19": "t1c"}.get(a.set, "pilot"))
    ok_ids = lambda i: P[i]["split"] == want or (a.set in ("3", "4", "5", "6", "7") and i in BRACKET)   # 집합 3은 holdout + 보드 이동 점검용 파일럿 자세(BRACKET)만 허용
    if any(not ok_ids(i) for i in SETS[a.set]): sys.exit("집합 %s는 %s 자세(집합 3은 점검 자세 %s 포함)만 허용한다" % (a.set, want, sorted(BRACKET)))
    moves = []; states = [list(start)]; notes = []
    # A) 접기 계단(J3 → J4, 4단계)
    cur = list(start)
    for k in range(1, 5):
        for j in (3, 4):
            tgt = start[j - 1] - 20 * k
            if a.set in ("12", "13", "14", "15", "16", "17", "18", "19", "20", "21", "22") and j == 3: tgt = max(tgt, 14)   # P1: 시작 J3=91이면 4단계가 11까지 내려가 검증된 기준 깊은 자세(J3 14, 모델 높이 ≥70 mm)보다 깊어진다. 이전 실행의 J3 14 수준으로 멈춘다
            moves.append({"joint": j, "target": tgt, "time_ms": STAIR_TIMES[k - 1], "phase": "A", "pose_id": None, "capture": False}); cur[j - 1] = tgt; states.append(list(cur))
    base = list(cur); base_z = z_of(base)
    # B) 파일럿 자세
    seg_lows = []; first_j12 = True; first_used = {3: True, 4: True}   # J3·J4는 접기 계단에서 이미 첫 이동(8000 ms)을 함
    for i in SETS[a.set]:
        tgt = P[i]["sdk_deg"][:4]; lo, mv = order_moves(cur, tgt); seg_lows.append((i, lo))
        for n_, (j, f, t) in enumerate(mv):
            note = ""
            if j in (1, 2) and first_j12: note = "접힌 자세에서 처음 하는 J1/J2 이동(처짐·케이블 확인)"; first_j12 = False
            tm = max(POSE_TIME[j], int(-(-MS_PER_DEG * abs(t - f) // 100) * 100))   # 동결 상한: 도당 60 ms 이상
            if not first_used.get(j): tm = FIRST_MS; first_used[j] = True
            moves.append({"joint": j, "target": t, "time_ms": tm, "phase": "B", "pose_id": i, "capture": False, "note": note}); cur[j - 1] = t; states.append(list(cur))
        if not mv: moves[-1]["capture"] = True
        else: moves[-1]["capture"] = True
    t_start = len(states)
    if a.set == "16":   # 소각 탐침(사전 등록: docs/gates/G07-소각응답-승인-요청-문서.md §3): 기준 b에서 크기 s=1~4마다 위(b+s→b)·아래(b−s→b) 탐침, 다시 s=2를 반복. 관절 J2(b=90)→J3(b=14), 이동 1500 ms
        assert cur[0] == 90 and cur[1] == 90 and cur[2] == 14 and cur[3] == 11, ("기준 자세에 도달하지 못했다", cur)
        for j_, b_ in ((2, 90), (3, 14)):
            seq = []
            for s_ in (1, 2, 3, 4): seq += [b_ + s_, b_, b_ - s_, b_]
            seq += [b_ + 2, b_, b_ - 2, b_]
            for t_ in seq:
                moves.append({"joint": j_, "target": t_, "time_ms": 1500, "phase": "T", "pose_id": None, "capture": False, "note": ""}); cur[j_ - 1] = t_; states.append(list(cur))
        for m_ in moves: m_["capture"] = False   # 카메라 없음
    if a.set == "17":   # T1-A(사전 등록: docs/gates/G07-소각응답T1-승인-요청-문서.md §2-1, 변경 이력 §3-1·§6 D1~D2 + 교차 자세 홀드아웃 추가)
        assert cur[0] == 90 and cur[1] == 90 and cur[2] == 14 and cur[3] == 11, ("기준 자세 B에 도달하지 못했다", cur)
        def probe(j_, b_, sizes, role):   # 묶음: b+s, b, b−s, b (1500 ms)
            for s_ in sizes:
                for t_ in (b_ + s_, b_, b_ - s_, b_):
                    moves.append({"joint": j_, "target": t_, "time_ms": 1500, "phase": "T", "role": role, "pose_id": None, "capture": False, "note": ""}); cur[j_ - 1] = t_; states.append(list(cur))
        def go(j_, t_, note):   # 기준 위치 이동(탐침 아님, 분석에서 제외)
            moves.append({"joint": j_, "target": t_, "time_ms": max(POSE_TIME[j_], int(-(-MS_PER_DEG * abs(t_ - cur[j_ - 1]) // 100) * 100)), "phase": "B", "role": "go", "pose_id": None, "capture": False, "note": note}); cur[j_ - 1] = t_; states.append(list(cur))
        # 모델 높이(UNVERIFIED) 때문에 J2 아래 방향은 J3 14에서 b−5(높이 60.7 mm)까지만 가능 → J2 보정·홀드아웃은 자세 C(J3 22)에서 하고, 교차 홀드아웃을 자세 B(J3 14, s=5)에서 한다(문서 §2-1·§3-1 변경 이력)
        go(3, 22, "J3 22(자세 C, 처음 쓰는 조합)로 이동")
        probe(2, 90, (1, 2, 3, 4, 8, 12, 2), "cal"); probe(2, 90, (10, 5, 6), "hold")   # J2: 보정 28 + 홀드아웃 12 (자세 C, J3 22)
        probe(3, 22, (1, 2, 3, 4, 8, 12, 2), "cal"); probe(3, 22, (10, 5, 6), "hold")   # J3: 보정 28 + 홀드아웃 12 (b=22, J2 90)
        go(3, 17, "J3 교차 자세 기준(17)으로 이동")
        probe(3, 17, (7,), "xhold")   # 교차 홀드아웃(J3): 보정은 b=22, 이번엔 b=17 (J3 10~24)
        go(3, 14, "자세 B(J3 14)로 이동")
        probe(2, 90, (5,), "xhold")   # 교차 홀드아웃(J2): 보정은 자세 C(J3 22), 이번엔 자세 B(J3 14, 아래 b−5 = 85까지)
        for m_ in moves: m_["capture"] = False   # 카메라 없음
    if a.set == "18":   # T1-B(사전 등록: docs/gates/G07-소각응답T1B-큰이동-승인-요청-문서.md §2-1): J2·J3 단일 관절 큰 이동 왕복(위 b→b+s, 아래 b+s→b), 시간 max(1500 ms, 60 ms/°×s). 기준 A = [90,90,22,11]
        assert cur[0] == 90 and cur[1] == 90 and cur[2] == 14 and cur[3] == 11, ("기준 자세 B에 도달하지 못했다", cur)
        def go(j_, t_, note):   # 기준 위치 이동(탐침 아님, 분석에서 제외)
            moves.append({"joint": j_, "target": t_, "time_ms": max(POSE_TIME[j_], int(-(-MS_PER_DEG * abs(t_ - cur[j_ - 1]) // 100) * 100)), "phase": "B", "role": "go", "pose_id": None, "capture": False, "note": note}); cur[j_ - 1] = t_; states.append(list(cur))
        def pairs(j_, b_, sizes, role):   # 왕복 한 쌍: b+s(위), b(아래), 1500 ms
            for s_ in sizes:
                for t_ in (b_ + s_, b_):
                    tm_ = max(1500, int(-(-MS_PER_DEG * abs(t_ - cur[j_ - 1]) // 100) * 100))   # 어댑터 동결 상한(도당 60 ms 이상)을 지킨다: 25°까지 1500 ms, 35° 2100 ms, 45° 2700 ms
                    moves.append({"joint": j_, "target": t_, "time_ms": tm_, "phase": "T", "role": role, "pose_id": None, "capture": False, "note": "큰 이동 %d°" % s_ if s_ >= 35 else ""}); cur[j_ - 1] = t_; states.append(list(cur))
        go(3, 22, "J3 22(기준 A)로 이동")
        pairs(2, 90, (15, 25, 35, 45, 15, 25, 35, 45), "cal"); pairs(2, 90, (20, 30, 40, 20, 30, 40), "hold")   # J2: 보정 16 + 홀드아웃 12 (자세 A)
        pairs(3, 22, (15, 25, 35, 45, 15, 25, 35, 45), "cal"); pairs(3, 22, (20, 30, 40, 20, 30, 40), "hold")   # J3: 보정 16 + 홀드아웃 12 (b=22, J2 90)
        for m_ in moves: m_["capture"] = False   # 카메라 없음
    if a.set == "19":   # T1-C(사전 등록: docs/gates/G07-소각응답T1C-크기구간-승인-요청-문서.md §2-1, E2 축소안): J2·J3 단일 관절, 크기 8·12·16·20·30° 각 위2·아래2(관절당 20이동), 고정 시드로 섞은 목표열, 시간 max(1500 ms, 60 ms/°×s)
        assert cur[0] == 90 and cur[1] == 90 and cur[2] == 14 and cur[3] == 11, ("기준 자세 B에 도달하지 못했다", cur)
        import random
        def seq_for(lo, hi, start, seed):   # 크기·방향 다중집합을 시드로 섞고 구간 안에 머무는 목표열(30°는 처음 3이동 금지)을 찾는다
            multi = [(s_, d_) for s_ in (8, 12, 16, 20, 30) for d_ in (1, -1) for _ in range(2)]; rng = random.Random(seed)
            for _ in range(20000):
                pool = list(multi); rng.shuffle(pool); pos = start; seq = []; ok = True
                while pool:
                    cand = [(i, (s_, d_)) for i, (s_, d_) in enumerate(pool) if lo <= pos + s_ * d_ <= hi and not (s_ == 30 and len(seq) < 3)]
                    if not cand: ok = False; break
                    i, (s_, d_) = rng.choice(cand); pool.pop(i); pos += s_ * d_; seq.append(pos)
                if ok: return seq
            raise SystemExit("목표열을 찾지 못했다")
        def go(j_, t_, note):   # 기준 위치 이동(탐침 아님, 분석에서 제외)
            moves.append({"joint": j_, "target": t_, "time_ms": max(POSE_TIME[j_], int(-(-MS_PER_DEG * abs(t_ - cur[j_ - 1]) // 100) * 100)), "phase": "B", "role": "go", "pose_id": None, "capture": False, "note": note}); cur[j_ - 1] = t_; states.append(list(cur))
        def walk(j_, targets):
            for t_ in targets:
                tm_ = max(1500, int(-(-MS_PER_DEG * abs(t_ - cur[j_ - 1]) // 100) * 100))
                moves.append({"joint": j_, "target": t_, "time_ms": tm_, "phase": "T", "role": "mix", "pose_id": None, "capture": False, "note": "큰 이동 %d°" % abs(t_ - cur[j_ - 1]) if abs(t_ - cur[j_ - 1]) >= 30 else ""}); cur[j_ - 1] = t_; states.append(list(cur))
        go(3, 22, "J3 22(기준 A)로 이동")
        walk(2, seq_for(90, 135, 90, 1801))   # J2: 90~135
        go(2, 90, "J2 기준 90으로 복귀(다음 J3 시험 전)") if cur[1] != 90 else None
        walk(3, seq_for(22, 67, 22, 1802))   # J3: 22~67 (J2 90에서)
        for m_ in moves: m_["capture"] = False   # 카메라 없음
    pose_zs = {i: z_of(P[i]["sdk_deg"][:4]) for i in SETS[a.set]}
    # 복귀 경로(모든 중간 상태)
    rets = []; ret_los = []; ret_full = []
    for k, st in enumerate(states):
        lo, route = return_route(st, ret_target); ret_full.append(route_min(st, route)); rets.append({"after_move": k, "state": st, "route": [{"joint": j, "target": t, "time_ms": tm} for j, t, tm in route]}); ret_los.append(lo)
    gate = {"pose_min_required": POSE_MIN[a.set], "base_min_required": BASE_MIN, "base_z": round(base_z, 1), "pose_z": {str(i): round(v, 1) for i, v in pose_zs.items()}, "transition_min": {str(i): round(l, 1) for i, l in seg_lows}, "return_j12_min": round(min(ret_los), 1), "return_full_min": round(min(ret_full), 1)}
    errs = []
    if base_z < BASE_MIN - EPS: errs.append("기준 깊은 자세 높이 %.1f < %.1f" % (base_z, BASE_MIN))
    for i, v in pose_zs.items():
        if v < POSE_MIN[a.set] - EPS: errs.append("자세 %d 높이 %.1f < %.1f" % (i, v, POSE_MIN[a.set]))
    for i, l in seg_lows:
        if l < min(BASE_MIN, POSE_MIN[a.set]) - EPS: errs.append("자세 %d 전이 경로 %.1f < %.1f" % (i, l, min(BASE_MIN, POSE_MIN[a.set])))
    if min(ret_full) < min(BASE_MIN, POSE_MIN[a.set]) - EPS: errs.append("복귀 전체 경로 최저 %.1f < %.1f" % (min(ret_full), min(BASE_MIN, POSE_MIN[a.set])))
    if min(ret_los) < min(BASE_MIN, POSE_MIN[a.set]) - EPS: errs.append("복귀 단계 1 경로 최저 %.1f < %.1f" % (min(ret_los), min(BASE_MIN, POSE_MIN[a.set])))
    # 자세 사이 전이(기준 자세 연결 제외)는 자세 기준을 적용
    pose_to_pose = [l for (i, l), prev in zip(seg_lows[1:], seg_lows[:-1])]
    if pose_to_pose and min(pose_to_pose) < POSE_MIN[a.set] - EPS: errs.append("자세 사이 전이 경로 최저 %.1f < %.1f" % (min(pose_to_pose), POSE_MIN[a.set]))
    for st_ in states[t_start:]:   # 탐침·기준 위치 이동 상태 전부 모델 높이 확인
        if z_of(st_) < POSE_MIN[a.set] - EPS: errs.append("탐침 상태 %s 높이 %.1f < %.1f" % (st_, z_of(st_), POSE_MIN[a.set]))
    plan = {"bracket_ids": sorted(BRACKET) if a.set in ("3", "4", "5", "6", "7") else [], "set": a.set, "start": {"1": s6[0], "2": s6[1], "3": s6[2], "4": s6[3], "5": s6[4], "6": s6[5]}, "base": base, "moves": moves, "return_routes": rets, "gate": gate, "gate_errors": errs, "hold_poses": ([14, 3] if a.set == "8" else ([0, 5] if a.set == "9" else ([14, 0, 5] if a.set == "10" else ([300, 301, 302, 303, 304] if a.set == "11" else ([1301] if a.set == "20" else ([2101, 2102, 2103, 2104] if a.set == "21" else ([2202, 2203, 2204, 2205] if a.set == "22" else []))))))), "hold_max_s": 180, "exposures": ([25] * 10 if a.set == "6" else ([25] * 3 if a.set == "7" else ([25] * 3 if a.set in ("8", "9", "10", "11") else ([25] * 30 if a.set in ("12", "13", "14", "15", "20") else [15, 25, 35])))), "settle_s": 2, "poses": {str(i): P[i]["sdk_deg"] for i in SETS[a.set]}}
    if a.set == "21": plan["no_camera"] = True; plan["hold_min_mm"] = {"2101": 40, "2102": 32, "2103": 22, "2104": 18}   # D0: 카메라 없음, 입력(실제 최저점 mm) 하한 미만이면 중단·복귀
    if a.set == "22": plan["no_camera"] = True; plan["gemini_photo"] = True; plan["hold_min_mm"] = {str(i): 12 for i in (2202, 2203, 2204, 2205)}   # D1-A: 눈 읽기 입력 12 mm 미만이면 중단·복귀, 자세마다 Gemini 사진(Mac 래퍼가 측정 대기 때 촬영)
    plan["return_target"] = list(ret_target); plan["return_nominal"] = bool(a.return_nominal)
    plan["raw_after"] = (a.set in ("16", "17", "18", "19"))   # 러너가 이동마다 원시 값 두 시점을 기록
    for n, m in enumerate(moves, 1): m["n"] = n
    json.dump(plan, open(a.out, "w"), ensure_ascii=False, indent=1)
    print("집합 %s, 시작 %s, 이동 %d회(접기 8 + 자세 %d), 기준 깊은 자세 %s 높이 %.1f mm" % (a.set, start, len(moves), len(moves) - 8, base, base_z))
    print("복귀 전체 경로(펴기 계단 포함) 최저 %.1f mm" % gate["return_full_min"]); print("자세별 높이(mm):", gate["pose_z"]); print("자세로의 전이 경로 최저(mm):", gate["transition_min"]); print("복귀(J1·J2 구간) 최저 %.1f mm, 상태 %d개" % (min(ret_los), len(rets)))
    if errs: print("게이트 실패:", errs); sys.exit(1)
    print("게이트 통과"); 


if __name__ == "__main__": main()
