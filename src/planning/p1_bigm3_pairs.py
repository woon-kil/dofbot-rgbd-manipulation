"""③ 큰 이동 M3 자세 쌍 후보(오프라인, 이동 명령 없음). 질문: 관측 자세에서 tip을 50~100 mm 옮기는 이동의 모델 예측 오차가 이동량에 어떻게 비례하는가.
조건: 두 자세 모두 마커(25 mm)가 4모서리·강건 가시성(마커 위치 ±6 mm 흔들림, 가장자리 40 px), 깊이 0.13~0.32 m(0.19 m 이하만 보면 후보끼리 tip 최대 24 mm 떨어져 큰 이동 쌍이 없어, 멀리서 보는 구간까지 넓힘 — 카메라 이동의 FK 예측 오차는 마커 깊이와 무관하게 비교 가능), 입사각 ≤ 70°, 캡 가림 없음,
이미 이동해 본 높이 영역 안(종이 여유 ≥ 55 mm = P1 자세 B의 56.4 mm 수준, 블록 ≥ 20 mm, 베이스 ≥ 25 mm), 각 자세 J3 ≥ 10·J4 ≥ 11.
이동은 단일 관절 순서의 경로 전체도 같은 여유를 만족해야 한다. 쌍: tip 이동 50~100 mm, 관절 한 개 이상이 ≥ 5°(회독 1°가 이동의 20 % 이하), 최대 관절 이동 ≤ 20°."""
import json
import sys
import itertools
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))

from kinematics.urdf_fk import UrdfFK                                   # noqa: E402
from planning.collision_3a import Box, load_link_hull                   # noqa: E402
from planning.dry_run import BEST, HULL_KEYS, TABLE_Z                   # noqa: E402
from planning.observe import PoseGrid                                   # noqa: E402
from planning import p1_poses as P                                      # noqa: E402
from planning import p1_b2_poses as B                                   # noqa: E402
from planning.p1_plan_check import eval_moves                           # noqa: E402

DEPTH = (0.148, 0.320)   # 하한: 결정 7이 유지하는 PnP 깊이 0.145 m + P1의 FK-PnP 차 약 3 mm(UNVERIFIED). 이전 값 0.130
GATES = {"table": 55.0, "box": 20.0, "base": 25.0}
TIP_MOVE = (0.050, 0.100)
BANDS = [("50", 50.0, 60.0), ("70", 65.0, 80.0), ("100", 90.0, 100.001)]   # tip 이동 구간(mm)
PER_START = 15   # 공통 출발 후보 경로 검사 수
PER_BAND = 40   # 구간당 경로 검사 상한(호출당 시간 실측 전 임시값)
J2_RANGE = (74.0, 105.0)   # 실행 기록 최저 J2 74, 최고 약 105(러너 창 70~110 안)


def main():
    fk = UrdfFK(); grid = PoseGrid(fk, BEST, j234_min=P.J234_MIN); hulls = {k: load_link_hull(v) for k, v in HULL_KEYS.items()}
    half = np.array([P.BLOCK[0] / 2 + 0.003, P.BLOCK[1] / 2 + 0.003, P.BLOCK[2] / 2]); box = Box([B.CENTER[0], B.CENTER[1], TABLE_Z + P.BLOCK[2] / 2], half)
    top = TABLE_Z + P.BLOCK[2]; h = P.MARKER / 2; M = np.array([B.CENTER[0], B.CENTER[1] + B.MARKER_OFFSET_Y, top])
    ok = np.ones(len(grid.sdk), bool)
    n_before_vis = int(ok.sum())
    for dx, dy in B.SHIFTS:
        for sx in (-1, 1):
            for sy in (-1, 1):
                o, _, _, _ = grid.visible_mask(M + np.array([dx + sx * h, dy + sy * h, 0.0]), 0.0, 40, max_tilt=90.0)   # tilt 한도는 파지 접근용이라 M3에는 제외
                ok &= o   # 2026-11-15 수정: 이 줄이 위 주석 안에 들어가 있어 가시성 필터가 한 번도 적용되지 않았다(이슈보고서 참고)
    print("가시성 필터(마커 4모서리 ±흔들림, 가장자리 40 px): 격자 %d → %d 자세" % (n_before_vis, int(ok.sum())), flush=True)   # 필터가 실제로 적용됐는지 눈으로 확인(2026-11-15 버그 재발 방지)
    assert int(ok.sum()) < n_before_vis, "가시성 필터가 아무 효과가 없다(코드 확인)"
    v = grid.cp - M; ok &= (v[:, 2] / np.linalg.norm(v, axis=1)) >= np.cos(np.radians(70.0))
    idx = np.flatnonzero(ok); depth = np.einsum("nji,nj->ni", grid.Rc[idx], M - grid.cp[idx])[:, 2]
    idx = idx[(depth >= DEPTH[0]) & (depth < DEPTH[1])]
    pts = [M] + [M + np.array([sx * h, sy * h, 0.0]) for sx in (-1, 1) for sy in (-1, 1)]
    S = []
    for i in idx:
        if B.occluded(fk, grid, i, pts):
            continue
        q = [90.0, *[float(x) for x in grid.sdk[i][1:]]]
        if not (J2_RANGE[0] <= q[1] <= J2_RANGE[1] and q[2] <= 105 and q[3] <= 105):
            continue   # 러너 승인 창(J2 70~110, J3·J4 10~105)과 지금까지 이동한 J2 범위 안으로 제한
        w = B.worst(fk, hulls, q, box); wm = {k: x * 1000 for k, x in w.items()}
        if all(wm[k] >= GATES[k] for k in GATES):
            S.append((i, q, wm))
    print("관측 후보 %d → 게이트 통과 %d" % (len(idx), len(S)), flush=True)
    pairs = []
    for (i, qi, wi), (j, qj, wj) in itertools.combinations(S, 2):
        d = np.abs(np.array(qi[1:]) - np.array(qj[1:])); mv = float(np.linalg.norm(grid.tip[i] - grid.tip[j]))
        if d.max() >= 5.0 and d.max() <= 20.0 and TIP_MOVE[0] <= mv <= TIP_MOVE[1]:
            pairs.append((i, j, qi, qj, mv, float(d.max())))
    print("쌍 후보(이동 조건) %d" % len(pairs), flush=True)
    hist = {}
    for pr in pairs:
        b = int(pr[4] * 100) * 10; hist[b] = hist.get(b, 0) + 1   # 10 mm 구간(하한 mm)
    print("tip 이동 분포(10 mm 구간 하한: 쌍 수):", dict(sorted(hist.items())), flush=True)
    out = {"n_obs": int(len(idx)), "n_pass": len(S), "n_pairs": len(pairs), "gates_mm": GATES, "tip_move_m": TIP_MOVE, "depth_m": DEPTH, "tip_hist_mm": {str(k): v for k, v in sorted(hist.items())}, "bands": {}, "pairs": []}
    pairs.sort(key=lambda p: (p[5], p[4]))   # 최대 관절 이동이 작은 순
    def check(i, j, qi, qj, mv, dm):
        cur = list(qi); moves = [(k + 1, qj[k]) for k in range(4) if qj[k] != qi[k]]   # 단일 관절 순서 경로(번호 순)
        w = eval_moves(fk, hulls, box, cur, moves)
        if not all(w[k] >= GATES[k] for k in GATES):
            return None
        pi = np.einsum("ji,j->i", grid.Rc[i], M - grid.cp[i]); pj = np.einsum("ji,j->i", grid.Rc[j], M - grid.cp[j])
        return {"from": qi, "to": qj, "tip_move_mm": round(mv * 1000, 1), "max_joint_delta_deg": dm, "joint_deltas": [round(qj[k] - qi[k], 1) for k in range(1, 4)],
                "depth_from_mm": round(float(pi[2]) * 1000, 1), "depth_to_mm": round(float(pj[2]) * 1000, 1), "predicted_marker_shift_cam_mm": np.round((pj - pi) * 1000, 1).tolist(), "path_worst_mm": w}
    import time
    for name, lo, hi in BANDS:   # 구간별 경로 검사(구간당 상한 PER_BAND, 호출당 시간 기록). 구간이 비면 비었다는 사실이 결과다(조건을 풀지 않는다)
        cand = [p for p in pairs if lo <= p[4] * 1000 < hi]; t0 = time.time(); got = []; tried = 0
        for pr in cand[:PER_BAND]:
            tried += 1; r = check(*pr)
            if r: got.append(r)
        dt = time.time() - t0
        out["bands"][name] = {"tip_range_mm": [lo, hi], "n_candidates": len(cand), "n_checked": tried, "n_path_pass": len(got), "sec_per_check": round(dt / max(tried, 1), 3), "pairs": got}
        print("%s구간 %d~%d mm: 후보 %d, 경로 검사 %d(호출당 %.2f s), 통과 %d" % (name, lo, hi, len(cand), tried, dt / max(tried, 1), len(got)), flush=True)
    # 공통 출발 자세 탐색: 한 자세 S에서 50·70·100 mm급 도착 자세 셋으로 가는 구성(기준 자세 하나, 처음 쓰는 조합 수 감소). 쌍은 순서가 없으므로 양 끝을 모두 출발 후보로 본다
    part = {}
    for pr in pairs:
        i, j, qi, qj, mv, dm = pr
        for s_, e_, qs, qe in ((i, j, qi, qj), (j, i, qj, qi)):
            for name, lo, hi in BANDS:
                if lo <= mv * 1000 < hi:
                    part.setdefault((s_, tuple(qs)), {}).setdefault(name, []).append((s_, e_, qs, qe, mv, dm))
    starts = [k for k, v in part.items() if len(v) == len(BANDS)]
    starts.sort(key=lambda k: sum(min(p[5] for p in part[k][n]) for n, _, _ in BANDS))   # 각 구간 최소 관절 이동의 합이 작은 순
    out["common_start"] = {"n_starts_all_bands": len(starts), "checked": []}
    print("세 구간 모두에 도착 후보가 있는 출발 자세: %d" % len(starts), flush=True)
    for k in starts[:PER_START]:
        ent = {"start": list(k[1]), "ends": {}}
        for name, _, _ in BANDS:
            cands = sorted(part[k][name], key=lambda p: (p[5], p[4]))[:3]
            ent["ends"][name] = [r for r in (check(*c) for c in cands) if r]
        out["common_start"]["checked"].append(ent)
        print("  출발", k[1], {n: len(v) for n, v in ent["ends"].items()}, flush=True)
    out["pairs"] = out["bands"].get("50", {}).get("pairs", [])   # 이전 출력 형식 호환(50 mm급)
    (ROOT / "evidence/G06/p1_bigm3_pairs.json").write_text(json.dumps(out, ensure_ascii=False, indent=1, default=float), encoding="utf-8")


if __name__ == "__main__":
    main()
