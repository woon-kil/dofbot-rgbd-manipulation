"""③ P1 실행 계획(`scripts/g01_s42_plan.py --set 12`가 만든 단일 관절 이동 목록)을 G06 충돌 모델로 검증한다(오프라인, 이동 명령 없음).
정상 경로 15이동과 중단 시의 상태별 복귀 경로 전부를, 블록(실측 위치, 반폭 +3 mm)·종이·베이스에 대해 0.5° 간격으로 최선+비관적 4변형과 그리퍼 캡 간격 2종(열림, 오므림 UNVERIFIED)에서 평가한다.
사용: python3 -I src/planning/p1_plan_check.py plan.json [out.json]   종료 코드 1 = 여유 기준 미달"""
import json
import os
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))

from kinematics.urdf_fk import UrdfFK                                   # noqa: E402
from planning.collision_3a import Box, clearances, finger_capsules, load_link_hull   # noqa: E402
from planning.dry_run import TRUTH, TABLE_Z, HULL_KEYS                  # noqa: E402
from planning import p1_poses as P                                      # noqa: E402

CENTER = (0.0014, 0.189)
GATE_MM = float(os.environ.get("P1_GATE_MM", "20"))   # P1: 20 mm, P1-b2(더 가까운 자세): 10 mm(승인 문서 제안)
# 캡 중심 x·쌍 중심 옵셋 조합: 열림, 오므림 가정 4.85 mm, 사진 대조값 7.1 mm(옵셋 −3 mm, UNVERIFIED) 등
CAP_GEOMS = [(0.0292, 0.0), (0.00485, 0.0), (0.0071, -0.003), (0.0071, 0.0), (0.012, -0.003)]
STEP = 0.5


_STATE_CACHE = {}   # 같은 관절 자세(소수 4자리)의 여유는 항상 같으므로 재사용한다(2026-11-15: 세트 16처럼 같은 상태가 반복되는 계획의 검증 시간 단축, 결과 불변)


def eval_state(fk, hulls, q, box):
    key = tuple(round(float(x), 4) for x in q)
    hit = _STATE_CACHE.get(key)
    if hit is not None:
        return dict(hit)
    res = _eval_state_uncached(fk, hulls, q, box); _STATE_CACHE[key] = dict(res)
    return res


def _eval_state_uncached(fk, hulls, q, box):
    w = {"box": 1e9, "table": 1e9, "base": 1e9}
    for t in TRUTH.values():
        f = fk.fk(np.array([list(q) + [90.0]]), gains=t.gains, tool_offset=t.tool_offset())
        for cx, off in CAP_GEOMS:
            c = clearances(f, hulls, finger_capsules(0.09 + t.tip_y, cx, off), TABLE_Z, dz=t.dz, boxes=[box])
            for k, v in (("box", c["box"]), ("table", c["table"]), ("base", c.get("base", 1e9))):
                w[k] = min(w[k], v)
    return w


def eval_moves(fk, hulls, box, start, moves):
    """moves: [(joint 1~4, target)]. 각 이동은 그 관절만 선형으로 바뀐다. 반환: 최악 여유(mm)"""
    cur = list(start); w = {"box": 1e9, "table": 1e9, "base": 1e9}
    for j, tgt in moves:
        n = max(2, int(np.ceil(abs(tgt - cur[j - 1]) / STEP)) + 1)
        for t in np.linspace(0, 1, n):
            q = list(cur); q[j - 1] = cur[j - 1] + (tgt - cur[j - 1]) * t
            e = eval_state(fk, hulls, q, box)
            for k in w:
                w[k] = min(w[k], e[k])
        cur[j - 1] = tgt
    return {k: round(v * 1000, 1) for k, v in w.items()}


def main():
    plan = json.load(open(sys.argv[1], encoding="utf-8"))
    fk = UrdfFK(); hulls = {k: load_link_hull(v) for k, v in HULL_KEYS.items()}
    half = np.array([P.BLOCK[0] / 2 + 0.003, P.BLOCK[1] / 2 + 0.003, P.BLOCK[2] / 2])
    box = Box([CENTER[0], CENTER[1], TABLE_Z + P.BLOCK[2] / 2], half)
    start = [plan["start"][str(i)] for i in (1, 2, 3, 4)]
    fwd = [(m["joint"], m["target"]) for m in plan["moves"]]
    res = {"plan_set": plan["set"], "start": start, "forward": eval_moves(fk, hulls, box, start, fwd), "forward_per_move": [], "return": []}
    cur = list(start)
    for k, (j, t) in enumerate(fwd, 1):
        res["forward_per_move"].append({"n": k, "move": "J%d→%d" % (j, t), "worst_mm": eval_moves(fk, hulls, box, cur, [(j, t)])}); cur[j - 1] = t
    for r in plan["return_routes"]:
        rm = [(m["joint"], m["target"]) for m in r["route"]]
        res["return"].append({"after_move": r["after_move"], "state": r["state"], "worst_mm": eval_moves(fk, hulls, box, r["state"], rm)})
    allv = [res["forward"]] + [x["worst_mm"] for x in res["return"]]
    res["min_mm"] = {k: min(v[k] for v in allv) for k in ("box", "table", "base")}
    res["gate_mm"] = GATE_MM; res["pass"] = all(v >= GATE_MM for v in res["min_mm"].values())
    out = Path(sys.argv[2]) if len(sys.argv) > 2 else ROOT / "evidence/G06/p1_plan_check.json"
    out.write_text(json.dumps(res, ensure_ascii=False, indent=1, default=float), encoding="utf-8")
    print("정상 경로 최악(mm)", res["forward"]); print("복귀 경로 상태 %d개 중 최악 최소(mm)" % len(res["return"]), {k: min(x["worst_mm"][k] for x in res["return"]) for k in ("box", "table", "base")})
    print("전체 최소", res["min_mm"], "기준", GATE_MM, "→", "통과" if res["pass"] else "미달")
    sys.exit(0 if res["pass"] else 1)


if __name__ == "__main__":
    main()
