"""③ 큰 이동 M3 자세 선정(오프라인, 이동 명령 없음). p1_bigm3_pairs.py의 출력(evidence/G06/p1_bigm3_pairs.json)에서 공통 출발 S 하나와 50·70·100 mm급 도착 자세를 고른다.
선정 규칙(결과를 본 뒤 두 번 바뀌었다 — ①처음 ‘과거 실행 상태와 가까운 순’(깊이 하한 0.130 m) ②임의 경로 여유 하한(블록 ≥ 100, 베이스 ≥ 70 mm) 추가 ③철회 ④결정 7 범위에 맞춘 깊이 하한 148 mm를 쌍 조건에 추가하고 재계산. 이력 전문은 docs/gates/G07-큰이동-M3-승인-요청-문서.md §0-2; 아래는 최종 규칙): ① 쌍 계산이 이미 걸러낸 조건(마커 가시성·깊이 하한 148 mm·여유 게이트·경로 검사)을 통과한 후보만 쓴다.
② 출발 S마다 각 구간에서 ‘과거 실행 상태와의 J2~J4 최대 편차’가 가장 작은(가장 덜 새로운) 도착을 고르고(동점은 최대 관절 이동이 작은 쪽), ③ S는 네 자세(S·E50·E70·E100)의 최악 편차가 작은 순(동점은 관절 이동 합)으로 정한다.
‘처음 조합’ 판정: 모든 실행 기록(`evidence/G01/raw/*real*.json`)의 각 이동 후 회독(`final`, J1~J4) 상태와 비교해 J2·J3·J4 모두 ±1° 안에 든 기록이 없으면 처음 조합이다(편차는 J2~J4 최대 편차)."""
import glob
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
TOL = 1.0   # 같은 조합으로 보는 J2~J4 편차(도)


def executed_states():
    out = []

    def walk(o):
        if isinstance(o, dict):
            f = o.get("final")
            if isinstance(f, dict) and all(k in f for k in "1234") and all(isinstance(f[k], (int, float)) for k in "1234"):
                out.append([f[k] for k in "1234"])
            for v in o.values():
                walk(v)
        elif isinstance(o, list):
            for v in o:
                walk(v)
    for fn in sorted(glob.glob(str(ROOT / "evidence/G01/raw/*real*.json"))):
        walk(json.load(open(fn, encoding="utf-8")))
    return np.array(out, float)


def novelty(St, q):
    """과거 실행 상태와의 J2~J4 최대 편차 중 최솟값(도). TOL 이하면 같은 조합이 이미 실행된 것."""
    return float(np.abs(St[:, 1:4] - np.array(q[1:4], float)).max(1).min())


def main():
    sys.path.insert(0, str(ROOT / "src"))
    St = executed_states()
    b = json.load(open(ROOT / "evidence/G06/p1_bigm3_pairs.json", encoding="utf-8"))
    names = list(b["bands"].keys())
    res = []
    for e in b["common_start"]["checked"]:
        ends = {n: min(e["ends"][n], key=lambda r: (novelty(St, r["to"]), r["max_joint_delta_deg"])) for n in names if e["ends"].get(n)}
        if len(ends) < len(names):
            continue
        worst = max([novelty(St, e["start"])] + [novelty(St, r["to"]) for r in ends.values()])
        res.append((worst, sum(r["max_joint_delta_deg"] for r in ends.values()), e["start"], ends))
    res.sort(key=lambda x: (x[0], x[1]))
    if not res:
        sys.exit("선정 가능한 공통 출발이 없다")
    worst, sj, S, ends = res[0]
    poses = [{"id": 1401, "sdk_deg": S + [90], "split": "bigm3", "name": "S", "novelty_deg": round(novelty(St, S), 1)}]
    for k, n in enumerate(names):
        r = ends[n]
        poses.append({"id": 1402 + k, "sdk_deg": r["to"] + [90], "split": "bigm3", "name": "E" + n, "novelty_deg": round(novelty(St, r["to"]), 1),
                      "tip_move_mm": r["tip_move_mm"], "joint_deltas_from_S": r["joint_deltas"], "depth_S_to_E_mm": [r["depth_from_mm"], r["depth_to_mm"]],
                      "predicted_marker_shift_cam_mm": r["predicted_marker_shift_cam_mm"], "path_worst_mm": r["path_worst_mm"]})
    out = {"note": "큰 이동 M3 자세(오프라인 모델, UNVERIFIED). S에서 E50·E70·E100으로 각각 단일 관절 순서 이동. 선정 규칙은 p1_bigm3_select.py 머리 주석.", "n_executed_states": int(len(St)), "same_combo_tol_deg": TOL,
           "n_common_start_checked": len(b["common_start"]["checked"]), "worst_novelty_deg": worst, "poses": poses,
           "ranking_top3": [{"start": s, "worst_novelty_deg": w, "joint_delta_sum": j} for w, j, s, _ in res[:3]]}
    (ROOT / "evidence/G06/p1_bigm3_poses_s42.json").write_text(json.dumps(out, ensure_ascii=False, indent=1, default=float), encoding="utf-8")
    print("실행 후 상태 %d개, 공통 출발 후보 %d개 중 선정(최악 처음조합 편차 %.1f°)" % (len(St), len(res), worst))
    for p in poses:
        print(" ", p["id"], p["name"], p["sdk_deg"][:4], "편차", p["novelty_deg"], {k: p[k] for k in ("tip_move_mm", "path_worst_mm") if k in p})


if __name__ == "__main__":
    main()
