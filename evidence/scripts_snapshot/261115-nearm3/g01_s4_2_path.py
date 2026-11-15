#!/usr/bin/env python3
"""S4-2 파일럿 자세 경로 계획(오프라인, URDF FK 모델, 하드웨어 없음). 파일럿 12개 자세를 가까운 순서로 정렬하고, 자세 사이를 **한 번에 한 관절씩**(J1→J2→J3→J4 순서 후보 중 클리어런스가 가장 큰 순서) 이동할 때 사슬(관절·TCP·DaBai 모서리)의
작업대 위 최저 높이를 1° 간격으로 계산한다. 모델 값=UNVERIFIED."""
import sys, json, itertools, numpy as np
sys.path.insert(0, "scripts")
import importlib.util
spec = importlib.util.spec_from_file_location("s3", "scripts/g01_s3_path.py"); m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m)
fk, box = m.fk, m.box
def chain_z(j):
    f = fk(np.array([[j[0], j[1], j[2], j[3], j[4]]], float)); dp, dR = f["dabai"]
    z = [f["tcp"][0][0][2], f["arm5"][0][0][2], f["Arm4_Joint"][0][0][2], f["Arm3_Joint"][0][0][2]] + [(dp[0] + dR[0] @ c)[2] for c in box]
    tcp = f["tcp"][0][0] * 1000
    return min(z) * 1000, tcp
def seg_min(a, b, joint):
    lo = 1e9
    n = max(abs(b[joint] - a[joint]), 1)
    for k in range(int(n) + 1):
        p = list(a); p[joint] = a[joint] + (b[joint] - a[joint]) * k / n
        lo = min(lo, chain_z(p)[0])
    return lo
def best_order(a, b, joints=(0, 1, 2, 3)):
    best = None
    for order in itertools.permutations(joints):
        cur = list(a); lo = 1e9
        for j in order:
            if cur[j] == b[j]: continue
            nxt = list(cur); nxt[j] = b[j]; lo = min(lo, seg_min(cur, nxt, j)); cur = nxt
        if best is None or lo > best[0]: best = (lo, order)
    return best
d = json.load(open("evidence/G01/b4_pose_candidates_y230_72x60_j34min10_clear60.json"))
pilot = [p for p in d["poses"] if p["split"] == "pilot"]
start = [90, 91, 14, 12, 90]    # S4-1b에서 실측으로 도달한 가장 깊은 자세(J3≈14, J4≈12)
rem = pilot[:]; seq = []; cur = start
while rem:
    nxt = min(rem, key=lambda p: sum(abs(x - y) for x, y in zip(cur, p["sdk_deg"]))); seq.append(nxt); rem.remove(nxt); cur = nxt["sdk_deg"]
out = {"start": start, "poses": [], "transitions": []}; cur = start; worst = (1e9, None)
for p in seq:
    lo, order = best_order(cur, p["sdk_deg"]); z, tcp = chain_z(p["sdk_deg"]); names = ["J1", "J2", "J3", "J4"]
    out["poses"].append({"id": p["id"], "sdk": p["sdk_deg"], "pose_min_z": round(z, 1)})
    out["transitions"].append({"to": p["id"], "from": cur, "order": [names[j] for j in order if cur[j] != p["sdk_deg"][j]], "path_min_z": round(lo, 1), "joint_deltas": [p["sdk_deg"][i] - cur[i] for i in range(4)]})
    worst = min(worst, (lo, p["id"])); cur = p["sdk_deg"]
    print("pose %2d %s <- %s 순서 %s 경로 최저 %.1f mm, 자세 최저 %.1f mm, 변화 %s" % (p["id"], p["sdk_deg"], out["transitions"][-1]["from"], "→".join(out["transitions"][-1]["order"]) or "-", lo, z, out["transitions"][-1]["joint_deltas"]))
# 복귀: 마지막 자세 → 시작(깊은 자세) 후 계단 펴기는 S3 순차와 같음
lo, order = best_order(cur, start); out["return_to_start"] = {"path_min_z": round(lo, 1), "order": [["J1", "J2", "J3", "J4"][j] for j in order if cur[j] != start[j]]}
print("복귀(마지막 자세 → 시작 깊은 자세) 경로 최저 %.1f mm, 순서 %s" % (lo, out["return_to_start"]["order"]))
print("전체 경로 최저 %.1f mm (자세 %s로 가는 구간), 총 이동 관절 수 %d" % (min(worst[0], lo), worst[1], sum(len(t["order"]) for t in out["transitions"])))
out["worst_path_min_z"] = round(min(worst[0], lo), 1)
json.dump(out, open("evidence/G01/s4_2_path_plan.json", "w"), ensure_ascii=False, indent=1)
