"""G06 부분(③ 하강 영역) 오프라인 dry-run 드라이버. 하드웨어 명령 없음.

계획 모델(최선 추정)로 관절을 계산하고, 같은 관절 명령을 "진짜" 모델 변형(비관적 포함)에서 평가해 여유를 구한다.
격자: 베이스 기준 극좌표 패치(반경, 방위각, 종이 기준 tip 높이). 마스크 v1 ∩ 반경 교차는 G03(Gemini→베이스 변환) 전에는 못 하므로
ADR-007의 반경 한계(약 203 mm) 안쪽 패치로 대신한다(블록 위치 결정 후 좁힌다).
"""
import json
import sys
from datetime import datetime
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))

from kinematics.urdf_fk import UrdfFK                      # noqa: E402
from planning.collision_3a import clearances, finger_capsules, load_link_hull   # noqa: E402
from planning.observe import PoseGrid                       # noqa: E402
from planning.reach import Model, solve_ik, tip_state       # noqa: E402

TABLE_Z = -0.011   # 흰 보드 면의 FK 월드 z (판 윗면 +11 mm, 사용자 실측 2026-11-14)
BEST = Model(gains=(1.0, 0.913, 1.0, 1.0), dz=-0.0104, tip_y=-0.004)
TRUTH = {"best": BEST}
for g2 in (0.863, 0.963):
    for ty in (-0.002, -0.006):
        TRUTH["pess_g2=%.3f_tip=%.0f" % (g2, ty * 1000)] = Model(gains=(1.0, g2, 1.0, 1.0), dz=-0.015, tip_y=ty)
HULL_KEYS = {"base_link": "base_link", "Arm1_Joint": "Arm1_Link", "Arm2_Joint": "Arm2_Link", "Arm3_Joint": "Arm3_Link",
             "Arm4_Joint": "Arm4_Link", "dabai": "DaBai_DCW2_Link", "arm5": "Arm5_Link"}
TILTS = (-15.0, -10.0, -5.0, 0.0, 5.0, 10.0, 15.0)
RADII = (0.11, 0.13, 0.15, 0.17, 0.19)
AZIMUTHS = (-30.0, -15.0, 0.0, 15.0, 30.0)
HEIGHTS_PAPER = (0.010, 0.020, 0.030, 0.045, 0.060)
CAL_NEAR_DEG = 15.0   # 보정 자세와의 관절 공간 거리 한도(제안, 미동결)


def calibration_poses():
    q = []
    for e in json.load(open(ROOT / "evidence/G02/pilot_pnp.json")):
        q.append(e["q"])
    for res in ("g01_5b4_s424_result-real-261113-015617.json", "g01_5b4_s425_result-real-261113-102151.json"):
        for p in json.load(open(ROOT / "evidence/G01/raw" / res))["poses"]:
            q.append([p["readback"][k] for k in "1234"])
    q += [[89, 90, 10, 11], [89, 97, 10, 11], [89, 104, 10, 11], [89, 98, 10, 11], [85, 90, 10, 11]]
    return np.array(q, float)


def eval_clearance(fk, hulls, caps_by_tip, q, truth):
    out = fk.fk(np.array([list(q) + [90.0]]), gains=truth.gains, tool_offset=truth.tool_offset())
    return clearances(out, hulls, caps_by_tip(truth.tip_y), TABLE_Z, dz=truth.dz)


def path_min_clearance(fk, hulls, caps_by_tip, q_obs, q_grasp, step_deg=0.4):
    """관측 자세 -> 파지 자세를 관절 공간에서 선형 보간해 변형별 최소 여유(table, base)를 구한다."""
    q0, q1 = np.array(q_obs, float), np.array(q_grasp, float)
    n = max(2, int(np.ceil(np.abs(q1 - q0).max() / step_deg)) + 1)
    res = {name: {"table": 1e9, "base": 1e9, "table_end": None} for name in TRUTH}
    for i, t in enumerate(np.linspace(0, 1, n)):
        q = q0 + (q1 - q0) * t
        for name, truth in TRUTH.items():
            c = eval_clearance(fk, hulls, caps_by_tip, q, truth)
            res[name]["table"] = min(res[name]["table"], c["table"])
            if "base" in c:
                res[name]["base"] = min(res[name]["base"], c["base"])
            if i == n - 1:
                res[name]["table_end"] = c["table"]
    return res


def run(out_dir=ROOT / "evidence/G06", progress=None):
    fk = UrdfFK()
    hulls = {k: load_link_hull(v) for k, v in HULL_KEYS.items()}
    caps_by_tip = lambda tip_y: finger_capsules(0.09 + tip_y)   # noqa: E731
    cal = calibration_poses()
    grid = PoseGrid(fk, BEST)
    records = []
    for r in RADII:
        for az in AZIMUTHS:
            for hp in HEIGHTS_PAPER:
                x, y = r * np.sin(np.radians(az)), r * np.cos(np.radians(az))
                target = np.array([x, y, hp + TABLE_Z])      # 실제(종이 기준 hp) tip 높이를 FK 월드 z로
                plane_target = np.array([0.0, r, hp + TABLE_Z])   # 방위각 대칭: 팔 평면으로 회전한 목표(관측 탐색용)
                feas = [tl for tl in TILTS if solve_ik(fk, target, tl, BEST).get("ok")]
                rec = {"r_mm": r * 1000, "az_deg": az, "tip_height_paper_mm": hp * 1000, "feasible_tilts": feas}
                if feas:
                    tl = min(feas, key=abs)
                    qg = solve_ik(fk, target, tl, BEST)["q"]
                    rec["tilt_used"] = tl
                    rec["q_grasp"] = [round(v, 2) for v in qg]
                    rec["cal_distance_deg"] = float(np.min(np.linalg.norm(cal - np.array(qg), axis=1)))
                    rec["cal_unverified"] = rec["cal_distance_deg"] > CAL_NEAR_DEG
                    c_end = {n: eval_clearance(fk, hulls, caps_by_tip, qg, t) for n, t in TRUTH.items()}
                    rec["grasp_table_clearance_mm"] = {n: round(c["table"] * 1000, 1) for n, c in c_end.items()}
                    rec["grasp_base_clearance_mm"] = {n: round(c.get("base", np.nan) * 1000, 1) for n, c in c_end.items()}
                    rec["observation"] = {}
                    for dmin in (0.2, 0.15, 0.0):
                        cands = grid.nearest_observations(plane_target, dmin, k=12)
                        entry = None
                        if cands:
                            entry = {"nearest_tip_distance_mm": round(cands[0][1] * 1000, 1), "collision_free": None}
                            for idx, dist, px in cands:
                                q_obs = [qg[0], *[float(v) for v in grid.sdk[idx][1:]]]
                                pc = path_min_clearance(fk, hulls, caps_by_tip, q_obs, qg)
                                if pc["best"]["table"] > 0 and pc["best"]["base"] > 0:
                                    entry.update({"collision_free": True, "path_tip_distance_mm": round(dist * 1000, 1), "tilt_obs": round(float(grid.tilt[idx]), 1),
                                                  "q_obs": [round(v, 1) for v in q_obs], "pixel": [round(px[0], 1), round(px[1], 1)], "depth_mm": round(px[2] * 1000, 1),
                                                  "path_min_table_clearance_mm": {n: round(v["table"] * 1000, 1) for n, v in pc.items()},
                                                  "path_min_base_clearance_mm": {n: round(v["base"] * 1000, 1) for n, v in pc.items()}})
                                    break
                        rec["observation"]["dmin_%.2f" % dmin] = entry
                records.append(rec)
                if progress:
                    progress(len(records), rec)
    stamp = datetime.now().strftime("%y%m%d-%H%M")
    out = {"date": datetime.now().isoformat(timespec="seconds"), "plan": "docs/261114-2246-G06-오프라인-dry-run-플랜.md", "scope": "G06 부분(③ 하강 영역), 하드웨어 명령 0건",
           "table_z_fk_m": TABLE_Z, "planning_model": BEST.__dict__, "truth_variants": {k: v.__dict__ for k, v in TRUTH.items()},
           "grid": {"radii_mm": [r * 1000 for r in RADII], "azimuth_deg": AZIMUTHS, "tip_height_paper_mm": [h * 1000 for h in HEIGHTS_PAPER], "tilts_deg": TILTS},
           "cal_near_deg": CAL_NEAR_DEG, "records": records}
    Path(out_dir).mkdir(parents=True, exist_ok=True)
    path = Path(out_dir) / ("dry_run_%s.json" % stamp)
    path.write_text(json.dumps(out, ensure_ascii=False, indent=1, default=float), encoding="utf-8")
    return path, out


if __name__ == "__main__":
    def prog(n, rec):
        print(n, rec["r_mm"], rec["az_deg"], rec["tip_height_paper_mm"], rec["feasible_tilts"], flush=True)
    p, _ = run(progress=prog)
    print("saved", p)
