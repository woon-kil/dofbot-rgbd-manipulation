#!/usr/bin/env python3
"""S4-10 영상에서 세 자세의 (a) 두 손가락 끝 (y, 높이) (b) 손가락 축 각도(이미지 세로 기준) (c) 카메라 롤을 읽는다. 로봇 명령 없음.
출력은 evidence/G02/s410_video_geometry.json. 사용: python3 scripts/s410_finger_geometry.py VIDEO OUT.json H_STRIP_MM"""
import sys, json, glob, subprocess, tempfile, os
import numpy as np, cv2
from scipy.signal import find_peaks
VIDEO, OUT, HS = sys.argv[1], sys.argv[2], float(sys.argv[3])
WINDOWS = {"14": (92, 36), "0": (145, 12), "5": (170, 35)}
tmp = tempfile.mkdtemp(); G = lambda p: cv2.cvtColor(cv2.imread(p), cv2.COLOR_BGR2GRAY)
def extract(name, ss, dur, fps):
    d = os.path.join(tmp, name); os.makedirs(d)
    subprocess.run(["ffmpeg", "-v", "error", "-y", "-ss", str(ss), "-t", str(dur), "-i", VIDEO, "-vf", "fps=%s" % fps, os.path.join(d, "%03d.png")], check=True)
    return sorted(glob.glob(d + "/*.png"))
bg = np.median(np.stack([G(p) for p in extract("bg", 5, 30, 0.2)]), axis=0).astype(np.float32); bgb = cv2.GaussianBlur(bg, (5, 5), 0)
def ticks(g):
    prof = g.astype(np.float32)[1056:1078, :].mean(axis=0); base = cv2.GaussianBlur(prof.reshape(1, -1), (0, 0), 15).ravel()
    pk, _ = find_peaks(base - prof, height=25, distance=60); return [int(p) for p in pk if 100 < p < 1340]
def subpix(g, x):
    prof = g.astype(np.float32)[1056:1078, :].mean(axis=0); xs = np.arange(x - 6, x + 7); w = prof[x - 6:x + 7].max() - prof[x - 6:x + 7]; return float((xs * w).sum() / w.sum())
def strip_top(g):   # 띠 윗면(앞 가장자리) 행: 열별 세로 기울기 최대
    gf = cv2.GaussianBlur(g, (0, 0), 2).astype(np.float32); rows = []
    for c in range(330, 1200, 30):
        col = gf[840:900, c]; d = np.diff(col); rows.append((c, 840 + int(np.argmin(d)) + 0.5))   # 밝은 면 위 어두운 선 → 아래쪽으로 밝아지는 곳
    return rows
res = {"video": os.path.basename(VIDEO), "h_strip_mm": HS, "poses": {}}
for name, (ss, dur) in WINDOWS.items():
    recs = []
    for p in extract("p" + name, ss, dur, 1):
        g = G(p); gf = cv2.GaussianBlur(g, (5, 5), 0).astype(np.float32)
        m = ((np.abs(gf - bgb) > 50) & (gf < 95)); m[860:, :] = False; m[:, :260] = False
        m = cv2.morphologyEx(m.astype(np.uint8), cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        k, lab, st, _ = cv2.connectedComponentsWithStats(m); comps = [j for j in range(1, k) if st[j, 4] > 800]
        tk = ticks(g)
        if not comps or len(tk) != 12: continue
        sub = np.array([subpix(g, x) for x in tk]); A = np.polyfit(sub, np.arange(110, -1, -10), 1); pxmm = abs(1 / A[0])
        allm = np.isin(lab, comps); cols = np.where(allm.any(axis=0))[0]; bottom = np.array([np.where(allm[:, c])[0].max() for c in cols])
        sm = np.convolve(bottom, np.ones(9) / 9, mode="same"); pk, _ = find_peaks(sm, distance=40, prominence=6)
        cand = sorted([(float(sm[i]), float(cols[i])) for i in pk], reverse=True)[:2]
        tips = []
        for low, lx in cand:
            sel = allm.copy(); sel[:int(low) - 170, :] = False; sel[int(low) + 3:, :] = False; sel[:, :int(lx) - 60] = False; sel[:, int(lx) + 60:] = False
            n2, lab2, st2, _ = cv2.connectedComponentsWithStats(sel.astype(np.uint8)); 
            if n2 < 2: continue
            j = 1 + int(np.argmax(st2[1:, 4])); ys, xs = np.where(lab2 == j); pts = np.stack([xs, ys], 1).astype(np.float32); mu = pts.mean(axis=0)
            u, s, vt = np.linalg.svd(pts - mu, full_matrices=False); ax = vt[0] if vt[0][1] > 0 else -vt[0]   # 아래쪽(+행)을 향하게
            ang = float(np.degrees(np.arctan2(ax[0], ax[1])))   # 이미지 세로에서 +y(오른쪽)로 기운 각도
            tips.append({"col": round(lx, 1), "row": round(low, 1), "m": round(float(np.polyval(A, lx)), 2), "axis_angle_deg": round(ang, 2)})
        top = strip_top(g); slope = np.polyfit([c for c, r in top], [r for c, r in top], 1)[0]
        recs.append({"px_per_mm": round(float(pxmm), 3), "tips": tips, "strip_top_row": round(float(np.median([r for c, r in top])), 1), "strip_top_slope_deg": round(float(np.degrees(np.arctan(slope))), 2)})
    def med(f): return float(np.median([f(r) for r in recs if r["tips"]]))
    low_tip = lambda r: max(r["tips"], key=lambda t: t["row"])
    out = {"frames": len(recs), "px_per_mm_median": round(med(lambda r: r["px_per_mm"]), 3), "strip_top_row_median": round(med(lambda r: r["strip_top_row"]), 1), "roll_deg_from_strip_top": round(med(lambda r: r["strip_top_slope_deg"]), 2),
           "lowest_tip": {"col": round(med(lambda r: low_tip(r)["col"]), 1), "row": round(med(lambda r: low_tip(r)["row"]), 1), "m": round(med(lambda r: low_tip(r)["m"]), 2), "axis_angle_deg": round(med(lambda r: low_tip(r)["axis_angle_deg"]), 2)}}
    out["lowest_tip"]["y_mm"] = round(199 - out["lowest_tip"]["m"], 1)
    out["lowest_tip"]["height_above_strip_top_mm"] = round((out["strip_top_row_median"] - out["lowest_tip"]["row"]) / out["px_per_mm_median"], 1)
    out["lowest_tip"]["height_above_paper_mm"] = round(out["lowest_tip"]["height_above_strip_top_mm"] + HS, 1)
    out["lowest_tip"]["height_above_plate_mm"] = round(out["lowest_tip"]["height_above_paper_mm"] - 11, 1)
    other = [t for r in recs for t in r["tips"] if abs(t["col"] - out["lowest_tip"]["col"]) > 40]
    if other: out["other_tip"] = {"col": round(float(np.median([t["col"] for t in other])), 1), "row": round(float(np.median([t["row"] for t in other])), 1), "axis_angle_deg": round(float(np.median([t["axis_angle_deg"] for t in other])), 2)}
    res["poses"][name] = out
json.dump(res, open(OUT, "w"), ensure_ascii=False, indent=1); print(json.dumps(res["poses"], ensure_ascii=False, indent=1))
