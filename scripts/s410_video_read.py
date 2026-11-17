#!/usr/bin/env python3
"""S4-10 영상 판독: 폰 영상의 세 자세 정지 구간에서 손끝 최저점의 눈금 m을 읽는다(손끝 y = 199 − m). 로봇 명령 없음, 영상 파일 읽기만.
사용: python3 scripts/s410_video_read.py VIDEO.mov OUT.json  (자세 정지 구간 시각은 아래 WINDOWS에 초)
방법: 영상 정지 구간을 1 fps로 뽑고 ① 첫 5~35초(팔이 프레임 밖)의 중앙값을 배경으로 둔다 ② 배경 차분으로 그리퍼(검정) 마스크를 만든다 ③ 모든 성분의 열별 하단 극값 중 가장 낮은 점을 손끝 최저점으로 본다 ④ 프레임 아래쪽에 보이는 띠의 긴 눈금(10 mm)의 세로선 12개(눈금 110, 100, ..., 0)를 서브픽셀로 찾아 직선 맞춤한 뒤 손끝 열을 눈금으로 환산한다.
"""
import sys, json, glob, subprocess, tempfile, os
import numpy as np, cv2
from scipy.signal import find_peaks

VIDEO, OUT = sys.argv[1], sys.argv[2]
WINDOWS = {"14": (92, 36), "0": (145, 12), "5": (170, 35)}   # (시작 초, 길이 초), 영상 IMG_0414의 정지 구간
BG = (5, 30)
tmp = tempfile.mkdtemp()
G = lambda p: cv2.cvtColor(cv2.imread(p), cv2.COLOR_BGR2GRAY)

def extract(name, ss, dur, fps):
    d = os.path.join(tmp, name); os.makedirs(d)
    subprocess.run(["ffmpeg", "-v", "error", "-y", "-ss", str(ss), "-t", str(dur), "-i", VIDEO, "-vf", "fps=%s" % fps, os.path.join(d, "%03d.png")], check=True)
    return sorted(glob.glob(d + "/*.png"))

bg = np.median(np.stack([G(p) for p in extract("bg", BG[0], BG[1], 0.2)]), axis=0).astype(np.float32)
bgb = cv2.GaussianBlur(bg, (5, 5), 0)

def ticks(g):
    prof = g.astype(np.float32)[1056:1078, :].mean(axis=0)
    base = cv2.GaussianBlur(prof.reshape(1, -1), (0, 0), 15).ravel()
    pk, _ = find_peaks(base - prof, height=25, distance=60)
    return [int(p) for p in pk if 100 < p < 1340]

def subpix(g, x):
    prof = g.astype(np.float32)[1056:1078, :].mean(axis=0)
    xs = np.arange(x - 6, x + 7); w = prof[x - 6:x + 7].max() - prof[x - 6:x + 7]
    return float((xs * w).sum() / w.sum())

result = {"video": os.path.basename(VIDEO), "mapping": "긴 눈금 12개 = 눈금 110, 100, ..., 0 (왼쪽부터), 손끝 y = 199 - m", "poses": {}}
for name, (ss, dur) in WINDOWS.items():
    rows = []
    for p in extract("p" + name, ss, dur, 1):
        g = G(p); gf = cv2.GaussianBlur(g, (5, 5), 0).astype(np.float32)
        m = ((np.abs(gf - bgb) > 50) & (gf < 95)); m[860:, :] = False; m[:, :260] = False
        m = cv2.morphologyEx(m.astype(np.uint8), cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        k, lab, st, _ = cv2.connectedComponentsWithStats(m)
        comps = [j for j in range(1, k) if st[j, 4] > 800]
        tk = ticks(g)
        if not comps or len(tk) != 12: continue
        sub = np.array([subpix(g, x) for x in tk]); A = np.polyfit(sub, np.arange(110, -1, -10), 1)
        resid = float(np.abs(np.polyval(A, sub) - np.arange(110, -1, -10)).max())
        allm = np.isin(lab, comps); cols = np.where(allm.any(axis=0))[0]
        bottom = np.array([np.where(allm[:, c])[0].max() for c in cols])
        sm = np.convolve(bottom, np.ones(9) / 9, mode="same")
        pk, _ = find_peaks(sm, distance=40, prominence=6)
        cand = [(float(sm[i]), float(cols[i])) for i in pk]
        low, lx = max(cand)
        rows.append({"frame": os.path.basename(p), "tip_col": round(lx, 1), "tip_row": round(low, 1), "m": round(float(np.polyval(A, lx)), 2), "fit_resid_mm": round(resid, 2)})
    ms = np.array([r["m"] for r in rows])
    result["poses"][name] = {"frames": len(rows), "m_median": round(float(np.median(ms)), 2), "m_std": round(float(ms.std()), 2), "m_min": float(ms.min()), "m_max": float(ms.max()),
                             "y_median": round(199 - float(np.median(ms)), 1), "max_fit_resid_mm": max(r["fit_resid_mm"] for r in rows), "samples": rows[:3]}
    mid = rows[len(rows) // 2]; im = cv2.imread([p for p in glob.glob(os.path.join(tmp, "p" + name, "*.png")) if os.path.basename(p) == mid["frame"]][0])
    cv2.circle(im, (int(mid["tip_col"]), int(mid["tip_row"])), 12, (0, 0, 255), 2); cv2.line(im, (int(mid["tip_col"]), int(mid["tip_row"])), (int(mid["tip_col"]), 1060), (0, 0, 255), 2)
    x0 = max(int(mid["tip_col"]) - 400, 0); y0 = max(int(mid["tip_row"]) - 330, 0)
    cv2.imwrite(OUT.replace(".json", "_pose%s.png" % name), im[y0:1080, x0:x0 + 800])
json.dump(result, open(OUT, "w"), ensure_ascii=False, indent=2)
print(json.dumps({k: {kk: vv for kk, vv in v.items() if kk != "samples"} for k, v in result["poses"].items()}, ensure_ascii=False, indent=1))
