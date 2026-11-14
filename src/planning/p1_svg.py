"""③ P1 받침대·자세 배치 도식(SVG) 생성과 역산 검증. 사진 위가 아니라 도식이다(사진 대조 미실시).
상단: 위에서 본 그림(위=베이스, 아래=사용자 쪽, 오른쪽=프로젝트 월드 +x). 하단: 옆면(왼쪽=베이스, 오른쪽=사용자 쪽, 위=높이, 종이=0)."""
import json
import re
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))
from kinematics.urdf_fk import UrdfFK                      # noqa: E402
from planning.dry_run import BEST, TABLE_Z                 # noqa: E402
from planning.observe import camera_pose                   # noqa: E402

POSES = json.load(open(ROOT / "evidence/G06/p1_pose_list_r190.json"))["poses"]
R = 190.0
BLOCK_H = 30.0                                   # 실제 노란 블록 높이 30 mm(사용자, 2026-11-15)
PLAT = (-15.0, R - 30.0, 15.0, R + 30.0)         # x0, y0, x1, y1 (mm)
MARK = (-12.5, R - 12.5, 12.5, R + 12.5)       # 25 mm 마커
BOARD = (-36.0, 199.0, 36.0, 259.0)              # 평판 ChArUco 검은 무늬(직전 위치, 현재 상태 미확인)
RULER = (37.0, 89.0, 53.0, 239.0)                # 금속 자(직전 위치, 현재 상태 미확인)
PLATE = (-92.0, -40.0, 92.0, 73.0)               # 베이스 판 앞부분: 앞 가장자리 y = 73 mm(사용자 실측 2026-11-13, evidence/G02/board_measurement.json J1_axis_behind_A_mm), 폭 ±92는 base_link 메시 bbox
CAP_X, CAP_T, CAP_L = 29.2, 9.7, 25.0
KT, KS = 2.0, 2.0                                # px/mm (상단, 측면)
TOP_O = (250.0, 90.0)                            # 상단 패널 원점(x=0,y=0 → px)
SIDE_O = (560.0, 640.0)                           # 측면 패널 원점(y=−40 → 80+, z=0 → 800)
SIDE_Y0 = -40.0

def tx(x): return TOP_O[0] + KT * x
def ty(y): return TOP_O[1] + KT * (y + 40.0)
def sx(y): return SIDE_O[0] + KS * (y - SIDE_Y0)
def sz(z): return SIDE_O[1] - KS * z

els = []   # (id, kind, svg_string, data_mm tuple for verification)
def rect_top(id_, box, cls, label=None):
    x0, y0, x1, y1 = box
    els.append((id_, "rect_top", '<rect id="%s" class="%s" x="%.2f" y="%.2f" width="%.2f" height="%.2f" data-mm="%s"/>' % (id_, cls, tx(x0), ty(y0), KT * (x1 - x0), KT * (y1 - y0), ",".join("%.2f" % v for v in box)), box))
    if label: els.append((id_ + "_l", "text", '<text x="%.1f" y="%.1f" class="lb">%s</text>' % (tx(x1) + 5, ty((y0 + y1) / 2), label), None))
def rect_side(id_, box, cls):
    y0, z0, y1, z1 = box
    els.append((id_, "rect_side", '<rect id="%s" class="%s" x="%.2f" y="%.2f" width="%.2f" height="%.2f" data-mm="%s"/>' % (id_, cls, sx(y0), sz(z1), KS * (y1 - y0), KS * (z1 - z0), ",".join("%.2f" % v for v in box)), box))
def dot_top(id_, x, y, cls, label, dy=0):
    els.append((id_, "dot_top", '<circle id="%s" class="%s" cx="%.2f" cy="%.2f" r="4" data-mm="%.2f,%.2f"/><text x="%.1f" y="%.1f" class="lb">%s</text>' % (id_, cls, tx(x), ty(y), x, y, tx(x) + 6, ty(y) - 5 + dy, label), (x, y)))
def dot_side(id_, y, z, cls, label, dy=0):
    els.append((id_, "dot_side", '<circle id="%s" class="%s" cx="%.2f" cy="%.2f" r="4" data-mm="%.2f,%.2f"/><text x="%.1f" y="%.1f" class="lb">%s</text>' % (id_, cls, sx(y), sz(z), y, z, sx(y) + 6, sz(z) - 5 + dy, label), (y, z)))

rect_top("plate", PLATE, "plate", "베이스 판(앞 가장자리 y=73 실측)")
rect_top("ruler", RULER, "ruler", "금속 자(직전 위치)")
rect_top("board", BOARD, "board", "평판 보드(직전 위치)")
rect_top("platform", PLAT, "plat", "블록 30×60×30 (r=190)")
rect_top("marker", MARK, "mark")
fk = UrdfFK()
tips = {}
for k in ("A", "B", "B2"):
    q = POSES[k]["sdk"]; tx_, ty_, tz_ = POSES[k]["tip_xyz_paper_mm"]; tau = np.radians(POSES[k]["tilt_deg"])
    cp, _ = camera_pose(fk, q, BEST); cpx, cpy, cpz = -cp[0] * 1000, cp[1] * 1000, (cp[2] - TABLE_Z) * 1000     # 프로젝트 x = −FK x
    tips[k] = dict(y=ty_, z=tz_, tau=tau, cam=(cpx, cpy, cpz))
    ycap = ty_ - CAP_L * np.sin(tau)
    for sgn, nm in ((-1, "L"), (1, "R")):
        x0 = sgn * CAP_X - CAP_T / 2
        rect_top("cap_%s_%s" % (k, nm), (x0, min(ty_, ycap), x0 + CAP_T, max(ty_, ycap)), "cap")
    dot_top("tip_top_" + k, -tx_, ty_, "tip", "")
    dot_top("cam_top_" + k, cpx, cpy, "cam", "")
    dot_side("tip_side_" + k, ty_, tz_, "tip", k + " tip", {"A": 0, "B": 16, "B2": 8}[k])
    dot_side("cam_side_" + k, cpy, cpz, "cam", k + " 카메라", {"A": 0, "B": 16, "B2": 8}[k])
# 측면
rect_side("paper", (-40.0, -2.0, 260.0, 0.0), "paperl")
rect_side("plate_s", (-40.0, 0.0, 73.0, 11.0), "plate")
rect_side("plat_s", (PLAT[1], 0.0, PLAT[3], BLOCK_H), "plat")
rect_side("mark_s", (MARK[1], BLOCK_H, MARK[3], BLOCK_H + 2.0), "mark")
lines = []
for k, t in tips.items():
    a = np.array([np.sin(t["tau"]), -np.cos(t["tau"])]); back = np.array([t["y"], t["z"]]) - CAP_L * a
    lines.append('<line class="capl" x1="%.2f" y1="%.2f" x2="%.2f" y2="%.2f"/>' % (sx(t["y"]), sz(t["z"]), sx(back[0]), sz(back[1])))
    cy_, cz_ = t["cam"][1], t["cam"][2]
    lines.append('<line class="ray" x1="%.2f" y1="%.2f" x2="%.2f" y2="%.2f"/>' % (sx(cy_), sz(cz_), sx(R), sz(BLOCK_H)))

def build():
    css = """<style>
.plate{fill:#d9d9d9;stroke:#666}.ruler{fill:#f3c5c5;stroke:#b33;stroke-dasharray:4 3}.board{fill:#f3c5c5;stroke:#b33;stroke-dasharray:4 3}
.plat{fill:#a8d5a2;stroke:#2c6e2c}.mark{fill:#222}.cap{fill:#555;opacity:.8}.tip{fill:#d22}.cam{fill:#27c}.paperl{fill:#bbb}
.lb{font:12px sans-serif;fill:#222}.hd{font:bold 16px sans-serif;fill:#000}.nt{font:13px sans-serif;fill:#333}.capl{stroke:#555;stroke-width:6}.ray{stroke:#27c;stroke-width:1;stroke-dasharray:5 4}
</style>"""
    out = ['<svg xmlns="http://www.w3.org/2000/svg" width="1250" height="1250" viewBox="0 0 1250 1250">', css, '<rect width="1250" height="1250" fill="#fff"/>']
    out.append('<text x="20" y="26" class="hd">③ P1 받침대·자세 배치 도식 (사진 대조 미실시, 승인 아님)</text>')
    out.append('<text x="20" y="46" class="nt">위에서 본 그림: 위=베이스, 아래=사용자 쪽, 오른쪽=프로젝트 +x. 단위 mm. 값 출처 evidence/G06/p1_pose_list_r190.json (최선 모델)</text>')
    out += [e[2] for e in els if e[1] in ("rect_top", "dot_top") or (e[1] == "text")]
    out.append('<text x="560" y="90" class="hd">옆면 (왼쪽=베이스, 오른쪽=사용자 쪽, 종이 z=0)</text>')
    out.append('<text x="20" y="80" class="hd">위에서 본 그림</text>')
    out += [e[2] for e in els if e[1] in ("rect_side", "dot_side")]
    out += lines
    out.append('<text x="20" y="770" class="nt">붉은 점 = tip(캡 끝면 가운데) A·B·B2 y=%.1f·%.1f·%.1f mm, 파란 점 = 카메라(위에서 본 그림 x=+14.4 mm) y=%.1f·%.1f·%.1f mm</text>' % (tips["A"]["y"], tips["B"]["y"], tips["B2"]["y"], tips["A"]["cam"][1], tips["B"]["cam"][1], tips["B2"]["cam"][1]))
    out.append('<text x="20" y="800" class="nt">붉은 점선 = 직전 배치 물체(보드·자): 받침대·손가락과 겹치거나 가까우므로 P1 전에 치워야 함(현재 상태는 사진으로 확인 필요).</text>')
    out.append('</svg>')
    return "\n".join(out)

def verify(svg_text):
    rows = []
    for m in re.finditer(r'<(rect|circle) id="([^"]+)" class="[^"]*"([^>]*?)data-mm="([^"]+)"', svg_text):
        pass
    for id_, kind, s, mm in els:
        if mm is None: continue
        a = dict(re.findall(r'([a-z]+)="([-0-9.]+)"', s.split(">")[0]))
        if kind == "rect_top":
            x0 = (float(a["x"]) - TOP_O[0]) / KT; y0 = (float(a["y"]) - TOP_O[1]) / KT - 40.0
            x1 = x0 + float(a["width"]) / KT; y1 = y0 + float(a["height"]) / KT; got = (x0, y0, x1, y1)
        elif kind == "rect_side":
            y0 = (float(a["x"]) - SIDE_O[0]) / KS + SIDE_Y0; z1 = (SIDE_O[1] - float(a["y"])) / KS
            y1 = y0 + float(a["width"]) / KS; z0 = z1 - float(a["height"]) / KS; got = (y0, z0, y1, z1)
        elif kind == "dot_top":
            got = ((float(a["cx"]) - TOP_O[0]) / KT, (float(a["cy"]) - TOP_O[1]) / KT - 40.0)
        else:
            got = ((float(a["cx"]) - SIDE_O[0]) / KS + SIDE_Y0, (SIDE_O[1] - float(a["cy"])) / KS)
        err = max(abs(g - w) for g, w in zip(got, mm))
        rows.append((id_, kind, np.round(mm, 2).tolist(), np.round(got, 2).tolist(), round(err, 3)))
    return rows

if __name__ == "__main__":
    svg = build()
    out = ROOT / "results/g07_p1_layout_v1.svg"
    out.write_text(svg, encoding="utf-8")
    rows = verify(svg)
    print("요소 %d개 역산 검증 (글의 mm ↔ SVG 좌표 역변환), 최대 오차 %.3f mm" % (len(rows), max(r[4] for r in rows)))
    for r in rows: print("  %-14s %-9s 글 %s → 역산 %s 오차 %.3f" % r)
    print("받침대 y 범위 %.0f~%.0f, 보드 y 범위 %.0f~%.0f → 겹침 %.0f mm" % (PLAT[1], PLAT[3], BOARD[1], BOARD[3], max(0, min(PLAT[3], BOARD[3]) - max(PLAT[1], BOARD[1]))))
    print("캡 바깥쪽 x %.2f mm, 금속 자 안쪽 x %.1f mm → 간격 %.2f mm" % (CAP_X + CAP_T / 2, RULER[0], RULER[0] - (CAP_X + CAP_T / 2)))
