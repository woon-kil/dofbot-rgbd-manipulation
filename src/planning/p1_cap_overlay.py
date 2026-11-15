"""③ P1 촬영 사진(A·B·B2) 위에 모델이 예측한 그리퍼 캡 원판 투영을 겹쳐 그린다(오프라인). 캡 중심 x 4.85(닫힘 가정)·12·29.2 mm(열림)를 색별로.
사용: python3 -I src/planning/p1_cap_overlay.py <촬영 폴더> <실행 결과 json> <출력 폴더>   (출력 PNG는 개인 공간이 찍혀 gitignore 폴더에 둔다)"""
import json
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))

from kinematics.urdf_fk import UrdfFK                                   # noqa: E402
from planning.collision_3a import finger_capsules                       # noqa: E402
from planning.dry_run import BEST                                       # noqa: E402
from planning.observe import camera_pose, K                             # noqa: E402

COLORS = {0.00485: (0, 0, 255), 0.012: (0, 200, 255), 0.0292: (255, 128, 0)}   # BGR: 빨강 닫힘, 노랑 12 mm, 파랑 열림


def main():
    folder, res, out = Path(sys.argv[1]), json.load(open(sys.argv[2])), Path(sys.argv[3]); out.mkdir(parents=True, exist_ok=True)
    fk = UrdfFK(); rb = {str(p["pose_id"]): p["readback"] for p in res["poses"]}; rows = {}
    for name, pid in (("A", "1201"), ("B", "1202"), ("B2", "1203")):
        q = [rb[pid][str(k)] for k in (1, 2, 3, 4)]
        im = cv2.imread(str(folder / ("p%s" % pid) / "exp_25.png"))
        o = fk.fk(np.array([q + [90.0]]), gains=BEST.gains, tool_offset=BEST.tool_offset())
        p5 = o["arm5"][0][0] + np.array([0, 0, BEST.dz]); R5 = o["arm5"][1][0]; cp, Rc = camera_pose(fk, q, BEST)
        proj = lambda P: (lambda p: (K[0] * p[0] / p[2] + K[2], K[1] * p[1] / p[2] + K[3], p[2]))(Rc.T @ (P - cp))   # noqa: E731
        rows[name] = {}
        for cx, col in COLORS.items():
            ends = []
            for a, b, rr in finger_capsules(0.09 + BEST.tip_y, cx)[::2]:   # 캡(금속 링크 연장 제외)
                for P in (a, b):
                    u, v, d = proj(p5 + R5 @ P); ends.append((round(u, 1), round(v, 1), round(d * 1000, 1)))
                    if d > 0:
                        cv2.circle(im, (int(round(u)), int(round(v))), max(1, int(round(K[0] * rr / d))), col, 2)
            rows[name]["cap_center_x_mm_%.2f" % (cx * 1000)] = ends
        cv2.imwrite(str(out / ("p1_cap_overlay_%s.png" % name)), im)
    (out / "p1_cap_overlay.json").write_text(json.dumps(rows, ensure_ascii=False, indent=1), encoding="utf-8"); print(json.dumps(rows, ensure_ascii=False, indent=1))


if __name__ == "__main__":
    main()
