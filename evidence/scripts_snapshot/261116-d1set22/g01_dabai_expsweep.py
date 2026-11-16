#!/usr/bin/env python3
"""DaBai 컬러 노출 스윕(Jetson, 로봇 명령·관절 조회 없음). 서비스 /<ns>/set_color_exposure 로 노출을 바꿔 가며 프레임을 저장하고 평균 밝기를 기록한다.
사용: python3 g01_dabai_expsweep.py 출력디렉터리 노출값1,노출값2,... [ns=dabai] [gain=없음]
"""
import json, sys, time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from orbbec_camera_msgs.srv import SetInt32
sys.path.insert(0, str(Path(__file__).resolve().parent))
from g01_dabai_probe import write_png, arrays, qos


class Sweep(Node):
    def __init__(self, ns):
        super().__init__("g01_expsweep"); self.ns = ns; self.last = None; self.count = 0
        self.create_subscription(Image, f"/{ns}/color/image_raw", self.on, qos())
        self.exp = self.create_client(SetInt32, f"/{ns}/set_color_exposure"); self.gain = self.create_client(SetInt32, f"/{ns}/set_color_gain")
    def on(self, m): self.last = arrays(m); self.count += 1
    def call(self, cli, v):
        if not cli.wait_for_service(timeout_sec=5.0): return {"success": False, "message": "서비스 없음"}
        req = SetInt32.Request(); req.data = int(v); fut = cli.call_async(req)
        t0 = time.monotonic()
        while not fut.done() and time.monotonic() - t0 < 5.0: rclpy.spin_once(self, timeout_sec=0.1)
        r = fut.result(); return {"success": bool(r.success), "message": r.message} if r else {"success": False, "message": "응답 없음"}
    def settle(self, sec):
        end = time.monotonic() + sec; self.count = 0
        while time.monotonic() < end: rclpy.spin_once(self, timeout_sec=0.05)


def main():
    out = Path(sys.argv[1]); out.mkdir(parents=True, exist_ok=True)
    values = [int(x) for x in sys.argv[2].split(",")]; ns = sys.argv[3] if len(sys.argv) > 3 else "dabai"; gain = int(sys.argv[4]) if len(sys.argv) > 4 else None
    rclpy.init(); n = Sweep(ns); n.settle(3.0); res = []
    if gain is not None: res.append({"gain_call": n.call(n.gain, gain)}); n.settle(1.0)
    seen = {}
    for v in values:
        r = n.call(n.exp, v); n.settle(2.0)
        if n.last is None: res.append({"exposure": v, "call": r, "frame": None}); continue
        img = n.last; g = img.mean(axis=2) if img.ndim == 3 else img
        k = seen.get(v, 0); seen[v] = k + 1; write_png(out / (f"exp_{v}.png" if k == 0 else f"exp_{v}_r{k}.png"), img)   # 같은 노출 반복(파일럿 항 a): 두 번째부터 _r<k> 접미사
        res.append({"exposure": v, "call": r, "frames_in_settle": n.count, "mean_brightness": round(float(g.mean()), 1), "p05_p50_p95": [round(float(x), 1) for x in np.percentile(g, [5, 50, 95])], "saturated_ratio": round(float((g >= 250).mean()), 3)})
        print(json.dumps(res[-1], ensure_ascii=False), flush=True)
    (out / "expsweep.json").write_text(json.dumps(res, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"); rclpy.shutdown()


if __name__ == "__main__":
    main()
