#!/usr/bin/env python3
"""P-A: 상시 구동 중인 Gemini 드라이버에서 '요청 이후 노출된' 첫 color·depth 프레임을 저장한다(컨테이너 안, ROS Humble). 로봇 명령 없음.
사용: cd /work && python3 pa_snap.py <출력디렉터리> <요청id> [타임아웃초=5] [네임스페이스=camera]
저장: <출력>/color.png(rgb8, 36 s 방식과 같은 write_png), depth_mm.npy, meta.json(요청 id·요청/프레임 시각). 이미지 변환·PNG 쓰기는 g01_gemini_probe(36 s 방식과 같은 코드)를 그대로 쓴다.
프레임 신선도: 헤더 stamp > 요청 수신 시각(time.time())인 첫 프레임만 쓴다(팔이 멈추기 전에 노출된 프레임을 쓰지 않기 위함). 타임아웃이면 프레임 없이 meta.json(ok=false)만 남기고 종료 코드 3.
이전 요청의 color.png·depth_mm.npy·meta.json은 시작하자마자 지운다(옛 파일을 새 결과로 오인하지 않게)."""
import json, sys, time
from pathlib import Path


class FreshCollector:
    """요청 수신 시각 req_recv 이후에 노출된(헤더 stamp > req_recv) 첫 프레임을 종류별로 모은다. ROS 없이 시험할 수 있는 순수 로직."""
    def __init__(self, req_recv, kinds=("color", "depth")):
        self.req_recv = req_recv; self.kinds = tuple(kinds); self.got = {}; self.stale = {k: 0 for k in self.kinds}

    def offer(self, kind, stamp, recv, payload, frame_id=""):
        if kind in self.got: return False
        if not (stamp > self.req_recv): self.stale[kind] += 1; return False
        self.got[kind] = {"stamp": stamp, "recv": recv, "payload": payload, "frame_id": frame_id}; return True

    def done(self): return all(k in self.got for k in self.kinds)


def build_meta(request_id, req_recv, col, now, ok, error=None, stale=None):
    m = {"request_id": request_id, "ok": bool(ok), "req_recv_time": req_recv, "saved_time": now, "stale_dropped": stale or {}}
    if error: m["error"] = error
    for k in ("color", "depth"):
        if col and k in col:
            f = col[k]; m[k] = {"stamp": f["stamp"], "recv": f["recv"], "recv_minus_stamp": f["recv"] - f["stamp"], "stamp_minus_req": f["stamp"] - req_recv, "frame_id": f["frame_id"]}
    return m


def main():
    req_recv = time.time()
    out = Path(sys.argv[1]); rid = sys.argv[2]; timeout = float(sys.argv[3]) if len(sys.argv) > 3 else 5.0; ns = sys.argv[4] if len(sys.argv) > 4 else "camera"
    out.mkdir(parents=True, exist_ok=True)
    for n in ("color.png", "depth_mm.npy", "meta.json"):
        try: (out / n).unlink()
        except FileNotFoundError: pass
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    import numpy as np
    from g01_gemini_probe import qos, arr, write_png
    col = FreshCollector(req_recv)
    rclpy.init()
    class N(Node):
        def __init__(self):
            super().__init__("pa_snap"); self.seen = set()
            for rel in (True, False):
                self.create_subscription(Image, f"/{ns}/color/image_raw", lambda m: self.on("color", m), qos(rel))
                self.create_subscription(Image, f"/{ns}/depth/image_raw", lambda m: self.on("depth", m), qos(rel))
        def on(self, kind, m):
            key = (kind, m.header.stamp.sec, m.header.stamp.nanosec)
            if key in self.seen: return
            self.seen.add(key); recv = time.time(); st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            if st > req_recv and kind not in col.got: col.offer(kind, st, recv, arr(m), m.header.frame_id)
            else: col.offer(kind, st, recv, None)
    node = N(); deadline = time.monotonic() + timeout
    while not col.done() and time.monotonic() < deadline: rclpy.spin_once(node, timeout_sec=0.05)
    ok = col.done(); now = time.time()
    if ok:
        write_png(out / "color.png", col.got["color"]["payload"]); np.save(out / "depth_mm.npy", col.got["depth"]["payload"])
    meta = build_meta(rid, req_recv, col.got, now, ok, None if ok else "타임아웃: 요청 이후 노출된 프레임을 받지 못했다(드라이버가 꺼졌거나 프레임이 오지 않음)", col.stale)
    (out / "meta.json").write_text(json.dumps(meta, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps({k: meta[k] for k in ("request_id", "ok")}, ensure_ascii=False)); rclpy.shutdown()
    sys.exit(0 if ok else 3)


if __name__ == "__main__": main()
