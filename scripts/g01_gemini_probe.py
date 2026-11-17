#!/usr/bin/env python3
"""G01 Gemini 335L 취득 프로브(컨테이너 안, ROS Humble). 로봇 명령 없음.
사용: python3 g01_gemini_probe.py <출력디렉터리> <초> [네임스페이스=camera]
"""
import json, math, struct, sys, time, zlib
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image

def qos(reliable):
    return QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10,
                      reliability=ReliabilityPolicy.RELIABLE if reliable else ReliabilityPolicy.BEST_EFFORT,
                      durability=DurabilityPolicy.VOLATILE)

def write_png(path, image):
    h, w = image.shape[:2]
    if image.ndim == 2:
        ct, bd, rows = 0, 16, image.astype(">u2")
    else:
        ct, bd, rows = 2, 8, image
    raw = b"".join(b"\x00" + np.ascontiguousarray(rows[r]).tobytes() for r in range(h))
    def chunk(tag, data):
        return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)
    path.write_bytes(b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, bd, ct, 0, 0, 0)) + chunk(b"IDAT", zlib.compress(raw, 6)) + chunk(b"IEND", b""))

def arr(m):
    if m.encoding == "rgb8":
        return np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, 3).copy()
    if m.encoding == "16UC1":
        return np.frombuffer(m.data, np.uint16).reshape(m.height, m.width).copy()
    raise RuntimeError("예상 밖 인코딩 " + m.encoding)

class Probe(Node):
    def __init__(self, ns):
        super().__init__("g01_gemini_probe")
        self.t0 = time.monotonic(); self.rx = {"color": [], "depth": []}; self.skew = {}
        self.seen = set(); self.valid = []; self.bright = []; self.last = {}; self.info = {}; self.first = {}
        for rel in (True, False):
            self.create_subscription(Image, f"/{ns}/color/image_raw", lambda m: self.on("color", m), qos(rel))
            self.create_subscription(Image, f"/{ns}/depth/image_raw", lambda m: self.on("depth", m), qos(rel))
            self.create_subscription(CameraInfo, f"/{ns}/color/camera_info", lambda m: self.inf("color", m), qos(rel))
            self.create_subscription(CameraInfo, f"/{ns}/depth/camera_info", lambda m: self.inf("depth", m), qos(rel))
    def on(self, kind, m):
        key = (kind, m.header.stamp.sec, m.header.stamp.nanosec)
        if key in self.seen: return  # RELIABLE/BEST_EFFORT 두 구독의 중복 수신 제거
        self.seen.add(key)
        now = time.monotonic(); self.first.setdefault(kind, now - self.t0)
        st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        self.rx[kind].append((now, st)); self.skew.setdefault(kind, []).append(time.time() - st)
        img = arr(m)
        if kind == "depth": self.valid.append(float((img > 0).sum() / img.size))
        else: self.bright.append(float(img.mean()))
        self.last[kind] = (m.encoding, m.width, m.height, m.header.frame_id, img)
    def inf(self, kind, m):
        self.info[kind] = {"width": m.width, "height": m.height, "frame_id": m.header.frame_id, "k": list(m.k), "d": list(m.d), "distortion_model": m.distortion_model}

def rate(v):
    if len(v) < 3: return None
    a = [x[0] for x in v]; s = [x[1] for x in v]
    return {"n": len(v), "arrival_hz": (len(v) - 1) / (a[-1] - a[0]), "stamp_hz": (len(v) - 1) / (s[-1] - s[0]) if s[-1] > s[0] else None}

def main():
    out = Path(sys.argv[1]); sec = float(sys.argv[2]); ns = sys.argv[3] if len(sys.argv) > 3 else "camera"
    out.mkdir(parents=True, exist_ok=True); rclpy.init(); node = Probe(ns); end = time.monotonic() + sec
    while time.monotonic() < end: rclpy.spin_once(node, timeout_sec=0.1)
    res = {"robot_commands": 0, "joint_read": 0, "seconds": sec, "namespace": ns, "first_frame_sec": node.first,
           "rate": {k: rate(v) for k, v in node.rx.items()},
           "host_minus_stamp_sec": {k: {"min": min(v), "max": max(v), "mean": sum(v)/len(v)} for k, v in node.skew.items() if v},
           "depth_valid_ratio_per_frame": node.valid, "color_mean_brightness_per_frame": node.bright, "info": node.info}
    for kind, (enc, w, h, fid, img) in node.last.items():
        res[f"{kind}_image"] = {"encoding": enc, "width": w, "height": h, "frame_id": fid}
        write_png(out / f"{kind}.png", img)
        if kind == "depth":
            nz = img[img > 0]
            res["depth_stats_mm"] = {"valid_ratio": float(nz.size / img.size), "min": int(nz.min()) if nz.size else None,
                "p05": float(np.percentile(nz, 5)) if nz.size else None, "median": float(np.median(nz)) if nz.size else None,
                "p95": float(np.percentile(nz, 95)) if nz.size else None, "max": int(nz.max()) if nz.size else None}
            np.save(out / "depth_mm.npy", img)
    for kind, i in node.info.items():
        if i["k"][0]: i["fov_deg_h_v"] = [math.degrees(2*math.atan(i["width"]/(2*i["k"][0]))), math.degrees(2*math.atan(i["height"]/(2*i["k"][4])))]
    (out / "probe.json").write_text(json.dumps(res, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps({k: res[k] for k in ("first_frame_sec", "rate")}, ensure_ascii=False)); rclpy.shutdown()

if __name__ == "__main__": main()
