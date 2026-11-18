"""Mac 사진 다리(g01_lift_bridge) 시험: 요청 줄 파싱, A 관문(침묵 금지), 실패·중복, 러너 FilePhotoChannel과의 종단 연결(가짜 snap·push)."""
import json, os, sys, tempfile, threading, time, unittest
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
import g01_lift_bridge as B, g01_lift_hook as LH, g01_lift_params as LP

REF = {"found": True, "center": [673.0, 267.0], "side_px": 29.3, "yaw_deg": -178.8}
PARAMS = dict(LP.PARAMS)


class Rec:
    def __init__(self, detect=REF, started=1000.0, snap_exc=None, status_exc=None, push_exc=None, delay=0.0):
        self.pushed = []; self.detect = detect; self.started = started; self.snap_exc = snap_exc; self.status_exc = status_exc; self.push_exc = push_exc; self.delay = delay
    def snap(self, name):
        time.sleep(self.delay)
        if self.snap_exc: raise self.snap_exc
        return dict(self.detect)
    def status(self):
        time.sleep(self.delay)
        if self.status_exc: raise self.status_exc
        return self.started
    def push(self, name, res):
        if self.push_exc: raise self.push_exc
        self.pushed.append((name, res))


def bridge(rec, **kw): return B.Bridge({"detect": REF, "started_epoch": rec.started}, rec.snap, rec.status, rec.push, P=PARAMS, say=lambda s: None, **kw)


class TestBridge(unittest.TestCase):
    def test_parse(self):
        self.assertEqual(B.parse_request("  [사진 요청 A id=run-A-123]"), ("A", "run-A-123"))
        self.assertIsNone(B.parse_request("[이동 1/2] J2")); self.assertIsNone(B.parse_request("[사진 요청 D id=x]"))

    def test_a_happy_path_echoes_token_and_epoch(self):
        r = Rec(); res = bridge(r).handle("[사진 요청 A id=tok1]")
        self.assertEqual(r.pushed[0][0], "A"); self.assertEqual(res["request_id"], "tok1"); self.assertEqual(res["started_epoch"], 1000.0); self.assertTrue(res["gate"]["go"])

    def test_gate_exception_is_written_not_silent(self):
        moved = dict(REF, center=[676.0, 267.0])                               # 불감대 밖(3 px) + G2 입력(d_x) 미확정 → ParamsNotFinal
        p = dict(PARAMS, g2_deadzone_metric="x"); r = Rec(detect=moved)
        res = B.Bridge({"detect": REF}, r.snap, r.status, r.push, P=p, say=lambda s: None).handle("[사진 요청 A id=t]")
        self.assertFalse(res["gate"]["go"]); self.assertIn("ParamsNotFinal", res["gate"]["reason"]); self.assertEqual(len(r.pushed), 1)   # 결과는 반드시 전달된다

    def test_snap_failure_still_pushes_with_reason(self):
        r = Rec(snap_exc=RuntimeError("프레임 없음")); res = bridge(r).handle("[사진 요청 A id=t]")
        self.assertIsNone(res["detect"]); self.assertFalse(res["gate"]["go"]); self.assertIn("프레임 없음", res["error_snap"]); self.assertEqual(len(r.pushed), 1)

    def test_status_failure_gives_none_epoch_so_runner_rejects_same_start(self):
        import g01_lift_logic as L
        r = Rec(status_exc=RuntimeError("ssh 실패")); res = bridge(r).handle("[사진 요청 B id=t]")
        self.assertIsNone(res["started_epoch"]); self.assertFalse(L.same_start([1000.0, res["started_epoch"]])); self.assertNotIn("gate", res)

    def test_duplicate_request_ignored_and_push_failure_not_cached(self):
        r = Rec(); b = bridge(r); b.handle("[사진 요청 B id=t1]"); self.assertIsNone(b.handle("[사진 요청 B id=t1]")); self.assertEqual(len(r.pushed), 1)
        r2 = Rec(push_exc=OSError("scp 실패")); b2 = bridge(r2); self.assertIsNone(b2.handle("[사진 요청 C id=t]")); self.assertIsNone(b2.done["C"])

    def test_snap_and_status_run_concurrently(self):
        r = Rec(delay=0.3); t0 = time.monotonic(); bridge(r).handle("[사진 요청 B id=t]"); self.assertLess(time.monotonic() - t0, 0.55)

    def test_make_ref(self):
        ref = B.make_ref("rid", REF, 1000.0, 1800, 1042.5)
        self.assertEqual((ref["run_id"], ref["elapsed_at_ref_s"], ref["maxsec"], ref["started_epoch"]), ("rid", 42.5, 1800, 1000.0))

    def test_end_to_end_with_runner_file_channel(self):
        with tempfile.TemporaryDirectory() as d:
            lines = []; ch = LH.FilePhotoChannel(lines.append, folder=d)
            r = Rec()
            def push(n, res):
                p = os.path.join(d, "lift_photo_%s.json" % n)
                with open(p + ".tmp", "w") as f: json.dump(res, f)
                os.replace(p + ".tmp", p)
            r.push = push
            b = bridge(r); out = {}
            def runner():
                tok = ch.request("A", "rid"); out["res"] = ch.wait("A", 3.0, tok); out["tok"] = tok
            t = threading.Thread(target=runner); t.start()
            while not lines: time.sleep(0.01)
            b.handle(lines[0]); t.join(5)
            self.assertEqual(out["res"]["request_id"], out["tok"]); self.assertTrue(out["res"]["gate"]["go"])


if __name__ == "__main__": unittest.main()
