"""Mac 사진 다리(g01_lift_bridge) 시험: 요청 줄 파싱, A 관문(침묵 금지), 실패·중복, 러너 FilePhotoChannel과의 종단 연결(가짜 snap·push)."""
import json, os, subprocess, sys, tempfile, threading, time, unittest
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
import g01_lift_bridge as B, g01_lift_hook as LH, g01_lift_params as LP

REF = {"found": True, "center": [673.0, 267.0], "side_px": 29.3, "yaw_deg": -178.8}
try:
    import cv2, numpy; HAS_CV = True
except ImportError: HAS_CV = False
need_cv = unittest.skipUnless(HAS_CV, "cv2·numpy 필요(venv 전용: 관문의 Δx_block 환산)")
PARAMS = dict(LP.PARAMS); P_UNSET = LP.UNSET


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

    @need_cv
    def test_a_happy_path_echoes_token_and_epoch(self):
        r = Rec(); res = bridge(r).handle("[사진 요청 A id=tok1]")
        self.assertEqual(r.pushed[0][0], "A"); self.assertEqual(res["request_id"], "tok1"); self.assertEqual(res["started_epoch"], 1000.0); self.assertTrue(res["gate"]["go"])

    @need_cv
    def test_gate_exception_is_written_not_silent(self):
        moved = dict(REF, center=[676.0, 267.0]); r = Rec(detect=moved)
        p = dict(PARAMS, g2_deadzone_metric=P_UNSET)                           # 판정 지표가 UNSET이면 관문 계산이 ParamsNotFinal
        res = B.Bridge({"detect": REF}, r.snap, r.status, r.push, P=p, say=lambda s: None).handle("[사진 요청 A id=t]")
        self.assertFalse(res["gate"]["go"]); self.assertIn("ParamsNotFinal", res["gate"]["reason"]); self.assertEqual(len(r.pushed), 1)   # 결과는 반드시 전달된다

    @need_cv
    def test_gate_records_dx_block_and_passes_normal_grip_shift(self):
        r = Rec(detect=dict(REF, center=[675.0, 267.0])); res = bridge(r).handle("[사진 요청 A id=t]")      # 정상 물림 규모의 x 이동(약 −1.6~−2 mm급): 불감대 밖이지만 3 mm 안
        self.assertTrue(res["gate"]["go"], res["gate"]); self.assertIsNotNone(res["gate"]["dx_block_mm"]); self.assertTrue(res["gate"]["dx_depth_assumed"])
        res = bridge(Rec(detect=dict(REF, center=[679.0, 267.0]))).handle("[사진 요청 A id=t2]")           # 약 4.8 px ≈ 3.6 mm > 3 → no-go, 원값은 남는다
        self.assertFalse(res["gate"]["go"]); self.assertIn("dx_block_mm", res["gate"])

    def test_result_carries_image_sha256_and_failure_gives_none(self):
        r = Rec(); res = B.Bridge({"detect": REF}, r.snap, r.status, r.push, P=PARAMS, say=lambda s: None, imghash=lambda n: "sha-" + n).handle("[사진 요청 B id=t]")
        self.assertEqual(res["image_sha256"], "sha-B")
        def bad(n): raise OSError("없음")
        res = B.Bridge({"detect": REF}, r.snap, r.status, r.push, P=PARAMS, say=lambda s: None, imghash=bad).handle("[사진 요청 C id=t2]"); self.assertIsNone(res["image_sha256"])   # 해시 실패는 R8이 판정 없음으로 처리
        self.assertEqual(B.make_ref("rid", REF, 1000.0, 1800, 1042.5, "abc")["image_sha256"], "abc")

    def test_pump_survives_a_line_that_raises_and_keeps_reading(self):
        seen = []
        class Br:
            def handle(s, line):
                if "boom" in line: raise RuntimeError("처리 오류")
                seen.append(line)
        out = []; B.pump(Br(), ["a\n", "boom\n", "c\n"], say=out.append); self.assertEqual(seen, ["a\n", "c\n"]); self.assertTrue(any("줄 처리 오류" in o for o in out))
        B.pump(Br(), ["boom\n", "d\n"], say=lambda s: (_ for _ in ()).throw(OSError("죽은 터미널"))); # say마저 실패해도 계속(여기서는 예외가 새지 않아야 한다)

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

    @need_cv
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


PLAN33 = {"lift": {"j2_max_target": 77, "j2_first_target": 74}}
Q0 = [86.3, 68.32, 11.37, 31.42]


@unittest.skipUnless((ROOT / ".venv").exists(), "venv 필요")
class TestTiltRecheck(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        try: import cv2, numpy, scipy
        except ImportError: raise unittest.SkipTest("venv 전용(FK 모듈)")
        import g01_s42_plan_lift as PL; cwd = os.getcwd(); cls.mods = PL._load_modules(); os.chdir(cwd)

    def test_parse_q_and_request_line_with_q(self):
        self.assertEqual(B.parse_q("[사진 요청 A id=t q=86.3,68.32,11.37,31.42]"), Q0); self.assertIsNone(B.parse_q("[사진 요청 A id=t]")); self.assertIsNone(B.parse_q("[사진 요청 A id=t q=1,2,3]"))
        self.assertEqual(B.parse_request("[사진 요청 A id=tok q=86.3,68.32,11.37,31.42]"), ("A", "tok"))

    def test_recorded_contact_pose_keeps_77(self):
        r = B.tilt_recheck(PLAN33, Q0, self.mods); self.assertTrue(r["ok"]); self.assertEqual(r["recomputed_max"], 77); self.assertAlmostEqual(r["peak_tilt_deg"], 29.79, places=2)
        self.assertAlmostEqual(r["margin"], 0.21, places=2); self.assertEqual(r["cap_at_contact"], 77)
        for k, v in r["slopes"].items(): self.assertAlmostEqual(v, 1.0, delta=0.01, msg=k)                      # 모델 자체 점검(t2·J3·J4 민감도 1)
        for dq in ([0, .08, 0, 0], [0, 0, .08, 0], [0, 0, 0, .08], [0, 0, -.08, 0]):                          # 측정된 접촉 원시 변동(0.08°)은 77을 유지
            self.assertTrue(B.tilt_recheck(PLAN33, [a + b for a, b in zip(Q0, dq)], self.mods)["ok"], dq)

    def test_run3_and_run4_recorded_contacts_pass(self):
        for q in ([86.3, 68.318, 11.455, 31.418], [86.3, 68.318, 11.373, 31.418]):                            # Run 3·4 기록의 접촉 원시 q(J1은 기록 없음 → Q0의 86.3)
            r = B.tilt_recheck(PLAN33, q, self.mods); self.assertTrue(r["ok"], q); self.assertGreaterEqual(r["cap_at_contact"], 77 if q[2] < 11.4 else 76)

    def test_pose_needing_lower_target_is_capped_not_refused(self):
        r = B.tilt_recheck(PLAN33, [86.3, 68.32, 11.62, 31.42], self.mods)                                    # J3 +0.25°: margin 음수 → cap 76이지만 E1 선(75) 위라 거부하지 않는다(사용자 결정: 거부가 아니라 cap)
        self.assertTrue(r["ok"]); self.assertLess(r["margin"], 0); self.assertEqual(r["cap_at_contact"], 76)
        r = B.tilt_recheck(PLAN33, [86.3, 68.32, 12.9, 31.42], self.mods)                                     # J3 +1.5°: cap 75 이하 → 거부
        self.assertFalse(r["ok"]); self.assertIn("E1 선", r["reason"])

    def test_missing_q_or_plan_is_fail_closed(self):
        self.assertFalse(B.tilt_recheck(PLAN33, None, self.mods)["ok"]); self.assertFalse(B.tilt_recheck({}, Q0, self.mods)["ok"])

    @need_cv
    def test_bridge_gate_goes_no_go_when_tilt_fails_even_if_s4_passes(self):
        r = Rec(); b = B.Bridge({"detect": REF}, r.snap, r.status, r.push, P=PARAMS, say=lambda s: None, tilt=lambda q: B.tilt_recheck(PLAN33, q, self.mods))
        res = b.handle("[사진 요청 A id=t q=86.3,68.32,12.9,31.42]"); self.assertFalse(res["gate"]["go"]); self.assertFalse(res["gate"]["tilt_check"]["ok"]); self.assertIn("P10 tilt 재확인 실패", res["gate"]["reason"])
        r2 = Rec(); res = B.Bridge({"detect": REF}, r2.snap, r2.status, r2.push, P=PARAMS, say=lambda s: None, tilt=lambda q: B.tilt_recheck(PLAN33, q, self.mods)).handle("[사진 요청 A id=t2 q=86.3,68.32,11.37,31.42]")
        self.assertTrue(res["gate"]["go"]); self.assertTrue(res["gate"]["tilt_check"]["ok"])                    # 양성 대조
        r3 = Rec(); res = B.Bridge({"detect": REF}, r3.snap, r3.status, r3.push, P=PARAMS, say=lambda s: None, tilt=lambda q: B.tilt_recheck(PLAN33, q, self.mods)).handle("[사진 요청 A id=t3]")
        self.assertFalse(res["gate"]["go"]); self.assertIn("접촉 원시 q가 요청에 없다", res["gate"]["tilt_check"]["reason"])    # q 없음 → 거부


class TestAlign(unittest.TestCase):
    def mk(self, align): return B.Bridge({"detect": REF}, Rec().snap, Rec().status, Rec().push, P=PARAMS, say=self.out.append, align=align)
    def setUp(self): self.out = []

    def test_align_line_runs_once_prints_and_does_not_push(self):
        r = Rec(); calls = []; b = B.Bridge({"detect": REF}, r.snap, r.status, r.push, P=PARAMS, say=self.out.append, align=lambda: calls.append(1) or "보고 본문")
        b.handle("[정렬 확인] Mac 화면의 정렬 보고를 읽으세요."); b.handle("[정렬 확인] 다시"); self.assertEqual(calls, [1]); self.assertIn("보고 본문", self.out); self.assertEqual(r.pushed, [])

    def test_align_failure_prints_N_guidance(self):
        def bad(): raise RuntimeError("스냅 실패")
        self.mk(bad).handle("[정렬 확인] x"); self.assertTrue(any("러너 질문에 N" in o and "스냅 실패" in o for o in self.out))

    def test_no_align_callable_is_ignored(self):
        b = B.Bridge({"detect": REF}, Rec().snap, Rec().status, Rec().push, P=PARAMS, say=self.out.append); self.assertIsNone(b.handle("[정렬 확인] x")); self.assertEqual(self.out, [])

    def test_report_timeout_and_nonzero_become_errors(self):
        def to(*a, **k): raise subprocess.TimeoutExpired("x", 1)
        with self.assertRaises(subprocess.TimeoutExpired): B.align_report("/nonexistent", "a", "b", run=to)
        class R: returncode = 1; stdout = ""; stderr = "boom"
        with self.assertRaises(RuntimeError): B.align_report("/nonexistent", "a", "b", run=lambda *a, **k: R())
        self.assertLess(B.ALIGN_TIMEOUT_S, 30)

    @unittest.skipUnless((ROOT / "data" / "g04" / "d1_261117-143127_31010.png").exists() and (ROOT / ".venv" / "bin" / "python").exists(), "원 사진 또는 venv 없음")
    def test_set33_without_ref_file_reports_NG_not_run1_fallback(self):
        """저장소 상태(재등록 여부)와 무관: 기준 파일이 없는 임시 루트에서 정렬 보고를 실행한다(스크립트·src·data는 심볼릭 링크, evidence/G06은 lift_marker_ref_*만 뺀 링크)."""
        import tempfile, shutil
        tmp = Path(tempfile.mkdtemp()); (tmp / "evidence").mkdir()
        try:
            for n in ("scripts", "src", "data"): os.symlink(ROOT / n, tmp / n)
            (tmp / "evidence" / "G06").mkdir()
            for f in (ROOT / "evidence" / "G06").iterdir():
                if not f.name.startswith("lift_marker_ref_"): os.symlink(f, tmp / "evidence" / "G06" / f.name)
            txt = B.align_report(str(tmp), str(ROOT / "data/g04/d1_261117-143127_31010.png"), str(ROOT / "data/g04/p2kref_261117-143127.png"), python=str(ROOT / ".venv/bin/python"))
        finally: shutil.rmtree(tmp, ignore_errors=True)
        self.assertIn("정렬 NG", txt); self.assertIn("마커 기준 파일이 0개", txt); self.assertNotIn("d_x =", txt); self.assertNotIn("678.2", txt)   # Run 1 위치·87.4로 계산한 보고가 아니다

    @unittest.skipUnless((ROOT / "data" / "g04" / "d1_261117-143127_31010.png").exists() and (ROOT / ".venv" / "bin" / "python").exists(), "원 사진 또는 venv 없음")
    def test_positive_control_reproduces_run3_dx(self):
        txt = B.align_report(str(ROOT), str(ROOT / "data/g04/d1_261117-143127_31010.png"), str(ROOT / "data/g04/p2kref_261117-143127.png"), python=str(ROOT / ".venv/bin/python"), set_arg="31")
        self.assertIn("d_x = +1.49 mm", txt); self.assertIn("36 s 도구와 동등성 미검증", txt); self.assertIn("|d_x| ≤ 3 mm", txt)   # Run 3 31010 정렬 사진의 기록 d_x와 일치(참고용)


@unittest.skipUnless((ROOT / "data" / "g04" / "p2kpost_261117-153705.png").exists() and (ROOT / ".venv").exists(), "원 사진(gitignore) 또는 venv 없음")
class TestDxBlockSign(unittest.TestCase):
    def test_run4_sign_and_value_reproduce(self):
        try: import cv2
        except ImportError: self.skipTest("cv2 필요")
        sys.path.insert(0, str(ROOT / "scripts")); import g01_e0g_analyze as E
        det = lambda n: dict(E.detect(cv2.imread(str(ROOT / "data" / "g04" / n))).get(7), found=True)
        v = B.dx_block_mm(det("p2kref_261117-153705.png"), det("p2kpost_261117-153705.png"))
        self.assertAlmostEqual(v, -1.93, delta=0.01)        # Run 4 보고서의 관측값(음수 = 프로젝트 x 감소)


if __name__ == "__main__": unittest.main()
