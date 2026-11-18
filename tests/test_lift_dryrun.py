"""사진 경로 dry run 도구 시험: 운반 줄이 래퍼와 같은 문자열, 스텁 일정, 스텁↔다리 로컬 루프(실제 FilePhotoChannel·Bridge), 판독 규칙 분기."""
import json, os, re, subprocess, sys, tempfile, threading, time, unittest
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
import g01_lift_dryrun_report as RP, g01_lift_dryrun_stub as ST, g01_lift_bridge as B

try:
    import cv2, numpy; HAS_CV = True
except ImportError: HAS_CV = False

W = (ROOT / "scripts" / "g01_lift_run.sh").read_text(encoding="utf-8"); D = (ROOT / "scripts" / "g01_lift_dryrun.sh").read_text(encoding="utf-8")


def line(text, start):
    ls = [l.strip() for l in text.splitlines() if l.strip().startswith(start)]; assert len(ls) == 1, (start, len(ls)); return ls[0]


class TestStringIdentity(unittest.TestCase):
    def test_syntax(self): self.assertEqual(subprocess.run(["bash", "-n", str(ROOT / "scripts" / "g01_lift_dryrun.sh")]).returncode, 0)

    def test_transport_lines_identical_to_wrapper(self):
        for start in ("PA_MAXSEC=1800", "pa_ssh()", "pa_stop()", "trap pa_stop EXIT", "PA_STARTED=1; pa_ssh start", 'bash "${ROOT}/scripts/pa_snap_mac.sh" "lift_${TS}_ref"', 'REF="${ROOT}',
                      'scp -q -o BatchMode=yes "${REF}"', "beep_watch()"):
            self.assertEqual(line(D, start), line(W, start), start)
        mk = lambda t: t[t.index('"${VPY}" -I -c "import sys,json,re,subprocess,time; sys.path.insert(0'):t.index("|| { echo \"P-A 기준 결과 생성 실패")]
        self.assertEqual(mk(D), mk(W))                                                                                  # make_ref(해시 포함) 블록
        self.assertEqual(line(D, 'ssh -t -o ConnectTimeout=8 "${H}" "${STUBCMD}"').replace("${STUBCMD}", "${RUNNER}").split(" || echo")[0], line(W, 'ssh -t -o ConnectTimeout=8 "${H}" "${RUNNER}" | tee >(beep_watch) >(').split(" || echo")[0])   # 같은 tee·브리지 CLI 줄
        self.assertIn('--plan "${PLAN}"', D)

    def test_masters_cleaned_for_all_three_control_paths_before_p_a_start(self):
        for cp in ("/tmp/pa-ssh-%C p620", "/tmp/lift-ssh-%C p620", "/tmp/lift-ssh-%C ${H}"): self.assertIn(cp, D)
        self.assertIn("-O exit", D); self.assertIn("-O check", D)
        order = [D.index(x) for x in ('read -r -p "dry run 전 확인', "-O exit", 'grep -q "컨테이너 없음"', "PA_STARTED=1; pa_ssh start", "g01_lift_dryrun_stub.py --run-id")]
        self.assertEqual(order, sorted(order))                                                                           # 사람 확인 → 마스터 정리 → P-A 상태 확인 → 기동 → 스텁
        self.assertNotIn("Arm_Lib", D); self.assertFalse([l for l in D.splitlines() if not l.lstrip().startswith("#") and "--real" in l and "echo" not in l])   # 주석·안내 문구 밖에 --real 호출이 없다


class TestStub(unittest.TestCase):
    def test_schedule(self):
        s = ST.schedule(); self.assertEqual(len(s), 23); self.assertEqual(s[0], ("cold_first", "A", 0)); warm = [x for x in s if x[0].startswith("warm_")]; self.assertEqual(len(warm), 20)
        names = [x[1] for x in warm]; self.assertEqual(names[:5], ["A"] * 5); self.assertEqual(names[5:10], ["B"] * 5); self.assertEqual(names[10:], ["A", "B"] * 5)
        self.assertEqual([x[2] for x in s[-2:]], [130, 310]); self.assertTrue(all(x[2] >= 5 for x in warm))

    def test_no_hardware_imports(self):
        src = (ROOT / "scripts" / "g01_lift_dryrun_stub.py").read_text(encoding="utf-8"); self.assertNotIn("import Arm_Lib", src); self.assertNotIn("Arm_Device", src); self.assertNotIn(".move(", src)

    @unittest.skipUnless(HAS_CV, "cv2 필요(venv 전용: A 관문의 Δx_block 환산)")
    def test_local_loop_with_real_channel_and_bridge(self):
        """스텁(서브프로세스)과 다리(여기)를 실제 FilePhotoChannel로 연결해 23회 + 정렬 줄을 돌린다(시간만 줄인 인자). D는 Jetson 쪽 시계로 기록된다."""
        with tempfile.TemporaryDirectory() as d:
            for m in ("g01_lift_params.py", "g01_lift_logic.py", "g01_lift_hook.py"): os.symlink(ROOT / "scripts" / m, Path(d) / m)
            os.symlink(ROOT / "scripts" / "g01_lift_dryrun_stub.py", Path(d) / "stub.py")
            ref = {"run_id": "tr", "started_epoch": 1.5, "elapsed_at_ref_s": 5.0, "maxsec": 1800, "detect": {"found": True, "center": [673.0, 267.0], "side_px": 29.0, "yaw_deg": -179.0}}
            Path(d, "lift_photo_ref.json").write_text(json.dumps(ref))
            def push(name, res):
                p = Path(d) / ("lift_photo_%s.json" % name); tmp = Path(d) / ("." + p.name + ".tmp"); tmp.write_text(json.dumps(res)); os.replace(tmp, p)
            snap = lambda n: dict(ref["detect"]); aligned = []; br_hist = []
            br = B.Bridge({"detect": ref["detect"]}, snap, lambda: 1.5, push, say=lambda s: None, align=lambda: aligned.append(1) or "정렬 NG(가짜)", imghash=lambda n: "h%d" % len(br_hist))
            p = subprocess.Popen([sys.executable, "stub.py", "--run-id", "tr", "--gap", "0.02", "--idle1", "1", "--idle2", "1", "--align-wait", "0.3", "--out", "r.json"], cwd=d, stdout=subprocess.PIPE, text=True, env=dict(os.environ, PYTHONDONTWRITEBYTECODE="1"))
            for l in p.stdout: br.handle(l); br_hist[:] = br.history
            self.assertEqual(p.wait(timeout=120), 0); self.assertEqual(aligned, [1])
            stub = json.load(open(Path(d) / "r.json")); self.assertEqual(len(stub["trials"]), 23); self.assertTrue(all(t["ok"] and t["request_id_match"] and t["same_start"] and t["image_sha256"] for t in stub["trials"]))
            self.assertTrue(stub["precheck"]["ok"]); self.assertEqual(len(br.history), 23)                                     # 같은 이름 반복 요청이 중복으로 무시되지 않는다(토큰 기준)
            a = [t for t in stub["trials"] if t["name"] == "A"]; self.assertTrue(all(t["gate"] is not None for t in a))       # A에는 q 필드와 관문이 있다
            rep = RP.report(stub, {"history": br.history, "results": {"align": "x", "align_seconds": 2.0}}); self.assertTrue(rep["window_met"]); self.assertEqual(rep["warm"]["n"], 20)


class TestReport(unittest.TestCase):
    def stub(self, **over):
        tr = [{"label": "cold_first", "D_s": 1.0, "ok": True, "request_id_match": True, "same_start": True, "image_sha256": True, "detect_present": True, "sha": "x", "name": "B"}] + [{"label": "warm_%02d" % i, "D_s": 1.0, "ok": True, "request_id_match": True, "same_start": True, "image_sha256": True, "detect_present": True, "sha": "x", "name": "B"} for i in range(1, 21)] + \
             [{"label": "idle_130s", "D_s": 1.5, "ok": True, "request_id_match": True, "same_start": True, "image_sha256": True, "detect_present": True, "sha": "x", "name": "B"}, {"label": "idle_310s", "D_s": 2.0, "ok": True, "request_id_match": True, "same_start": True, "image_sha256": True, "detect_present": True, "sha": "x", "name": "B"}]
        for i, t in enumerate(tr): t["sha"] = "sha%02d" % i; t["name"] = "A" if i % 2 == 0 else "B"; t["gate"] = {"go": True} if t["name"] == "A" else None
        return dict({"trials": tr, "precheck": {"ok": True}, "python": "3.8"}, **over)
    BR = {"history": [], "results": {"align": "NG", "align_seconds": 3.0}}

    def test_window_met_and_failures_are_grade_a(self):
        self.assertTrue(RP.report(self.stub(), self.BR)["window_met"])
        s = self.stub(); s["trials"][-1]["D_s"] = 3.2; r = RP.report(s, self.BR); self.assertFalse(r["window_met"]); self.assertIn("idle_310s", r["over_cap_labels"]); self.assertIn("등급 A", r["grade"])
        s = self.stub(); s["trials"][5]["ok"] = False; s["trials"][5]["D_s"] = None; self.assertFalse(RP.report(s, self.BR)["window_met"])
        s = self.stub(); s["trials"][3]["request_id_match"] = False; self.assertFalse(RP.report(s, self.BR)["window_met"])
        s = self.stub(); s["trials"][3]["same_start"] = False; self.assertFalse(RP.report(s, self.BR)["window_met"])
        s = self.stub(); s["precheck"] = {"ok": False}; self.assertFalse(RP.report(s, self.BR)["window_met"])
        self.assertFalse(RP.report(self.stub(), None)["window_met"])                                                      # 다리 로그 없음 → 정렬 경로 판정 못 함 → A
        self.assertFalse(RP.report(self.stub(), {"history": [], "results": {"align": "NG", "align_seconds": 16.0}})["window_met"])      # 15 s 초과
        self.assertFalse(RP.report(self.stub(trials=self.stub()["trials"][:-1]), self.BR)["window_met"])                  # 시험 수 부족

    def test_snap_success_conditions_preregistered_before_data(self):
        s = self.stub(); s["trials"][4]["sha"] = s["trials"][2]["sha"]; self.assertFalse(RP.report(s, self.BR)["window_met"])                 # 같은 해시(옛 이미지 재사용)
        s = self.stub(); s["trials"][4]["detect_present"] = False; self.assertFalse(RP.report(s, self.BR)["window_met"])                       # detect 없음
        s = self.stub(); s["trials"][4]["error_snap"] = "스냅 실패"; self.assertFalse(RP.report(s, self.BR)["window_met"])
        s = self.stub(); s["trials"][6]["gate"] = {"go": False, "reason": "스냅 실패"}; r = RP.report(s, self.BR); self.assertFalse(r["window_met"]); self.assertEqual(r["bad"]["a_gate_not_go"], 1)   # A 관문 no-go
        s = self.stub(); s["trials"][3]["error_status"] = "ssh 실패"; self.assertFalse(RP.report(s, self.BR)["window_met"])

    def test_boundary_is_inclusive_at_cap(self):
        s = self.stub(); s["trials"][4]["D_s"] = 3.0; self.assertTrue(RP.report(s, self.BR)["window_met"]); s["trials"][4]["D_s"] = 3.0001; self.assertFalse(RP.report(s, self.BR)["window_met"])


class TestRealSnapNaming(unittest.TestCase):
    """pa_snap_mac.sh는 이미 있는 이름을 거부한다. 같은 이름 반복 요청(dry run)이 가짜 통과가 되지 않는지 그 동작을 흉내 낸 실행기로 확인한다."""
    def make(self, root, fail_at=()):
        pa = Path(root) / "data" / "g04" / "pa"; pa.mkdir(parents=True, exist_ok=True); calls = []
        class R:
            def __init__(s, rc, out=""): s.returncode = rc; s.stdout = out; s.stderr = ""
        def run(cmd, **k):
            name = cmd[-1]; calls.append(name)
            if (pa / (name + ".color.png")).exists(): return R(2, "이미 있음: " + name)                                       # pa_snap_mac.sh와 같은 거부
            if len(calls) in fail_at: return R(4, "실패: p620 snap")
            (pa / (name + ".color.png")).write_bytes(("img%d" % len(calls)).encode()); (pa / (name + ".detect.json")).write_text(json.dumps({"found": True, "center": [1, 2], "side_px": 3.0, "yaw_deg": 0.0})); return R(0)
        return run, calls

    def test_repeats_get_suffix_and_every_hash_is_fresh(self):
        with tempfile.TemporaryDirectory() as d:
            run, calls = self.make(d); snap = B.real_snap(d, "lift_T", run=run); h = B.real_imghash(d, snap); hashes = []
            for i in range(23): nm = "A" if i % 2 == 0 else "B"; snap(nm); hashes.append(h(nm))
            self.assertEqual(calls[0], "lift_T_A"); self.assertEqual(calls[1], "lift_T_B"); self.assertEqual(calls[2], "lift_T_A_r1"); self.assertEqual(len(set(calls)), 23)
            self.assertEqual(len(set(hashes)), 23)                                                                        # 23개 해시가 모두 다르다(매번 새 스냅)

    def test_old_behaviour_would_have_failed_21_of_23(self):
        with tempfile.TemporaryDirectory() as d:
            run, calls = self.make(d); fails = 0
            for i in range(23):                                                                                           # 접미사 없이 같은 이름을 반복하면(이전 구현) 첫 A·B 뒤로 전부 거부된다
                nm = "lift_T_" + ("A" if i % 2 == 0 else "B"); r = run(["bash", "x", nm]); fails += r.returncode != 0
            self.assertEqual(fails, 21)

    def test_failed_snap_attaches_no_hash(self):
        with tempfile.TemporaryDirectory() as d:
            run, calls = self.make(d, fail_at=(2,)); snap = B.real_snap(d, "lift_T", run=run); h = B.real_imghash(d, snap)
            snap("A"); self.assertTrue(h("A"))
            with self.assertRaises(RuntimeError): snap("A")                                                               # 두 번째 A 스냅 실패
            with self.assertRaises(RuntimeError): h("A")                                                                  # 첫 A의 옛 파일을 해시하지 않는다
            det = {"found": True, "center": [1, 2], "side_px": 3.0, "yaw_deg": 0}
            res = B.Bridge({"detect": det}, snap, lambda: 1.0, lambda n, r: None, say=lambda s: None, imghash=h).handle("[사진 요청 A id=t9]")   # 세 번째 호출(성공): 새 해시
            self.assertIsNotNone(res["image_sha256"])
            run2, _ = self.make(d + "/x", fail_at=(1,)); snap2 = B.real_snap(d + "/x", "lift_T", run=run2); h2 = B.real_imghash(d + "/x", snap2)
            res2 = B.Bridge({"detect": det}, snap2, lambda: 1.0, lambda n, r: None, say=lambda s: None, imghash=h2).handle("[사진 요청 A id=t8]")
            self.assertIsNone(res2["image_sha256"]); self.assertIsNone(res2["detect"]); self.assertIn("error_snap", res2); self.assertFalse(res2["gate"]["go"])   # 스냅 실패: 해시 없음, 관문 no-go, 사유 기록


if __name__ == "__main__": unittest.main()
