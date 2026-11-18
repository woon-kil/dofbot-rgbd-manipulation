"""R8(g01_lift_r8) 시험: 기준별 판정표(E3 사진 C 경우 포함), 이미지 해시 결속, 사진 다리 로그 대조, 판정 기록 수 정의, 실제 가짜 실행 결과로 필드명 확인. venv 전용(cv2)."""
import copy, hashlib, json, os, shutil, sys, tempfile, unittest
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / "scripts"), str(ROOT / "tests")]
try:
    import cv2, numpy as np, g01_lift_r8 as R8, g01_lift_logic as L, pa_detect
except ImportError: R8 = None
SRC = ROOT / "data" / "g04" / "pa"; P3 = "pa_261118-090521_"
TS = "T1"


def sha(p): return hashlib.sha256(open(p, "rb").read()).hexdigest()


@unittest.skipUnless(R8 is not None and (SRC / (P3 + "snap01.color.png")).exists(), "cv2 또는 원 사진 없음")
class R8Test(unittest.TestCase):
    def setUp(self):
        self.root = Path(tempfile.mkdtemp()); pa = self.root / "data" / "g04" / "pa"; pa.mkdir(parents=True)
        self.names = {"ref": "snap01", "A": "snap02", "B": "snap10", "C": "snap11"}
        for n, s in self.names.items(): shutil.copy(SRC / (P3 + s + ".color.png"), pa / ("lift_%s_%s.color.png" % (TS, n)))
        det = lambda n: pa_detect.detect(cv2.imread(str(pa / ("lift_%s_%s.color.png" % (TS, n)))))
        self.det = det; self.pa = pa
        self.ref = {"run_id": TS, "started_epoch": 1.5, "detect": det("ref"), "image_sha256": sha(pa / ("lift_%s_ref.color.png" % TS))}
        ph = lambda n, rid, ep=1.5: {"request_id": rid, "started_epoch": ep, "detect": det(n), "image_sha256": sha(pa / ("lift_%s_%s.color.png" % (TS, n)))}
        self.lift = {"photos": {"A": ph("A", "a"), "B": ph("B", "b"), "C": ph("C", "c")}, "photo_times": {"A": {"request_id": "a"}, "B": {"request_id": "b"}, "C": {"request_id": "c"}}, "hold_done": True, "photo_c_done": True,
                     "j6_after_close": 146, "j6_after_lift": 146, "moves": [{"step": "lift1"}, {"step": "lift2"}, {"step": "lift3"}], "clock_log": [["photo_c", 20.0, 3.0, True]], "second_j2": {"target": 77}}
        self.r2 = {"close_steps": [{"target": 150, "stall": True}], "stall_readback": 146, "stall_command": 150, "lift": self.lift}

    def tearDown(self): shutil.rmtree(self.root, ignore_errors=True)

    def result(self, r2=None): return {"mode": "real", "j6_adapter_logs": {"contact": [["write", 145, 1500, "real"], ["write", 150, 1500, "real"]]}, "poses": [{"pose_id": 3104, "run2": r2 or self.r2}]}
    def judge(self, r2=None, bridge=None, ref=True): return R8.judge(self.result(r2), str(self.root), TS, self.ref if ref else None, bridge, detect_fn=pa_detect.detect)

    def test_happy_plumbing_static_frames_judged_but_e1_fails(self):
        j = self.judge(); v = j["verdicts"]
        self.assertEqual((v["E2"], v["E3"], v["Q3"]), ("합격", "합격", "합격")); self.assertEqual(v["E1"], "불합격")                      # 정지 장면이라 위 이동 없음 = E1 불합격(기대)
        self.assertTrue(j["judged_record"]); self.assertFalse(j["success"]); self.assertEqual(j["state"], "들어올림 완료"); self.assertIn("독립 증거가 아니다", j["criteria"]["E2"]["note"])
        self.assertEqual(j["criteria"]["E2"]["minus_146"], 0); self.assertTrue(j["r8_code_commit"]); self.assertIn("E1'", " ".join(j["notes"]))

    def test_tampered_image_hash_voids_all_photo_criteria(self):
        p = self.pa / ("lift_%s_B.color.png" % TS); shutil.copy(SRC / (P3 + "snap20.color.png"), p)                                      # 다른 사진으로 교체(정지 장면은 검출값이 같을 수 있다 → 해시로만 잡힌다)
        j = self.judge(); self.assertEqual(j["verdicts"]["E1"], "판정 없음"); self.assertEqual(j["verdicts"]["Q3"], "판정 없음"); self.assertEqual(j["verdicts"]["E3"], "판정 불가")
        self.assertIn("해시 불일치", j["criteria"]["E1"]["reason"]); self.assertFalse(j["judged_record"])

    def test_missing_hash_in_result_is_not_usable(self):
        del self.lift["photos"]["A"]["image_sha256"]; j = self.judge(); self.assertEqual(j["verdicts"]["E1"], "판정 없음"); self.assertIn("image_sha256 없음", j["criteria"]["E1"]["reason"])

    def test_e3_photo_c_cases(self):
        # ① C 요청 안 함 + 시계 거부 + 유지 완료 → P5 J6 회독판만(합격)
        l = self.lift; l["photos"]["C"] = None; l["photo_times"].pop("C"); l["photo_c_done"] = False; l["clock_log"] = [["photo_c", 25.0, 3.0, False]]
        j = self.judge(); self.assertEqual(j["verdicts"]["E3"], "합격"); self.assertIn("①", j["criteria"]["E3"]["case"])
        l["j6_after_lift"] = 149; self.assertEqual(self.judge()["verdicts"]["E3"], "불합격")                                            # J6 회독판이 판정(±1° 초과)
        l["j6_after_lift"] = 146
        # 미요청인데 시계 거부 기록이 없다 → 판정 불가
        l["clock_log"] = [["photo_c", 20.0, 3.0, True]]; self.assertEqual(self.judge()["verdicts"]["E3"], "판정 불가")
        # ② 요청했으나 결과 없음 → 판정 불가(자체 결정)
        l["photo_times"]["C"] = {"request_id": "c"}; j = self.judge(); self.assertEqual(j["verdicts"]["E3"], "판정 불가"); self.assertIn("②", j["criteria"]["E3"]["case"])
        # ③ C 받았으나 기동 불일치 → 판정 불가
        l["photos"]["C"] = {"request_id": "c", "started_epoch": 9.9, "detect": self.det("C"), "image_sha256": sha(self.pa / ("lift_%s_C.color.png" % TS))}; j = self.judge(); self.assertEqual(j["verdicts"]["E3"], "판정 불가"); self.assertIn("③", j["criteria"]["E3"]["case"])
        # 유지 생략이면 C 경우와 무관하게 판정 불가(E-2 원래 규칙)
        l["hold_done"] = False; l["photos"]["C"] = None; l["photo_times"].pop("C"); l["clock_log"] = [["hold", 25.0, 3.0, False]]; j = self.judge(); self.assertEqual(j["verdicts"]["E3"], "판정 불가"); self.assertIn("유지", j["criteria"]["E3"]["rule"])

    def test_same_start_mismatch_blocks_e1_q3(self):
        self.lift["photos"]["B"]["started_epoch"] = 9.9; j = self.judge(); self.assertEqual(j["verdicts"]["E1"], "판정 없음"); self.assertEqual(j["verdicts"]["Q3"], "판정 없음"); self.assertIn("R5", j["criteria"]["E1"]["reason"])

    def test_bridge_log_disagreement_and_absence(self):
        ok = {n: {"request_id": self.lift["photos"][n]["request_id"], "started_epoch": 1.5} for n in "ABC"}
        self.assertEqual(self.judge(bridge=ok)["verdicts"]["E1"], "불합격"); self.assertIn("대조함", " ".join(self.judge(bridge=ok)["notes"]))
        bad = copy.deepcopy(ok); bad["A"]["request_id"] = "x"; j = self.judge(bridge=bad); self.assertEqual(j["verdicts"]["E1"], "판정 없음"); self.assertIn("다리 로그", j["criteria"]["E1"]["reason"])
        self.assertIn("없음", " ".join(self.judge(bridge=None)["notes"]))                                                                # 로그 없음은 계산을 막지 않는다

    def test_no_lift_and_partial_lift_are_labelled_not_scored(self):
        self.lift["moves"] = []; self.lift["code"] = "G2"; j = self.judge(); self.assertEqual(j["verdicts"]["E1"], "판정 불가(들어올림 없음)"); self.assertFalse(j["judged_record"]); self.assertFalse(j["success"])
        self.lift["moves"] = [{"step": "lift1"}]; self.lift["code"] = "SAG_LATE"; j = self.judge(); self.assertEqual(j["verdicts"]["Q3"], "판정 불가(들어올림 중단)"); self.assertEqual(j["state"], "들어올림 중단(첫 이동 뒤 열림)")

    def test_post_return_record_is_record_only(self):
        self.assertFalse(self.judge()["post_return_record"]["available"])                                                               # 복귀 뒤 스냅 없음
        (self.pa / ("lift_%s_post.detect.json" % TS)).write_text(json.dumps({"found": True, "center": [675.0, 268.0], "side_px": 29.0, "yaw_deg": -170.0}))
        j = self.judge(); pr = j["post_return_record"]; self.assertTrue(pr["available"]); self.assertEqual(j["verdicts"], self.judge()["verdicts"]); self.assertIn("yaw_delta_deg", pr); self.assertIn("기록 전용", pr["note"])
        self.assertEqual(j["verdicts"]["E2"], "합격")                                                                                       # 판정에 영향이 없다

    def test_e2_rules(self):
        for rb, cmd, exp in ((146, 150, "합격"), (147, 150, "합격"), (148, 150, "불합격"), (146, 145, "불합격"), (None, 150, "판정 불가")):
            self.r2.update(stall_readback=rb, stall_command=cmd); self.assertEqual(self.judge()["criteria"]["E2"]["verdict"], exp, (rb, cmd))

    def test_weak_signal_label_counts_separately(self):
        # E1 불합격 + 처짐 cap → 라벨. 판정 기록 수와 라벨은 따로 보고한다.
        self.lift["sag"] = {"late": {"cap": 76, "plan_max": 77}}; j = self.judge(); self.assertTrue(j["weak_signal_label"]); self.assertTrue(j["judged_record"]); self.assertTrue(j["criteria"]["E1"]["signal_shortfall_possible"])

    def test_q3_registered_box_has_no_real_positive_control_but_synthetic_occlusion_is_caught(self):
        self.assertEqual(self.judge()["verdicts"]["Q3"], "합격")                                                                           # 정지 장면(등록 영역)은 합격
        p = self.pa / ("lift_%s_B.color.png" % TS); im = cv2.imread(str(p)); im[10:90, 820:880] = 255; cv2.imwrite(str(p), im)             # 등록 영역 (800,0,900,100) 안에 합성 가림(실사진 양성 대조는 이 영역에 없다)
        self.lift["photos"]["B"]["image_sha256"] = sha(p); self.lift["photos"]["B"]["detect"] = pa_detect.detect(cv2.imread(str(p)))
        j = self.judge(); self.assertEqual(j["verdicts"]["Q3"], "판정 불가"); self.assertIn("ZNCC", j["criteria"]["Q3"]["reason"])


try:
    import vclock, g01_p2k_test as R, g01_s42_plan_lift as PL, test_lift_hook as TH
except Exception: vclock = None


@unittest.skipIf(R8 is None or vclock is None, "의존성 없음")
class R8RealFakeRunTest(unittest.TestCase):
    """훅이 실제로 쓰는 필드명으로 R8을 돌린다(mode만 real): photos·photo_times·j6_after_close/lift·hold_done·clock_log·stall 필드를 읽는지."""
    def test_reads_real_hook_record(self):
        TH.LiftHookTest.setUpClass(); pa = None
        root = Path(tempfile.mkdtemp()); pa = root / "data" / "g04" / "pa"; pa.mkdir(parents=True)
        try:
            for n, s in {"ref": "snap01", "A": "snap02", "B": "snap10", "C": "snap11"}.items(): shutil.copy(SRC / (P3 + s + ".color.png"), pa / ("lift_%s_%s.color.png" % (TS, n)))
            det = lambda n: pa_detect.detect(cv2.imread(str(pa / ("lift_%s_%s.color.png" % (TS, n)))))
            shaf = lambda n: sha(pa / ("lift_%s_%s.color.png" % (TS, n)))
            ref = dict(TH.REF, detect=det("ref"), image_sha256=shaf("ref"))
            ph = {n: {"started_epoch": 1.5, "detect": det(n), "image_sha256": shaf(n)} for n in "ABC"}; ph["A"]["gate"] = {"go": True, "tilt_check": {"ok": True, "margin": 0.21, "plan_max": 77}}
            t = TH.LiftHookTest("test_happy_path_lifts_drops_and_returns_with_j6_closed"); r = t.run33(TH.FakeChannel(ref, ph))
            r = copy.deepcopy(r); r["mode"] = "real"; j = R8.judge(r, str(root), TS, ref, None, detect_fn=pa_detect.detect)
            self.assertEqual(j["state"], "들어올림 완료"); self.assertEqual(j["verdicts"]["E2"], "합격"); self.assertEqual(j["verdicts"]["E3"], "합격"); self.assertEqual(j["verdicts"]["Q3"], "합격")
            self.assertEqual(j["verdicts"]["E1"], "불합격"); self.assertTrue(j["judged_record"]); self.assertTrue(j["classification"]["counted"]); self.assertEqual(j["classification"]["outcome"], "lifted")
        finally: shutil.rmtree(root, ignore_errors=True); TH.R.LIFT_CHANNEL = None


if __name__ == "__main__": unittest.main()
