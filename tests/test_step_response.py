"""소각 응답 분석(src/planning/p1_step_response_analyze.py) 합성 데이터 시험: 방향별 정지 대역(H2)·부하 오프셋(H1)·정상을 구분하고, 움직임 없음을 센다."""
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from planning import p1_step_response_analyze as A   # noqa: E402


def synth(d_up, o_down, base=90.0, joint=2, creep=0.0):
    """J를 기준 b에서 탐침(사전 등록 순서)하는 결과를 만든다. 서보 모델: 증가 방향은 d_up만큼 모자라게 멈추고(요청이 d_up 이하이면 움직이지 않음), 감소 방향은 o_down만큼 넘어간다."""
    seq = []
    for s in (1, 2, 3, 4): seq += [base + s, base, base - s, base]
    seq += [base + 2, base, base - 2, base]
    pos = base; steps = []; ra = lambda p: {"a": {str(k): {"deg": (p if k == joint else 0.0)} for k in (1, 2, 3, 4)}, "b": {str(k): {"deg": (p + creep if k == joint else 0.0)} for k in (1, 2, 3, 4)}}
    steps.append({"n": 1, "joint": joint, "phase": "B", "target": base, "cmd_before": {str(joint): base}, "raw_after": ra(pos)})
    cmdb = base
    for n, t in enumerate(seq, 2):
        if t > pos: new = max(pos, t - d_up)
        elif t < pos: new = t - o_down
        else: new = pos
        pa = pos; pos = new
        steps.append({"n": n, "joint": joint, "phase": "T", "target": t, "cmd_before": {str(joint): cmdb}, "raw_after": {"a": {str(k): {"deg": (pos if k == joint else 0.0)} for k in (1, 2, 3, 4)}, "b": {str(k): {"deg": (pos + creep if k == joint else 0.0)} for k in (1, 2, 3, 4)}}})
        cmdb = t
    return {"steps": steps}


class StepResponseTest(unittest.TestCase):
    def test_direction_dependent_band_h2(self):          # 증가 1.5° 모자람, 감소 0.3° 넘어감 → H2, 증가 1~1.5° 요청은 움직임 없음
        o = A.analyze(synth(1.5, 0.3))["joints"]["2"]
        self.assertEqual(o["verdict"], "H2 방향별 정지 대역"); self.assertTrue(o["hypotheses"]["H2_direction_dependent_band"])
        self.assertLess(o["up"]["stop_err_mean"], -1.0); self.assertGreater(o["down"]["stop_err_mean"], -0.6)
        self.assertGreaterEqual(o["up"]["bins"][0]["n_no_move"], 1)
        self.assertEqual(o["up"]["min_effective_width_bin"], (2.5, 3.5))          # 1.5° 모자람이면 실현 비율이 0.5를 처음 넘는 구간은 2.5~3.5°
        self.assertAlmostEqual(o["common_component"], -(1.5 + 0.3) / 2, delta=0.1); self.assertAlmostEqual(o["band_halfwidth"], (1.5 - 0.3) / 2, delta=0.1)

    def test_load_offset_h1(self):                       # 양방향 모두 아래에서 멈춤(증가 1.0 모자람, 감소 0.8 넘어감)
        o = A.analyze(synth(1.0, 0.8))["joints"]["2"]
        self.assertEqual(o["verdict"], "H1 부하 오프셋")

    def test_normal_no_deviation(self):
        o = A.analyze(synth(0.0, 0.0))["joints"]["2"]; self.assertEqual(o["up"]["min_effective_width_bin"], (0.5, 1.5)); self.assertAlmostEqual(o["common_component"], 0.0, places=3)
        self.assertEqual(o["verdict"], "정상(편차 없음)"); self.assertEqual(sum(b["n_no_move"] for b in o["up"]["bins"]), 0)

    def test_repeat_pairs_and_counts(self):
        out = A.analyze(synth(1.5, 0.3)); o = out["joints"]["2"]
        self.assertEqual(out["n_probe_moves"], 20)            # 위·아래 1~4° 각 4 + 반복 4 = 20
        self.assertEqual(len(o["repeat_abs_diff"]), 4)       # (b+2,b), (b,b−2) 등 반복 쌍이 잡힌다... 실제 반복 쌍 수는 시퀀스에 따름
        self.assertAlmostEqual(o["creep_median_abs"], 0.0, places=6)

    def test_creep_recorded(self):
        o = A.analyze(synth(1.5, 0.3, creep=0.2))["joints"]["2"]; self.assertAlmostEqual(o["creep_median_abs"], 0.2, places=3)

    def test_skips_when_raw_after_missing(self):          # raw_after가 없는 스텝(접기 계단 등)은 직전 값으로 쓰지 않는다
        r = synth(1.5, 0.3); r["steps"][5].pop("raw_after"); out = A.analyze(r)
        self.assertLess(out["n_probe_moves"], 20)


if __name__ == "__main__":
    unittest.main()
