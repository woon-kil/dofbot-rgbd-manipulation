"""T1-C 분석 스크립트(p1_t1c_analyze) 합성 시험: 큰 이동 전이 판정, 크기 구간별 SD 비, 사전 등록 문구 유지."""
import sys, random
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]; sys.path.insert(0, str(ROOT / "src" / "planning"))
import p1_t1c_analyze as C

SEQ = {2: [106, 98, 118, 134, 118, 102, 132, 112, 104, 112, 120, 108, 96, 108, 120, 100, 130, 100, 120, 90],
       3: [42, 54, 38, 46, 62, 54, 66, 36, 24, 32, 62, 54, 24, 44, 32, 48, 32, 62, 42, 22]}


def synth(d_up=1.56, o_down=0.643, small_noise=0.0, large_noise=0.0, seed=3):
    """세트 19의 J2·J3 목표열을 합성 팔(M1 + 크기별 잡음)로 실행한 결과 json(steps)."""
    rng = random.Random(seed); steps = []; n = 0; pos = {1: 90.0, 2: 90.0, 3: 22.0, 4: 11.0}; cmd = {2: 90, 3: 22}
    def raw(): return {"b": {str(j): {"deg": pos[j]} for j in (1, 2, 3, 4)}, "a": {str(j): {"deg": pos[j]} for j in (1, 2, 3, 4)}}
    steps.append({"n": 0, "joint": 3, "target": 22, "phase": "B", "role": "go", "cmd_before": {"2": 90, "3": 22}, "raw_after": raw()})
    for j in (2, 3):
        for t in SEQ[j]:
            n += 1; cur = pos[j]; before = dict(cmd); sz = abs(t - cmd[j]); nz = small_noise if sz < 13 else large_noise
            new = (max(cur, t - d_up) if t > cur else t - o_down) + (rng.gauss(0, nz) if nz else 0.0)
            pos[j] = new; cmd[j] = t
            steps.append({"n": n, "joint": j, "target": t, "phase": "T", "role": "mix", "cmd_before": {str(i): c for i, c in before.items()}, "raw_after": raw()})
    return {"steps": steps}


def test_sizes_and_counts():
    out = C.analyze(synth())
    for j in ("2", "3"):
        i = out["joints"][j]; assert i["n"] == 20 and i["n_large"] == 12 and i["n_small"] == 8, i


def test_regime_noise_supports_size_transition():
    out = C.analyze(synth(small_noise=0.4, large_noise=0.04, seed=5))
    for j in ("2", "3"):
        i = out["joints"][j]; assert i["c2_verdict"] is not None
    assert out["joints"]["3"]["c1_verdict"].startswith("큰 이동 전이 성립"), out["joints"]["3"]["c1_large"]


def test_uniform_noise_no_transition():
    ratios = [C.analyze(synth(small_noise=0.3, large_noise=0.3, seed=s))["joints"]["3"]["c2_ratio_down_small_over_large"] for s in range(11, 20)]
    assert sum(r is not None and r >= 2 for r in ratios) <= 3, ratios   # 같은 잡음이면 비가 2 이상인 경우가 드물다


def test_wrong_model_fails_transfer():
    out = C.analyze(synth(d_up=0.3, o_down=0.1, small_noise=0.05, large_noise=0.05))
    assert out["joints"]["3"]["c1_verdict"] .startswith("전이 실패"), out["joints"]["3"]["c1_large"]


def test_scope_text_kept():
    assert "6b 판정 보류" in C.analyze(synth())["interpretation_scope"]


def test_j2_filled_m3_recovers_with_zero_noise():
    """J2 고정 모델(M3, 식별 안 된 칸은 같은 방향 계수로 채움)과 같은 합성 팔 → 같은 방향 연속 이동에서도 잔차 ≈ 0."""
    P = C.T1B_PARAMS[2]; import p1_t1_model_analyze as A
    rng = random.Random(1); steps = []; n = 0; pos = {1: 90.0, 2: 90.0, 3: 22.0, 4: 11.0}; cmd = {2: 90, 3: 22}
    def raw(): return {"b": {str(j): {"deg": pos[j]} for j in (1, 2, 3, 4)}, "a": {str(j): {"deg": pos[j]} for j in (1, 2, 3, 4)}}
    steps.append({"n": 0, "joint": 3, "target": 22, "phase": "B", "role": "go", "cmd_before": {"2": 90, "3": 22}, "raw_after": raw()})
    prev_dir = 1
    for t in SEQ[2]:
        n += 1; cur = pos[2]; before = dict(cmd); dq = t - cur; c = "up" if dq > 0.5 else "down"
        row = {"dreq": dq, "prev_dir": prev_dir, "prev_actual": cur}
        pos[2] = cur + A.predict_dact(P, row); cmd[2] = t; prev_dir = 1 if t > before[2] else -1
        steps.append({"n": n, "joint": 2, "target": t, "phase": "T", "role": "mix", "cmd_before": {str(i): c_ for i, c_ in before.items()}, "raw_after": raw()})
    for t in SEQ[3]:
        n += 1; before = dict(cmd); pos[3] += (t - pos[3]) - (1.56 if t > pos[3] else -0.6434); cmd[3] = t
        steps.append({"n": n, "joint": 3, "target": t, "phase": "T", "role": "mix", "cmd_before": {str(i): c_ for i, c_ in before.items()}, "raw_after": raw()})
    out = C.analyze({"steps": steps})["joints"]["2"]
    assert out["c1_large"]["rms"] < 0.05 and out["c1_verdict"].startswith("큰 이동 전이 성립"), out["c1_large"]
    assert out["c3_history_large"]["same_dir"] is None or out["c3_history_large"]["same_dir"]["rms"] < 0.05, out["c3_history_large"]
