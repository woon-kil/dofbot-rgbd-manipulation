"""T1-A 분석 스크립트(p1_t1_model_analyze) 합성 시험: 알려진 설정값 복원, 모델이 틀린 입력에서 G3, 교차 자세 보고."""
import sys, random
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]; sys.path.insert(0, str(ROOT / "src" / "planning"))
import p1_t1_model_analyze as T


def synth(d_up=2.0, o_down=0.3, noise=0.0, seed=1, chaos=False):
    """J2·J3 계획(보정 28 + 홀드 12 + 교차 4)을 합성 팔로 실행한 결과 json(steps)."""
    rng = random.Random(seed); steps = []; n = 0; pos = {2: 90.0, 3: 22.0}; cmd = {2: 90, 3: 22}
    def raw(): return {"b": {str(j): {"deg": pos[j]} for j in (1, 2, 3, 4)}, "a": {str(j): {"deg": pos[j]} for j in (1, 2, 3, 4)}}
    for k in (1, 2, 3, 4): pos[1] = 90.0; pos[4] = 11.0
    def move(j, tgt, role):
        nonlocal n; n += 1; cur = pos[j]
        if tgt > cur: new = max(cur, tgt - d_up) + (rng.gauss(0, noise) if noise else 0.0)
        elif tgt < cur: new = tgt - o_down + (rng.gauss(0, noise) if noise else 0.0)
        else: new = cur
        if chaos: new = cur + (tgt - cur) * rng.uniform(0.0, 1.0)
        steps.append({"n": n, "joint": j, "target": tgt, "phase": "T", "role": role, "cmd_before": {str(i): c for i, c in cmd.items()}, "raw_after": None}); cmd[j] = tgt; pos[j] = new; steps[-1]["raw_after"] = raw()
    steps.append({"n": 0, "joint": 3, "target": 22, "phase": "B", "role": "go", "cmd_before": {"2": 90, "3": 22}, "raw_after": raw()})
    for j, b in ((2, 90), (3, 22)):
        for role, sizes in (("cal", (1, 2, 3, 4, 8, 12, 2)), ("hold", (10, 5, 6))):
            for s in sizes:
                for t in (b + s, b, b - s, b): move(j, t, role)
    for t in (24, 17, 10, 17): move(3, t, "xhold")
    for t in (95, 90, 85, 90): move(2, t, "xhold")
    return {"steps": steps}


def test_recover_and_g1():
    out = T.analyze(synth(d_up=2.0, o_down=0.3, noise=0.03))
    for j in ("2", "3"):
        i = out["joints"][j]; assert i["selected"] in ("M1", "M2", "M3"), i["selected"]; assert i["n_cal"] == 28 and i["n_hold"] == 12 and i["n_xhold"] == 4
        assert i["verdict"].startswith("G1"), (j, i["selected_holdout"], i["verdict"])
        assert i["cv_rms"]["M0"] > 1.0 and i["selected_holdout"]["rms"] < 0.25
    P = T.fit("M1", [r for r in T.rows_from_result(synth(noise=0.0)) if r["joint"] == 2 and r["role"] == "cal"]); assert abs(P["d_up"] - 2.0) < 0.1 and abs(P["o_down"] - 0.3) < 0.05, P


def test_wrong_model_gives_g3():
    out = T.analyze(synth(d_up=0.5, o_down=0.3, noise=0.9, seed=3))
    assert all(out["joints"][j]["verdict"] == "G3 미달" for j in ("2", "3")), {j: (out["joints"][j]["selected_holdout"], out["joints"][j]["verdict"]) for j in ("2", "3")}


def test_chaos_not_g1():
    out = T.analyze(synth(chaos=True, seed=5)); assert all(not out["joints"][j]["verdict"].startswith("G1") for j in ("2", "3"))


def test_scope_text():
    assert "6b 판정 보류" in T.analyze(synth())["interpretation_scope"]
