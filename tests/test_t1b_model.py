"""T1-B 분석 스크립트(p1_t1b_analyze) 합성 시험: 전이 성립/실패(Q1), 쌍 단위 묶음, 알려진 설정값 복원."""
import sys, random
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]; sys.path.insert(0, str(ROOT / "src" / "planning"))
import p1_t1b_analyze as B


def synth(d_up=1.95, o_down=0.22, k_up=0.0, noise=0.0, seed=2):
    """J2·J3 세트 18 계획(보정 16 + 홀드 12, 왕복 쌍)을 합성 팔로 실행한 결과 json(steps). k_up = 크기 기울기(위 모자람 += k·폭)."""
    rng = random.Random(seed); steps = []; n = 0; pos = {1: 90.0, 2: 90.0, 3: 22.0, 4: 11.0}; cmd = {2: 90, 3: 22}
    def raw(): return {"b": {str(j): {"deg": pos[j]} for j in (1, 2, 3, 4)}, "a": {str(j): {"deg": pos[j]} for j in (1, 2, 3, 4)}}
    def move(j, tgt, role):
        nonlocal n; n += 1; cur = pos[j]; before = dict(cmd)
        if tgt > cur: new = max(cur, tgt - (d_up + k_up * (tgt - cur))) + (rng.gauss(0, noise) if noise else 0.0)
        elif tgt < cur: new = tgt - o_down + (rng.gauss(0, noise) if noise else 0.0)
        else: new = cur
        pos[j] = new; cmd[j] = tgt
        steps.append({"n": n, "joint": j, "target": tgt, "phase": "T", "role": role, "cmd_before": {str(i): c for i, c in before.items()}, "raw_after": raw()})
    steps.append({"n": 0, "joint": 3, "target": 22, "phase": "B", "role": "go", "cmd_before": {"2": 90, "3": 22}, "raw_after": raw()})
    for j, b in ((2, 90), (3, 22)):
        for role, sizes in (("cal", (15, 25, 35, 45, 15, 25, 35, 45)), ("hold", (20, 30, 40, 20, 30, 40))):
            for s in sizes:
                for t in (b + s, b): move(j, t, role)
    return {"steps": steps}


def test_pairs_and_transfer_holds():
    out = B.analyze(synth(noise=0.05))
    for j in ("2", "3"):
        i = out["joints"][j]; assert i["n_cal"] == 16 and i["n_hold"] == 12
        assert len(B.pairs(list(range(16)))) == 8
        assert i["q1_transfer_hold"]["verdict_q1"].startswith("전이 성립"), i["q1_transfer_hold"]
        assert i["verdict_q2"].startswith("G1"), (j, i["selected_holdout"])


def test_size_slope_breaks_transfer():
    out = B.analyze(synth(k_up=0.03, noise=0.05))
    for j in ("2", "3"):
        i = out["joints"][j]; assert i["q1_transfer_hold"]["verdict_q1"].startswith("전이 실패"), i["q1_transfer_hold"]
        assert i["selected_holdout"]["rms"] < i["q1_transfer_hold"]["rms"], "재적합이 전이보다 좋아야 한다"


def test_scope_text_kept():
    assert "6b 판정 보류" in B.analyze(synth())["interpretation_scope"]
