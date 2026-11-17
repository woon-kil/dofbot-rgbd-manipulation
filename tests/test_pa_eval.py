"""P-A 평가(scripts/pa_eval.py)와 프레임 신선도 수집기(scripts/pa_snap.py FreshCollector) 시험. pytest 방식(unittest discover에는 잡히지 않는다)."""
import json, sys, tempfile
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
import pa_eval as E
import pa_snap as S


def test_fresh_collector_drops_stale_and_takes_first_fresh():
    c = S.FreshCollector(100.0)
    assert not c.offer("color", 99.9, 100.1, "old") and c.stale["color"] == 1
    assert not c.offer("color", 100.0, 100.1, "eq")                      # stamp == 요청 시각은 쓰지 않는다(>)
    assert c.offer("color", 100.05, 100.1, "A") and not c.offer("color", 100.2, 100.3, "B")   # 첫 신선 프레임만
    assert not c.done() and c.offer("depth", 100.07, 100.12, "D") and c.done() and c.got["color"]["payload"] == "A"


def test_build_meta_fields_and_failure():
    col = {"color": {"stamp": 10.5, "recv": 10.56, "payload": None, "frame_id": "f"}}
    m = S.build_meta("rid", 10.0, col, 11.0, True, None, {"color": 2, "depth": 0})
    assert m["request_id"] == "rid" and abs(m["color"]["recv_minus_stamp"] - 0.06) < 1e-9 and abs(m["color"]["stamp_minus_req"] - 0.5) < 1e-9
    f = S.build_meta("rid", 10.0, {}, 11.0, False, "타임아웃")
    assert f["ok"] is False and "error" in f and "color" not in f


def test_angle_wrap():
    assert abs(E.ang_diff(179.0, -179.0) - (-2.0)) < 1e-9
    assert abs(E.mean_angle_near([178.0, -178.0], 178.0) - 180.0) < 1e-9       # ±180 경계 평균


def _snap(i, lat=1.0, rms=0.05, smr=0.4, found=True, center=(676.0, 266.5), side=28.5, yaw=-178.2, rid=True):
    n = "S_snap%02d" % i
    meta = {"ok": True, "request_id": n + "-1" if rid else "x", "color": {"stamp_minus_req": smr, "recv_minus_stamp": rms}, "depth": {"stamp_minus_req": smr, "recv_minus_stamp": rms}}
    det = {"found": True, "center": list(center), "side_px": side, "yaw_deg": yaw} if found else {"found": False}
    return {"name": n, "marks": {"m_start": 0.0, "m_detect_done": lat}, "meta": meta, "detect": det, "req_id": n + "-1", "bright": 116.4, "valid": 0.764}


def test_m1_all_must_be_within_3s():
    assert E.m1([_snap(i) for i in range(1, 21)])["pass"]
    s = [_snap(i) for i in range(1, 21)]; s[7]["marks"]["m_detect_done"] = 3.01
    assert not E.m1(s)["pass"] and not E.m1(s[:19])["pass"]


def test_m1_prime_freshness_and_undecidable():
    s = [_snap(i) for i in range(1, 21)]
    assert E.m1p(s)["pass"] is True
    s[3]["meta"]["color"]["stamp_minus_req"] = -0.01
    assert E.m1p(s)["pass"] is False                                           # 요청 전에 노출된 프레임
    s = [_snap(i, rms=0.5) for i in range(1, 21)]
    assert E.m1p(s)["pass"] is None and E.m1p(s)["undecidable"]                # stamp가 호스트 시계 기준이 아닐 때 판정 불가


def test_m1_double_prime_request_id_and_cycles():
    s = [_snap(i) for i in range(1, 21)]
    assert E.m1pp(s, {"snap_after_stop_failed": True, "auto_exit_snap_failed": True})["pass"]
    assert not E.m1pp(s, {"snap_after_stop_failed": True, "auto_exit_snap_failed": False})["pass"]
    s[0]["meta"]["request_id"] = "old"
    assert not E.m1pp(s, {"snap_after_stop_failed": True, "auto_exit_snap_failed": True})["pass"]


BASE = {"found": True, "center": [676.0, 266.5], "side_px": 28.5, "yaw_deg": -178.2}
BASES4 = [BASE, BASE, BASE, BASE]


def test_m2_bias_pass_and_yaw_wrap():
    s = [_snap(i, yaw=179.0 if i % 2 else -179.0) for i in range(1, 21)]      # ±180 경계에 걸친 yaw
    bs = [dict(BASE, yaw_deg=179.5), dict(BASE, yaw_deg=-179.5), dict(BASE, yaw_deg=179.5), dict(BASE, yaw_deg=-179.5)]
    r = E.m2(s, bs)
    assert r["m2a"]["pass"] and r["m2b"]["pass"] is True, r["m2b"]


def test_m2_bias_fail_and_front_back_gate_undecidable():
    s = [_snap(i, center=(676.5, 266.5)) for i in range(1, 21)]                 # 중심 편향 0.5 > 0.32
    assert E.m2(s, BASES4)["m2b"]["pass"] is False
    moved = [BASE, BASE, dict(BASE, center=[677.0, 266.5]), dict(BASE, center=[677.0, 266.5])]   # 앞 평균 대 뒤 평균 1.0 > 0.75 → 장면이 움직임
    r2 = E.m2(s, moved)
    assert r2["m2b"]["pass"] is None and r2["m2b"]["undecidable"]
    exact = [BASE, BASE, dict(BASE, center=[676.75, 266.5]), dict(BASE, center=[676.75, 266.5])]  # 정확히 0.75 → '이하이면 정상'
    assert E.m2(s, exact)["m2b"]["undecidable"] is False


def test_within_pair_noise_alone_does_not_make_undecidable():
    s = [_snap(i) for i in range(1, 21)]
    noisy = [BASE, dict(BASE, side_px=29.4), BASE, dict(BASE, side_px=29.4)]    # A1-A2·B1-B2 변 길이 차 0.9 > 0.75이지만 앞·뒤 평균은 같다
    r = E.m2(s, noisy)
    assert r["m2b"]["undecidable"] is False and r["m2b"]["within_pair_diff_reference"]["A1-A2"][1] > 0.75


def test_m2c_within_start_18_pairs_and_cross_start_pair_excluded():
    s = [_snap(i, side=28.5 + (0.45 if i % 2 else 0.0)) for i in range(1, 21)]
    r = E.m2(s, BASES4)["m2c"]
    assert r["pass"] and r["n_pairs"] == 18                                     # 18쌍(기동 안), 최대 0.45 < 0.7
    s2 = [_snap(i, side=28.5 if i <= 10 else 29.4) for i in range(1, 21)]       # 기동 안은 모두 같고 기동 #1→#2에서만 0.9 차이
    assert E.m2(s2, BASES4)["m2c"]["pass"]                                       # snap10(#1)-snap11(#2) 쌍은 제외되어 통과
    s2[3]["detect"]["side_px"] = 29.4                                            # 같은 기동 안 쌍 0.9 ≥ 0.7
    assert not E.m2(s2, BASES4)["m2c"]["pass"]


def test_m2a_requires_all_20_detected():
    s = [_snap(i) for i in range(1, 21)]; s[2]["detect"] = {"found": False}
    assert not E.m2(s, BASES4)["m2a"]["pass"]


def _bases3(vals, valid=0.764):
    return [{"bright": b, "valid": valid} for b in vals]


def test_m3_per_start_means_against_four_start_range():
    s = [_snap(i) for i in range(1, 21)]                                         # 모든 프레임 116.4
    assert E.m3(s, _bases3([116.0, 118.0, 115.5, 119.0]))["pass"]                # 기동 평균 116.4 ∈ [113.5, 121.0]
    far = _bases3([120.0, 121.0, 122.0, 123.0])                                  # 범위 [118.0, 125.0] 밖
    assert not E.m3(s, far)["pass"]
    for i in range(10, 20): s[i]["bright"] = 124.0                                # 기동 #2 평균 124 → #1은 안, #2는 밖이면 실패
    assert not E.m3(s, _bases3([116.0, 118.0, 115.5, 119.0]))["pass"]
    assert E.m3(s, [{"bright": None, "valid": None}] * 4)["pass"] is None


def test_m3_depth_all_frames_within_baseline_range():
    s = [_snap(i) for i in range(1, 21)]; s[4]["valid"] = 0.80
    assert not E.m3(s, _bases3([116.0, 117.0, 116.5, 116.8]))["pass"]


def test_m4_per_file_usb_counts_and_clean_stops():
    p = {"rate": {"color": {"arrival_hz": 30.0}, "depth": {"arrival_hz": 29.9}}}
    ok = {"usb_counts": {"a": 0, "b": 0, "c": 0}, "containers_left_after_stop": [0, 0, 0]}
    assert E.m4(p, p, ok)["pass"]
    assert not E.m4(p, p, dict(ok, usb_counts={"a": 0, "b": 1, "c": 0}))["pass"]            # 어느 로그든 끊김이 있으면 실패
    assert not E.m4(p, p, dict(ok, usb_counts={"a": 0, "b": 0}))["pass"]                     # 로그 수가 3이 아니면(다른 세션 로그 혼입·누락) 판정하지 않는다
    assert not E.m4(p, p, dict(ok, containers_left_after_stop=[0, 1, 0]))["pass"]
    assert not E.m4(p, {"rate": {"color": {"arrival_hz": 28.0}, "depth": {"arrival_hz": 30.0}}}, ok)["pass"]


def test_evaluate_full_with_synthetic_files_and_frozen_constants():
    with tempfile.TemporaryDirectory() as d:
        d = Path(d)
        for i in range(1, 21):
            n = "S_snap%02d" % i
            (d / (n + ".mac_timing.txt")).write_text("m_start 0.0\nm_detect_done 1.0\n")
            (d / (n + ".meta.json")).write_text(json.dumps({"ok": True, "request_id": n + "-1", "color": {"stamp_minus_req": 0.3, "recv_minus_stamp": 0.05}, "depth": {"stamp_minus_req": 0.3, "recv_minus_stamp": 0.04}}))
            (d / (n + ".detect.json")).write_text(json.dumps(BASE))
            (d / (n + ".req_id.txt")).write_text(n + "-1\n")
        for t in E.BASES: (d / ("S_base_%s.detect.json" % t)).write_text(json.dumps(BASE))
        (d / "S_cycles.json").write_text(json.dumps({"snap_after_stop_failed": True, "auto_exit_snap_failed": True, "usb_counts": {"l1": 0, "l2": 0, "l3": 0}, "containers_left_after_stop": [0, 0, 0]}))
        p = {"rate": {"color": {"arrival_hz": 30.0}, "depth": {"arrival_hz": 30.0}}}
        (d / "S_probe_start.json").write_text(json.dumps(p)); (d / "S_probe_end.json").write_text(json.dumps(p))
        r = E.evaluate("S", d)
        assert r["M1"]["pass"] and r["M1'"]["pass"] and r["M1''"]["pass"] and r["m2a"]["pass"] and r["m2b"]["pass"] and r["m2c"]["pass"] and r["M4"]["pass"]
        assert r["overall"] in ("성립", "판정 불가 포함")                         # 이미지 파일이 없으면 M3만 판정 불가
    assert (E.M1_MAX_S, E.BIAS_CENTER_PX, E.BIAS_SIDE_PX, E.BIAS_YAW_DEG, E.GATE_CENTER_PX, E.GATE_SIDE_PX, E.GATE_YAW_DEG, E.E1_SIDE_PX, E.M3_BRIGHT, E.M3_VALID, E.M4_MIN_HZ) == (3.0, 0.32, 0.25, 1.0, 0.75, 0.75, 3.0, 0.7, 2.0, 0.02, 29.0)
    assert E.BASES == ("a1", "a2", "b1", "b2") and E.N_STARTS_LOGS == 3


def _full_session_files(d, cycles_extra):
    for i in range(1, 21):
        n = "S_snap%02d" % i
        (d / (n + ".mac_timing.txt")).write_text("m_start 0.0\nm_detect_done 1.0\n")
        (d / (n + ".meta.json")).write_text(json.dumps({"ok": True, "request_id": n + "-1", "color": {"stamp_minus_req": 0.3, "recv_minus_stamp": 0.05}, "depth": {"stamp_minus_req": 0.3, "recv_minus_stamp": 0.04}}))
        (d / (n + ".detect.json")).write_text(json.dumps(BASE)); (d / (n + ".req_id.txt")).write_text(n + "-1\n")
    for t in E.BASES: (d / ("S_base_%s.detect.json" % t)).write_text(json.dumps(BASE))
    cyc = {"snap_after_stop_failed": True, "auto_exit_snap_failed": True, "usb_counts": {"l1": 0, "l2": 0, "l3": 0}, "containers_left_after_stop": [0, 0, 0]}; cyc.update(cycles_extra)
    (d / "S_cycles.json").write_text(json.dumps(cyc))
    p = {"rate": {"color": {"arrival_hz": 30.0}, "depth": {"arrival_hz": 30.0}}}
    (d / "S_probe_start.json").write_text(json.dumps(p)); (d / "S_probe_end.json").write_text(json.dumps(p))


def test_aborted_or_fast_session_is_not_a_p_a_verdict():
    with tempfile.TemporaryDirectory() as d:
        d = Path(d); _full_session_files(d, {"aborted": "status_pattern_start1"})
        r = E.evaluate("S", d)
        assert r["overall"] == "시험 중단(판정 없음): status_pattern_start1"          # 지표가 FAIL이어도 불성립으로 세지 않는다
    with tempfile.TemporaryDirectory() as d:
        d = Path(d); _full_session_files(d, {"fast_mode": True})
        assert E.evaluate("S", d)["overall"] == "시험용 FAST 세션(판정 없음)"
    with tempfile.TemporaryDirectory() as d:
        d = Path(d); _full_session_files(d, {})                                       # 정상 세션은 지표로 판정(이미지 없음 → M3 판정 불가)
        assert E.evaluate("S", d)["overall"] in ("성립", "판정 불가 포함")
