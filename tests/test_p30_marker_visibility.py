"""P3-0 마커 가시성 도구(scripts/g01_p30_marker_visibility.py) 시험: 예측 표, 가짜 영상 검출·판정, 로봇 코드 부재."""
import sys, tempfile
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]; sys.path.insert(0, str(ROOT / "scripts"))
import g01_p30_marker_visibility as P


def test_predict_matches_design_table():
    assert P.predict(0)["centre_px"] == [375, 532] and not P.predict(0)["all_corners_inside"]
    assert P.predict(43)["all_corners_inside"] is False          # 하단행 ≈ 480 경계
    assert P.predict(50)["all_corners_inside"] and P.predict(60)["all_corners_inside"]
    assert abs(P.predict(10)["centre_px"][1] - 501) <= 1


def test_fake_images_detect_and_judge():
    import pytest; pytest.importorskip("cv2")
    with tempfile.TemporaryDirectory() as d:
        P.fake_images(d)
        r = P.analyze(d)
    assert r["robot_commands"] == 0 and r["missing_images_h"] == []
    assert r["Q1"].startswith("예측과 일치")                      # h=0은 영상 밖이라 검출 안 됨
    assert r["Q2_min_h_all_detected"] in (50, 60)                 # 가장자리 여유 때문에 50은 실패할 수 있음
    for row in r["rows"]:
        if row["n_detected"]:
            assert row["centre_matches_pred"] is True


def test_missing_images_reported():
    with tempfile.TemporaryDirectory() as d:
        r = P.analyze(d)
    assert r["missing_images_h"] == P.H_LIST and r["Q2_min_h_all_detected"] is None and r["Q1"] == "촬영 없음"


def test_no_robot_code():
    src = (ROOT / "scripts/g01_p30_marker_visibility.py").read_text(encoding="utf-8")
    for bad in ("gripper_adapter", "Arm_serial", "Arm_Lib", "serial", "write_servo", "set_servo"):
        assert bad not in src, bad
