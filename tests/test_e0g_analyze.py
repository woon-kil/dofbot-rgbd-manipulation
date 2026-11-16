"""E-0G 분석 도구(scripts/g01_e0g_analyze.py) 시험: 합성 영상(평행 이동)으로 이동량·모델 판정·재현성·누락 처리."""
import sys, tempfile
from pathlib import Path
import numpy as np
ROOT = Path(__file__).resolve().parents[1]; sys.path.insert(0, str(ROOT / "scripts"))
import unittest
try:
    import cv2
except ImportError:                      # cv2 없는 환경(시스템 python)에서는 모듈 전체를 건너뜀
    raise unittest.SkipTest("cv2 없음")
import g01_e0g_analyze as E


def make(d, prefix, h, r, shift_y, jitter=0.0):
    img = np.full((720, 1280, 3), (200, 200, 200), np.uint8)
    rng = np.random.default_rng(1); img[0:100, 480:900] = rng.integers(60, 220, (100, 420, 1), dtype=np.uint8)   # 고정 구조 질감
    cv2.rectangle(img, (672, 300 + int(round(-shift_y))), (737, 395 + int(round(-shift_y))), (30, 215, 215), -1)      # 노랑 BGR
    tag = cv2.aruco.generateImageMarker(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11), 7, 70)
    tag = cv2.copyMakeBorder(tag, 12, 12, 12, 12, cv2.BORDER_CONSTANT, value=255)
    y0 = 303 - int(round(shift_y)); x0 = 669
    img[y0:y0 + tag.shape[0], x0:x0 + tag.shape[1]] = tag[..., None]
    cv2.imwrite(str(Path(d) / f"{prefix}_h{h}_r{r}.png"), img)


def center_of(d):
    im = cv2.imread(str(Path(d) / "e0g_h0_r1.png")); return E.detect(im)[7]["center"]


def test_synthetic_shift_matches_model():
    with tempfile.TemporaryDirectory() as d:
        for r in (1, 2): make(d, "e0g", 0, r, 0)
        c0 = np.array(center_of(d))
        for h in (30, 60):
            dy = float(c0[1] - E.predict_px(c0, h / 1000)[1])      # 예측 y 이동(양수 = 위)
            for r in (1, 2): make(d, "e0g", h, r, dy)
        res = E.analyze(d, "e0g", [0, 30, 60], 2)
    assert res["robot_commands"] == 0 and res["repeat_ok"] and res["model_ok_all"]
    h30 = [x for x in res["heights"] if x["h_mm"] == 30][0]
    assert 0.8 <= h30["ratio_to_pred"] <= 1.2 and h30["blob_bottom_shift_px"] > 10 and res["Q3_ok"] is True


def test_wrong_scale_fails_model():
    with tempfile.TemporaryDirectory() as d:
        for r in (1, 2): make(d, "e0g", 0, r, 0)
        c0 = np.array(center_of(d))
        dy = float(c0[1] - E.predict_px(c0, 0.030)[1])
        for r in (1, 2): make(d, "e0g", 30, r, dy * 2)           # 2배 이동 = 모델 불일치
        res = E.analyze(d, "e0g", [0, 30], 2)
    assert res["model_ok_all"] is False


def test_missing_zero_height_reports_error():
    with tempfile.TemporaryDirectory() as d:
        res = E.analyze(d, "e0g", [0, 30], 2)
    assert "error" in res


def test_no_robot_or_camera_code():
    src = (ROOT / "scripts/g01_e0g_analyze.py").read_text(encoding="utf-8")
    for bad in ("subprocess", "ssh", "gripper_adapter", "Arm_Lib", "serial"):
        assert bad not in src, bad


def test_q3_fails_when_base_moves():
    with tempfile.TemporaryDirectory() as d:
        make(d, "e0g", 0, 1, 0); make(d, "e0g", 0, 2, 0)
        im = cv2.imread(str(Path(d) / "e0g_h0_r2.png")); sh = np.roll(im, 6, axis=1)          # 가로로 6 px 밀림
        cv2.imwrite(str(Path(d) / "e0g_h0_r2.png"), sh)
        res = E.analyze(d, "e0g", [0], 2)
    assert res["Q3_ok"] is False
