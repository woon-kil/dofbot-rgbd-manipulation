"""출발 전 공통 점검(P1-b2 승인 요청 문서 2-1절, 큰 이동 M3 공통). 읽기 전용: 팔·카메라에 어떤 명령도 보내지 않고, 이미 읽은 회독값·사진·스크립트 파일만 판정한다.
하나라도 실패하면 종료 코드 1(팔 이동 없이 중단). 판정 기준은 2-1절 ‘제안’ 값이며 사용자 승인 전에는 미동결이다.
사용: python3 -I src/planning/p1_preflight.py --readback J1,J2,J3,J4,J5,J6 [--image 사진 --ref-image 기준사진 --marker-ref 702.8,273.0] [--manifest MANIFEST.sha256 --scripts-dir scripts] [--out 기록.json]
사진 점검(cv2 필요)은 --image를 줄 때만 실행한다. 사진 촬영(p620 Gemini)은 기존 절차를 쓰고 이 도구는 받은 파일만 판정한다."""
import argparse
import hashlib
import json
import sys
from pathlib import Path

K1 = (89, 91, 91, 91, 90, 179)                      # 기준 K1 기립(오므림) 자세: evidence/G01/raw/g01_5b4_s4212_result-real-261115-022536.json
TOL = {1: 2.0, 2: 2.0, 3: 2.0, 4: 2.0, 5: 1.0, 6: 1.0}   # J1~J4 ±2°, J5 ±1°, J6 ±1°(오므림 유지 확인)  — 2-1절 점검 2 제안값
MARKER_ID = 7
MARKER_TOL_PX = 4.0     # 마커 중심 허용(약 3 mm) — 점검 3 제안값
STILL_TOL_PX = 0.5      # 기준 사진 대비 카메라 이동(위상 상관) 참고 폭 — 점검 3 제안값. 2026-11-15 실기기에서 카메라가 2.46 px 움직여 게이트에서 기록 항목으로 내렸다(아래 check_scene)
SCENE_ROI = (480, 0, 900, 400)   # S1(사용자 승인 2026-11-16 11:45): 이동·응답 추정 영역 = 로봇 베이스+종이(x0, y0, x1, y1). 왼쪽 물건·사람·아래 테이블처럼 시험마다 달라지는 배경이 블록과 무관하게 게이트를 막지 않게 한다. 응답 최소·블록 허용은 그대로
RESPONSE_MIN = 0.5      # 전체 이동 추정(위상 상관 응답)의 신뢰 하한. 같은 장면의 영역별 응답은 0.74~0.98이었다. 낮으면 ‘블록의 장면 내 이동’ 계산을 믿을 수 없어 실패 처리(제안값)


def check_readback(rb, k1=K1, tol=TOL):
    """회독 J1~J6(길이 6, None 허용)을 기준 K1과 비교한다. 각 관절의 차이를 기록하고, None이거나 허용 밖이면 실패."""
    rows = []
    for j in range(1, 7):
        v = rb[j - 1] if j - 1 < len(rb) else None
        d = None if v is None else v - k1[j - 1]
        rows.append({"joint": j, "readback": v, "expected": k1[j - 1], "diff": d, "tol": tol[j], "ok": d is not None and abs(d) <= tol[j]})
    return {"name": "readback_vs_K1", "ok": all(r["ok"] for r in rows), "rows": rows}


def check_scene(image_path, ref_path, marker_ref_px, marker_tol=MARKER_TOL_PX, still_tol=STILL_TOL_PX):
    """Gemini 사진 점검. 게이트: **블록이 장면 안에서 확정 자리에서 움직이지 않았다** = (마커 id 7 중심 이동) − (기준 사진 대비 전체 영상 이동) 의 크기가 marker_tol(약 3 mm) 이하, 그리고 전체 이동 추정 신뢰(위상 상관 응답) ≥ RESPONSE_MIN.
    기록(게이트 아님): 카메라 이동(ROI 이동 px, 전체 영상 추정은 full_image_info에 참고로 기록). 카메라가 조금 움직여도 마커와 배경이 같이 움직이면 블록은 그대로다 — 이 점검이 막으려는 것은 블록 이동이고, 큰 이동 M3의 팔 이동에는 Gemini를 쓰지 않는다.
    (변경 이력: 처음 게이트는 ‘카메라 이동 ≤ 0.5 px 그리고 마커 중심 ≤ 4 px’였고 2026-11-15 실기기에서 카메라가 균일하게 2.46 px 움직여 실패했다. 영역별로 전 영역이 같은 방향으로 움직이고 마커 잔차는 0.23 px였다.)"""
    import cv2
    import numpy as np
    im = cv2.imread(str(image_path)); rf = cv2.imread(str(ref_path))
    if im is None or rf is None:
        return {"name": "scene", "ok": False, "error": "사진을 읽을 수 없다", "image": str(image_path), "ref": str(ref_path)}
    if im.shape != rf.shape:
        return {"name": "scene", "ok": False, "error": "사진 크기가 기준과 다르다 %s vs %s" % (im.shape, rf.shape)}
    a = cv2.cvtColor(rf, cv2.COLOR_BGR2GRAY).astype(np.float32); b = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY).astype(np.float32)
    x0, y0, x1, y1 = SCENE_ROI
    (sx, sy), resp = cv2.phaseCorrelate(a[y0:y1, x0:x1], b[y0:y1, x0:x1], cv2.createHanningWindow((x1 - x0, y1 - y0), cv2.CV_32F))   # S1: ROI만으로 추정(전체 영상이 아님)
    (fsx, fsy), fresp = cv2.phaseCorrelate(a, b, cv2.createHanningWindow((a.shape[1], a.shape[0]), cv2.CV_32F))   # 참고 기록: 전체 영상 추정(게이트 아님)
    shift = float(np.hypot(sx, sy))
    det = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11), cv2.aruco.DetectorParameters())
    corners, ids, _ = det.detectMarkers(b.astype(np.uint8))
    cen = None
    if ids is not None:
        for c, i in zip(corners, ids.ravel()):
            if int(i) == MARKER_ID:
                cen = c.reshape(4, 2).mean(0)
    marker_move = None if cen is None else (float(cen[0] - marker_ref_px[0]), float(cen[1] - marker_ref_px[1]))
    block_rel = None if cen is None else float(np.hypot(marker_move[0] - sx, marker_move[1] - sy))   # 장면 안에서 블록이 움직인 크기(px)
    response_ok = float(resp) >= RESPONSE_MIN; marker_ok = block_rel is not None and block_rel <= marker_tol
    return {"name": "scene", "ok": marker_ok and response_ok, "block_move_in_scene_px": None if block_rel is None else round(block_rel, 2), "marker_tol_px": marker_tol,
            "camera_shift_px": round(shift, 3), "camera_shift_xy_px": [round(float(sx), 2), round(float(sy), 2)], "camera_shift_ref_px": still_tol, "phase_response": round(float(resp), 3), "response_min": RESPONSE_MIN,
            "marker_center_px": None if cen is None else [round(float(cen[0]), 2), round(float(cen[1]), 2)], "marker_ref_px": list(marker_ref_px), "marker_move_px": None if marker_move is None else [round(marker_move[0], 2), round(marker_move[1], 2)],
            "camera_still_ok": shift <= still_tol, "marker_ok": marker_ok, "response_ok": response_ok,
            "roi": list(SCENE_ROI), "full_image_info": {"shift_xy_px": [round(float(fsx), 2), round(float(fsy), 2)], "response": round(float(fresp), 3)}}


def check_projection(poses_path, margin=47.0):
    """게이트: 자세 파일의 모든 자세에서 블록 위 마커 4모서리(위치 ±흔들림 포함)가 FK 카메라 자세로 투영했을 때 영상 가장자리에서 margin px 이상 안쪽인가.
    선정 코드(PoseGrid.visible_mask)와 별개 경로(camera_pose + project)다. 2026-11-15 큰 이동 M3에서 가시성 필터가 주석에 묻혀 마커가 6/6 미검출이었던 사고의 재발 방지.
    margin 47 px = max(40, 2 × P1 실촬영 FK 투영 최대 오차 23.5 px). 무거운 의존(FK·numpy)은 이 함수 안에서만 가져온다."""
    import sys as _s
    root = str(Path(__file__).resolve().parents[1]); _s.path.insert(0, root)
    import numpy as np
    from kinematics.urdf_fk import UrdfFK
    from planning import p1_b2_poses as B, p1_poses as P
    from planning.dry_run import TABLE_Z
    from planning.p1_near_m3 import corner_margins
    fk = UrdfFK(); M = np.array([B.CENTER[0], B.CENTER[1] + B.MARKER_OFFSET_Y, TABLE_Z + P.BLOCK[2]]); h = P.MARKER / 2
    rows = []
    for p in json.loads(Path(poses_path).read_text(encoding="utf-8"))["poses"]:
        m, cen = corner_margins(fk, [float(x) for x in p["sdk_deg"][:4]], M, h, B.SHIFTS)
        rows.append({"id": p["id"], "name": p.get("name"), "q": p["sdk_deg"][:4], "edge_margin_px": round(m, 1), "center_px_depth_mm": None if cen is None else [round(cen[0], 1), round(cen[1], 1), round(cen[2] * 1000, 1)], "ok": m >= margin})
    return {"name": "marker_projection", "ok": bool(rows) and all(r["ok"] for r in rows), "margin_px": margin, "rows": rows,
            "poses_sha256": hashlib.sha256(Path(poses_path).read_bytes()).hexdigest()}


def check_scripts(manifest_path, scripts_dir, name="scripts_sha256"):
    """승인 때 고정한 MANIFEST(형식 `sha256  파일명`, 파일명은 base 디렉터리 기준 상대 경로)의 모든 파일이 실제 파일과 같은지 비교한다. 스크립트(scripts/)와 충돌 검증 코드·자세 파일·URDF(저장소 루트 기준) 두 곳에 쓴다."""
    rows = []
    for line in Path(manifest_path).read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        sha, fname = line.split(None, 1); fname = fname.strip().lstrip("*")
        p = Path(scripts_dir) / fname
        actual = hashlib.sha256(p.read_bytes()).hexdigest() if p.is_file() else None
        rows.append({"file": fname, "expected": sha, "actual": actual, "ok": actual == sha})
    return {"name": name, "ok": bool(rows) and all(r["ok"] for r in rows), "n": len(rows), "mismatch": [r["file"] for r in rows if not r["ok"]], "manifest_sha256": hashlib.sha256(Path(manifest_path).read_bytes()).hexdigest()}


def _lib_int(raw, jid):
    """Arm_Lib 1.0.3 `Arm_serial_servo_read`의 환산·절삭(원시 900~3100 = 0~180, int 절삭, J2~J4는 180 − 값). 범위 밖은 None."""
    p = int(180 * (raw - 900) / 2200)
    if p > 180 or p < 0:
        return None
    return 180 - p if jid in (2, 3, 4) else p


def _norm(text):
    import textwrap
    return "\n".join(l.rstrip() for l in textwrap.dedent(text).strip().splitlines())


def func_hash(source, name="Arm_serial_servo_read"):
    """소스 텍스트에서 함수 `name`만 뽑아(ast) 들여쓰기·줄 끝 공백을 정규화한 sha256(앞 16자). Jetson 쪽은 `inspect.getsource`로 같은 함수를 뽑아 같은 정규화를 적용한다."""
    import ast
    lines = source.splitlines()
    for n in ast.walk(ast.parse(source)):
        if isinstance(n, ast.FunctionDef) and n.name == name:
            return hashlib.sha256(_norm("\n".join(lines[n.lineno - 1:n.end_lineno])).encode()).hexdigest()[:16]
    return None


def check_arm_lib(installed_func_hash, ref_path, error=None, installed_file_sha=None):
    """게이트: Jetson에 설치된 `Arm_serial_servo_read` 함수 소스가 읽은 소스와 같은가(원시 환산 가정이 기대는 식은 이 함수 하나다). 파일 전체 sha는 기록만 한다(다른 곳이 바뀌어도 가정은 깨지지 않는다).
    확인 불가(설치 형태 때문에 소스를 못 읽음 등)는 불일치와 구별해 실패로 처리한다 — 통과로 치지 않는다."""
    ref_src = Path(ref_path).read_text(encoding="utf-8"); ref_fn = func_hash(ref_src); ref_file = hashlib.sha256(ref_src.encode()).hexdigest()
    rec = {"name": "arm_lib_version", "reference_func_hash": ref_fn, "reference_file_sha256": ref_file, "installed_file_sha256": installed_file_sha, "file_sha_equal": (installed_file_sha == ref_file) if installed_file_sha else None}
    if error or not installed_func_hash:
        return {**rec, "ok": False, "status": "unverifiable", "note": "Jetson의 Arm_Lib 읽기 함수 소스를 확인하지 못했다(%s) — 통과로 치지 않는다. 사용자에게 알린다" % (error or "값 없음")}
    same = installed_func_hash == ref_fn
    return {**rec, "ok": same, "status": "match" if same else "mismatch", "installed_func_hash": installed_func_hash,
            "note": None if same else "설치된 Arm_serial_servo_read가 읽은 소스와 다르다 — 사용자에게 알리고 원시 환산 가정을 다시 확인한다"}


def check_raw_samples(samples, min_consistent=0.95):
    """J1~J4 원시 값 표본(`{"1": {"int": [...], "raw": [...]}, ...}`): 정수 회독과 원시 환산이 맞는지, 원시 값의 퍼짐이 정수 반올림 흔들림(균등 ±0.5°의 SD 0.289°)보다 작은지 기록한다.
    gate=False: 결과는 기록·보고만 하고 팔 이동 판단에는 쓰지 않는다(원시 읽기가 안 맞으면 결정 8의 효과를 다시 평가한다)."""
    rows = []
    for j in ("1", "2", "3", "4"):
        sm = samples.get(j) or {}; ints = sm.get("int") or []; raws = sm.get("raw") or []
        pairs = [(i, r) for i, r in zip(ints, raws) if i is not None and r is not None]
        n_none = len(raws) - len([r for r in raws if r is not None])
        if not pairs:
            rows.append({"joint": int(j), "n": 0, "n_none": n_none, "ok": False}); continue
        rs = [r for _, r in pairs]; mean = sum(rs) / len(rs); sd = (sum((r - mean) ** 2 for r in rs) / len(rs)) ** 0.5
        cons = sum(1 for i, r in pairs if (_lib_int(r, int(j)) is not None and abs(_lib_int(r, int(j)) - i) <= 1)) / len(pairs)
        rows.append({"joint": int(j), "n": len(pairs), "n_none": n_none, "raw_min": min(rs), "raw_max": max(rs), "raw_sd_counts": round(sd, 2), "raw_sd_deg": round(sd * 180 / 2200, 3),
                     "int_vs_raw_consistent": round(cons, 3), "sd_below_rounding": sd * 180 / 2200 < 0.289, "ok": cons >= min_consistent and n_none == 0})
    return {"name": "raw_samples", "ok": all(r["ok"] for r in rows), "gate": False, "rows": rows, "rounding_sd_deg": 0.289}


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--readback", required=True, help="J1,J2,J3,J4,J5,J6 (None은 'None')")
    ap.add_argument("--image"); ap.add_argument("--ref-image"); ap.add_argument("--marker-ref", help="x,y(기준 사진의 마커 id 7 중심 px)")
    ap.add_argument("--manifest"); ap.add_argument("--scripts-dir", default="scripts"); ap.add_argument("--out")
    ap.add_argument("--manifest-src", help="충돌 검증 코드·자세 파일·URDF·메시의 MANIFEST(저장소 루트 기준 경로)"); ap.add_argument("--root", default=str(Path(__file__).resolve().parents[2]))
    ap.add_argument("--armlib-func-hash", help="Jetson 설치본 Arm_serial_servo_read 정규화 해시(앞 16자)"); ap.add_argument("--armlib-file-sha", help="설치본 파일 전체 sha256(기록만)"); ap.add_argument("--armlib-error", help="설치본 확인 실패 사유"); ap.add_argument("--armlib-ref", default=str(Path(__file__).resolve().parents[2] / "data/refs/yahboom-Arm_Lib-1.0.3-Orin.py"))
    ap.add_argument("--raw-samples", help="원시 값 표본 JSON 파일"); ap.add_argument("--poses-file", help="자세 파일: 모든 자세의 마커 투영 가시성(FK, 선정과 독립)을 게이트로 확인")
    a = ap.parse_args(argv)
    rb = [None if t.strip() == "None" else float(t) for t in a.readback.split(",")]
    results = [check_readback(rb)]
    if a.image:
        if not (a.ref_image and a.marker_ref):
            sys.exit("--image에는 --ref-image와 --marker-ref가 함께 필요하다")
        results.append(check_scene(a.image, a.ref_image, tuple(float(t) for t in a.marker_ref.split(","))))
    if a.manifest:
        results.append(check_scripts(a.manifest, a.scripts_dir))
    if a.poses_file:
        results.append(check_projection(a.poses_file))
    if a.manifest_src:
        results.append(check_scripts(a.manifest_src, a.root, name="src_sha256"))
    if a.armlib_func_hash or a.armlib_error:
        results.append(check_arm_lib(a.armlib_func_hash, a.armlib_ref, a.armlib_error, a.armlib_file_sha))
    if a.raw_samples:
        results.append(check_raw_samples(json.loads(Path(a.raw_samples).read_text(encoding="utf-8"))))
    ok = all(r["ok"] for r in results if r.get("gate", True))
    rec = {"ok": ok, "checks": results, "note": "읽기 전용 점검. 하나라도 실패하면 팔을 움직이지 않는다."}
    for r in results:
        tag = ("통과" if r["ok"] else "실패") if r.get("gate", True) else ("기록(정상)" if r["ok"] else "기록(주의, 이동 판단에는 쓰지 않음)")
        print("[%s] %s" % (tag, r["name"]), {k: v for k, v in r.items() if k not in ("name", "ok", "rows")} if r["name"] not in ("readback_vs_K1", "raw_samples", "marker_projection") else
              ([(x["joint"], x["readback"], x["diff"]) for x in r["rows"]] if r["name"] == "readback_vs_K1" else ([(x["joint"], x.get("raw_sd_deg"), x.get("int_vs_raw_consistent")) for x in r["rows"]] if r["name"] == "raw_samples" else [(x["name"], x["edge_margin_px"], x["center_px_depth_mm"]) for x in r["rows"]])))
    if a.out:
        Path(a.out).write_text(json.dumps(rec, ensure_ascii=False, indent=1) + "\n", encoding="utf-8")
    print("출발 전 공통 점검:", "통과" if ok else "실패(팔 이동 없이 중단)")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
