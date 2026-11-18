#!/usr/bin/env python3
"""`--real` 직전 스냅샷(승인 요청 문서 §7). 코드 변경 없는 커밋에서 실행한다. `evidence/scripts_snapshot/<이름>/`에
 - `scripts/`의 실행 파일 복사본 + `MANIFEST.sha256`(기존 규칙: `해시  이름`, 이름은 `scripts/<이름>`),
 - `scripts/` 밖의 코드·입력 데이터는 `MANIFEST-extra.sha256`(저장소 상대 경로 `해시  경로`)에 해시를 남기고 작은 코드 파일은 `extra/<경로>`로 복사(메시·URDF 같은 큰 데이터는 해시만),
 - `SNAPSHOT-INFO.txt`(HEAD, 시각, 대상 경로의 추적 파일 변경 여부, `shasum -c` 결과)를 만든다.
추적 파일이 대상 경로에서 수정 상태이거나 목록의 파일이 없으면 실패한다. 스냅샷 뒤 코드를 바꾸면 이 스냅샷은 무효다. 검증: 저장소 루트에서 `shasum -a 256 -c evidence/scripts_snapshot/<이름>/MANIFEST-extra.sha256`.
사용: python3 tools/make_lift_snapshot.py <이름>"""
import hashlib, os, shutil, subprocess, sys, datetime
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = ["g01_lift_run.sh", "g01_lift_bridge.py", "g01_lift_r8.py", "g01_lift_classify.py", "g01_lift_marker_ref.py", "g01_lift_params.py", "g01_lift_logic.py", "g01_lift_hook.py", "g01_s42_plan_lift.py",
           "g01_s42_plan_p2k.py", "g01_p2k_test.py", "g01_5a_gripper_test.py", "g01_5b4_joint_test.py", "gripper_adapter.py", "g01_s4_2_path.py", "g01_s3_path.py", "g01_pose_generator.py", "g01_dabai_probe.py",
           "g01_dabai_expsweep.py", "g01_e0g_analyze.py", "p2k_align_report.py", "p1_m3_gemini_photo.sh", "g01_gemini_check.sh", "pa_driver.sh", "pa_snap_mac.sh", "pa_snap.py", "pa_detect.py",
           "g01_lift_dryrun.sh", "g01_lift_dryrun_stub.py", "g01_lift_dryrun_report.py"]
EXTRA_CODE = ["src/planning/p1_plan_check.py", "src/planning/collision_3a.py", "src/planning/dry_run.py", "src/planning/p1_poses.py", "src/kinematics/urdf_fk.py", "src/control/gripper_adapter.py",
              "evidence/G06/run2_contact_clearance_261117.py", "evidence/G06/p2k_analyze_261117.py", "evidence/G06/carried_block_path_261117.py", "evidence/G06/gemini_fingertip_detect_r15.py", "requirements.txt"]
EXTRA_DATA = ["evidence/G06/p2k_poses_s42.json", "evidence/G06/p2k_marker_ref.txt", "data/refs/dofbot-pro-DOFBOT_Pro-V24.urdf", "evidence/G06/d3_height_model.json", "evidence/G01/s3_path_fk.json"] + \
             ["data/refs/dofbot_meshes/%s.STL" % n for n in ("base_link", "Arm1_Link", "Arm2_Link", "Arm3_Link", "Arm4_Link", "DaBai_DCW2_Link", "Arm5_Link")]
OPTIONAL = {"evidence/G06/d3_height_model.json", "evidence/G01/s3_path_fk.json"}   # 생성기가 열 수 있는 입력(소스 grep): 있으면 포함, 없으면 기록


def check_name(name):
    """스냅샷 이름 검증: `-`로 시작(옵션처럼 보이는 `--help`)하거나 비어 있거나 `/`·공백·`..`를 포함하면 거부한다(폴더를 만들기 전에)."""
    import re
    if not name or name.startswith("-") or ".." in name or not re.fullmatch(r"[0-9A-Za-z._][0-9A-Za-z._-]*", name): raise SystemExit("스냅샷 이름이 올바르지 않다: %r (영문·숫자·._- 만, `-`로 시작·`..`·`/`·공백 금지)" % (name,))
    return name


def sha(p): return hashlib.sha256(p.read_bytes()).hexdigest()


def git(*a): return subprocess.run(["git", *a], cwd=ROOT, capture_output=True, text=True).stdout


def main():
    if len(sys.argv) != 2: sys.exit(__doc__)
    name = check_name(sys.argv[1]); out = ROOT / "evidence" / "scripts_snapshot" / name
    if out.exists(): sys.exit("이미 있다(기존 스냅샷은 지우지 않는다): %s" % out)
    dirty = [l for l in git("status", "--porcelain", "--untracked-files=no", "--", "scripts", "src", "evidence/G06", "requirements.txt", "tools").splitlines() if l.strip()]
    if dirty: sys.exit("대상 경로에 수정된 추적 파일이 있다(커밋 뒤 다시): " + "; ".join(dirty))
    miss = [f for f in SCRIPTS if not (ROOT / "scripts" / f).exists()] + [f for f in EXTRA_CODE if not (ROOT / f).exists()] + [f for f in EXTRA_DATA if f not in OPTIONAL and not (ROOT / f).exists()]
    if miss: sys.exit("없는 파일: " + ", ".join(miss))
    out.mkdir(parents=True); (out / "extra").mkdir()
    lines = []
    for f in SCRIPTS: shutil.copy2(ROOT / "scripts" / f, out / f); lines.append("%s  %s" % (sha(ROOT / "scripts" / f), f))
    (out / "MANIFEST.sha256").write_text("\n".join(lines) + "\n", encoding="utf-8")
    ex = []; absent = []
    for f in EXTRA_CODE: d = out / "extra" / f; d.parent.mkdir(parents=True, exist_ok=True); shutil.copy2(ROOT / f, d); ex.append("%s  %s" % (sha(ROOT / f), f))
    for f in EXTRA_DATA:
        p = ROOT / f
        if p.exists(): ex.append("%s  %s" % (sha(p), f))
        else: absent.append(f)
    ex += [("%s  %s" % (sha(p), p.relative_to(ROOT))) for p in sorted((ROOT / "evidence" / "G06").glob("lift_marker_ref_*.txt"))]
    (out / "MANIFEST-extra.sha256").write_text("\n".join(ex) + "\n", encoding="utf-8")
    chk = subprocess.run(["shasum", "-a", "256", "-c", str((out / "MANIFEST-extra.sha256").relative_to(ROOT))], cwd=ROOT, capture_output=True, text=True)
    chk2 = subprocess.run(["bash", "-c", "cd scripts && shasum -a 256 -c ../%s" % (out / "MANIFEST.sha256").relative_to(ROOT)], cwd=ROOT, capture_output=True, text=True)
    info = ["스냅샷 %s" % name, "시각 %s" % datetime.datetime.now().isoformat(timespec="seconds"), "HEAD %s" % git("rev-parse", "HEAD").strip(), "HEAD 제목 %s" % git("log", "-1", "--format=%s").strip(),
            "대상 경로 추적 파일 변경: 없음(있으면 실패)", "없는 선택 입력(기록): %s" % (absent or "없음"), "", "[shasum -c MANIFEST-extra.sha256, 저장소 루트에서]", chk.stdout.strip(), chk.stderr.strip(), "[shasum -c MANIFEST.sha256, scripts/에서]", chk2.stdout.strip(), chk2.stderr.strip()]
    (out / "SNAPSHOT-INFO.txt").write_text("\n".join(info) + "\n", encoding="utf-8")
    ok = chk.returncode == 0 and chk2.returncode == 0
    print("스냅샷 %s: scripts %d개, extra %d개, 검증 %s, 없는 선택 입력 %s" % (out, len(SCRIPTS), len(ex), "통과" if ok else "실패", absent or "없음")); sys.exit(0 if ok else 1)


if __name__ == "__main__": main()
