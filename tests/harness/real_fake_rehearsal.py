#!/usr/bin/env python3
"""`g01_lift_run.sh --real` 전체 흐름의 가짜 리허설(로봇·카메라·네트워크 없음). 래퍼의 `--real` 전용 코드(마커 관문, REFINFO, P-A 기동·기준 스냅·기준 JSON, 사진 다리 CLI ↔ 러너 파일 채널, 분류기, R8)를 실제로 돌린다.
대역: ssh·scp(임시 'Jetson' 폴더로 매핑), afplay, p620 P-A 상태·기동·정지, `p1_m3_gemini_photo.sh`·`pa_snap_mac.sh`(저장된 P-A 정지 사진을 복사), 러너(`--real` → `--fake`, 같은 코드), 간격 게이트 두 개(느림, 이미 `--plan-check`로 통과 확인).
임시 루트에서 래퍼 복사본을 실행하므로 저장소는 바뀌지 않는다. 사용: .venv/bin/python -I tests/harness/real_fake_rehearsal.py [--keep]   (약 5~8분: 러너가 가짜 모드라도 실시간으로 기다린다)"""
import os, pty, re, select, shutil, signal, subprocess, sys, tempfile, time
from pathlib import Path
ROOT = Path(__file__).resolve().parents[2]; VENV = ROOT / ".venv" / "bin" / "python"
SRC_PA = ROOT / "data" / "g04" / "pa"; P3 = "pa_261118-090521_"
IMG = {"ref": "snap01", "A": "snap02", "B": "snap10", "C": "snap11", "align": "snap20", "post": "snap20"}

SSH = r'''#!/usr/bin/env python3
import os, subprocess, sys
J = os.environ["REH_JET"]; args = sys.argv[1:]; log = open(os.environ["REH_LOG"], "a")
cmd = args[-1]; host = [a for a in args[:-1] if a == "p620" or a.startswith("jetson@")]; host = host[-1] if host else ""
log.write("ssh %s | %s\n" % (host, cmd[:160])); log.flush()
def local(c, **k): return subprocess.call(["bash", "-c", c], cwd=J, env=dict(os.environ, HOME=J), **k)
if host == "p620":
    ef = os.path.join(J, ".pa_epoch")
    if "ACTION=status" in cmd: print("컨테이너 실행 중\n드라이버 프로세스 있음\nstarted_epoch %s\n경과 5 s" % open(ef).read().strip())
    elif "ACTION=start" in cmd:
        import time; open(ef, "w").write("%.3f" % (time.time() - 5)); print("시작 완료: 워밍업 12 s, 최대 가동 1800 s")
    else: print("컨테이너 정리: 0개 남음")
    sys.exit(0)
if "import Arm_Lib" in cmd and "Arm_Device()" not in cmd: print("Arm_Lib_ok"); sys.exit(0)
if cmd.startswith("ps -eo"): sys.exit(0)
if "Arm_serial_servo_read" in cmd: print("88,92,92,91,90,177"); sys.exit(0)
if "g01_p2k_test.py" in cmd and "--plan" in cmd:     # 러너: --real → --fake(같은 코드), Jetson 작업 폴더에서 실행
    c = cmd.replace("--real", "--fake --fake-ceil 6:146 --fake-hyst 1.9,0.27"); rc = local(c)
    import json; rp = os.path.join(J, "dofbot-g01_5b4", "result.json")
    if os.path.exists(rp): j = json.load(open(rp)); j["mode"] = "real"; json.dump(j, open(rp, "w"), ensure_ascii=False)   # 리허설 대역: 가짜 실행 결과를 real로 표시(분류기·R8·래퍼 확인 경로를 지나가게)
    sys.exit(rc)
sys.exit(local(cmd))
'''
SCP = r'''#!/usr/bin/env python3
import os, shutil, sys
J = os.environ["REH_JET"]; a = [x for x in sys.argv[1:] if not x.startswith("-") and x not in ("BatchMode=yes",)]
a = [x for x in a if "=" not in x or ":" in x]; *srcs, dst = a
def m(p): return os.path.join(J, p.split(":", 1)[1]) if ":" in p and not p.startswith("/") else p
open(os.environ["REH_LOG"], "a").write("scp %s -> %s\n" % (srcs, dst))
for s in srcs:
    d = m(dst); s2 = m(s)
    if os.path.isdir(d): d = os.path.join(d, os.path.basename(s2))
    os.makedirs(os.path.dirname(d), exist_ok=True); shutil.copy(s2, d)
'''
GEM = '#!/usr/bin/env bash\nset -e\nROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"\ncp "${ROOT}/data/g04/lift_markerchk_1.png" "${ROOT}/data/g04/${1}.png"\necho "저장: ${1}"\n'
SNAP = r'''#!/usr/bin/env bash
set -e
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"; N="$1"; D="${ROOT}/data/g04/pa"; mkdir -p "${D}"
[ ! -e "${D}/${N}.color.png" ] || { echo "이미 있음: ${N}"; exit 2; }
case "${N}" in *_ref) S=%(ref)s;; *_A|*_A_r*) S=%(A)s;; *_B|*_B_r*) S=%(B)s;; *_C|*_C_r*) S=%(C)s;; *_align) S=%(align)s;; *) S=%(post)s;; esac
cp "${ROOT}/pa_src/%(p3)s${S}.color.png" "${D}/${N}.color.png"
"${ROOT}/.venv/bin/python" -I -c "import sys,json,cv2; sys.path.insert(0,'${ROOT}/scripts'); import pa_detect; json.dump(pa_detect.detect(cv2.imread('${D}/${N}.color.png')), open('${D}/${N}.detect.json','w'))"
echo "OK ${N}"
''' % dict(IMG, p3=P3)
SHIM = '#!/bin/bash\ncase "$*" in *p1_plan_check.py*|*run2_contact_clearance*) echo "(리허설 대역: 간격 게이트 생략, --plan-check로 별도 확인)"; exit 0;; esac\nexec "%s" "$@"\n' % VENV


def main():
    keep = "--keep" in sys.argv; no_on = [x for x in (sys.argv[sys.argv.index("--no-on") + 1].split(",") if "--no-on" in sys.argv else []) if x]   # --no-on "패턴1,패턴2": 그 질문에 N으로 답한다(N 경로 확인)
    T = Path(tempfile.mkdtemp(prefix="reh_")); JET = T / "jetson"; JET.mkdir(); bindir = T / "bin"; bindir.mkdir()
    (T / "scripts").mkdir(); (T / "evidence" / "G01" / "raw").mkdir(parents=True); (T / "data" / "g04" / "pa").mkdir(parents=True)
    for f in (ROOT / "scripts").iterdir():
        if f.name in ("p1_m3_gemini_photo.sh", "pa_snap_mac.sh", "g01_lift_run.sh") or f.is_dir(): continue
        os.symlink(f, T / "scripts" / f.name)
    shutil.copy(ROOT / "scripts" / "g01_lift_run.sh", T / "scripts" / "g01_lift_run.sh")
    (T / "scripts" / "p1_m3_gemini_photo.sh").write_text(GEM); (T / "scripts" / "pa_snap_mac.sh").write_text(SNAP)
    for n in ("src",): os.symlink(ROOT / n, T / n)
    os.symlink(ROOT / ".venv", T / ".venv"); os.symlink(ROOT / "data" / "refs", T / "data" / "refs"); os.symlink(ROOT / "evidence" / "G06", T / "evidence" / "G06"); os.symlink(SRC_PA, T / "pa_src_dir") if False else None
    (T / "pa_src").mkdir()
    for s in IMG.values(): shutil.copy(SRC_PA / (P3 + s + ".color.png"), T / "pa_src" / (P3 + s + ".color.png"))
    shutil.copy(ROOT / "data" / "g04" / "lift_markerchk_1.png", T / "data" / "g04" / "lift_markerchk_1.png")
    for n, body in (("ssh", SSH), ("scp", SCP), ("python3", SHIM.replace("%s", "")), ("afplay", "#!/bin/bash\nexit 0\n"), ("pyshim", SHIM)): (bindir / n).write_text(body if n not in ("python3",) else SHIM)
    for p in bindir.iterdir(): p.chmod(0o755)
    (T / "scripts" / "pa_snap_mac.sh").chmod(0o755); (T / "scripts" / "p1_m3_gemini_photo.sh").chmod(0o755)
    log = T / "stub_calls.log"
    env = dict(os.environ, PATH="%s:%s" % (bindir, os.environ["PATH"]), JETSON_HOST="jetson@stub", REH_JET=str(JET), REH_LOG=str(log), P1_CHECK_PY=str(bindir / "pyshim"), TERM="dumb")
    pid, fd = pty.fork()
    if pid == 0:
        os.chdir(T); os.execvpe("bash", ["bash", "scripts/g01_lift_run.sh", "--real"], env)
    out = b""; t0 = time.time(); answered = 0; status = None
    while time.time() - t0 < 1500:
        r, _, _ = select.select([fd], [], [], 1.0)
        if r:
            try: chunk = os.read(fd, 65536)
            except OSError: break
            if not chunk: break
            out += chunk; sys.stdout.write(chunk.decode("utf-8", "replace")); sys.stdout.flush()
            tail = out[-300:].decode("utf-8", "replace")
            if re.search(r"\[y/N\]\s*$", tail) and out.count(b"[y/N]") > answered:
                answered = out.count(b"[y/N]"); ans = b"n\r" if any(p in out[-1500:].decode("utf-8", "replace") for p in no_on) else b"y\r"; os.write(fd, ans)   # 긴 질문은 마지막 300자 밖에 패턴이 있어 1500자를 본다
        p, st = os.waitpid(pid, os.WNOHANG)
        if p: status = st; break
    if status is None:                                  # pty가 먼저 닫혀 읽기가 끝난 경우: 자식 종료를 잠시 기다린다
        for _ in range(100):
            p, st = os.waitpid(pid, os.WNOHANG)
            if p: status = st; break
            time.sleep(0.1)
    if status is None:
        try: os.kill(pid, signal.SIGKILL)
        except Exception: pass
        print("\n[리허설] 시간 초과"); rc = 99
    else: rc = os.waitstatus_to_exitcode(status); print("\n[리허설] 래퍼 종료 코드", rc)
    text = out.decode("utf-8", "replace")
    checks = {"종료 코드 0": rc == 0, "마커 관문 통과": "관문 <= 1.0" in text and "마커 관문 실패" not in text, "기준 결과 전달": "기준 결과 전달 실패" not in text, "정렬 보고 문구": "[P-A 정렬 보고]" in text,
              "사진 A 전달(관문)": "사진 A 전달" in text, "사진 B 전달": "사진 B 전달" in text, "사진 C 전달": "사진 C 전달" in text, "기록 확인": "기록 확인:" in text, "분류기": "분류: 시도" in text, "R8": "R8 판정" in text,
              "낙하 뒤 질문": "블록이 낙하했습니다" in text, "마지막 확인": "마지막 확인 1회" in text}
    if no_on:
        import glob, json; print("\n=== N 경로 요약(--no-on %s) ===" % no_on); print("질문 순서:", [m.group(0)[:36] for m in re.finditer(r"(위 8개를 모두 확인했습니까|정렬 확인이 모두 맞습니까|블록이 낙하했습니다|J6가 시작값[^?]*\?|팔이 시작 자세가 아닐 수[^?]*\?|팔이 K1로 돌아왔습니다\. J6를|마지막 확인 1회|기준 사진 전 확인)", text)])
        for f in glob.glob(str(T / "evidence" / "G01" / "raw" / "g01_5b4_s4233_result-real-*.json")):
            j = json.load(open(f)); print("ended", j.get("ended"), "| returned_to_start", j.get("returned_to_start"), "| abort", (j.get("abort_reason") or "")[:60], "| J6 로그", {k: [(l[1], l[4] if len(l) > 4 else "") for l in v if l[0] == "write"] for k, v in (j.get("j6_adapter_logs") or {}).items()})
        print("복귀 안내 줄:", [l for l in text.splitlines() if "복귀 뒤 사진" in l])
        print("임시 루트:", T); sys.exit(0)
    print("\n=== 리허설 점검 ===")
    for k, v in checks.items(): print(("통과 " if v else "실패 ") + k)
    print("임시 루트:", T, "(--keep이면 보존)"); ok = all(checks.values())
    if not keep and ok: shutil.rmtree(T, ignore_errors=True)
    sys.exit(0 if ok else 1)


if __name__ == "__main__": main()
