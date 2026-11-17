#!/usr/bin/env python3
"""P-A 시험 오케스트레이터(Mac에서 실행, 사람 입회, 로봇 명령 없음, p620 전용 컨테이너·Gemini만). 플랜 §3 순서:
 ① 기존 36 s 도구로 기준 사진 A → ② pa_driver start → ③ 15 s probe(M4 시작) → ④ snap 20회(간격 ≥ 5 s) → ⑤ 구동 20분까지 대기 → ⑥ 15 s probe(M4 끝) → ⑦ stop(컨테이너 0개 확인) → ⑧ 기준 사진 B
 → ⑨ stop 뒤 snap 실패 확인 → ⑩ start/stop 2회 더(두 번째는 MAXSEC=45 자동 종료 시험: 드라이버가 꺼진 뒤 snap 실패, 컨테이너 종료 확인) → 평가(scripts/pa_eval.py).
사용: python3 scripts/pa_test_session.py [--dry]      (--dry: 계획만 출력, p620 접속·쓰기 없음)
모든 질문 직전 입력 버퍼를 비운다(P-B 입력 버퍼 이슈의 교훈). 문턱은 pa_eval.py의 사전 등록 상수이며 여기서 바꾸지 않는다."""
import json, os, shutil, subprocess, sys, time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
D = ROOT / "data" / "g04" / "pa"
PY = os.environ.get("P1_CHECK_PY", str(ROOT / ".venv" / "bin" / "python"))
SSH = ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=8", "p620"]
WARMUP, SNAP_GAP_S, HOLD_S = 12, 5.0, 1200


def flush_stdin():
    try:
        import termios; termios.tcflush(sys.stdin.fileno(), termios.TCIFLUSH)
    except Exception: pass


def ask(msg):
    flush_stdin(); return input(msg + " [y/N] ").strip().lower() in ("y", "yes")


def run(cmd, **kw): return subprocess.run(cmd, capture_output=True, text=True, **kw)


def pa_driver(action, **env):
    e = " ".join("%s=%s" % (k, v) for k, v in env.items())
    return subprocess.run(SSH + ["ACTION=%s %s bash -s" % (action, e)], stdin=open(ROOT / "scripts" / "pa_driver.sh"), capture_output=True, text=True)


def containers_left():
    r = run(SSH + ["docker ps -a --format '{{.Names}}' | grep -c '^dofbot-g01-pa$' || true"])
    try: return int(r.stdout.strip().splitlines()[-1])
    except Exception: return None


def snap(name): return run(["bash", str(ROOT / "scripts" / "pa_snap_mac.sh"), name])


def base_photo(S, tag):
    name = "%s_base_%s" % (S, tag); orig = "pa_%s" % name
    r = run(["bash", str(ROOT / "scripts" / "p1_m3_gemini_photo.sh"), orig]); print(r.stdout[-300:])
    p = ROOT / "data" / "g04" / (orig + ".png")
    if not p.exists(): return False
    shutil.copy(p, D / (name + ".color.png"))
    run(["scp", "-q", "-o", "BatchMode=yes", "p620:dofbot-g01-work/%s/depth_mm.npy" % orig, str(D / (name + ".depth_mm.npy"))])
    (D / (name + ".detect.json")).write_text(run([PY, "-I", str(ROOT / "scripts" / "pa_detect.py"), str(D / (name + ".color.png"))]).stdout)
    return True


def probe(S, tag):
    n = "%s_probe_%s" % (S, tag)
    run(SSH + ["docker exec dofbot-g01-pa bash -lc 'set +u; source /opt/ros/humble/setup.bash; source /opt/cad-less-install/setup.bash; set -u; cd /work && python3 g01_gemini_probe.py /work/pa/%s 15 camera'" % n])
    run(["scp", "-q", "-o", "BatchMode=yes", "p620:dofbot-g01-work/pa/%s/probe.json" % n, str(D / (n + ".json"))])


def main():
    dry = "--dry" in sys.argv
    S = "pa_" + time.strftime("%y%m%d-%H%M%S"); D.mkdir(parents=True, exist_ok=True)
    plan = ["① 36 s 도구로 기준 사진 A", "② pa_driver start(워밍업 %d s)" % WARMUP, "③ 15 s probe(M4 시작)", "④ snap 20회(간격 ≥ %.0f s)" % SNAP_GAP_S, "⑤ 구동 %d s까지 대기" % HOLD_S,
            "⑥ 15 s probe(M4 끝)", "⑦ stop, 컨테이너 0개 확인", "⑧ 기준 사진 B", "⑨ stop 뒤 snap 실패 확인", "⑩ start/stop 2회(둘째는 MAXSEC=45 자동 종료 시험)", "평가(pa_eval.py)"]
    print("P-A 시험 계획(세션 %s, 약 30분):" % S); [print("  " + p) for p in plan]
    if dry: print("--dry: 접속·쓰기 없음. 끝."); return 0
    print("사전 확인:\n  1. p620 전원 ON, Gemini USB 연결, 다른 프로그램이 Gemini를 쓰지 않는다\n  2. **장면은 정지, 마커 id 7이 보이고 사람·손은 화면 밖**, 조명 변화 없음(약 30분)\n  3. 로봇 명령은 없다(DOFBOT 전원·자세는 그대로 둔다)")
    if not ask("위 3개를 모두 확인했습니까? (질문은 이 한 번입니다)"): print("중단: 아무것도 실행하지 않았다."); return 1
    r = run(["scp", "-q", "-o", "BatchMode=yes", str(ROOT / "scripts" / "pa_snap.py"), "p620:dofbot-g01-work/pa_snap.py"])
    if r.returncode: print("pa_snap.py를 p620로 보내지 못했다(전원·접속 확인):", r.stderr); return 2
    cycles = {"session": S}
    try:
        print("① 기준 사진 A(약 40 s)"); base_photo(S, "a")
        print("② start"); r = pa_driver("start", WARMUP=WARMUP); print(r.stdout[-200:]);
        if r.returncode: print("start 실패:", r.stdout, r.stderr); return 3
        start = time.time()
        print("③ probe(15 s)"); probe(S, "start")
        for i in range(1, 21):
            n = "%s_snap%02d" % (S, i); t0 = time.time(); r = snap(n); print(" ", r.stdout.strip()[:110] or ("실패 " + r.stderr[-100:]))
            time.sleep(max(0.0, SNAP_GAP_S - (time.time() - t0)))
        while time.time() - start < HOLD_S: print("  구동 %.0f / %d s ..." % (time.time() - start, HOLD_S), end="\r"); time.sleep(10)
        print("\n⑥ probe(15 s)"); probe(S, "end")
        cycles["usb_disconnect_lines"] = int((run(SSH + ["grep -ci 'disconnect\\|device lost\\|lost connection' dofbot-g01-work/pa/driver.log || true"]).stdout.strip().splitlines() or ["-1"])[-1])
        left = []
        print("⑦ stop"); pa_driver("stop"); left.append(containers_left())
        print("⑧ 기준 사진 B(약 40 s)"); base_photo(S, "b")
        print("⑨ stop 뒤 snap(실패해야 한다)"); r = snap("%s_after_stop" % S); cycles["snap_after_stop_failed"] = bool(r.returncode != 0 and not (D / ("%s_after_stop.color.png" % S)).exists())
        print("⑩ start/stop 2회째(정상)"); r = pa_driver("start", WARMUP=WARMUP); pa_driver("stop"); left.append(containers_left())
        print("   3회째: MAXSEC=45 자동 종료 시험"); r = pa_driver("start", WARMUP=WARMUP, MAXSEC=45); t3 = time.time()
        while time.time() - t3 < 40: time.sleep(5)
        r = snap("%s_after_autoexit" % S); cycles["auto_exit_snap_failed"] = bool(r.returncode != 0 and not (D / ("%s_after_autoexit.color.png" % S)).exists())
        pa_driver("stop"); left.append(containers_left()); cycles["containers_left_after_stop"] = left
    except KeyboardInterrupt:
        print("\n중단(Ctrl-C): 드라이버를 정리합니다."); 
    finally:
        pa_driver("stop"); (D / ("%s_cycles.json" % S)).write_text(json.dumps(cycles, ensure_ascii=False, indent=2) + "\n")
    print(run([PY, "-I", str(ROOT / "scripts" / "pa_eval.py"), S]).stdout); print("세션:", S); return 0


if __name__ == "__main__": sys.exit(main())
