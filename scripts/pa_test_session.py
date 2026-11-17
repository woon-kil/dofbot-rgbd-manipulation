#!/usr/bin/env python3
"""P-A 시험 오케스트레이터(Mac에서 실행, 사람 입회, 로봇 명령 없음, p620 전용 컨테이너·Gemini만). 플랜 §3 순서:
 ① 기존 36 s 도구로 기준 사진 A → ② pa_driver start → ③ 15 s probe(M4 시작) → ④ snap 20회(간격 ≥ 5 s) → ⑤ 구동 20분까지 대기 → ⑥ 15 s probe(M4 끝) → ⑦ stop(컨테이너 0개 확인) → ⑧ 기준 사진 B
 → ⑨ stop 뒤 snap 실패 확인 → ⑩ start/stop 2회 더(두 번째는 MAXSEC=45 자동 종료 시험: 드라이버가 꺼진 뒤 snap 실패, 컨테이너 종료 확인) → 평가(scripts/pa_eval.py).
확인용 snap(<S>_warmcheck, 평가 제외, 사전 등록)이나 기준 사진 A가 실패하면 즉시 중단한다. 자동 종료 시험은 status를 폴링해 드라이버가 꺼진 것을 확인한 뒤 snap한다.
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


def driver_alive(): return "드라이버 프로세스 있음" in pa_driver("status").stdout     # 시험 장치 점검(평가 지표 아님, 사전 등록): start 직후 status가 살아 있는 드라이버를 '있음'으로 잡는지


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
        print("① 기준 사진 A(약 40 s)")
        if not base_photo(S, "a"): print("기준 사진 A 실패 → 중단(드라이버 시작 전)"); cycles["aborted"] = "base_a"; return 4
        print("② start"); r = pa_driver("start", WARMUP=WARMUP); print(r.stdout[-200:]);
        if r.returncode: print("start 실패:", r.stdout, r.stderr); cycles["aborted"] = "start"; return 3
        start = time.time()
        if not driver_alive(): print("시험 장치 점검 실패: start 직후 status가 '드라이버 프로세스 있음'을 내지 않았다 → 즉시 중단(패턴 결함 의심)"); cycles["aborted"] = "status_pattern"; return 7
        cycles["status_alive_main"] = True
        print("   확인용 snap 1회(평가 제외)"); r = snap("%s_warmcheck" % S)
        if r.returncode != 0 or not r.stdout.startswith("OK"): print("확인용 snap 실패 → 즉시 중단:", r.stdout[-200:], r.stderr[-200:]); cycles["aborted"] = "warmcheck"; return 5
        print("③ probe(15 s)"); probe(S, "start")
        fails = 0
        for i in range(1, 21):
            n = "%s_snap%02d" % (S, i); t0 = time.time(); r = snap(n); ok = r.returncode == 0; fails = 0 if ok else fails + 1
            print(" ", r.stdout.strip()[:110] or ("실패 " + r.stderr[-100:]))
            if fails >= 3: print("snap 3회 연속 실패 → 중단"); cycles["aborted"] = "snap_fail"; return 6
            time.sleep(max(0.0, SNAP_GAP_S - (time.time() - t0)))
        while time.time() - start < HOLD_S: print("  구동 %.0f / %d s ..." % (time.time() - start, HOLD_S), end="\r"); time.sleep(10)
        print("\n⑥ probe(15 s)"); probe(S, "end")
        pat = "disconnect\\|device lost\\|lost connection"
        cycles["usb_disconnect_lines"] = int((run(SSH + ["grep -ci '%s' dofbot-g01-work/pa/driver.log || true" % pat]).stdout.strip().splitlines() or ["-1"])[-1])
        cycles["usb_disconnect_text"] = run(SSH + ["grep -i '%s' dofbot-g01-work/pa/driver.log | head -5 | cut -c1-200 || true" % pat]).stdout
        left = []
        print("⑦ stop"); pa_driver("stop"); left.append(containers_left())
        print("⑧ 기준 사진 B(약 40 s)"); base_photo(S, "b")
        print("⑨ stop 뒤 snap(실패해야 한다)"); r = snap("%s_after_stop" % S); cycles["snap_after_stop_failed"] = bool(r.returncode != 0 and not (D / ("%s_after_stop.color.png" % S)).exists())
        print("⑩ start/stop 2회째(정상)"); r = pa_driver("start", WARMUP=WARMUP); cycles["status_alive_cycle2"] = driver_alive(); pa_driver("stop"); left.append(containers_left())
        if not cycles["status_alive_cycle2"]: print("시험 장치 점검 실패(2회째): status가 '있음'을 내지 않음 → 중단"); cycles["aborted"] = "status_pattern_cycle2"; cycles["containers_left_after_stop"] = left; return 7
        print("   3회째: MAXSEC=45 자동 종료 시험(타임아웃 시각 이후 status 폴링으로 드라이버가 꺼진 것을 확인한 뒤 snap)"); r = pa_driver("start", WARMUP=WARMUP, MAXSEC=45); t3 = time.time(); cycles["cycle3_start_rc"] = r.returncode; cycles["status_alive_cycle3"] = driver_alive()
        if not cycles["status_alive_cycle3"]: print("시험 장치 점검 실패(3회째): status가 '있음'을 내지 않음 → 자동 종료 시험 중단"); cycles["aborted"] = "status_pattern_cycle3"; pa_driver("stop"); left.append(containers_left()); cycles["containers_left_after_stop"] = left; return 7
        exited, last = False, ""
        time.sleep(45 - WARMUP + 2); cycles["auto_exit_first_poll_after_start_s"] = round(time.time() - t3, 1)   # start는 launch 후 적어도 WARMUP s가 지나야 반환하므로 첫 polling은 늘 launch+47 s 이후(타임아웃 45 s 뒤). 첫 세션: 패턴 결함으로 0.3 s에 거짓 '꺼짐'
        cycles["auto_exit_poll_bound_after_start_s"] = 60
        while time.time() - t3 < 60:      # launch+70 s(45+15+10) 최악 경우를 덮는다       # launch 기준 45+15+10 s 안에 꺼져야 한다(start 반환 시각은 launch보다 약 15 s 늦다)
            last = pa_driver("status").stdout
            if "드라이버 프로세스 없음" in last or "컨테이너 종료됨" in last: exited = True; break
            time.sleep(3)
        cycles["auto_exit_polled_s"] = round(time.time() - t3, 1); cycles["auto_exit_status"] = last[-300:]; cycles["auto_exit_driver_gone"] = exited
        r = snap("%s_after_autoexit" % S); cycles["auto_exit_snap_failed"] = bool(exited and r.returncode != 0 and not (D / ("%s_after_autoexit.color.png" % S)).exists())
        pa_driver("stop"); left.append(containers_left()); cycles["containers_left_after_stop"] = left
    except KeyboardInterrupt:
        print("\n중단(Ctrl-C): 드라이버를 정리합니다."); 
    finally:
        pa_driver("stop"); (D / ("%s_cycles.json" % S)).write_text(json.dumps(cycles, ensure_ascii=False, indent=2) + "\n")
        run(["scp", "-q", "-o", "BatchMode=yes", "p620:dofbot-g01-work/pa/driver_*.log", str(D)])      # start마다 따로 남긴 드라이버 로그(원인 분석 증거)
        run(["ssh", "-o", "ControlPath=/tmp/pa-ssh-%C", "-O", "exit", "p620"])      # 재사용 연결 정리
    print(run([PY, "-I", str(ROOT / "scripts" / "pa_eval.py"), S]).stdout); print("세션:", S); return 0


if __name__ == "__main__": sys.exit(main())
