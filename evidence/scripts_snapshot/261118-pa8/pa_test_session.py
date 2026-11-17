#!/usr/bin/env python3
"""P-A 시험 오케스트레이터 v2(2회차 구조, 플랜 §6 개정 2, 새 데이터에만 적용). Mac에서 실행, 사람 입회, 로봇 명령 없음, p620 전용 컨테이너·Gemini만.
순서: A1, A2(36 s 도구 기준 사진 앞 2장) → P-A 기동 #1(확인용 snap → probe 시작 → snap 01~10 → 구동 20분까지 대기 → probe 끝 → stop) → P-A 기동 #2(확인용 snap → snap 11~20 → stop)
 → B1, B2(기준 사진 뒤 2장) → stop 뒤 snap 실패 확인 → **자동 종료 시험(마지막, 고정)**: MAXSEC=45 start → 첫 polling은 launch+47 s 이후, 상한 60 s → 드라이버가 꺼진 것을 확인한 뒤 snap 실패 확인 → 평가(scripts/pa_eval.py).
시험 장치 점검(평가 지표 아님, 사전 등록): 기동 #1·#2·자동 종료 시험 start 직후 status '드라이버 프로세스 있음', 확인용 snap(기동마다 1회, 평가 제외), 기준 사진 A1·A2가 실패하면 즉시 중단. snap 3회 연속 실패도 중단.
사용: python3 scripts/pa_test_session.py [--dry]      (--dry: 계획만 출력, p620 접속·쓰기 없음)
모든 질문 직전 입력 버퍼를 비운다(P-B 입력 버퍼 이슈의 교훈). 문턱은 pa_eval.py의 사전 등록 상수이며 여기서 바꾸지 않는다. 1회차(v1) 구조는 스냅샷 261117-pa2·pa5의 코드로 재현한다."""
import json, os, shutil, subprocess, sys, time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
D = ROOT / "data" / "g04" / "pa"
PY = os.environ.get("P1_CHECK_PY", str(ROOT / ".venv" / "bin" / "python"))
SSH = ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=8", "p620"]
WARMUP, SNAP_GAP_S, HOLD_S = 12, 5.0, 1200
MAXSEC_AUTOEXIT = 45                                         # 자동 종료 시험의 MAXSEC(드라이버 timeout)
AUTOEXIT_FIRST_POLL_S = MAXSEC_AUTOEXIT - WARMUP + 2         # start는 launch 후 적어도 WARMUP s 뒤 반환 → 첫 polling은 늘 launch+MAXSEC+2 s 이후
AUTOEXIT_POLL_BOUND_S = 60                                   # launch+MAXSEC+15+10 s 최악 경우를 덮는다
USB_PAT = "disconnect\\|device lost\\|lost connection"
if os.environ.get("PA_SESSION_FAST_FOR_TEST") == "1":      # 가짜 ssh로 오케스트레이션 흐름만 시험할 때의 시간 단축(문턱·판정 규칙은 바뀌지 않는다). 실제 시험에서는 쓰지 않는다
    SNAP_GAP_S, HOLD_S, AUTOEXIT_FIRST_POLL_S, AUTOEXIT_POLL_BOUND_S = 0.05, 3, 1.5, 6
    print("[FAST(시험용) 모드: 시간 단축, 가짜 환경 전용]")


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


def driver_alive(): return "드라이버 프로세스 있음" in pa_driver("status").stdout


def containers_left():
    r = run(SSH + ["docker ps -a --format '{{.Names}}' | grep -c '^dofbot-g01-pa$' || true"])
    try: return int(r.stdout.strip().splitlines()[-1])
    except Exception: return None


def snap(name): return run(["bash", str(ROOT / "scripts" / "pa_snap_mac.sh"), name])


def base_photo(S, tag):
    name = "%s_base_%s" % (S, tag); orig = "pa_%s" % name
    r = run(["bash", str(ROOT / "scripts" / "p1_m3_gemini_photo.sh"), orig]); print(r.stdout[-200:])
    p = ROOT / "data" / "g04" / (orig + ".png")
    if not p.exists(): return False
    shutil.copy(p, D / (name + ".color.png"))
    run(["scp", "-q", "-o", "BatchMode=yes", "p620:dofbot-g01-work/%s/depth_mm.npy" % orig, str(D / (name + ".depth_mm.npy"))])
    run(["scp", "-q", "-o", "BatchMode=yes", "p620:dofbot-g01-work/%s/probe.json" % orig, str(D / (name + ".probe.json"))])      # 참고(기동의 449프레임 평균 밝기)
    (D / (name + ".detect.json")).write_text(run([PY, "-I", str(ROOT / "scripts" / "pa_detect.py"), str(D / (name + ".color.png"))]).stdout)
    return True


def probe(S, tag):
    n = "%s_probe_%s" % (S, tag)
    run(SSH + ["docker exec dofbot-g01-pa bash -lc 'set +u; source /opt/ros/humble/setup.bash; source /opt/cad-less-install/setup.bash; set -u; cd /work && python3 g01_gemini_probe.py /work/pa/%s 15 camera'" % n])
    run(["scp", "-q", "-o", "BatchMode=yes", "p620:dofbot-g01-work/pa/%s/probe.json" % n, str(D / (n + ".json"))])


def usb_counts(start_epoch):
    """이번 세션 시작 epoch 이후의 driver_<epoch>.log만, 원격에서 파일별로 센다(scp glob에 기대지 않는다)."""
    cmd = ("for f in dofbot-g01-work/pa/driver_*.log; do e=${f##*driver_}; e=${e%%.log}; [ \"$e\" -ge %d ] 2>/dev/null && echo \"$f $(grep -ci '%s' \"$f\")\"; done || true" % (int(start_epoch), USB_PAT))
    out = {}
    for line in run(SSH + [cmd]).stdout.splitlines():
        t = line.split()
        if len(t) == 2 and t[1].isdigit(): out[t[0].rsplit("/", 1)[-1]] = int(t[1])
    return out


def snap_series(S, first, last, cycles):
    fails = 0
    for i in range(first, last + 1):
        n = "%s_snap%02d" % (S, i); t0 = time.time(); r = snap(n); ok = r.returncode == 0; fails = 0 if ok else fails + 1
        print(" ", r.stdout.strip()[:110] or ("실패 " + r.stderr[-100:]))
        if fails >= 3: print("snap 3회 연속 실패 → 중단"); cycles["aborted"] = "snap_fail"; return False
        time.sleep(max(0.0, SNAP_GAP_S - (time.time() - t0)))
    return True


def start_p_a(S, label, cycles):
    """P-A 기동: start → 시험 장치 점검(status '있음') → 확인용 snap(평가 제외). 실패하면 사유를 cycles에 남기고 False."""
    r = pa_driver("start", WARMUP=WARMUP); print(r.stdout[-120:].strip()); cycles["start_rc_" + label] = r.returncode
    if r.returncode: print("start 실패:", r.stdout, r.stderr); cycles["aborted"] = "start_" + label; return False
    alive = driver_alive(); cycles["status_alive_" + label] = alive
    if not alive: print("시험 장치 점검 실패: start 직후 status가 '드라이버 프로세스 있음'을 내지 않았다 → 즉시 중단(패턴 결함 의심)"); cycles["aborted"] = "status_pattern_" + label; return False
    r = snap("%s_warmcheck%s" % (S, label[-1]))
    if r.returncode != 0 or not r.stdout.startswith("OK"): print("확인용 snap 실패 → 즉시 중단:", r.stdout[-200:], r.stderr[-200:]); cycles["aborted"] = "warmcheck_" + label; return False
    return True


def main():
    dry = "--dry" in sys.argv
    S = "pa_" + time.strftime("%y%m%d-%H%M%S"); D.mkdir(parents=True, exist_ok=True)
    plan = ["① 36 s 도구로 기준 사진 A1·A2(독립 기동 2회)", "② P-A 기동 #1: start(워밍업 %d s) → status '있음' 점검 → 확인용 snap(평가 제외)" % WARMUP, "③ probe 15 s(M4 시작) → snap 01~10(간격 ≥ %.0f s)" % SNAP_GAP_S,
            "④ 구동 %d s까지 대기 → probe 15 s(M4 끝) → stop(컨테이너 0개 확인)" % HOLD_S, "⑤ P-A 기동 #2: start → '있음' 점검 → 확인용 snap → snap 11~20 → stop", "⑥ 36 s 도구로 기준 사진 B1·B2",
            "⑦ stop 뒤 snap 실패 확인", "⑧ 자동 종료 시험(마지막): MAXSEC=%d start → '있음' 점검 → 첫 polling launch+%d s 이후(상한 %d s) → 꺼진 뒤 snap 실패 확인 → stop" % (MAXSEC_AUTOEXIT, MAXSEC_AUTOEXIT + 2, AUTOEXIT_POLL_BOUND_S), "⑨ USB 끊김을 로그 3개 파일별로 원격 계수 → 평가(pa_eval.py)"]
    print("P-A 시험 계획 v2(세션 %s, 약 35분, 최대 50분):" % S); [print("  " + p) for p in plan]
    if dry: print("--dry: 접속·쓰기 없음. 끝."); return 0
    print("사전 확인:\n  1. p620 전원 ON, Gemini USB 연결, 다른 프로그램이 Gemini를 쓰지 않는다\n  2. **장면은 정지, 마커 id 7이 보이고 사람·손은 화면 밖**, 조명 변화 없음(약 35~50분)\n  3. 로봇 명령은 없다(DOFBOT 전원·자세는 그대로 둔다)")
    if not ask("위 3개를 모두 확인했습니까? (질문은 이 한 번입니다)"): print("중단: 아무것도 실행하지 않았다."); return 1
    r = run(["scp", "-q", "-o", "BatchMode=yes", str(ROOT / "scripts" / "pa_snap.py"), "p620:dofbot-g01-work/pa_snap.py"])
    if r.returncode: print("pa_snap.py를 p620로 보내지 못했다(전원·접속 확인):", r.stderr); return 2
    cycles = {"session": S}; left = []; session_start = time.time()
    if os.environ.get("PA_SESSION_FAST_FOR_TEST") == "1": cycles["fast_mode"] = True      # 가짜 환경 세션은 평가에서 판정 없음으로 처리된다
    try:
        for tag in ("a1", "a2"):
            print("① 기준 사진 %s(약 40 s)" % tag.upper())
            if not base_photo(S, tag): print("기준 사진 %s 실패 → 중단" % tag.upper()); cycles["aborted"] = "base_" + tag; return 4
        print("② P-A 기동 #1")
        if not start_p_a(S, "start1", cycles): return 3
        start = time.time()
        print("③ probe(15 s) → snap 01~10"); probe(S, "start")
        if not snap_series(S, 1, 10, cycles): return 6
        while time.time() - start < HOLD_S: print("  구동 %.0f / %d s ..." % (time.time() - start, HOLD_S), end="\r"); time.sleep(10)
        print("\n④ probe(15 s)"); probe(S, "end")
        print("   stop"); pa_driver("stop"); left.append(containers_left())
        print("⑤ P-A 기동 #2")
        if not start_p_a(S, "start2", cycles): return 3
        if not snap_series(S, 11, 20, cycles): return 6
        pa_driver("stop"); left.append(containers_left())
        for tag in ("b1", "b2"):
            print("⑥ 기준 사진 %s(약 40 s)" % tag.upper()); base_photo(S, tag)
        print("⑦ stop 뒤 snap(실패해야 한다)"); r = snap("%s_after_stop" % S); cycles["snap_after_stop_failed"] = bool(r.returncode != 0 and not (D / ("%s_after_stop.color.png" % S)).exists())
        print("⑧ 자동 종료 시험(MAXSEC=%d)" % MAXSEC_AUTOEXIT)
        r = pa_driver("start", WARMUP=WARMUP, MAXSEC=MAXSEC_AUTOEXIT); t3 = time.time(); cycles["start_rc_autoexit"] = r.returncode
        cycles["status_alive_autoexit"] = driver_alive()
        if r.returncode or not cycles["status_alive_autoexit"]:
            print("시험 장치 점검 실패(자동 종료 시험 start): 중단"); cycles["aborted"] = "status_pattern_autoexit"; pa_driver("stop"); left.append(containers_left()); cycles["containers_left_after_stop"] = left; return 7
        exited, last = False, ""
        time.sleep(AUTOEXIT_FIRST_POLL_S); cycles["auto_exit_first_poll_after_start_s"] = round(time.time() - t3, 1); cycles["auto_exit_poll_bound_after_start_s"] = AUTOEXIT_POLL_BOUND_S
        while time.time() - t3 < AUTOEXIT_POLL_BOUND_S:
            last = pa_driver("status").stdout
            if "드라이버 프로세스 없음" in last or "컨테이너 종료됨" in last: exited = True; break
            time.sleep(3)
        cycles["auto_exit_polled_s"] = round(time.time() - t3, 1); cycles["auto_exit_status"] = last[-300:]; cycles["auto_exit_driver_gone"] = exited
        r = snap("%s_after_autoexit" % S); cycles["auto_exit_snap_failed"] = bool(exited and r.returncode != 0 and not (D / ("%s_after_autoexit.color.png" % S)).exists())
        pa_driver("stop"); left.append(containers_left()); cycles["containers_left_after_stop"] = left
        cycles["usb_counts"] = usb_counts(session_start)
    except KeyboardInterrupt:
        print("\n중단(Ctrl-C): 드라이버를 정리합니다.")
    finally:
        pa_driver("stop"); (D / ("%s_cycles.json" % S)).write_text(json.dumps(cycles, ensure_ascii=False, indent=2) + "\n")
        run(["scp", "-q", "-o", "BatchMode=yes", "p620:dofbot-g01-work/pa/driver_*.log", str(D)])      # 증거 보존용(판정은 원격 파일별 계수)
        run(["ssh", "-o", "ControlPath=/tmp/pa-ssh-%C", "-O", "exit", "p620"])
    print(run([PY, "-I", str(ROOT / "scripts" / "pa_eval.py"), S]).stdout); print("세션:", S); return 0


if __name__ == "__main__": sys.exit(main())
