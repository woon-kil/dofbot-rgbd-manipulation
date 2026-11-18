#!/usr/bin/env python3
"""들어올림 러너(세트 33)의 사진 다리(Mac 쪽, M2c 계약 플랜 §3-2). 로봇 명령 없음.
러너가 출력하는 `[사진 요청 <A|B|C> id=<토큰>]` 줄을 stdin으로 받아 P-A 스냅(p620 카메라) → 마커 검출 → (A만) S4' 관문 → Jetson에 `lift_photo_<이름>.json`을 원자적으로(임시 파일 → mv) 둔다.
결과 JSON: {request_id(요청 토큰), name, started_epoch, detect, [gate], mac: {시각들}}. 관문 계산이 예외(ParamsNotFinal·입력 없음 등)면 침묵하지 않고 gate={go: False, reason}로 쓴다(러너가 시간 초과로 기다리다 이유 없이 끝나지 않게).
사용(래퍼가 호출): ssh ... | tee >(python3 g01_lift_bridge.py --run-id ID --ref ref.json --host H --rd RD --name-prefix lift_ID)"""
import argparse, json, os, re, subprocess, sys, time
from concurrent.futures import ThreadPoolExecutor
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

ALIGN_MARK = "[정렬 확인]"
ALIGN_FAIL = "\n[정렬 보고 없음] 보고를 만들지 못했다(%s) → 러너 질문에 N(하강 없이 복귀)\n"
ALIGN_NOTE = "※ 참고 수치: P-A 스냅 기반, 36 s 도구와 동등성 미검증. 허용값 |d_x| ≤ 3 mm, 캡 옆 여유 ≥ 6 mm, 마커 관문 ≤ 1.0 px. 보고가 없거나 NG면 N."
ALIGN_TIMEOUT_S = 15   # 보고 계산이 멈춰도 뒤의 사진 A 처리를 막지 않도록 하강 시간보다 훨씬 짧게
REQ_RE = re.compile(r"\[사진 요청 ([ABC]) id=(\S+?)(?: q=[-0-9.,]+)?\]")


def parse_request(line):
    """러너 출력 한 줄에서 (이름, 토큰) 또는 None."""
    m = REQ_RE.search(line)
    return (m.group(1), m.group(2)) if m else None


def dx_block_mm(ref_detect, a_detect, root=None):
    """Δx_block(프로젝트 x, mm) = x(A 마커) − x(기준 마커), 사전 등록 `p2k_analyze_261117.inv_P`(높이 0.030 m 평면 역투영)로 환산.
    부호 규약 확인(2026-11-18): Run 4 기준·복귀 뒤 사진에서 −1.93 mm가 재현된다(Run 4 보고서 §3의 관측 −1.93과 일치, tests/test_lift_bridge.py). 프로젝트 x는 FK x와 반대 부호지만 이 함수 안에서 일관되게 프로젝트 x만 쓴다."""
    PA = load_pa(root)
    xr = PA.inv_P(ref_detect["center"][0], ref_detect["center"][1], 0.030)[0]; xa = PA.inv_P(a_detect["center"][0], a_detect["center"][1], 0.030)[0]
    return float(xa - xr)


_PA = None


def load_pa(root=None):
    """사전 등록 분석 모듈을 한 번만 읽는다(메시·FK 적재가 있어 사진 요청 경로 안에서 매번 읽으면 3.0 s 창을 먹는다). 다리 시작 때 미리 부른다."""
    global _PA
    if _PA is None:
        import importlib.util
        root = root or os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        for sub in ("scripts", os.path.join("evidence", "G06"), "src"): sys.path.insert(0, os.path.join(root, sub))
        cwd = os.getcwd(); os.chdir(root)                                   # 모듈이 상대 경로(data/refs)를 읽는다
        try:
            spec = importlib.util.spec_from_file_location("p2k_analyze_261117", os.path.join(root, "evidence", "G06", "p2k_analyze_261117.py")); m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m)
            m.inv_P(600.0, 300.0, 0.030); _PA = m                            # 지연 적재가 있으면 여기서 끝낸다
        finally: os.chdir(cwd)
    return _PA


Q_RE = re.compile(r" q=([-0-9.,]+)")


def parse_q(line):
    """요청 줄의 ' q=J1,J2,J3,J4'(접촉 자세 원시 각도, 도) → 4개 실수 목록 또는 None."""
    m = Q_RE.search(line)
    if not m: return None
    try: v = [float(x) for x in m.group(1).split(",")]
    except ValueError: return None
    return v if len(v) == 4 else None


def tilt_recheck(plan, q, modules=None):
    """P10 런타임 확인(사용자 결정 2026-11-18, --real 전 필수): 실제 접촉 원시 q로 같은 규칙(FK 최악 정점 tilt ≤ 30°인 최대 정수 두 번째 J2 목표)을 다시 풀어
    계획의 j2_max_target보다 작으면 거부한다. q 없음·계산 실패도 거부(fail-closed). 새 숫자 없음. FK UNVERIFIED."""
    try:
        import g01_s42_plan_lift as PL
        L = plan["lift"]; planned = int(L["j2_max_target"]); first = int(L["j2_first_target"])
        if q is None: return {"ok": False, "reason": "접촉 원시 q가 요청에 없다", "planned_max": planned}
        C = (modules or PL._load_modules())[1]
        m = PL.max_target(C, list(q), first)
        if m is None: return {"ok": False, "reason": "실제 접촉 q에서 허용 목표 없음", "q": q, "planned_max": planned}
        return {"ok": m[0] >= planned, "recomputed_max": m[0], "peak_tilt_deg": round(m[1], 2), "planned_max": planned, "q": q,
                "reason": "" if m[0] >= planned else "실제 접촉 q의 최대 목표 %d < 계획 %d" % (m[0], planned)}
    except Exception as e: return {"ok": False, "reason": "%s: %s" % (type(e).__name__, e)}


def s4_gate_safe(ref_detect, detect, P=None):
    """S4' 관문. 계산 불가(ParamsNotFinal 등)는 go=False와 사유로 돌려준다. G2 배선 전(d_x 미확정)에는 불감대 밖이면 거부 쪽이다."""
    import g01_lift_logic as L
    try:
        if not (detect or {}).get("found"): return {"go": False, "reason": "사진 A에서 마커 id 7 미검출"}
        dxb = None
        if (ref_detect or {}).get("found") and detect.get("found"): dxb = dx_block_mm(ref_detect, detect)      # 원값은 gate 결과에 항상 기록
        return L.s4_gate(ref_detect, detect, dx_block_mm=dxb, dx_depth_mm=None, P=P)
    except Exception as e:                                           # ParamsNotFinal 포함: 기록하고 no-go
        return {"go": False, "reason": "%s: %s" % (type(e).__name__, e)}


class Bridge:
    """의존 주입: snap(이름) → detect dict, status() → started_epoch(float), push(이름, 결과 dict) → None, now() → epoch."""
    def __init__(self, ref, snap, status, push, now=time.time, P=None, say=print, align=None, tilt=None):
        self.ref, self.snap, self.status, self.push, self.now, self.P, self.say, self.align, self.tilt = ref, snap, status, push, now, P, say, align, tilt
        self.done = {}

    def handle(self, line):
        if ALIGN_MARK in line:                                       # 정렬 자세의 러너 질문 직전 줄: 정렬 보고를 화면에 띄운다(한 번만, Jetson 전달 없음)
            if self.align is None or "align" in self.done: return None
            try: text = self.align()
            except Exception as e: text = ALIGN_FAIL % repr(e)
            self.done["align"] = text; self.say(text); return None
        req = parse_request(line)
        if req is None: return None
        name, tok = req
        if name in self.done: self.say("[다리] 사진 %s 요청이 다시 왔다(무시, 이미 처리)" % name); return None
        t0 = self.now()
        with ThreadPoolExecutor(max_workers=2) as ex:               # 기동 시각 읽기는 스냅과 동시에(시간 한도 3.0 s 안에서 직렬로 더하지 않는다)
            fs, ft = ex.submit(self.status), ex.submit(self.snap, name)
            try: started = fs.result()
            except Exception as e: started = None; err_s = repr(e)
            try: detect = ft.result()
            except Exception as e: detect = None; err_d = repr(e)
        t1 = self.now()
        res = {"request_id": tok, "name": name, "started_epoch": started, "detect": detect, "mac": {"req_seen": t0, "snap_done": t1}}
        if started is None: res["error_status"] = err_s
        if detect is None: res["error_snap"] = err_d
        if name == "A":
            res["gate"] = {"go": False, "reason": "스냅 실패"} if detect is None else s4_gate_safe(self.ref["detect"], detect, self.P)
            if self.tilt is not None:                                    # P10 런타임 확인: 설정되면 반드시 통과해야 go
                tr = self.tilt(parse_q(line)); res["gate"]["tilt_check"] = tr
                if not tr.get("ok"): res["gate"]["go"] = False; res["gate"]["reason"] = (res["gate"].get("reason", "") + " P10 tilt 재확인 실패: " + tr.get("reason", "")).strip()
        res["mac"]["gate_done"] = self.now()
        try: self.push(name, res); res["mac"]["push_done"] = self.now()
        except Exception as e: self.say("[다리] 사진 %s 전달 실패: %r" % (name, e)); self.done[name] = None; return None
        self.done[name] = res; self.say("[다리] 사진 %s 전달 %.2f s%s" % (name, self.now() - t0, " gate=%s" % res["gate"].get("go") if "gate" in res else "")); return res


def make_ref(run_id, detect, started_epoch, maxsec, now_epoch):
    """기준 사진 결과(lift_photo_ref.json). elapsed_at_ref_s = 이 파일을 쓰기 직전의 P-A 경과 — 러너 기동까지의 몇 초는 포함하지 않는다(여유 300 s 대비 무시 가능, 플랜 §3-2에 기록)."""
    return {"run_id": run_id, "started_epoch": started_epoch, "elapsed_at_ref_s": now_epoch - started_epoch, "maxsec": maxsec, "detect": detect, "ref_written_epoch": now_epoch}


def align_report(root, img, ref, python=None, timeout=ALIGN_TIMEOUT_S, run=subprocess.run, set_arg="33"):
    """검증된 scripts/p2k_align_report.py(세트 33 인자: 재등록 기준 파일만 사용, 없으면 NG 보고. 세트 31 인자 '31'은 양성 대조용)를 수정 없이 호출해 정렬 보고 문자열을 만든다. 실패·시간 초과·출력 없음은 예외(상위가 'N' 안내로 바꾼다)."""
    r = run([python or sys.executable, "-I", os.path.join(root, "scripts", "p2k_align_report.py"), img, ref, set_arg], cwd=root, capture_output=True, text=True, timeout=timeout)
    if r.returncode != 0 or "결과:" not in r.stdout: raise RuntimeError("정렬 보고 계산 실패: " + (r.stdout + r.stderr).strip()[-200:])
    return r.stdout.rstrip() + "\n" + ALIGN_NOTE + "\n"


def marker_line(label, path):
    """두 기준 사진(P-A·36 s)의 마커 중심·변 길이를 기록해 실행 뒤 같은 장면 쌍으로 대조할 수 있게 한다."""
    import cv2, pa_detect
    d = pa_detect.detect(cv2.imread(path))
    return "기록: %s 마커 %s" % (label, ("(%.2f, %.2f) side %.2f px" % (d["center"][0], d["center"][1], d["side_px"])) if d.get("found") else "미검출")


# ---- 실기기 구현(시험에서는 가짜로 대체) ----
SO = ["-o", "BatchMode=yes", "-o", "ConnectTimeout=8", "-o", "ControlMaster=auto", "-o", "ControlPath=/tmp/lift-ssh-%C", "-o", "ControlPersist=300"]


def real_snap(root, prefix):
    def snap(name):
        n = "%s_%s" % (prefix, name); r = subprocess.run(["bash", os.path.join(root, "scripts", "pa_snap_mac.sh"), n], capture_output=True, text=True, timeout=30)
        if r.returncode != 0: raise RuntimeError("스냅 실패: " + (r.stdout + r.stderr).strip()[-200:])
        return json.load(open(os.path.join(root, "data", "g04", "pa", n + ".detect.json"), encoding="utf-8"))
    return snap


def real_align(root, ts, prefix, snap):
    def align():
        snap("align")
        pa = os.path.join(root, "data", "g04", "pa"); img = os.path.join(pa, "%s_align.color.png" % prefix); ref = os.path.join(pa, "%s_ref.color.png" % prefix)
        text = align_report(root, img, ref)
        extra = []
        for lab, pth in (("P-A 기준", ref), ("36 s 기준", os.path.join(root, "data", "g04", "p2kref_%s.png" % ts)), ("P-A 정렬 자세", img)):
            try: extra.append(marker_line(lab, pth))
            except Exception as e: extra.append("기록: %s 마커 읽기 실패(%r)" % (lab, e))
        text = "\n" + "=" * 60 + "\n[P-A 정렬 보고]\n" + text + "\n".join(extra) + "\n"
        out = os.path.join(root, "evidence", "G01", "raw", "lift_align_%s.txt" % ts)
        with open(out, "w", encoding="utf-8") as f: f.write(text)
        return text
    return align


def real_status(root):
    def status():
        r = subprocess.run(["ssh"] + SO + ["p620", "ACTION=status bash -s"], stdin=open(os.path.join(root, "scripts", "pa_driver.sh")), capture_output=True, text=True, timeout=15)
        m = re.search(r"started_epoch\s+([0-9.]+)", r.stdout)
        if not m: raise RuntimeError("P-A 기동 시각을 읽지 못했다: " + r.stdout.strip()[-200:])
        return float(m.group(1))
    return status


def real_push(host, rd, tmpdir):
    def push(name, res):
        tmp = os.path.join(tmpdir, ".lift_photo_%s.json" % name)
        with open(tmp, "w", encoding="utf-8") as f: json.dump(res, f, ensure_ascii=False)
        subprocess.run(["scp", "-q"] + SO + [tmp, "%s:%s/.lift_photo_%s.json.tmp" % (host, rd, name)], check=True, timeout=15)
        subprocess.run(["ssh"] + SO + [host, "mv %s/.lift_photo_%s.json.tmp %s/lift_photo_%s.json" % (rd, name, rd, name)], check=True, timeout=15)   # 원자적 교체(러너는 완성된 파일만 본다)
    return push


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--run-id", required=True); ap.add_argument("--ref", required=True); ap.add_argument("--host", required=True); ap.add_argument("--rd", required=True)
    ap.add_argument("--plan", required=True); ap.add_argument("--name-prefix", required=True); ap.add_argument("--tmpdir", default="/tmp"); ap.add_argument("--log", default=None); a = ap.parse_args()
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__))); ref = json.load(open(a.ref, encoding="utf-8"))
    assert ref.get("run_id") == a.run_id, "기준 사진 run_id 불일치"
    logf = open(a.log, "a", encoding="utf-8") if a.log else None
    def say(s):
        print(s, flush=True)
        if logf: logf.write(s + "\n"); logf.flush()
    t0 = time.time(); load_pa(root); say("[다리] 분석 모듈 적재 %.2f s(사진 요청 경로 밖)" % (time.time() - t0))
    plan = json.load(open(a.plan, encoding="utf-8")); import g01_s42_plan_lift as PL; mods = PL._load_modules()   # chdir root(생성기 모듈이 상대 경로를 읽음)
    snap = real_snap(root, a.name_prefix)
    b = Bridge(ref, snap, real_status(root), real_push(a.host, a.rd, a.tmpdir), say=say, align=real_align(root, a.run_id, a.name_prefix, snap), tilt=lambda q: tilt_recheck(plan, q, mods))
    for line in sys.stdin: b.handle(line)
    if logf: logf.write(json.dumps({"results": {k: v for k, v in b.done.items()}}, ensure_ascii=False) + "\n"); logf.close()


if __name__ == "__main__": main()
