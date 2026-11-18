"""들어올림(세트 33) 러너 훅 본체. g01_p2k_test.run2_hook의 접촉 분기가 정체(명령 150)를 확인한 뒤 호출한다. 의존은 ctx로 주입한다(순환 import·단위 시험 목적).
흐름(플랜 §2 R1~R12, §2-2 추적표): 사진 A → S4' 관문 → 들어올림 3이동(첫 J2, 두 번째 J2(실제 각도 기준·최대 목표), J4 −5) → 사진 B → 유지 → 사진 C → 낙하 열기.
모든 단계는 PressClock('경과 + 단계 상한 ≤ 26.8 s')으로 시작을 확인한다. 반환: 'no_lift'(들어올리지 않음: 호출부가 기존 열기 경로를 이어서 실행) 또는 'lifted'(들어올렸고 낙하 열기까지 끝냄: 호출부가 lift_done을 세팅).
사진 전달 프로토콜(Mac 래퍼 M2c): 러너는 '[사진 요청 <이름>]' 줄을 출력하고 작업 폴더의 lift_photo_<이름>.json이 나타나기를 폴링한다(이름: ref(실행 전, 이미 존재)·A·B·C)."""
import datetime, json, os, time
import g01_lift_logic as L
import g01_lift_params as LP

REF_NAME = "ref"


class PhotoChannel:
    """사진 요청·결과 수신 채널(시험에서 가짜로 대체). request(name)은 요청 줄을 출력하고, wait(name, timeout_s)는 결과 dict 또는 None(시간 초과)을 돌려준다."""
    def __init__(self, say): self.say = say
    def request(self, name, run_id, extra=""):
        """고유 토큰을 만들어 요청 줄에 넣는다(결과 JSON의 request_id와 대조). extra는 줄 끝에 붙는 추가 필드(예: ' q=86.3,68.3,11.4,31.4'). 반환: 토큰."""
        tok = "%s-%s-%d" % (run_id, name, time.time_ns()); self.say("[사진 요청 %s id=%s%s]" % (name, tok, extra)); return tok
    def read_ref(self): raise NotImplementedError
    def wait(self, name, timeout_s, token): raise NotImplementedError


class FilePhotoChannel(PhotoChannel):
    """실기기: Mac 래퍼가 scp로 놓는 lift_photo_<이름>.json을 0.02 s 간격으로 폴링한다(원자적 교체는 래퍼 책임)."""
    POLL_S = 0.02
    def __init__(self, say, folder="."): super().__init__(say); self.folder = folder
    def _path(self, name): return os.path.join(self.folder, "lift_photo_%s.json" % name)
    def read_ref(self):
        p = self._path(REF_NAME)
        return json.load(open(p, encoding="utf-8")) if os.path.exists(p) else None
    def request(self, name, run_id, extra=""):
        try: os.remove(self._path(name))                                 # 요청 직전에 같은 이름의 옛 결과를 지운다
        except FileNotFoundError: pass
        return super().request(name, run_id, extra)
    def wait(self, name, timeout_s, token):
        t0 = time.monotonic()
        while time.monotonic() - t0 <= timeout_s:
            p = self._path(name)
            if os.path.exists(p):
                try:
                    r = json.load(open(p, encoding="utf-8"))
                    if r.get("request_id") == token: return r              # 이번 요청의 응답만 받는다(옛 파일·다른 요청은 무시하고 계속 폴링)
                except Exception: pass                                   # 쓰는 중이면 다음 폴링에서 다시 읽는다
            time.sleep(self.POLL_S)
        return None


def new_clock(params):
    return L.PressClock(time.monotonic, params)


def precheck_before_close(ch, params, t_runner_start, run_id=None, now=None):
    """R5·R6: 닫기 전에 기준 사진(실행 전, 같은 P-A 기동)이 있고 P-A 경과가 여유 안인지 확인한다. 반환 (ok, 사유, ref dict)."""
    now = time.monotonic() if now is None else now
    ref = ch.read_ref()
    if not ref or not ref.get("detect") or ref.get("started_epoch") is None: return False, "기준 사진 결과(lift_photo_ref.json)가 없다", ref
    if run_id is not None and ref.get("run_id") != run_id: return False, "기준 사진이 이번 실행(run_id %s)의 것이 아니다" % run_id, ref
    elapsed = float(ref.get("elapsed_at_ref_s", 1e9)) + (now - t_runner_start)
    if not L.pa_elapsed_ok(elapsed, float(ref.get("maxsec", 0)), params): return False, "P-A 기동 경과 %.0f s가 MAXSEC − %d s를 넘었다" % (elapsed, params["pa_elapsed_margin_s"]), ref
    return True, "", ref


def _adapter(ctx, joint, target):
    """이동마다 새 어댑터(F5): 시작 회독을 읽고 [시작, 목표] 목록을 고정한다. first_move_min_ms=None(P7), J2 창은 WINDOWS(세트 33 [66,110], P8)."""
    today = datetime.date.today()
    ad = ctx["Adapter"](ctx["arm"], {"approved": True, "expires": today.isoformat(), "allowed_times_ms": [ctx["params"]["move_time_ms"]], "first_move_min_ms": ctx["params"]["first_move_min_ms"],
                                     "max_writes": 3, "delta_deg": 1, "joint_id": joint, "direction": 1, "angle_window": list(ctx["WINDOWS"][joint]), "pattern": "list"}, today=today, dry_run=False)
    s = ad.read_start(); ad.set_targets([s, int(target)]); return ad


def classify_abort(exc):
    """중단 사유 분류(기록 전용): 들어올리는 동안 J6 회독이 눌린 값에서 3° 넘게 벗어나면 E3 '미끄러짐 의심'."""
    m = str(exc)
    return "E3 미끄러짐 의심(J6 이탈)" if ("J6" in m and "벗어났" in m) else "기타 중단"


def _raw(ctx, joint):
    """원시 각도(실수) 읽기, None이면 3회 재시도 후 명시적으로 중단(높이 관문과 같은 방식)."""
    for _ in range(3):
        v = ctx["raw_set"](ctx["arm"])[str(joint)]["deg"]
        if v is not None: return v
        time.sleep(0.05)
    raise ctx["Abort"]("J%d 원시 각도 읽기 실패(None). 추가 이동 없음." % joint)


def _int_read(raw, joint):
    """원시 각도 → do_return이 쓰는 SDK 정수 회독과 같은 규칙(J2~J4는 올림, J1은 내림; D0~D2 기록으로 확인된 Arm_Lib 환산)."""
    import math
    return int(math.ceil(raw - 1e-9)) if joint in (2, 3, 4) else int(math.floor(raw + 1e-9))


def _stage_idx(ctx):
    """현재 상태에 가장 가까운 복귀 항목의 색인: 접촉 자세 항목(첫 이동이 실패한 경우)과 들어올린 상태 항목들 중에서 J2·J4 정수 회독 거리가 가장 작은 것.
    (접촉 항목이 후보에 없으면 첫 이동 실패 때 들어올린 상태 경로를 접촉 깊이에서 실행하게 된다 — 16라운드)"""
    plan = ctx["plan"]; k = plan["lift"]["contact_moves_done"]; cs = plan["return_routes"][k]["state"]
    j2, j4 = _int_read(_raw(ctx, 2), 2), _int_read(_raw(ctx, 4), 4); cand = [(abs(cs[1] - j2) + abs(cs[3] - j4), k, "contact")]
    for key, idx in plan["lift"]["return_index"].items():
        st = plan["return_routes"][idx]["state"]; cand.append((abs(st[1] - j2) + abs(st[3] - j4), idx, key))
    d, idx, key = min(cand); ctx["rec"].setdefault("lift", {})["return_index_used"] = {"index": idx, "key": key, "distance": d, "j2": j2, "j4": j4}
    return idx


def _idx(plan, stage, j2_read):
    """readback에 가장 가까운 들어올린 상태의 복귀 색인(F6). 후보가 없으면 None."""
    ri = plan["lift"]["return_index"]; cand = [(abs(int(k.split(":")[1]) - j2_read), v) for k, v in ri.items() if k.startswith(stage + ":")]
    return min(cand)[1] if cand else None


def lift_after_stall(ctx):
    """ctx 키: plan, arm, adc(J6 어댑터), cfg(run2), rec(접촉 자세 기록 dict), params, clock(145 쓰기에서 시작됨), channel, ref, run_move, raw_set, Adapter, WINDOWS, PROG, P2K,
    say, cmd, start_all, ads, mode, stalled(정체 회독), n_plan(계획 이동 수), release(callable: J6 열기 1회 + 도착 확인, 마지막 회독 반환)."""
    plan, arm, rec, P, clk, ch = ctx["plan"], ctx["arm"], ctx["rec"], ctx["params"], ctx["clock"], ctx["channel"]; say = ctx["say"]; PROG, P2K = ctx["PROG"], ctx["P2K"]
    lift = rec.setdefault("lift", {"photos": {}, "moves": []}); ubs = dict(L.step_upper_bounds(P)); lift["clock_ub"] = ubs
    lift["j6_after_close"] = ctx["stalled"]
    def done(reason): lift["ended_reason"] = reason; lift["clock_log"] = list(clk.log)
    def photo(name):
        extra = ""
        if name == "A":   # P10 런타임 확인용: 접촉 자세의 실제 원시 q(높이 관문에서 읽은 값)를 요청 줄에 실어 Mac 관문이 두 번째 J2 최대 목표를 다시 풀게 한다. 없으면 필드를 생략(관문이 거부)
            q_ = (ctx["rec"] or {}).get("raw_q_at_contact")
            if q_ and all(v is not None for v in q_): extra = " q=" + ",".join("%.3f" % v for v in q_)
        if not clk.may_start("photo_" + name.lower(), ubs["photo_" + name.lower()]): return None
        t_req, e_req = time.monotonic(), time.time(); tok = ch.request(name, ctx["run_id"], extra); r = ch.wait(name, P["photo_wait_max_s"], tok)
        if r is not None and r.get("request_id") != tok: r = None                                  # 요청 토큰이 다르면 응답이 아니다(방어)
        lift["photos"][name] = r; lift.setdefault("photo_times", {})[name] = {"req_mono": t_req, "rx_mono": time.monotonic(), "wait_s": round(time.monotonic() - t_req, 3), "req_epoch": e_req, "rx_epoch": time.time(), "request_id": tok}
        if r is None: say("  ! 사진 %s 결과가 %.1f s 안에 오지 않았다(판정 없음)." % (name, P["photo_wait_max_s"]))
        return r
    # --- 사진 A·S4' 관문(들어올리기 전) ---
    a = photo("A")
    if a is None: done("사진 A 없음(시계 또는 대기 초과): 들어올리지 않음"); return "no_lift"
    ref = ctx["ref"]
    if not L.same_start([ref.get("started_epoch"), a.get("started_epoch")]): done("기준·A가 다른 P-A 기동: 판정 없음"); return "no_lift"
    lift["gate"] = a.get("gate")
    if not (a.get("gate") or {}).get("go"): done("S4' no-go 또는 관문 결과 없음: 들어올리지 않음"); return "no_lift"
    # --- 들어올림 3이동 ---
    cmd, start_all, ads0 = ctx["cmd"], ctx["start_all"], ctx["ads"]; ms = P["move_time_ms"]; n0 = ctx["n_plan"]; lifted = False
    start_all[6] = ctx["stalled"]   # R12: 들어올리기 직전에 J6 감시 기준을 눌린 값(정체 회독)으로 갱신(run_move가 J6를 start_all과 ±3°로 감시). 들어올리지 않으면 아래에서 되돌린다
    def do_move(step, joint, target, k):
        nonlocal lifted
        if not clk.may_start(step, ubs[step]): raise _ClockStop(step)
        # 복귀 색인은 쓰기 직전에 '이 이동이 끝난 뒤' 상태로 지정한다(이미 보낸 이동은 서보가 끝까지 수행, 쓰기 전에 중단되면 do_return이 3° 불일치로 쓰기 없이 멈춘다)
        PROG["sent"] = _idx(plan, "before_j4", int(target)) if joint == 2 else _idx(plan, "after_j4", int(round(_raw(ctx, 2))))
        ad = _adapter(ctx, joint, target); ads = dict(ads0); ads[joint] = ad
        mv = {"joint": joint, "target": int(target), "time_ms": ms, "phase": "L", "pose_id": None}
        try: st = ctx["run_move"](ads, arm, mv, cmd, start_all, rec, n0 + k, n0 + 3, ctx["mode"], P["move_countdown_s"])
        except BaseException as exc_:
            lift["abort_class"] = classify_abort(exc_)
            try:
                PROG["sent"] = _stage_idx(ctx)                                                     # 중단되면 실제 상태에 가장 가까운 복귀 항목(불일치면 do_return이 쓰기 없이 멈춘다)
                for j_, v_ in zip((1, 2, 3, 4), plan["return_routes"][PROG["sent"]]["state"]): cmd[j_] = v_   # run_move가 중단 전에 cmd에 적은 '목표'(예: 무반응이었던 77)를 실제 상태 항목 값으로 되돌린다(복귀의 비명령 관절 이탈 검사)
            except BaseException: pass
            lift["moves"].append({"step": step, "joint": joint, "target": int(target), "aborted": True}); lifted = True; raise
        lift["moves"].append(dict(st, step=step, target=int(target))); lifted = True; PROG["sent"] = _stage_idx(ctx)
        return st
    try:
        j2_start = _raw(ctx, 2); first = plan["lift"]["j2_first_target"]
        if first > plan["lift"]["j2_max_target"]: raise ValueError("계획의 첫 J2 명령이 최대 목표를 넘는다")
        st = do_move("lift1", 2, first, 1); j2_read = _raw(ctx, 2)
        t2, info = L.second_j2_command(first, j2_start, j2_read, dict(P, lift_j2_max_target=plan["lift"]["j2_max_target"])); lift["second_j2"] = dict(info, target=t2, j2_start=j2_start, j2_after_first=j2_read)
        if t2 is not None and t2 - j2_read < P["lift_j2_min_effective_step_deg"]:   # 요청 폭이 서보 최소 유효 폭(위 방향, 세트 16)보다 작으면 보내지 않는다(조용한 반쪽 이동 방지, P10 조건 ①)
            lift["second_j2"]["skipped_reason"] = "요청 폭 %.2f° < 최소 유효 폭 %.1f°" % (t2 - j2_read, P["lift_j2_min_effective_step_deg"]); say("  두 번째 J2 이동 생략: %s" % lift["second_j2"]["skipped_reason"]); t2 = None
        if t2 is not None:
            do_move("lift2", 2, t2, 2); moved = _raw(ctx, 2) - j2_read; lift["second_j2"]["moved_deg"] = moved
            if moved < P["lift_j2_no_response_deg"]: lift["second_j2"]["no_response"] = True; say("  ! 두 번째 J2 무반응 의심(원시 변화 %.2f°). 재전송하지 않고 J4로 진행합니다(덜 들어올린 쪽)." % moved)
        do_move("lift3", 4, int(cmd[4]) + plan["lift"]["j4_delta"], 3)
        # --- 사진 B → 유지 → 사진 C ---
        b = photo("B"); lift["hold_done"] = False; lift["photo_c_done"] = False
        if b is not None and L.same_start([ref.get("started_epoch"), b.get("started_epoch")]):
            if clk.may_start("hold", ubs["hold"]):
                time.sleep(P["hold_s"]); lift["hold_done"] = True
                c = photo("C"); lift["photo_c_done"] = c is not None and L.same_start([ref.get("started_epoch"), c.get("started_epoch")])
        lift["j6_after_lift"] = ctx["adc"].read_position()
        done("완료" if lift["photo_c_done"] else ("유지·사진 C 일부 생략" if lift["hold_done"] else "사진 B 이후 생략"))
    except _ClockStop as e:
        done("시계 규칙으로 단계 '%s' 시작 전 중단(경과 + 상한 > %.1f s): 즉시 열기" % (e.args[0], P["clock_limit_s"]))
    # --- 낙하 열기(시계의 끝) ---
    if not lifted: start_all[6] = ctx["cfg"]["open_j6"]; return "no_lift"
    lift["open_readback"] = ctx["release"](); lift["elapsed_at_open_s"] = clk.elapsed(); ctx["start_all"][6] = ctx["cfg"]["open_j6"]   # 열린 뒤 J6 감시 기준을 열림 값으로 되돌린다
    return "lifted"


class _ClockStop(Exception): pass


def close_j6_at_k1(ctx):
    """K1 도착 뒤 J6를 시작값(오므림)으로 닫는다(플랜 추적표 단계 12, 새 코드): 100 → 140 → close_to. 어댑터는 새로 만든다."""
    cfg, plan, arm = ctx["cfg"], ctx["plan"], ctx["arm"]; today = datetime.date.today()
    fin = ctx["Adapter"](arm, {"approved": True, "expires": today.isoformat(), "allowed_times_ms": [3000, 4800], "first_move_min_ms": None, "max_writes": 4, "delta_deg": 5, "joint_id": 6, "direction": -1,
                                "angle_window": list(plan["p2k"]["window"]), "pattern": "list"}, today=today, dry_run=False)
    s = fin.read_start(); mid = cfg.get("open_mid", 140)
    if abs(s - cfg["open_j6"]) > 2: raise ctx["Abort"]("K1 도착 뒤 J6 회독 %s가 열림 %d와 다르다. J6 닫기 없음." % (s, cfg["open_j6"]))
    fin.set_targets([s, mid, cfg["close_to"]]); last = None
    for i, t in enumerate((mid, cfg["close_to"]), 1):
        ctx["say"]("\n[이동 %d/2] J6 -> %d (time %d ms): 3초 뒤 닫습니다. ▶ **J6(그리퍼)를 지켜보세요. 손가락 근처에 손을 넣지 마세요.**" % (i, t, cfg["close_to_ms"])); time.sleep(3.0)   # 래퍼 beep_watch가 '[이동 N/M]' 줄에 소리를 낸다
        fin.confirm_hands_clear(); time.sleep(ctx["QUIET_S"]); fin.move(t, cfg["close_to_ms"])
        series = ctx["poll"](fin, 2 * cfg["close_to_ms"] + 1000); last = next((v for _, v in reversed(series) if v is not None), None)
    ctx["P2K"]["cur6"] = cfg["close_to"]; ctx["P2K"]["start6"] = cfg["close_to"]; ctx["start_all"][6] = cfg["close_to"]
    return last
