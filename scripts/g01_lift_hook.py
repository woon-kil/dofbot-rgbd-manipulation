"""들어올림(세트 33) 러너 훅 본체. g01_p2k_test.run2_hook의 접촉 분기가 정체(명령 150)를 확인한 뒤 호출한다. 의존은 ctx로 주입한다(순환 import·단위 시험 목적).
흐름(플랜 §2 R1~R12, §2-2 추적표): 사진 A → S4' 관문 → 들어올림 3이동(첫 J2, 두 번째 J2(실제 각도 기준·최대 목표), J4 −5) → 사진 B → 유지 → 사진 C → 낙하 열기.
모든 단계는 PressClock('경과 + 단계 상한 ≤ 26.8 s')으로 시작을 확인한다. 반환: 'no_lift'(들어올리지 않음: 호출부가 기존 열기 경로를 이어서 실행) 또는 'lifted'(들어올렸고 낙하 열기까지 끝냄: 호출부가 lift_done을 세팅).
사진 전달 프로토콜(Mac 래퍼 M2c): 러너는 '[사진 요청 <이름>]' 줄을 출력하고 작업 폴더의 lift_photo_<이름>.json이 나타나기를 폴링한다(이름: ref(실행 전, 이미 존재)·A·B·C)."""
import datetime, json, os, time
import g01_lift_logic as L
import g01_lift_params as LP

REF_NAME = "ref"
ALIGN_TIMEOUT_S = 15   # g01_lift_bridge.ALIGN_TIMEOUT_S와 같은 값(사용자 승인 2026-11-19: 용도가 다른 기존 상수를 Jetson 정렬 판정 대기 상한에도 사용, 시간 초과는 자동 N)
ALIGN_NAME = "align_verdict"


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


def align_gate(ch, run_id, timeout_s=None):
    """M6: Mac 다리가 보낸 정렬 판정을 최대 ALIGN_TIMEOUT_S 기다린다. 반환 (통과 여부, 사유). **OK는 이번 요청 토큰·이번 run_id의 응답이 형식대로 `verdict == "OK"`일 때만 통과**한다
    (NG·없음·시간 초과·옛 요청(토큰 불일치)·다른 실행(run_id 불일치)·형식 오류는 모두 통과 아님 → 호출부가 사람 입력 없이 N 처리)."""
    tok = ch.request(ALIGN_NAME, run_id); res = ch.wait(ALIGN_NAME, ALIGN_TIMEOUT_S if timeout_s is None else timeout_s, tok)
    if res is None: return False, "정렬 판정이 %d s 안에 오지 않았다(시간 초과)" % (ALIGN_TIMEOUT_S if timeout_s is None else timeout_s)
    if not isinstance(res, dict): return False, "정렬 판정 형식 오류"
    if res.get("request_id") != tok: return False, "정렬 판정이 이번 요청의 것이 아니다(옛 파일)"
    if res.get("run_id") != run_id: return False, "정렬 판정의 run_id %r가 이번 실행 %r와 다르다" % (res.get("run_id"), run_id)
    v = res.get("verdict")
    if v == "OK": return True, ""
    return False, "정렬 판정 %r" % (v,)


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
    if ctx.get("_lift_adapters") is not None: ctx["_lift_adapters"].append((joint, int(target), ad))   # 기록 전용: read_start가 None으로 실패해도 어댑터 로그("read", None)가 결과 JSON에 남는다(재실행 #1: 로그가 어댑터째 사라졌다)
    s = _read_start_retry(ad); ad.set_targets([s, int(target)]); return _BeforeRetry(ad, ctx["Abort"])


def _set_label(ctx, label):
    try: f = getattr(ctx.get("arm"), "set_label", None)
    except (AttributeError, AssertionError): return   # 가드 없는 팔(가짜 팔은 알 수 없는 속성에 AssertionError)
    if f is not None: f(label)


def _last_read_was_none(ad): return bool(getattr(ad, "log", None)) and ad.log[-1] == ("read", None)


def _read_start_retry(ad):
    """M5b(플랜 v2, 사용자 승인 2026-11-19): `read_start`가 None이면 기존 `_raw`의 3회 규칙(총 3회)으로 다시 읽는다. 재시도 간격은 가드(g01_read_guard)의 같은 서보 ≥ 0.05 s가 보장한다(별도 sleep 없음).
    None일 때만 재시도한다: 승인 창 밖 값 등 다른 사유의 거부는 그대로 던진다. 실패하면 어댑터 `_start`가 비어 있어 재호출할 수 있다."""
    for i in range(3):
        try: return ad.read_start()
        except Exception:
            if i == 2 or not _last_read_was_none(ad): raise


class _BeforeRetry(object):
    """M5b: 이 어댑터의 첫 `read_position`(= `run_move`의 `before`)이 None이면 같은 3회 규칙으로 다시 읽고, 끝내 None이면 쓰기 전에 중단한다(새 중단 경로: 기존에는 `width0 = 0`으로 무반응·재전송 검사가 조용히 꺼진 채 쓰기가 나갔다).
    이후의 읽기(watch 폴링)는 그대로 통과한다. 어댑터의 나머지 속성·메서드는 그대로 전달한다."""
    def __init__(self, ad, abort_cls): self.__dict__.update(_ad=ad, _abort=abort_cls, _first=True)
    def read_position(self):
        if not self._first: return self._ad.read_position()
        self.__dict__["_first"] = False
        for _ in range(3):
            v = self._ad.read_position()
            if v is not None: return v
        e = self._abort("이동 전 회독(before)이 3회 모두 None이다: 쓰기 없이 중단합니다(쓰기 0회).")
        e.before_none = True; raise e
    def __getattr__(self, name): return getattr(self.__dict__["_ad"], name)


def classify_abort(exc):
    """중단 사유 분류(기록 전용): 들어올리는 동안 J6 회독이 눌린 값에서 3° 넘게 벗어나면 E3 '미끄러짐 의심'."""
    m = str(exc)
    return "E3 미끄러짐 의심(J6 이탈)" if ("J6" in m and "벗어났" in m) else "기타 중단"


def _read4(ctx, label=None):
    """원시 각도 J1~J4를 한 번 읽는다(raw_set 1회). label이 있으면 lift["raw_reads"]에 기록한다(P10 처짐 자료: 기록만, 읽기를 추가하지 않는다)."""
    r = ctx["raw_set"](ctx["arm"]); q = {j: (r.get(str(j)) or {}).get("deg") for j in (1, 2, 3, 4)}
    if label: ctx["rec"].setdefault("lift", {}).setdefault("raw_reads", []).append({"label": label, "q": [q[j] for j in (1, 2, 3, 4)]})
    return q


def _raw(ctx, joint, label=None):
    """원시 각도(실수) 읽기, None이면 3회 재시도 후 명시적으로 중단(높이 관문과 같은 방식). 재시도 규칙은 변경 전과 같다."""
    for _ in range(3):
        v = _read4(ctx, label)[joint]
        if v is not None: return v
        time.sleep(0.05)
    raise ctx["Abort"]("J%d 원시 각도 읽기 실패(None). 추가 이동 없음." % joint)


def _raw_j24(ctx):
    """한 번의 `raw_set`에서 J2·J4를 함께 얻는다(둘 중 하나라도 None이면 `_raw`와 같은 3회·0.05 s 재시도). `_stage_idx`가 읽기를 두 번 하던 것을 한 번으로 줄인다(같은 서보 재읽기 간격 가드의 대기와 시계 소모를 줄임, 새 숫자 없음)."""
    for _ in range(3):
        q = _read4(ctx)
        if q[2] is not None and q[4] is not None: return q
        time.sleep(0.05)
    raise ctx["Abort"]("J%d 원시 각도 읽기 실패(None). 추가 이동 없음." % (2 if q[2] is None else 4))


def _raw_all(ctx, label):
    """같은 읽기에서 J1~J4를 모두 돌려준다. 재시도는 변경 전 `_raw(ctx, 2)`와 같게 J2가 None일 때만(J3·J4 None은 재읽기 없이 호출자가 기록·처리한다)."""
    for _ in range(3):
        q = _read4(ctx, label)
        if q[2] is not None: return q
        time.sleep(0.05)
    raise ctx["Abort"]("J2 원시 각도 읽기 실패(None). 추가 이동 없음.")


def _int_read(raw, joint):
    """원시 각도 → do_return이 쓰는 SDK 정수 회독과 같은 규칙(J2~J4는 올림, J1은 내림; D0~D2 기록으로 확인된 Arm_Lib 환산)."""
    import math
    return int(math.ceil(raw - 1e-9)) if joint in (2, 3, 4) else int(math.floor(raw + 1e-9))


def _stage_idx(ctx):
    """현재 상태에 가장 가까운 복귀 항목의 색인: 접촉 자세 항목(첫 이동이 실패한 경우)과 들어올린 상태 항목들 중에서 J2·J4 정수 회독 거리가 가장 작은 것.
    (접촉 항목이 후보에 없으면 첫 이동 실패 때 들어올린 상태 경로를 접촉 깊이에서 실행하게 된다 — 16라운드)"""
    plan = ctx["plan"]; k = plan["lift"]["contact_moves_done"]; cs = plan["return_routes"][k]["state"]
    q = _raw_j24(ctx); j2, j4 = _int_read(q[2], 2), _int_read(q[4], 4); cand = [(abs(cs[1] - j2) + abs(cs[3] - j4), k, "contact")]
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
    adapters = ctx.setdefault("_lift_adapters", [])
    def save_logs():   # 기록 전용(성공·실패 무관): 어댑터 로그와 시계 로그(M7: 실패·중단 경로에서도 clock_log를 남긴다, 재실행 #1은 clock_log가 None이었다), 읽기 가드 대기 기록(M5)
        lift["adapter_logs"] = [{"joint": j_, "target": t_, "log": [list(l_) for l_ in a_.log]} for j_, t_, a_ in adapters]; lift["clock_log"] = list(clk.log)
        wl = getattr(ctx.get("arm"), "wait_log", None) if type(ctx.get("arm")).__name__ == "ReadGuardArm" else None
        if wl is not None: lift["guard_waits"] = [list(w) for w in wl]
    def photo(name):
        extra = ""
        if name == "A":   # P10 런타임 확인용: 접촉 자세의 실제 원시 q(높이 관문에서 읽은 값)를 요청 줄에 실어 Mac 관문이 두 번째 J2 최대 목표를 다시 풀게 한다. 없으면 필드를 생략(관문이 거부)
            q_ = (ctx["rec"] or {}).get("raw_q_at_contact")
            if q_ and all(v is not None for v in q_): extra = " q=" + ",".join("%.3f" % v for v in q_)
        if not clk.may_start("photo_" + name.lower(), ubs["photo_" + name.lower()]): return None
        _set_label(ctx, "photo_" + name.lower())
        t_req, e_req = time.monotonic(), time.time(); tok = ch.request(name, ctx["run_id"], extra); r = ch.wait(name, P["photo_wait_max_s"], tok)
        if r is not None and r.get("request_id") != tok: r = None                                  # 요청 토큰이 다르면 응답이 아니다(방어)
        lift["photos"][name] = r; lift.setdefault("photo_times", {})[name] = {"req_mono": t_req, "rx_mono": time.monotonic(), "wait_s": round(time.monotonic() - t_req, 3), "req_epoch": e_req, "rx_epoch": time.time(), "request_id": tok}
        if r is None: say("  ! 사진 %s 결과가 %.1f s 안에 오지 않았다(판정 없음)." % (name, P["photo_wait_max_s"]))
        return r
    # --- 사진 A·S4' 관문(들어올리기 전) ---
    a = photo("A")
    if a is None: lift["code"] = "PHOTO_A"; done("사진 A 없음(시계 또는 대기 초과): 들어올리지 않음"); return "no_lift"
    ref = ctx["ref"]
    if not L.same_start([ref.get("started_epoch"), a.get("started_epoch")]): lift["code"] = "P_A_START"; done("기준·A가 다른 P-A 기동: 판정 없음"); return "no_lift"
    lift["gate"] = a.get("gate")
    if not (a.get("gate") or {}).get("go"):
        g_ = a.get("gate") or {}; lift["code"] = "P10" if (g_.get("tilt_check") is not None and not g_["tilt_check"].get("ok")) else (",".join(k.upper() for k in ("g1", "g2", "g4", "g5") if g_.get(k) is False) or "GATE")
        done("S4' no-go 또는 관문 결과 없음: 들어올리지 않음"); return "no_lift"
    # --- 들어올림 3이동 ---
    cmd, start_all, ads0 = ctx["cmd"], ctx["start_all"], ctx["ads"]; ms = P["move_time_ms"]; n0 = ctx["n_plan"]; lifted = False
    start_all[6] = ctx["stalled"]   # R12: 들어올리기 직전에 J6 감시 기준을 눌린 값(정체 회독)으로 갱신(run_move가 J6를 start_all과 ±3°로 감시). 들어올리지 않으면 아래에서 되돌린다
    def do_move(step, joint, target, k):
        nonlocal lifted
        if not clk.may_start(step, ubs[step]): raise _ClockStop(step)
        _set_label(ctx, step)   # 읽기 가드의 대기 기록용 단계 라벨(기록 전용, 가드가 없으면 아무 일도 하지 않는다)
        # 어댑터 생성(read_start)은 쓰기가 나가기 전이다. 여기서 실패하면(예: 서보 회독 None) 복귀 색인·cmd를 실제 상태 기준으로 돌린 뒤 다시 던진다(재실행 #1 사고: PROG["sent"]가 '이 이동이 끝난 뒤' 색인으로 남아 복귀가 쓰기 없이 멈췄다)
        try: ad = _adapter(ctx, joint, target)
        except BaseException as exc_:
            lift["adapter_error"] = {"joint": joint, "target": int(target), "error": repr(exc_), "type": type(exc_).__name__}; save_logs()
            if type(exc_).__name__ == "GripperTestError": lift["code"] = "ADAPTER_BEFORE_WRITE"; lift["lifted_once"] = bool(lifted)   # 분류 라벨은 어댑터 거부일 때만(Ctrl-C·Hangup은 라벨 없이 복원만)
            try:
                PROG["sent"] = _stage_idx(ctx)
                for j_, v_ in zip((1, 2, 3, 4), plan["return_routes"][PROG["sent"]]["state"]): cmd[j_] = v_
            except BaseException: pass
            raise
        # 복귀 색인은 쓰기 직전에 '이 이동이 끝난 뒤' 상태로 지정한다(이미 보낸 이동은 서보가 끝까지 수행, 쓰기 전에 중단되면 do_return이 3° 불일치로 쓰기 없이 멈춘다)
        PROG["sent"] = _idx(plan, "before_j4", int(target)) if joint == 2 else _idx(plan, "after_j4", int(round(_raw(ctx, 2, "J4 이동 전"))))
        ads = dict(ads0); ads[joint] = ad
        mv = {"joint": joint, "target": int(target), "time_ms": ms, "phase": "L", "pose_id": None}
        try: st = ctx["run_move"](ads, arm, mv, cmd, start_all, rec, n0 + k, n0 + 3, ctx["mode"], P["move_countdown_s"])
        except BaseException as exc_:
            lift["abort_class"] = classify_abort(exc_)
            if getattr(exc_, "before_none", False): lift["code"] = "BEFORE_NONE"; lift["lifted_once"] = bool(lifted)   # 쓰기 전 중단: 이 이동 전까지의 상태(181행 `lifted = True`보다 먼저 잡는다)
            save_logs()
            try:
                PROG["sent"] = _stage_idx(ctx)                                                     # 중단되면 실제 상태에 가장 가까운 복귀 항목(불일치면 do_return이 쓰기 없이 멈춘다)
                for j_, v_ in zip((1, 2, 3, 4), plan["return_routes"][PROG["sent"]]["state"]): cmd[j_] = v_   # run_move가 중단 전에 cmd에 적은 '목표'(예: 무반응이었던 77)를 실제 상태 항목 값으로 되돌린다(복귀의 비명령 관절 이탈 검사)
            except BaseException: pass
            lift["moves"].append({"step": step, "joint": joint, "target": int(target), "aborted": True}); lifted = True; raise
        fv_ = (st.get("final") or {}).get(str(joint))   # 이동 성공 뒤 그 관절의 명령 기준점을 run_move가 이미 모은 실제 정수 회독으로 갱신한다(사용자 승인 2026-11-19; 새로 읽지 않음, None이면 목표값 유지, HOLD_TOL 3·run_move 불변)
        if fv_ is not None: cmd[joint] = fv_                # 비명령 관절 이탈 검사가 서보 정상 부족분(약 2°)을 섞지 않게 한다
        lift["moves"].append(dict(st, step=step, target=int(target))); lifted = True; PROG["sent"] = _stage_idx(ctx)
        return st
    try:
        rd0 = _raw_all(ctx, "누름 중(첫 이동 전)"); j2_start = rd0[2]; first = plan["lift"]["j2_first_target"]
        if first > plan["lift"]["j2_max_target"]: raise ValueError("계획의 첫 J2 명령이 최대 목표를 넘는다")
        tc = (lift.get("gate") or {}).get("tilt_check") or {}; pmax = plan["lift"]["j2_max_target"]
        def sag(phase, rd):   # P10 처짐 cap(사용자 결정 2026-11-18): 이동 0회 거부(누름 중) 또는 cap(첫 이동 뒤). 둘 다 같은 30° 규칙(L.sag_cap). 입력 없음은 거부(fail-closed)
            r = L.sag_cap(pmax, tc.get("margin"), (rec or {}).get("raw_q_at_contact"), rd, P["lift_j2_refuse_at_or_below"], first); lift.setdefault("sag", {})[phase] = r
            if r["refuse"]: raise _SagStop(phase, r)
            return r["cap"]
        sag("early", rd0)
        st = do_move("lift1", 2, first, 1); rd1 = _raw_all(ctx, "첫 이동 뒤"); j2_read = rd1[2]; cap1 = sag("late", rd1)
        t2, info = L.second_j2_command(first, j2_start, j2_read, dict(P, lift_j2_max_target=cap1)); lift["second_j2"] = dict(info, target=t2, j2_start=j2_start, j2_after_first=j2_read)
        if t2 is not None and t2 - j2_read < P["lift_j2_min_effective_step_deg"]:   # 요청 폭이 서보 최소 유효 폭(위 방향, 세트 16)보다 작으면 보내지 않는다(조용한 반쪽 이동 방지, P10 조건 ①)
            lift["second_j2"]["skipped_reason"] = "요청 폭 %.2f° < 최소 유효 폭 %.1f°" % (t2 - j2_read, P["lift_j2_min_effective_step_deg"]); say("  두 번째 J2 이동 생략: %s" % lift["second_j2"]["skipped_reason"]); t2 = None
        if t2 is not None:
            do_move("lift2", 2, t2, 2); moved = _raw(ctx, 2, "둘째 이동 뒤") - j2_read; lift["second_j2"]["moved_deg"] = moved
            if moved < P["lift_j2_no_response_deg"]: lift["second_j2"]["no_response"] = True; say("  ! 두 번째 J2 무반응 의심(원시 변화 %.2f°). 재전송하지 않고 J4로 진행합니다(덜 들어올린 쪽)." % moved)
        do_move("lift3", 4, int(cmd[4]) + plan["lift"]["j4_delta"], 3)
        # --- 사진 B → 유지 → 사진 C ---
        b = photo("B"); lift["hold_done"] = False; lift["photo_c_done"] = False
        if b is not None and L.same_start([ref.get("started_epoch"), b.get("started_epoch")]):
            if clk.may_start("hold", ubs["hold"]):
                _set_label(ctx, "hold")
                time.sleep(P["hold_s"]); lift["hold_done"] = True
                c = photo("C"); lift["photo_c_done"] = c is not None and L.same_start([ref.get("started_epoch"), c.get("started_epoch")])
        lift["j6_after_lift"] = ctx["adc"].read_position()
        done("완료" if lift["photo_c_done"] else ("유지·사진 C 일부 생략" if lift["hold_done"] else "사진 B 이후 생략"))
    except _SagStop as e:
        # 누름 중(early)이면 이동 0회라 lifted=False → 아래에서 no_lift(러너의 기존 열기 경로). 첫 이동 뒤(late)면 이미 첫 J2 이동(74)이 나간 상태: 시계 중단과 같은 경로(그 높이에서 열기 1회 + 들어올린 상태 복귀)로 끝낸다.
        # 러너 내부 반환값 "lifted"는 복귀 경로를 위한 값일 뿐이고, 기록·분류는 lift["code"]/lifted_once로 구분한다(시도 + no_lift, 처짐 과다)
        lift["code"] = "SAG_" + e.args[0].upper(); lift["lifted_once"] = bool(lifted); done("처짐 과다(%s): %s — %s" % (e.args[0], e.args[1].get("reason"), "첫 이동 뒤 그 높이에서 열기" if lifted else "이동 없이 열기"))
    except _ClockStop as e:
        lift["code"] = "CLOCK"; done("시계 규칙으로 단계 '%s' 시작 전 중단(경과 + 상한 > %.1f s): 즉시 열기" % (e.args[0], P["clock_limit_s"]))
    save_logs()
    # --- 낙하 열기(시계의 끝) ---
    if not lifted: start_all[6] = ctx["cfg"]["open_j6"]; return "no_lift"
    lift["elapsed_before_open_s"] = clk.elapsed()   # 기록 전용(B): 열기 명령 직전 경과 = 누름 구간(플랜 R2 "D 제외" 예산)의 끝. `elapsed_at_open_s`는 열기 뒤 폴링이 끝난 시각이라 낙하(D) 시간을 포함한다
    lift["open_readback"] = ctx["release"](); lift["elapsed_at_open_s"] = clk.elapsed(); ctx["start_all"][6] = ctx["cfg"]["open_j6"]   # 열린 뒤 J6 감시 기준을 열림 값으로 되돌린다
    return "lifted"


class _ClockStop(Exception): pass


class _SagStop(Exception): pass


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
