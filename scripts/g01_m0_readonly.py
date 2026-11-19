#!/usr/bin/env python3
"""M0 읽기 전용 정지 측정(플랜 M0, 사용자 입회, 쓰기 0·팔 정지). 서보 읽기 None 비율을 읽기 패턴별로 표로 기록한다. 판정 없음.
사용(Jetson): python3 g01_m0_readonly.py --out m0.json [--n N] [--sanity] [--fake]
사전 등록(측정 전 고정, 변경은 새 이름 도구로만):
 - 시행 = 아래 조건의 읽기 패턴 1회(읽기 2개, 위치 0·1을 따로 센다). 시행 사이 조용한 시간 0.3 s(기존 QUIET_S 재사용). 조건은 시행 번호마다 고정 순서로 한 바퀴씩 돈다(시간 흐름이 한 조건에 몰리지 않게).
 - 조건: C01~C04 간격 없음(같은 J2, 같은 J4, J2→J4, J4→J2), C05~C08 같은 서보 간격 0.05 s/0.1 s(J2·J4 각각),
   C09~C12 ④ raw_set(J1~J4 원시 4회) → 간격 0.05/0.1 s → SDK 읽기(J2·J4 각각), C13~C16 ④ SDK 읽기(read_start 대응) → 간격 0.05/0.1 s → SDK 읽기(J2·J4 각각; 호출 순서는 C05~C08과 구성상 동일, 반복 측정 역할),
   C17~C18 대조군: 관찰된 J4 실패 순서 그대로 raw_set → 간격 없음(0 s) → SDK 읽기(J2·J4 각각, 기록 전용 항목).
 - 세는 규칙: None(원시 읽기는 반환 None 또는 예외)만 센다. 재측정으로 고르지 않는다. --sanity(시행 3회씩, 시간 실측용) 실행의 None은 표에 넣지 않는다. 판정 문구 없음.
 - 쓰기·토크 API는 대리 객체가 예외로 막는다(허용: Arm_serial_servo_read, 읽기 요청 바이트(레지스터 0x31~0x36)와 read_word_data만).
 - 중지 조건: 쓰기가 필요해지거나(대리 객체 예외) Ctrl-C·서보 이상 신호 시 즉시 중지하고 그때까지의 기록을 저장한다."""
import argparse, datetime, json, sys, time

QUIET_S = 0.3   # 기존 쓰기 직전 조용한 시간 상수(g01_5b4_s42_test.QUIET_S = 0.3)를 재사용(새 숫자 없음)
READ_REGS = {0x31, 0x32, 0x33, 0x34, 0x35, 0x36}   # jid + 0x30 (jid 1~6)


class WriteBlocked(Exception): pass


class _BusRO:
    def __init__(self, bus): self._bus = bus
    def write_byte_data(self, addr, reg, val):
        if reg not in READ_REGS or val != 0: raise WriteBlocked("버스 쓰기 차단: reg=%r val=%r" % (reg, val))
        return self._bus.write_byte_data(addr, reg, val)   # 읽기 요청 바이트(SDK와 같은 호출)
    def read_word_data(self, addr, reg): return self._bus.read_word_data(addr, reg)
    def __getattr__(self, name): raise WriteBlocked("버스 API 차단: " + name)


class ReadOnlyArm:
    """허용 목록: Arm_serial_servo_read, bus(읽기 요청만), addr. 나머지(쓰기·토크·부저 등)는 모두 예외."""
    def __init__(self, arm):
        object.__setattr__(self, "_arm", arm); object.__setattr__(self, "_bus", _BusRO(arm.bus) if hasattr(arm, "bus") and arm.bus is not None else None)
    def Arm_serial_servo_read(self, jid): return self._arm.Arm_serial_servo_read(jid)
    @property
    def bus(self): return self._bus
    @property
    def addr(self): return self._arm.addr
    def __getattr__(self, name):
        if name == "raw_servo_read": return getattr(self._arm, name)   # 가짜 팔 전용 읽기
        raise WriteBlocked("팔 API 차단: " + name)
    def __setattr__(self, k, v): raise WriteBlocked("속성 쓰기 차단: " + k)


def read_raw(arm, jid):
    """g01_5b4_s42_test.read_raw와 같은 원시 읽기(읽기 전용). 실패하면 None."""
    try:
        f = getattr(arm, "raw_servo_read", None)
        if f is not None: return f(jid)
        arm.bus.write_byte_data(arm.addr, jid + 0x30, 0); time.sleep(0.003)
        w = arm.bus.read_word_data(arm.addr, jid + 0x30)
        return None if w == 0 else ((w >> 8 & 0xff) | (w << 8 & 0xff00))
    except WriteBlocked: raise
    except Exception: return None


def raw_set(arm):
    return {k: read_raw(arm, k) for k in (1, 2, 3, 4)}


def _sdk(arm, j): return arm.Arm_serial_servo_read(j)


def _cond_table():
    t = []
    t += [("C01", "같은 J2, 간격 없음", lambda a, g: (lambda: _sdk(a, 2), lambda: _sdk(a, 2), 0.0)),
          ("C02", "같은 J4, 간격 없음", lambda a, g: (lambda: _sdk(a, 4), lambda: _sdk(a, 4), 0.0)),
          ("C03", "J2→J4, 간격 없음", lambda a, g: (lambda: _sdk(a, 2), lambda: _sdk(a, 4), 0.0)),
          ("C04", "J4→J2, 간격 없음", lambda a, g: (lambda: _sdk(a, 4), lambda: _sdk(a, 2), 0.0))]
    for i, (j, g) in enumerate([(2, 0.05), (4, 0.05), (2, 0.1), (4, 0.1)], 5):
        t.append(("C%02d" % i, "같은 J%d, 간격 %.2f s" % (j, g), (lambda j, g: lambda a, _g: (lambda: _sdk(a, j), lambda: _sdk(a, j), g))(j, g)))
    for i, (j, g) in enumerate([(2, 0.05), (4, 0.05), (2, 0.1), (4, 0.1)], 9):
        t.append(("C%02d" % i, "④ raw_set→%.2f s→SDK J%d" % (g, j), (lambda j, g: lambda a, _g: (lambda: raw_set(a), lambda: _sdk(a, j), g))(j, g)))
    for i, (j, g) in enumerate([(2, 0.05), (4, 0.05), (2, 0.1), (4, 0.1)], 13):
        t.append(("C%02d" % i, "④ SDK J%d→%.2f s→SDK J%d" % (j, g, j), (lambda j, g: lambda a, _g: (lambda: _sdk(a, j), lambda: _sdk(a, j), g))(j, g)))
    for i, j in ((17, 2), (18, 4)):
        t.append(("C%02d" % i, "대조군 raw_set→0 s→SDK J%d" % j, (lambda j: lambda a, _g: (lambda: raw_set(a), lambda: _sdk(a, j), 0.0))(j)))
    return t


def _none(v):
    if isinstance(v, dict): return any(x is None for x in v.values())   # raw_set: 4회 중 하나라도 None
    return v is None


def run(arm, n, sanity=False, quiet_s=QUIET_S):
    conds = _cond_table(); rec = {c[0]: {"label": c[1], "none": [0, 0], "n": 0, "t_first": [], "t_second": []} for c in conds}; err = None
    try:
        for i in range(n):
            for cid, _lab, mk in conds:
                f1, f2, gap = mk(arm, None); a = time.perf_counter(); v1 = f1(); b = time.perf_counter()
                if gap: time.sleep(gap)
                c = time.perf_counter(); v2 = f2(); d = time.perf_counter(); r = rec[cid]
                r["n"] += 1; r["none"][0] += _none(v1); r["none"][1] += _none(v2); r["t_first"].append(round(b - a, 4)); r["t_second"].append(round(d - c, 4))
                time.sleep(quiet_s)
    except BaseException as e: err = repr(e)   # 쓰기 차단·Ctrl-C·SIGHUP(SystemExit)·서보 이상 등 어떤 중지든 그때까지의 기록을 돌려준다
    return rec, err


def table(rec, sanity=False):
    if sanity:   # sanity는 None을 화면에도 JSON에도 내지 않는다(사전 등록: sanity의 None은 표에 넣지 않음, N을 None 수로 정하지 않기 위함)
        lines = ["조건  내용                                  시행  평균 소요 첫째/둘째(ms)"]
        for cid, r in rec.items():
            n = r["n"]; m1 = 1000 * sum(r["t_first"]) / n if n else 0; m2 = 1000 * sum(r["t_second"]) / n if n else 0
            lines.append("%s  %-34s %5d  %.1f / %.1f" % (cid, r["label"], n, m1, m2))
        return "\n".join(lines)
    lines = ["조건  내용                                  시행  None(첫째)  None(둘째)  평균 소요 첫째/둘째(ms)"]
    for cid, r in rec.items():
        n = r["n"]; m1 = 1000 * sum(r["t_first"]) / n if n else 0; m2 = 1000 * sum(r["t_second"]) / n if n else 0
        lines.append("%s  %-34s %5d  %5d/%-5d %5d/%-5d  %.1f / %.1f" % (cid, r["label"], n, r["none"][0], n, r["none"][1], n, m1, m2))
    return "\n".join(lines)


class FakeArm:
    """로컬 시험용: 읽기만 있는 가짜 팔. none_every=k이면 k번째 호출마다 None."""
    def __init__(self, none_every=0): self.calls = 0; self.none_every = none_every; self.bus = None
    def Arm_serial_servo_read(self, jid):
        self.calls += 1
        return None if self.none_every and self.calls % self.none_every == 0 else 90
    def raw_servo_read(self, jid):
        self.calls += 1
        return None if self.none_every and self.calls % self.none_every == 0 else 2000
    def Arm_serial_servo_write(self, *a): raise AssertionError("쓰기가 호출되면 안 된다")


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--out", required=True); ap.add_argument("--n", type=int, default=0); ap.add_argument("--sanity", action="store_true"); ap.add_argument("--fake", action="store_true")
    a = ap.parse_args()
    if a.sanity: n = 3
    elif a.n > 0: n = a.n
    else: sys.exit("--n(사전 등록 값) 또는 --sanity가 필요하다")
    import signal; signal.signal(signal.SIGHUP, lambda *_: sys.exit("SIGHUP"))   # 터미널이 끊겨도 부분 기록을 저장한다
    arm = FakeArm(7) if a.fake else __import__("Arm_Lib").Arm_Device()
    ro = ReadOnlyArm(arm); t0 = time.perf_counter()
    rec, err = run(ro, n, a.sanity, 0.0 if a.fake else QUIET_S)
    if a.sanity:
        for r in rec.values(): r["none"] = "제외(sanity)"
    out = {"tool": "g01_m0_readonly", "mode": "fake" if a.fake else "real_readonly", "sanity": a.sanity, "n_per_condition": n, "started_at": datetime.datetime.now().isoformat(timespec="seconds"),
           "elapsed_s": round(time.perf_counter() - t0, 1), "stopped": err, "quiet_s": QUIET_S, "conditions": rec,
           "loop_s_per_round": round((time.perf_counter() - t0) / n, 2), "note": "판정 없음. sanity 실행의 None은 표에 넣지 않는다. 정지 측정이라 누름 하중 상태의 실패와 다를 수 있다."}
    json.dump(out, open(a.out, "w"), ensure_ascii=False, indent=1)
    print(table(rec, a.sanity)); print("소요 %.1f s, 중지 사유: %s" % (out["elapsed_s"], err or "없음"))


if __name__ == "__main__": main()
