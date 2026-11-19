"""서보별 최소 재읽기 간격 가드(M5, 플랜 v2 `docs/261119-1206-읽기-None-정렬-게이트-수정-플랜-v2.md` §4, 사용자 승인 2026-11-19).
M0(n=136, 정지 상태): 같은 서보를 직전 읽기 직후 간격 0으로 다시 읽으면 None 100%, 다른 서보를 끼우거나 0.05 s 이상 간격이면 0/136. 이 가드는 `arm` 한 곳에서
**같은 서보 번호의 읽기(SDK `Arm_serial_servo_read`, 원시 버스 읽기 `jid + 0x30`, 가짜 팔의 `raw_servo_read`)가 그 서보의 직전 읽기 응답이 돌아온 시각으로부터 ≥ 0.05 s 뒤**에 나가게 한다
(사이에 다른 읽기가 있어도 적용, M0 간격 정의와 같음). 0.05 s는 `g01_lift_hook._raw` 재시도에 이미 쓰는 값을 새 용도로 쓴다. 쓰기·그 밖의 호출은 가드를 거치지 않고 그대로 전달한다.
대기할 때마다 (서보, 대기 시간 s, 단계 라벨)을 `wait_log`에 남긴다(기록 전용). 세트 33 실행에서 `Arm_Device`/가짜 팔을 고르는 한 줄 바로 뒤에서 감싼다(실행 모드와 무관)."""
import time

MIN_GAP_S = 0.05
_REG_LO, _REG_HI = 0x31, 0x36   # 읽기 요청 레지스터 = jid + 0x30 (jid 1~6)


class _GuardBus:
    def __init__(self, guard, bus): object.__setattr__(self, "_g", guard); object.__setattr__(self, "_bus", bus)
    def write_byte_data(self, addr, reg, val):
        if _REG_LO <= reg <= _REG_HI and val == 0: self._g._wait(reg - 0x30)   # 읽기 요청 바이트: 같은 서보 직전 읽기 응답 뒤 ≥ 0.05 s
        return self._bus.write_byte_data(addr, reg, val)
    def read_word_data(self, addr, reg):
        try: return self._bus.read_word_data(addr, reg)
        finally:
            if _REG_LO <= reg <= _REG_HI: self._g._done(reg - 0x30)
    def __getattr__(self, name): return getattr(self._bus, name)


class ReadGuardArm:
    MIN_GAP_S = MIN_GAP_S
    def __init__(self, arm, sleep=None, now=None, min_gap_s=MIN_GAP_S):
        d = self.__dict__; d["_arm"] = arm; d["_sleep"] = sleep; d["_now"] = now; d["_gap"] = float(min_gap_s); d["_end"] = {}; d["_label"] = "창 밖"; d["wait_log"] = []
    def _t(self): return self._now() if self._now else time.monotonic()
    def _wait(self, jid):
        last = self._end.get(jid)
        if last is None: return
        w = self._gap - (self._t() - last)
        if w > 1e-9:   # 부동소수점 잔여는 대기·기록하지 않는다
            self.wait_log.append((jid, round(w, 6), self._label)); (self._sleep or time.sleep)(w)
    def _done(self, jid): self._end[jid] = self._t()
    def set_label(self, label): self.__dict__["_label"] = label
    def Arm_serial_servo_read(self, jid):
        self._wait(jid)
        try: return self._arm.Arm_serial_servo_read(jid)
        finally: self._done(jid)
    @property
    def bus(self):
        try: b = self._arm.bus
        except (AttributeError, AssertionError): raise AttributeError("bus")
        if b is None: return None
        return _GuardBus(self, b)
    def __getattr__(self, name):
        if name == "raw_servo_read":   # 가짜 팔 전용(실제 Arm_Device에는 없다): 같은 시각표를 거친다
            try: f = getattr(self._arm, name)
            except (AttributeError, AssertionError): raise AttributeError(name)
            def raw(jid):
                self._wait(jid)
                try: return f(jid)
                finally: self._done(jid)
            return raw
        if name.startswith("__"): raise AttributeError(name)
        return getattr(self._arm, name)
    def __setattr__(self, name, value):
        if name in self.__dict__ or name in ("wait_log",): self.__dict__[name] = value
        else: setattr(self._arm, name, value)
