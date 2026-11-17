"""총 누름 시간 상한(코드 중단 조건). 사용자 승인 전에는 limit_s=None(비활성)이라 기존 시험 동작을 바꾸지 않는다.
누름 = J6 명령이 회독보다 3° 이상 앞서 있는 구간(정체 포함)의 누적 시간. start()는 누름이 시작될 때, stop()은 누름이 풀릴 때(열기 명령 직전) 호출한다.
check()는 누적(진행 중 구간 포함)이 limit_s를 넘으면 BudgetExceeded를 낸다. 호출자는 즉시 열기 경로로 간다(중단 시 추가 이동 없음 규칙과 함께 사용)."""
import time


class BudgetExceeded(Exception):
    pass


class PressBudget:
    def __init__(self, limit_s=None, clock=time.monotonic):
        self.limit_s = limit_s; self._clock = clock; self._t0 = None; self._acc = 0.0

    def start(self):
        if self._t0 is None: self._t0 = self._clock()

    def stop(self):
        if self._t0 is not None: self._acc += self._clock() - self._t0; self._t0 = None

    def used_s(self):
        return self._acc + ((self._clock() - self._t0) if self._t0 is not None else 0.0)

    def check(self):
        if self.limit_s is not None and self.used_s() > self.limit_s:
            raise BudgetExceeded("총 누름 시간 %.1f s가 상한 %.1f s를 넘었다. 즉시 엽니다." % (self.used_s(), self.limit_s))
        return self.used_s()
