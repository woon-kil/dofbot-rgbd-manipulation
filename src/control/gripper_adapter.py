"""G01.5a 그리퍼(서보 id 6) 시험 명령 어댑터. 하드웨어에 직접 연결하지 않는다.

SDK 소스(Arm_Lib.py)에서 확인한 사실: Arm_serial_servo_write에는 각도 범위 검사가 없고 id 0은 6개 서보 전체 쓰기다.
그래서 이 어댑터는 id를 6으로 고정하고(인자로 받지 않음), 승인된 절대 목표 목록·시간·횟수만 허용한다.
"""
import datetime

GRIPPER_ID = 6


class GripperTestError(Exception):
    pass


class GripperTestAdapter:
    def __init__(self, arm, approval, today=None, dry_run=True):
        self._arm = arm
        self._dry_run = bool(dry_run)
        today = today or datetime.date.today()
        if approval.get("approved") is not True:
            raise GripperTestError("승인되지 않았다")
        if datetime.date.fromisoformat(approval["expires"]) < today:
            raise GripperTestError("승인이 만료되었다")
        self._times = tuple(int(t) for t in approval["allowed_times_ms"])
        if not self._times or any(t < 1500 for t in self._times):
            raise GripperTestError("허용 이동 시간은 1500 ms 이상이어야 한다")
        self._max_writes = int(approval["max_writes"])
        self._delta = int(approval["delta_deg"])
        if self._delta <= 0:
            raise GripperTestError("소각도는 양수여야 한다")
        self._start = None
        self._targets = None
        self._next = 0
        self._writes = 0
        self._hands_clear = False
        self.log = []

    def read_start(self):
        """시작값 S를 한 번 읽는다(이전 프로젝트 규칙 5의 명시적 예외 1건). None이면 중단."""
        if self._start is not None:
            raise GripperTestError("시작값은 한 번만 읽는다")
        value = self._arm.Arm_serial_servo_read(GRIPPER_ID)
        self.log.append(("read", value))
        if value is None:
            raise GripperTestError("회독이 None이다")
        if not isinstance(value, int) or not 0 <= value <= 180:
            raise GripperTestError("회독 값이 범위 밖이다")
        self._start = value
        return value

    def set_targets(self, targets):
        """사용자가 확인한 절대 목표 목록(S, S-Δ, S-2Δ, S)을 고정한다. 이후 회독으로 다시 계산하지 않는다."""
        if self._start is None:
            raise GripperTestError("시작값을 먼저 읽어야 한다")
        if self._targets is not None:
            raise GripperTestError("목표 목록은 한 번만 설정한다")
        s, d = self._start, self._delta
        if list(targets) != [s, s - d, s - 2 * d, s]:
            raise GripperTestError("목표 목록이 S, S-Δ, S-2Δ, S와 다르다")
        if any(not 0 <= t <= 180 for t in targets):
            raise GripperTestError("목표가 0~180 밖이다")
        self._targets = tuple(targets)
        self._next = 1  # 첫 값 S는 현재 위치(측정만, 쓰지 않음). 쓰기는 두 번째 값부터

    def confirm_hands_clear(self):
        """사용자가 손과 캘리퍼를 뗐다고 명시적으로 답한 뒤에만 호출한다. 이동 한 번에 한 번만 유효."""
        self._hands_clear = True

    def move(self, target, time_ms):
        if self._targets is None:
            raise GripperTestError("목표 목록이 설정되지 않았다")
        if not self._hands_clear:
            raise GripperTestError("손 제거 확인이 없다")
        if self._next >= len(self._targets):
            raise GripperTestError("목표 목록을 모두 사용했다")
        if target != self._targets[self._next]:
            raise GripperTestError("다음 승인된 절대 목표가 아니다")
        if time_ms not in self._times:
            raise GripperTestError("승인된 이동 시간이 아니다")
        if self._writes >= self._max_writes:
            raise GripperTestError("최대 쓰기 횟수를 넘는다")
        self._hands_clear = False
        self._writes += 1
        self._next += 1
        self.log.append(("write", target, time_ms, "dry_run" if self._dry_run else "real"))
        if not self._dry_run:
            self._arm.Arm_serial_servo_write(GRIPPER_ID, target, time_ms)

    def read_position(self):
        """도착 확인용 회독(서보 6만)."""
        value = self._arm.Arm_serial_servo_read(GRIPPER_ID)
        self.log.append(("read", value))
        return value
