"""G01.5a/b 단일 서보 시험 명령 어댑터(그리퍼 id 6, G01.5b-1 손목 회전 id 5, G01.5b-2 밑동 회전 id 1, G01.5b-3 피치 관절 id 2~4). 하드웨어에 직접 연결하지 않는다.

SDK 소스(Arm_Lib.py)에서 확인한 사실: Arm_serial_servo_write에는 각도 범위 검사가 없고 id 0은 6개 서보 전체 쓰기다.
그래서 이 어댑터는 id를 6으로 고정하고(인자로 받지 않음), 승인된 절대 목표 목록·시간·횟수만 허용한다.
"""
import datetime

GRIPPER_ID = 6
# 이 어댑터가 허용하는 서보와 SDK 각도 범위(Arm_Lib.py 원문). id 0(6개 서보 전체)과 1~6 밖은 코드로 거부한다. J2~J4(id 2~4, 중력 하중이 걸리는 피치 관절)는 G01.5b-3 시험용으로 허용하며 실제 실행은 관절별 시험 당일 확인 문구 입력 때만 가능하다.
ALLOWED_JOINTS = {1: (0, 180), 2: (0, 180), 3: (0, 180), 4: (0, 180), 5: (0, 270), 6: (0, 180)}


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
        # 동결 상한의 "첫 이동" 규칙(ADR-005): 첫 쓰기의 이동 시간이 이 값보다 짧으면 거부한다. 승인 객체에 키가 없으면 검사하지 않는다(G01.5a 그리퍼 시험도 이 규칙을 적용하지 않았다. 목적이 단위 실측이었다는 사정이 있었으나 이탈이 아니었다고 하지 않는다).
        fm = approval.get("first_move_min_ms")
        self._first_min = int(fm) if fm is not None else None
        if self._first_min is not None and self._first_min not in self._times:
            raise GripperTestError("first_move_min_ms는 허용 이동 시간 목록에 있어야 한다")
        self._delta = int(approval["delta_deg"])
        if self._delta <= 0:
            raise GripperTestError("소각도는 양수여야 한다")
        self._id = int(approval.get("joint_id", GRIPPER_ID))
        if self._id not in ALLOWED_JOINTS:
            raise GripperTestError("허용되지 않은 서보 id: %r" % self._id)
        self._max_deg = ALLOWED_JOINTS[self._id][1]
        self._dir = int(approval.get("direction", -1))
        if self._dir not in (-1, 1):
            raise GripperTestError("direction은 -1 또는 +1이어야 한다")
        window = approval.get("angle_window")
        self._window = (int(window[0]), int(window[1])) if window else (0, self._max_deg)
        if not 0 <= self._window[0] < self._window[1] <= self._max_deg:
            raise GripperTestError("angle_window가 SDK 범위 밖이다")
        self._pattern = approval.get("pattern", "ramp")
        if self._pattern not in ("ramp", "pair", "stair", "list"):
            raise GripperTestError("pattern은 ramp, pair, stair, list 중 하나여야 한다")
        self._stairs = int(approval.get("stair_steps", 0))
        if self._pattern == "stair" and self._stairs < 1:
            raise GripperTestError("stair 패턴은 stair_steps >= 1이 필요하다")
        # 동결 상한(ADR-005): 이동 한 번의 각도는 90° 이하, 이동 시간은 60 ms/° 이상. 시간 목록만 보던 이전 검사에 각도 기준을 코드로 추가했다.
        self._max_move_deg = 90
        self._min_ms_per_deg = 60
        self._cur = None
        self._start = None
        self._last_write = None
        self._repeat_used = False
        self._targets = None
        self._next = 0
        self._writes = 0
        self._hands_clear = False
        self._returned = False
        self.log = []

    def read_start(self):
        """시작값 S를 한 번 읽는다(이전 프로젝트 규칙 5의 명시적 예외 1건). None이면 중단."""
        if self._start is not None:
            raise GripperTestError("시작값은 한 번만 읽는다")
        value = self._arm.Arm_serial_servo_read(self._id)
        self.log.append(("read", value))
        if value is None:
            raise GripperTestError("회독이 None이다")
        if not isinstance(value, int) or not self._window[0] <= value <= self._window[1]:
            raise GripperTestError("회독 값이 승인 범위 밖이다")
        self._start = value
        self._cur = value
        return value

    def set_targets(self, targets):
        """사용자가 확인한 절대 목표 목록(S, S-Δ, S-2Δ, S)을 고정한다. 이후 회독으로 다시 계산하지 않는다."""
        if self._start is None:
            raise GripperTestError("시작값을 먼저 읽어야 한다")
        if self._targets is not None:
            raise GripperTestError("목표 목록은 한 번만 설정한다")
        s, d = self._start, self._delta
        k = self._dir
        if self._pattern == "list":
            # 명시 목표 목록(S4-2): 사용자·계획이 확인한 절대 목록. 첫 항목은 시작 읽기값 S여야 하고, 이동마다 폭·속도·범위 검사는 move()가 한다.
            lst = [int(x) for x in targets]
            if len(lst) < 2 or lst[0] != s:
                raise GripperTestError("list 패턴의 첫 목표는 시작 읽기값이어야 하고 목표가 하나 이상 있어야 한다")
            if any(abs(lst[i + 1] - lst[i]) > self._max_move_deg for i in range(len(lst) - 1)):
                raise GripperTestError("list 패턴의 이동 폭이 1회 상한을 넘는다")
            expected = lst
        elif self._pattern == "ramp": expected = [s, s + k * d, s + 2 * k * d, s]
        elif self._pattern == "pair": expected = [s, s + k * d, s, s + k * d, s]
        else:   # stair: S에서 Δ씩 n단계 접고 같은 계단으로 S까지 되돌린다
            n = self._stairs; expected = [s] + [s + k * d * i for i in range(1, n + 1)] + [s + k * d * i for i in range(n - 1, -1, -1)]
        if list(targets) != expected:
            raise GripperTestError("목표 목록이 승인된 패턴(%s)과 다르다" % self._pattern)
        if any(not self._window[0] <= t <= self._window[1] for t in targets):
            raise GripperTestError("목표가 승인 범위 밖이다")
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
        if self._first_min is not None and self._writes == 0 and time_ms < self._first_min:
            raise GripperTestError("첫 이동은 %d ms 이상이어야 한다(동결 상한)" % self._first_min)
        if self._writes >= self._max_writes:
            raise GripperTestError("최대 쓰기 횟수를 넘는다")
        self._check_speed(target, time_ms)
        self._hands_clear = False
        self._writes += 1
        self._next += 1
        self._cur = target
        self._last_write = (target, time_ms); self._repeat_used = False
        self.log.append(("write", target, time_ms, "dry_run" if self._dry_run else "real"))
        if not self._dry_run:
            self._arm.Arm_serial_servo_write(self._id, target, time_ms)

    def _check_speed(self, target, time_ms):
        """직전 명령 위치(없으면 시작값)에서 target까지의 이동이 90° 이하이고 time_ms >= 60 ms/° 인지 검사한다."""
        dist = abs(int(target) - int(self._cur))
        if dist > self._max_move_deg:
            raise GripperTestError("이동 폭 %d°가 1회 상한 %d°를 넘는다" % (dist, self._max_move_deg))
        if time_ms < self._min_ms_per_deg * dist:
            raise GripperTestError("이동 시간 %d ms가 %d ms/° × %d° = %d ms보다 짧다(동결 상한)" % (time_ms, self._min_ms_per_deg, dist, self._min_ms_per_deg * dist))

    def repeat_last(self, time_ms):
        """직전 이동 쓰기와 **같은 절대 목표·같은 시간**을 한 번만 다시 보낸다(쓰기 누락 의심 시). 손 제거 확인 필요, 이동마다 1회, 쓰기 상한에 포함."""
        if self._writes == 0 or self._last_write is None:
            raise GripperTestError("다시 보낼 직전 쓰기가 없다")
        if self._repeat_used:
            raise GripperTestError("같은 이동의 재전송은 한 번만 가능하다")
        if not self._hands_clear:
            raise GripperTestError("손 제거 확인이 없다")
        target, last_ms = self._last_write
        if time_ms != last_ms:
            raise GripperTestError("재전송 시간은 직전 쓰기와 같아야 한다")
        if self._writes >= self._max_writes:
            raise GripperTestError("최대 쓰기 횟수를 넘는다")
        self._hands_clear = False; self._repeat_used = True; self._writes += 1
        self.log.append(("write", target, time_ms, "dry_run" if self._dry_run else "real", "repeat"))
        if not self._dry_run:
            self._arm.Arm_serial_servo_write(self._id, target, time_ms)

    def read_position(self):
        """도착 확인용 회독(서보 6만)."""
        value = self._arm.Arm_serial_servo_read(self._id)
        self.log.append(("read", value))
        return value

    def return_to_start(self, time_ms):
        """실패·중단 후 시작값 S로 복귀하는 쓰기 1회. 운영자가 명시적으로 요청할 때만 호출한다.
        손 제거 확인이 필요하고 max_writes에 포함되며 한 번만 가능하다. 쓰기가 한 번도 없었으면(이미 S) 거부한다."""
        if self._start is None:
            raise GripperTestError("시작값이 없다")
        if self._returned:
            raise GripperTestError("복귀는 한 번만 가능하다")
        if self._writes == 0:
            raise GripperTestError("쓰기 이력이 없어 복귀가 필요 없다")
        if not self._hands_clear:
            raise GripperTestError("손 제거 확인이 없다")
        if time_ms not in self._times:
            raise GripperTestError("승인된 이동 시간이 아니다")
        if self._writes >= self._max_writes:
            raise GripperTestError("최대 쓰기 횟수를 넘는다")
        self._check_speed(self._start, time_ms)
        self._hands_clear = False
        self._returned = True
        self._cur = self._start
        self._writes += 1
        self.log.append(("write", self._start, time_ms, "dry_run" if self._dry_run else "real", "return_to_start"))
        if not self._dry_run:
            self._arm.Arm_serial_servo_write(self._id, self._start, time_ms)


JointTestAdapter = GripperTestAdapter   # 일반 이름(허용 id는 ALLOWED_JOINTS)


class SixServoTestAdapter:
    """여섯 서보 동시 쓰기(제조사 `Arm_serial_servo_write6`) 어댑터. 움직일 서보(J3·J4 등)는 계단 목표만, 나머지는 **시작 읽기값 그대로**만 허용한다.
    제조사 write6는 6개 값을 한 번에 보내므로 움직이지 않는 서보에도 현재값을 넣는다(2026-11-12 S3 후속: 개별 쓰기 연속 시 J4 누락 이슈).
    쓰기 API는 write6 하나, 읽기 API는 read 하나뿐이다. 이동 폭 90° 이하, 이동 시간 60 ms/° 이상(직전 명령 기준 6축 중 최대 폭), 첫 이동 시간 하한, 쓰기 횟수 상한."""
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
        fm = approval.get("first_move_min_ms")
        self._first_min = int(fm) if fm is not None else None
        if self._first_min is not None and self._first_min not in self._times:
            raise GripperTestError("first_move_min_ms는 허용 이동 시간 목록에 있어야 한다")
        self._moving = tuple(int(j) for j in approval.get("moving_ids", ()))
        if any(j not in ALLOWED_JOINTS for j in self._moving):
            raise GripperTestError("허용되지 않은 서보 id가 moving_ids에 있다")
        self._pattern = approval.get("pattern", "stair")
        if self._pattern not in ("stair", "hold"):
            raise GripperTestError("pattern은 stair 또는 hold여야 한다")
        if self._pattern == "hold" and self._moving:
            raise GripperTestError("hold 패턴은 moving_ids가 비어 있어야 한다")
        if self._pattern == "stair" and not self._moving:
            raise GripperTestError("stair 패턴은 moving_ids가 필요하다")
        self._delta = int(approval.get("delta_deg", 0))
        self._stairs = int(approval.get("stair_steps", 0))
        if self._pattern == "stair" and (self._delta <= 0 or self._stairs < 1):
            raise GripperTestError("stair 패턴은 delta_deg > 0, stair_steps >= 1이 필요하다")
        self._dir = int(approval.get("direction", -1))
        if self._dir not in (-1, 1):
            raise GripperTestError("direction은 -1 또는 +1이어야 한다")
        window = approval.get("angle_window")
        self._window = (int(window[0]), int(window[1])) if window else (0, 180)
        self._hold_tol = int(approval.get("hold_tolerance_deg", 0))
        self._max_move_deg = 90
        self._min_ms_per_deg = 60
        self._start = None      # {id: 회독값}
        self._cur = None
        self._expected = None
        self._next = 0
        self._writes = 0
        self._hands_clear = False
        self._returned = False
        self.log = []

    def read_start6(self):
        """서보 1~6을 각 1회 읽는다. 하나라도 None이면 중단. 움직일 서보는 승인 범위 안이어야 한다."""
        if self._start is not None:
            raise GripperTestError("시작값은 한 번만 읽는다")
        vals = {}
        for i in range(1, 7):
            v = self._arm.Arm_serial_servo_read(i)
            self.log.append(("read", i, v))
            if v is None or not isinstance(v, int):
                raise GripperTestError("서보 %d 회독이 None이다" % i)
            vals[i] = v
        for j in self._moving:
            if not self._window[0] <= vals[j] <= self._window[1]:
                raise GripperTestError("서보 %d 회독 %d가 승인 범위 밖이다" % (j, vals[j]))
        self._start = dict(vals); self._cur = dict(vals)
        return dict(vals)

    def expected_targets(self):
        """승인된 6축 목표 목록(첫 항목은 시작 읽기값). 움직일 서보는 계단, 나머지는 시작값 그대로."""
        if self._start is None:
            raise GripperTestError("시작값을 먼저 읽어야 한다")
        s = self._start; k = self._dir; d = self._delta
        if self._pattern == "hold":
            return [tuple(s[i] for i in range(1, 7)), tuple(s[i] for i in range(1, 7))]
        n = self._stairs
        seq = [0] + list(range(1, n + 1)) + list(range(n - 1, -1, -1))
        return [tuple((s[i] + k * d * m) if i in self._moving else s[i] for i in range(1, 7)) for m in seq]

    def set_targets(self, targets):
        if self._expected is not None:
            raise GripperTestError("목표 목록은 한 번만 설정한다")
        exp = self.expected_targets()
        if [tuple(t) for t in targets] != exp:
            raise GripperTestError("목표 목록이 승인된 패턴(%s)과 다르다" % self._pattern)
        for t in exp:
            for j in self._moving:
                if not self._window[0] <= t[j - 1] <= self._window[1]:
                    raise GripperTestError("목표가 승인 범위 밖이다")
        self._expected = tuple(exp); self._next = 1

    def confirm_hands_clear(self):
        self._hands_clear = True

    def _check_speed(self, target6, time_ms):
        dist = max(abs(int(target6[i - 1]) - int(self._cur[i])) for i in range(1, 7))
        if dist > self._max_move_deg:
            raise GripperTestError("이동 폭 %d°가 1회 상한 %d°를 넘는다" % (dist, self._max_move_deg))
        if time_ms < self._min_ms_per_deg * dist:
            raise GripperTestError("이동 시간 %d ms가 %d ms/° × %d° = %d ms보다 짧다(동결 상한)" % (time_ms, self._min_ms_per_deg, dist, self._min_ms_per_deg * dist))

    def move6(self, target6, time_ms):
        if self._expected is None:
            raise GripperTestError("목표 목록이 설정되지 않았다")
        if not self._hands_clear:
            raise GripperTestError("손 제거 확인이 없다")
        if self._next >= len(self._expected):
            raise GripperTestError("목표 목록을 모두 사용했다")
        target6 = tuple(int(x) for x in target6)
        if target6 != self._expected[self._next]:
            raise GripperTestError("다음 승인된 6축 목표가 아니다")
        if time_ms not in self._times:
            raise GripperTestError("승인된 이동 시간이 아니다")
        if self._first_min is not None and self._writes == 0 and time_ms < self._first_min:
            raise GripperTestError("첫 이동은 %d ms 이상이어야 한다(동결 상한)" % self._first_min)
        if self._writes >= self._max_writes:
            raise GripperTestError("최대 쓰기 횟수를 넘는다")
        self._check_speed(target6, time_ms)
        self._hands_clear = False; self._writes += 1; self._next += 1
        self._cur = {i: target6[i - 1] for i in range(1, 7)}
        self.log.append(("write6", list(target6), time_ms, "dry_run" if self._dry_run else "real"))
        if not self._dry_run:
            self._arm.Arm_serial_servo_write6(*target6, time_ms)

    def read_position(self, joint_id):
        if joint_id not in range(1, 7):
            raise GripperTestError("서보 id는 1~6이어야 한다")
        v = self._arm.Arm_serial_servo_read(joint_id)
        self.log.append(("read", joint_id, v))
        return v

    def return_to_start6(self, time_ms):
        """중단 후 6축을 시작 읽기값으로 되돌리는 쓰기 1회. 손 제거 확인 필요, 쓰기 횟수에 포함, 쓰기 이력이 없으면 거부."""
        if self._start is None:
            raise GripperTestError("시작값이 없다")
        if self._returned:
            raise GripperTestError("복귀는 한 번만 가능하다")
        if self._writes == 0:
            raise GripperTestError("쓰기 이력이 없어 복귀가 필요 없다")
        if not self._hands_clear:
            raise GripperTestError("손 제거 확인이 없다")
        if time_ms not in self._times:
            raise GripperTestError("승인된 이동 시간이 아니다")
        if self._writes >= self._max_writes:
            raise GripperTestError("최대 쓰기 횟수를 넘는다")
        target6 = tuple(self._start[i] for i in range(1, 7))
        self._check_speed(target6, time_ms)
        self._hands_clear = False; self._returned = True; self._writes += 1
        self._cur = dict(self._start)
        self.log.append(("write6", list(target6), time_ms, "dry_run" if self._dry_run else "real", "return_to_start"))
        if not self._dry_run:
            self._arm.Arm_serial_servo_write6(*target6, time_ms)
