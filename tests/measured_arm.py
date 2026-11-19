"""M8 측정 모델 가짜 팔(시험 전용, 플랜 v2 §4): M0(n=136) 측정을 그대로 모델로 쓴다 — 읽기 요청이 직전 버스 요청과 같은 레지스터(같은 서보)이고
그 사이에 다른 서보 요청이 없으며 직전 요청의 응답이 끝난 시각으로부터 0.05 s 미만이면 None(C01·C02·C18 재현, C17·C03·C04·간격 ≥ 0.05 s는 성공).
읽기 1번은 가상 시계로 4.5 ms가 걸린다(sanity 실측 호출당 4~5 ms). SDK 읽기와 원시 읽기(`raw_servo_read`)가 같은 시각표를 쓴다(같은 버스).
`forced_value`: {서보: [값, ...]}이면 그 서보의 다음 SDK 읽기들이 그 값을 차례로 돌려준다(None이 아닌 창 밖 값 모사). `forced_none`: {서보: 남은 횟수}면 그 서보의 다음 SDK 읽기를 그 횟수만큼 None으로 만든다(이동 중 설명되지 않은 None 모사)."""
import time


def make(base):
    class MeasuredArm(base):
        READ_S = 0.0045; GAP_S = 0.05
        bus = None; addr = 0x15   # 가짜 팔은 버스가 없다(M0 대리 객체·가드가 hasattr로 확인)
        def _req(self, jid, f, sdk):
            now = time.monotonic(); last = self.__dict__.get("_last")
            bad = last is not None and last[0] == jid and now - last[1] < self.GAP_S - 1e-9
            v = f(jid); time.sleep(self.READ_S); self._last = (jid, time.monotonic())
            forced = self.__dict__.setdefault("forced_none", {})
            if sdk and forced.get(jid, 0) > 0: forced[jid] -= 1; bad = True
            fv = self.__dict__.setdefault("forced_value", {})
            if sdk and not bad and fv.get(jid): v = fv[jid].pop(0)   # 다음 SDK 읽기 값을 지정(예: 승인 창 밖 값), 한 번씩 소모
            self.__dict__.setdefault("events", []).append((round(now, 4), jid, "None" if bad else "ok", "sdk" if sdk else "raw"))
            return None if bad else v
        def Arm_serial_servo_read(self, id): return self._req(id, lambda j: base.Arm_serial_servo_read(self, j), True)
        def raw_servo_read(self, id): return self._req(id, lambda j: base.raw_servo_read(self, j), False)
    return MeasuredArm
