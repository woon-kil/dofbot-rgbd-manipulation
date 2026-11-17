#!/usr/bin/env python3
"""P-A 0단계: 사진 1장 36 s 구성 시간 분해 보고(읽기 전용, 판정·한도 없음, 측정 기록일 뿐이다).
사용: python3 scripts/pa0_timing_report.py data/g04/<이름>   (파일: <이름>.mac_timing.txt, .host_timing.txt, .container_timing.txt, .p620_clock.txt)
시계 규칙: Mac 시계 구간과 p620 시계 구간은 서로 다른 기계라 **구간 길이만 같은 기계 안에서** 뺀다. p620 호스트와 컨테이너는 같은 기계(같은 커널 시계)라 함께 쓴다.
Mac–p620 오프셋은 참고값(Mac 앞뒤 사이 p620 epoch 1회, 불확실도 = 왕복/2)이며 구간 계산에 쓰지 않는다."""
import sys
from pathlib import Path

MAC = ["m_start", "m_p620_check_start", "m_p620_check_done", "m_capture_start", "m_capture_done", "m_scp_done", "m_verify_done"]
P620 = ["h_script_start", "h_checks_done", "h_old_container_removed", "h_container_up", "h_exec_start", "c_cfg_start", "c_driver_launched",
        "c_serial_reported", "c_sleep12_done", "c_probe_done", "c_kill_sleep4_done", "c_pgrep_stage_done", "c_end", "h_exec_done", "h_cleanup_start", "h_cleanup_end"]
CONST = {"c_serial_reported→c_sleep12_done": 12.0, "c_sleep12_done→c_probe_done": 15.0, "c_probe_done→c_kill_sleep4_done": 4.0}   # 코드 상수(원본 g01_gemini_check.sh)


def parse(path):
    d = {}
    p = Path(path)
    if not p.exists(): return d
    for line in p.read_text(encoding="utf-8", errors="replace").splitlines():
        parts = line.split()
        if len(parts) == 2:
            try: d[parts[0]] = float(parts[1])
            except ValueError: pass
    return d


def stages(marks, order):
    out = []
    have = [k for k in order if k in marks]
    for a, b in zip(have, have[1:]): out.append((a, b, marks[b] - marks[a]))
    missing = [k for k in order if k not in marks]
    return out, missing


def clock_offset(mac, p620_clock_path):
    p = Path(p620_clock_path)
    if not p.exists() or "m_clk_a" not in mac or "m_clk_b" not in mac: return None
    first = (p.read_text(errors="replace").splitlines() or [""])[0].strip()
    try: remote = float(first)
    except ValueError: return None
    mid = (mac["m_clk_a"] + mac["m_clk_b"]) / 2; unc = (mac["m_clk_b"] - mac["m_clk_a"]) / 2
    return remote - mid, unc


def report(base):
    base = str(base)
    mac = parse(base + ".mac_timing.txt"); p620 = {**parse(base + ".host_timing.txt"), **parse(base + ".container_timing.txt")}
    lines = ["P-A 0단계 시간 분해 (측정 기록일 뿐, 판정·한도 없음)", "[Mac 시계 구간]"]
    ms, miss = stages(mac, MAC)
    for a, b, d in ms: lines.append("  %-20s → %-20s %7.2f s" % (a, b, d))
    if "m_start" in mac and "m_verify_done" in mac: lines.append("  Mac 전체(m_start→m_verify_done): %.2f s" % (mac["m_verify_done"] - mac["m_start"]))
    if miss: lines.append("  (없는 표시: %s)" % ", ".join(miss))
    lines.append("[p620 시계 구간(호스트+컨테이너 같은 기계)]")
    ps, miss2 = stages(p620, P620)
    for a, b, d in ps:
        k = "%s→%s" % (a, b); extra = ("  (코드 상수 %.0f s)" % CONST[k]) if k in CONST else ""
        lines.append("  %-24s → %-24s %7.2f s%s" % (a, b, d, extra))
    if "h_script_start" in p620 and "h_cleanup_end" in p620: lines.append("  p620 스크립트 전체(h_script_start→h_cleanup_end): %.2f s" % (p620["h_cleanup_end"] - p620["h_script_start"]))
    if miss2: lines.append("  (없는 표시: %s)" % ", ".join(miss2))
    if all(k in mac for k in ("m_capture_start", "m_capture_done")) and all(k in p620 for k in ("h_script_start", "h_cleanup_end")):
        lines.append("[구간 길이 비교(같은 기계 안에서만 뺌)]  Mac 촬영 구간 %.2f s − p620 스크립트 %.2f s = ssh·전송 오버헤드 %.2f s" % (
            mac["m_capture_done"] - mac["m_capture_start"], p620["h_cleanup_end"] - p620["h_script_start"],
            (mac["m_capture_done"] - mac["m_capture_start"]) - (p620["h_cleanup_end"] - p620["h_script_start"])))
    off = clock_offset(mac, base + ".p620_clock.txt")
    if off: lines.append("[참고] Mac–p620 시계 오프셋 %+.3f s (불확실도 ±%.3f s, 구간 계산에 쓰지 않음)" % off)
    return "\n".join(lines)


if __name__ == "__main__":
    if len(sys.argv) != 2: sys.exit("사용: python3 scripts/pa0_timing_report.py data/g04/<이름>")
    print(report(sys.argv[1]))
