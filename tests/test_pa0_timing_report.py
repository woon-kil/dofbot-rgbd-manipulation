"""P-A 0단계 보고 스크립트(scripts/pa0_timing_report.py) 시험: 합성 타임스탬프로 구간 계산·누락 처리·시계 분리."""
import sys, tempfile
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
import pa0_timing_report as R


def _write(d, name, text): (Path(d) / name).write_text(text, encoding="utf-8")


def test_stages_and_constants_and_clock_separation():
    with tempfile.TemporaryDirectory() as d:
        b = str(Path(d) / "x")
        _write(d, "x.mac_timing.txt", "m_start 1000.000\nm_p620_check_start 1000.100\nm_p620_check_done 1000.600\nm_capture_start 1000.700\nm_capture_done 1036.700\nm_scp_done 1037.200\nm_verify_done 1037.500\nm_clk_a 1037.600\nm_clk_b 1037.800\n")
        _write(d, "x.host_timing.txt", "h_script_start 5000.0\nh_checks_done 5000.5\nh_old_container_removed 5000.6\nh_container_up 5001.6\nh_exec_start 5001.7\nh_exec_done 5035.9\nh_cleanup_start 5036.0\nh_cleanup_end 5036.4\n")
        _write(d, "x.container_timing.txt", "c_cfg_start 5002.0\nc_driver_launched 5002.1\nc_serial_reported 5004.0\nc_sleep12_done 5016.0\nc_probe_done 5031.1\nc_kill_sleep4_done 5035.2\nc_pgrep_stage_done 5035.4\nc_end 5035.5\n")
        _write(d, "x.p620_clock.txt", "9999.000000000\nNTPSynchronized=yes\n")
        out = R.report(b)
        assert "Mac 전체(m_start→m_verify_done): 37.50 s" in out
        assert "p620 스크립트 전체(h_script_start→h_cleanup_end): 36.40 s" in out
        assert "c_serial_reported" in out and "(코드 상수 12 s)" in out and "(코드 상수 15 s)" in out and "(코드 상수 4 s)" in out
        assert "Mac 촬영 구간 36.00 s − p620 스크립트 36.40 s = ssh·전송 오버헤드 -0.40 s" in out
        assert "시계 오프셋" in out and "±0.100 s" in out                      # 오프셋은 참고값으로만 표시


def test_missing_marks_are_reported_not_crashing():
    with tempfile.TemporaryDirectory() as d:
        b = str(Path(d) / "y")
        _write(d, "y.mac_timing.txt", "m_start 1.0\nm_capture_start 2.0\n")
        out = R.report(b)
        assert "없는 표시" in out and "m_verify_done" in out


def test_no_files_at_all():
    with tempfile.TemporaryDirectory() as d:
        out = R.report(str(Path(d) / "none"))
        assert "측정 기록일 뿐" in out
