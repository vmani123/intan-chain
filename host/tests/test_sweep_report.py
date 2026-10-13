"""End to end with no hardware and no sockets: the --dry-run path."""
import json

from chain import report, sweep


def test_dry_run_writes_one_json_per_cell_and_reports_clean(tmp_path, capsys):
    rc = sweep.main(["--dry-run", "--seconds", "0.2", "--rates", "5000", "10000",
                     "--out", str(tmp_path)])
    assert rc == 0
    files = sorted(tmp_path.glob("*.json"))
    assert len(files) == 2
    r = json.loads(files[0].read_text())
    assert r["accuracy"]["bit_errors"] == 0 and r["loss"]["missing"] == 0
    assert r["counters"]["FRAMES"] > 0  # the counter read path ran
    assert r["throughput"]["samples_per_s_per_ch"] > 0

    out = report.main(["--dir", str(tmp_path)])
    assert out == 0
    printed = capsys.readouterr().out
    assert "| rate |" in printed and "5000" in printed


def test_dropped_datagrams_show_up_as_loss(tmp_path):
    sweep.main(["--dry-run", "--seconds", "0.2", "--rates", "5000",
                "--drop-every", "4", "--out", str(tmp_path)])
    r = json.loads(next(iter(tmp_path.glob("*.json"))).read_text())
    assert r["loss"]["missing"] > 0
    assert r["accuracy"]["bit_errors"] == 0  # loss must not knock the pattern out of step


def test_phase_scan_covers_every_phase(tmp_path, capsys):
    sweep.main(["--dry-run", "--phase-scan", "--phase-rate", "5000", "--seconds", "0.05",
                "--out", str(tmp_path)])
    assert len(sorted(tmp_path.glob("phase*.json"))) == 12
    assert "PHASE_SELECT 11" in capsys.readouterr().out


def test_sample_period_respects_the_sequencer_floor():
    assert sweep.period_for(1_000_000) == sweep.MIN_PERIOD  # 35 cmds x 16 bits x 4 ticks
    assert sweep.period_for(30_000) == 3200


def test_listen_only_scores_traffic_without_a_control_port(tmp_path, monkeypatch):
    # Stage 1: the ESP synthesises its own data and there is no FPGA, so there
    # is no serial port to open and no counters to read.
    def no_ctrl(*a, **k):
        raise AssertionError("listen-only opened the control port")
    monkeypatch.setattr(sweep, "Ctrl", no_ctrl)
    rc = sweep.main(["--listen-only", "--dry-run", "--seconds", "0.2", "--rates", "5000",
                     "--out", str(tmp_path)])
    assert rc == 0
    r = json.loads(next(iter(tmp_path.glob("*.json"))).read_text())
    assert r["listen_only"] is True and r["counters"] is None
    assert r["accuracy"]["bit_errors"] == 0
    assert r["throughput"]["samples_per_s_per_ch"] > 0


def test_listen_only_refuses_a_phase_scan():
    # A phase scan writes PHASE_SELECT; without the control port it would
    # silently measure one phase twelve times.
    import pytest
    with pytest.raises(SystemExit):
        sweep.main(["--listen-only", "--phase-scan", "--dry-run"])


def test_emulator_source_scores_against_the_playback_memory(tmp_path):
    rc = sweep.main(["--dry-run", "--source", "emulator", "--seconds", "0.2", "--rates", "5000",
                     "--out", str(tmp_path)])
    assert rc == 0
    r = json.loads(next(iter(tmp_path.glob("*.json"))).read_text())
    assert r["source"] == "emulator" and r["accuracy"]["source"] == "emulator"
    assert r["accuracy"]["bit_errors"] == 0


def test_phase_scan_runs_the_emulator_not_the_internal_pattern(tmp_path, monkeypatch):
    # PHASE_SELECT only matters when the receiver reads real SPI (MODE bit 2
    # clear). Scanning it with the internal pattern measured nothing.
    modes = []
    real_write = sweep.Ctrl.write
    def spy(self, reg, value):
        if reg == "MODE":
            modes.append(value)
        return real_write(self, reg, value)
    monkeypatch.setattr(sweep.Ctrl, "write", spy)
    sweep.main(["--dry-run", "--phase-scan", "--phase-rate", "5000", "--seconds", "0.05",
                "--out", str(tmp_path)])
    assert modes and all(m & 0x4 == 0 for m in modes)
    r = json.loads(sorted(tmp_path.glob("phase*.json"))[0].read_text())
    assert r["source"] == "emulator"
