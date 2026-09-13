#!/usr/bin/env python3
"""Static guards for the motor-driven V46 attitude-validation path."""

from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"


def main() -> None:
    config = (SRC / "config.h").read_text(encoding="utf-8")
    runner = (SRC / "experiment_runner.cpp").read_text(encoding="utf-8")
    web = (SRC / "web_ui.cpp").read_text(encoding="utf-8")
    roller = (SRC / "roller485_manager.cpp").read_text(encoding="utf-8")
    main_cpp = (SRC / "main.cpp").read_text(encoding="utf-8")

    # Frozen V7 motor experiment shape.
    for token in (
        '"energy_control_autonomous_v7_side_response_correction_rwlog30s"',
        "ENERGY_CONTROL_AUTONOMOUS_DURATION_MS = 30000UL",
        "ENERGY_CONTROL_AUTONOMOUS_START_KICK_CURRENT_MA = 300",
        "ENERGY_CONTROL_AUTONOMOUS_START_KICK_PULSE_MS = 100",
        "ENERGY_CONTROL_AUTONOMOUS_CURRENT_MA = 300",
        "ENERGY_CONTROL_AUTONOMOUS_MAX_PULSE_MS = 100",
    ):
        assert token in config, token

    # The web UI must expose the actual V7 motor start and ESTOP paths.
    assert 'server_->on("/start-energy-control-autonomous"' in web
    assert "runner_->startEnergyControlAutonomousCapture()" in web
    assert "Start autonomous energy control" in web
    assert 'server_->on("/stop"' in web
    assert 'runner_->requestEmergencyStop("web_estop")' in web

    # The visible page intentionally freezes during a measurement, but polling
    # must remain parseable and must resume automatically when running becomes
    # false. V46's intentionally-disabled estimator fields may be NaN inside
    # firmware but must never escape as invalid JSON numeric tokens.
    for token in (
        "displayFrozen",
        "refreshInFlight",
        "if(lastStatus.running){displayFrozen=true;applyFrozenState();return;}",
        "if(displayFrozen)displayFrozen=false;apply(lastStatus);",
        'json.replace(":nan", ":null")',
        'json.replace(":NaN", ":null")',
        'json.replace(":inf", ":null")',
    ):
        assert token in web, token

    # Actual motor authority remains isolated to explicitly authorized V7 pulses.
    assert "energy_control_autonomous_pulse_live" in runner
    assert "energy_control_autonomous_pulse_authorized_" in runner
    assert "beginEnergyControlAutonomousStartKickPulse" in runner
    assert "beginEnergyControlAutonomousPulse" in runner
    assert "updateEnergyControlAutonomousPulse" in runner

    # Once output is already known off, idle/FINISHED service calls must not
    # hammer the Roller with redundant zero-current I2C writes. If telemetry
    # later reports OUTPUT=1, the same stop() path writes zero again.
    assert "if (command_mA_ == 0 && telemetry_.output_raw == 0) return true;" in roller
    assert "telemetry_.output_raw = current_mA == 0 ? 0 : 1;" in roller

    # MEKF is the adopted controller/detector attitude; dynamic-beta Madgwick is
    # retained only as synchronized comparison data during autonomous capture.
    assert "const bool v46_mekf_dynamic_compare = energy_control_autonomous_mode_" in runner
    assert "status_.pitch_mekf_deg" in runner
    assert "pitch_madgwick_dynamic_abs_deg" in runner
    assert "v46_mekf_adopted_dynamic_beta_compare_20260912" in config

    # Startup identity must not claim that this build is motor-off only.
    assert "V46 MEKF motor-driven dynamic validation" in main_cpp
    assert "V7 MOTOR VALIDATION" in main_cpp
    assert "motor output OFF" not in main_cpp

    print("V46 motor-driven validation source guards passed")


if __name__ == "__main__":
    main()
