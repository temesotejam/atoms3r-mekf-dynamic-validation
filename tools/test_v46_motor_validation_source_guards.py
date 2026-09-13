#!/usr/bin/env python3
"""Static guards for the motor-driven V46c attitude-validation path."""

from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"


def main() -> None:
    config = (SRC / "config.h").read_text(encoding="utf-8")
    runner = (SRC / "experiment_runner.cpp").read_text(encoding="utf-8")
    web = (SRC / "web_ui.cpp").read_text(encoding="utf-8")
    roller = (SRC / "roller485_manager.cpp").read_text(encoding="utf-8")
    main_cpp = (SRC / "main.cpp").read_text(encoding="utf-8")
    upright = (SRC / "upright_pose_guide.h").read_text(encoding="utf-8")

    for token in (
        '"energy_control_autonomous_v7_side_response_correction_rwlog30s"',
        "ENERGY_CONTROL_AUTONOMOUS_DURATION_MS = 30000UL",
        "ENERGY_CONTROL_AUTONOMOUS_START_KICK_CURRENT_MA = 300",
        "ENERGY_CONTROL_AUTONOMOUS_START_KICK_PULSE_MS = 100",
        "ENERGY_CONTROL_AUTONOMOUS_CURRENT_MA = 300",
        "ENERGY_CONTROL_AUTONOMOUS_MAX_PULSE_MS = 100",
    ):
        assert token in config, token

    assert 'server_->on("/start-energy-control-autonomous"' in web
    assert "runner_->startEnergyControlAutonomousCapture()" in web
    assert "Start autonomous energy control" in web
    assert 'server_->on("/stop"' in web
    assert 'runner_->requestEmergencyStop("web_estop")' in web

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

    assert "energy_control_autonomous_pulse_live" in runner
    assert "energy_control_autonomous_pulse_authorized_" in runner
    assert "beginEnergyControlAutonomousStartKickPulse" in runner
    assert "beginEnergyControlAutonomousPulse" in runner
    assert "updateEnergyControlAutonomousPulse" in runner

    assert "if (command_mA_ == 0 && telemetry_.output_raw == 0) return true;" in roller
    assert "telemetry_.output_raw = current_mA == 0 ? 0 : 1;" in roller

    for token in (
        "GUIDE_LED_ON_AFTER_BOOT_MS = 10000UL",
        "REF_AX = 0.021626f",
        "REF_AY = 0.033568f",
        "REF_AZ = -0.999202f",
        "UPRIGHT_STABLE_HOLD_MS = 400UL",
        "MEKF_REINIT_AVERAGE_MS = 800UL",
        "MEKF_REINIT_MIN_SAMPLES = 60UL",
    ):
        assert token in upright, token
    assert "if (startup_upright_confirmed || runner.running()) return;" in main_cpp
    assert "digitalWrite(Config::SYNC_LED_PIN, HIGH);" in main_cpp
    assert "startup_upright_confirmed = true;" in main_cpp
    assert "digitalWrite(Config::SYNC_LED_PIN, LOW);" in main_cpp

    # V46c: proper right-handed Rx(pi) sensor-to-body rotation. Raw upright is
    # approximately -Z; body/filter upright must become +Z without the old
    # ~180-degree Euler-roll branch. The reinit average must use the same map.
    for token in (
        "return {r.ax_g, -r.ay_g, -r.az_g};",
        "mekf6::degToRad(r.gx_dps), mekf6::degToRad(-r.gy_dps), mekf6::degToRad(-r.gz_dps)",
        "mekf6::degToRad(bx_dps), mekf6::degToRad(-by_dps), mekf6::degToRad(-bz_dps)",
        "mean_accel_raw.x, -mean_accel_raw.y, -mean_accel_raw.z",
        "R=diag(+1,-1,-1)",
    ):
        assert token in runner, token

    for token in (
        'status_.last_error = "upright_pose_required_before_start"',
        "g_v46_mekf_run_reinit.active = true",
        "sync_step_ == 0",
        "UprightPoseGuide::MEKF_REINIT_AVERAGE_MS",
        "mekf_.reset();",
        "mekf_.initializeFromAccel(mean_accel)",
        "mekf_.setGyroBiasRadS(mekfStartupBiasFromRaw(",
        'requestEmergencyStop("mekf_reinit_upright_not_stable")',
        'Serial.printf("MEKF run reinit:',
    ):
        assert token in runner, token

    assert "const bool v46_mekf_dynamic_compare = energy_control_autonomous_mode_" in runner
    assert "status_.pitch_mekf_deg" in runner
    assert "pitch_madgwick_dynamic_abs_deg" in runner
    assert "MEKF_VIDEO_OUTPUT_SCALE = 0.908911f" in config
    assert "MEKF_VIDEO_OUTPUT_SIGN = -1.0f" in config
    assert runner.count("Config::MEKF_VIDEO_OUTPUT_SCALE * raw_mekf_pitch_abs_deg_") == 2
    assert "status_.pitch_mekf_deg = raw_mekf_pitch_abs_deg_ - offset_mekf_pitch_deg_" in runner
    assert "v46d_mekf_video_calibrated_rx180_upright_reinit_20260913" in config

    assert "V46d MEKF motor-driven dynamic validation" in main_cpp
    assert "V7 MOTOR VALIDATION" in main_cpp
    assert "motor output OFF" not in main_cpp

    print("V46c motor-driven validation source guards passed")


if __name__ == "__main__":
    main()
