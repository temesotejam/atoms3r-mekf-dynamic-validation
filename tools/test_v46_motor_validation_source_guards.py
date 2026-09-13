#!/usr/bin/env python3
"""Static guards for the motor-driven V46e attitude-validation path."""

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

    # Web start/ESTOP and post-run recovery remain intact.
    for token in (
        'server_->on("/start-energy-control-autonomous"',
        "runner_->startEnergyControlAutonomousCapture()",
        "Start autonomous energy control",
        'server_->on("/stop"',
        'runner_->requestEmergencyStop("web_estop")',
        "displayFrozen",
        "refreshInFlight",
        "if(lastStatus.running){displayFrozen=true;applyFrozenState();return;}",
        "if(displayFrozen)displayFrozen=false;apply(lastStatus);",
        'json.replace(":nan", ":null")',
        'json.replace(":NaN", ":null")',
        'json.replace(":inf", ":null")',
    ):
        assert token in web, token

    # Existing V7 motor safety/authority remains the sole output path.
    for token in (
        "energy_control_autonomous_pulse_live",
        "energy_control_autonomous_pulse_authorized_",
        "beginEnergyControlAutonomousStartKickPulse",
        "beginEnergyControlAutonomousPulse",
        "updateEnergyControlAutonomousPulse",
    ):
        assert token in runner, token
    assert "if (command_mA_ == 0 && telemetry_.output_raw == 0) return true;" in roller
    assert "telemetry_.output_raw = current_mA == 0 ? 0 : 1;" in roller

    # Startup 10-s guide LED and measured-upright gating are unchanged.
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

    # V46e proper right-handed R_y(pi)=diag(-1,+1,-1) body frame.
    assert "return {-r.ax_g, r.ay_g, -r.az_g};" in runner
    assert "r.gy_dps * Config::MEKF_GYRO_Y_SCALE" in runner
    assert "by_dps * Config::MEKF_GYRO_Y_SCALE" in runner
    assert "-mean_accel_raw.x, mean_accel_raw.y, -mean_accel_raw.z" in runner

    # No display-only calibration may remain: absolute and control pitch both
    # originate from the same MEKF state.
    assert "MEKF_VIDEO_OUTPUT_SCALE" not in config
    assert "MEKF_VIDEO_OUTPUT_SIGN" not in config
    assert "status_.pitch_mekf_abs_deg = raw_mekf_pitch_abs_deg_;" in runner
    assert "status_.pitch_mekf_deg = raw_mekf_pitch_abs_deg_ - offset_mekf_pitch_deg_" in runner
    assert "MEKF_GYRO_Y_SCALE = 0.908911f" in config
    assert "r.gy_dps * Config::MEKF_GYRO_Y_SCALE" in runner
    assert "by_dps * Config::MEKF_GYRO_Y_SCALE" in runner
    assert "mekf_bias_y_dps = mekf6::radToDeg(b.y) / Config::MEKF_GYRO_Y_SCALE" in runner

    # With physical/video-sign pitch, return-to-centre rate is opposite peak
    # side and physical peak side equals detector side. Zero-cross physical side
    # remains selected from +gy rate, preserving Q/motor direction semantics.
    assert "rate_sign == -energy_control_autonomous_candidate_detector_side_" in runner
    assert "energy_control_autonomous_candidate_peak_ms_,\n      energy_control_autonomous_candidate_detector_side_," in runner
    assert "event.physical_next_peak_side = rate_dps >= 0.0f ? 1 : -1;" in runner
    assert "event.q_command_direction = event.physical_next_peak_side;" in runner

    # Run-start MEKF reinitialization and accel-rejection path remain present.
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
    assert "pitch_madgwick_dynamic_abs_deg" in runner
    assert "v46f_mekf_ry180_gyro_y_calibrated_upright_reinit_20260913" in config
    assert "V46f MEKF motor-driven dynamic validation" in main_cpp
    assert "V7 MOTOR VALIDATION" in main_cpp
    assert "motor output OFF" not in main_cpp

    print("V46e physical-frame motor validation source guards passed")


if __name__ == "__main__":
    main()
