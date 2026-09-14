from pathlib import Path

config = Path("src/config.h").read_text(encoding="utf-8")
main = Path("src/main.cpp").read_text(encoding="utf-8")
runner_h = Path("src/experiment_runner.h").read_text(encoding="utf-8")
runner = Path("src/experiment_runner.cpp").read_text(encoding="utf-8")
web = Path("src/web_ui.cpp").read_text(encoding="utf-8")
logger = Path("src/psram_logger.cpp").read_text(encoding="utf-8")
log_types = Path("src/log_types.h").read_text(encoding="utf-8")
manifest = Path("site/manifest.json").read_text(encoding="utf-8")

assert "v46l_fast_solver_shadow_upright_gate_fix_20260914" in config
assert '"version": "0.46.12"' in manifest

# Existing physical limits remain unchanged.
for token in (
    "CURRENT_AUDIT_FAST_READ_PERIOD_US = 2000UL",
    "CURRENT_AUDIT_LOG_PERIOD_US = 2000UL",
    "ENERGY_CONTROL_AUTONOMOUS_CURRENT_MA = 300",
    "ENERGY_CONTROL_AUTONOMOUS_START_KICK_CURRENT_MA = 300",
    "ENERGY_CONTROL_AUTONOMOUS_START_KICK_PULSE_MS = 100",
    "UPRIGHT_STABLE_HOLD_MS = 400UL",
    "UPRIGHT_MAX_DIRECTION_ERROR_DEG = 8.0f",
    "UPRIGHT_MIN_ACCEL_NORM_G = 0.85f",
    "UPRIGHT_MAX_ACCEL_NORM_G = 1.15f",
    "UPRIGHT_MAX_GYRO_NORM_DPS = 5.0f",
):
    assert token in config or token in Path("src/upright_pose_guide.h").read_text(encoding="utf-8"), token
assert "sizeof(LogSample) == 226" in log_types
assert "RWLOG_FORMAT_VERSION = 46" in logger

# The runner start gate consumes the continuously maintained latch, not a
# one-sample duplicate gate at HTTP-start time.
start_begin = runner.index("bool ExperimentRunner::startEnergyControlAutonomousCapture()")
start_end = runner.index("bool ExperimentRunner::setEnergyControlAutonomousTarget", start_begin)
start_region = runner[start_begin:start_end]
assert "if (!startup_upright_confirmed_)" in start_region
assert 'status_.last_error = "upright_pose_required_before_start"' in start_region
assert "isUprightStableSample(autonomous_start_reading)" not in start_region
assert "setStartupUprightConfirmed" in runner_h
assert "startupUprightConfirmed" in runner_h
assert "startup_upright_confirmed_ = false" in runner_h

# The main-loop guide actively revokes confirmation before WebUi handles POSTs.
guide_begin = main.index("static void updateStartupPoseGuide()")
guide_end = main.index("void setup()", guide_begin)
guide = main[guide_begin:guide_end]
for token in (
    "const bool upright_sample_ok = UprightPoseGuide::isUprightStableSample(r)",
    "if (startup_upright_confirmed)",
    "startup_upright_confirmed = false",
    "startup_upright_since_ms = 0",
    "runner.setStartupUprightConfirmed(false)",
    "runner.setStartupUprightConfirmed(true)",
    "UPRIGHT_STABLE_HOLD_MS",
):
    assert token in guide, token
assert main.index("updateStartupPoseGuide();") < main.index("web.update();")

# Live gate diagnostics are observable from status.json.
for token in (
    "startup_upright_confirmed",
    "startup_upright_direction_error_deg",
    "startup_upright_accel_norm_g",
    "startup_upright_gyro_norm_dps",
):
    assert token in web, token

# V46l solver shadow remains post-pulse and metadata-only.
assert "runEnergyControlAutonomousSolverShadow();" in runner
assert runner.index("stopActivePulse(now_ms);") < runner.index("runEnergyControlAutonomousSolverShadow();")
assert "beginEnergyControlAutonomousPulse(now_ms, t_test_ms, event.q_command_direction, selected_width_ms)" in runner

print("V46l-r1 upright-gate safety guards passed")
