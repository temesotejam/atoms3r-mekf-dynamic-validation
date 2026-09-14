from pathlib import Path

config = Path("src/config.h").read_text(encoding="utf-8")
main = Path("src/main.cpp").read_text(encoding="utf-8")
runner_h = Path("src/experiment_runner.h").read_text(encoding="utf-8")
runner = Path("src/experiment_runner.cpp").read_text(encoding="utf-8")
logger_h = Path("src/psram_logger.h").read_text(encoding="utf-8")
logger = Path("src/psram_logger.cpp").read_text(encoding="utf-8")
log_types = Path("src/log_types.h").read_text(encoding="utf-8")
manifest = Path("site/manifest.json").read_text(encoding="utf-8")

assert "v46k_pulse_start_timing_probe_20260914" in config
assert "V46k" in main
assert '"version": "0.46.10"' in manifest

# Physical/control behavior is deliberately frozen from V46j.
assert "CURRENT_AUDIT_FAST_READ_PERIOD_US = 2000UL" in config
assert "CURRENT_AUDIT_LOG_PERIOD_US = 2000UL" in config
assert "ENERGY_CONTROL_AUTONOMOUS_CURRENT_MA = 300" in config
assert "ENERGY_CONTROL_AUTONOMOUS_START_KICK_CURRENT_MA = 300" in config
assert "ENERGY_CONTROL_AUTONOMOUS_START_KICK_PULSE_MS = 100" in config
assert "IMU_POLL_PERIOD_US = 1000UL" in config
assert "BMI270_GYRO_ODR_HZ = 400" in config
assert "BMI270_ACCEL_ODR_HZ = 200" in config
assert "sizeof(LogSample) == 226" in log_types
assert "RWLOG_FORMAT_VERSION = 46" in logger

# Timing probes are metadata-only and cannot be consulted by control selection.
for token in (
    "TimingProbeEvent", "addTimingProbeEvent", "v46k_timing_probe_events",
    "set_current_us", "state_update_us", "current_model_us",
    "update_pulse_model_us", "pulse_begin_total_us",
    "first_audit_log_us", "imu_update_call_us", "runner_update_call_us",
    "first_imu_dt_after_start_us", "first_imu_sample_offset_us",
):
    assert token in logger_h or token in logger, token

for token in (
    "recordTimingProbeLoop", "startTimingProbe", "maybeFinalizeTimingProbe",
    "set_current_t0_us", "pulse_model_t0_us", "probe_log",
    "r.gyro_sequence != timing_probe_event_.gyro_sequence_at_start",
):
    assert token in runner_h or token in runner, token

assert "v46k_timing_probe_active" in main
assert "runner.recordTimingProbeLoop" in main

# Never print timing inside a Run: that would perturb the quantity being measured.
probe_region = runner[runner.index("void ExperimentRunner::startTimingProbe"):runner.index("void ExperimentRunner::captureAngleOffsets")]
assert "Serial." not in probe_region
assert "printf" not in probe_region

# The logger remains observational: controller code before logSampleNow may not
# read timing values for pulse selection or stopping.
controller_region = runner[:runner.index("void ExperimentRunner::logSampleNow()")]
for forbidden in (
    "timing_probe_event_.set_current_us >",
    "timing_probe_event_.first_imu_dt_after_start_us >",
    "timing_probe_event_.runner_update_call_us >",
):
    assert forbidden not in controller_region

print("V46k timing-probe guards passed")
