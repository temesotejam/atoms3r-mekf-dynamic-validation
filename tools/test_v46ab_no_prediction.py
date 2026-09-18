#!/usr/bin/env python3
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
runner = (ROOT / "src/experiment_runner.cpp").read_text(encoding="utf-8")
logger = (ROOT / "src/psram_logger.cpp").read_text(encoding="utf-8")
config = (ROOT / "src/config.h").read_text(encoding="utf-8")

assert "v46ab_no_control_prediction_20260918" in config

display = runner[
    runner.index("void ExperimentRunner::updateDisplayedAngles"):
    runner.index("void ExperimentRunner::updateCurrentRollState")
]
compare_update = display.index("updateMekfComparisonRelativeAngles();")
override = display.index("if (energy_control_autonomous_mode_)")
assert compare_update < override
assert "status_.pitch_mekf_deg = status_.pitch_mekf_measurement_relative_deg;" in display
assert "status_.pitch_mekf_detector_relative_deg =" in display
assert display.count("status_.pitch_mekf_measurement_relative_deg;") >= 2

def method(name: str) -> str:
    start = runner.index(name)
    brace = runner.index("{", start)
    depth = 1
    end = brace + 1
    while depth:
        depth += (runner[end] == "{") - (runner[end] == "}")
        end += 1
    return runner[start:end]

autonomous_timing = "\n".join(method(name) for name in (
    "void ExperimentRunner::updateEnergyControlAutonomousMotion",
    "void ExperimentRunner::updateEnergyControlAutonomousPeakTracker",
    "void ExperimentRunner::updateEnergyControlAutonomousAtZeroCross",
))

assert "const float detector_relative_angle_deg = status_.pitch_mekf_detector_relative_deg;" in autonomous_timing
for forbidden in (
    "pitch_mekf_predicted_abs_deg",
    "raw_mekf_predicted_abs_deg_",
    "mekf_detector_zero_predicted_abs_deg",
    "mekf_prediction_horizon_us",
):
    assert forbidden not in autonomous_timing, forbidden

# Prediction still exists as a diagnostic computation only; removing it from the
# controller does not remove the ability to compare posterior vs predicted offline.
filter_series = method("void ExperimentRunner::updateFilterSeries")
assert "raw_mekf_predicted_abs_deg_ = mekf_.predictEulerDeg" in filter_series
assert "status_.mekf_prediction_horizon_us = horizon_us" in filter_series

# Physical output limits and abnormal-stop guards remain unchanged.
for token in (
    "ENERGY_CONTROL_AUTONOMOUS_CURRENT_MA = 300",
    "ENERGY_CONTROL_AUTONOMOUS_MAX_PULSE_MS = 100",
):
    assert token in config, token
assert "autonomous_control_prediction_enabled" in logger
assert "false" in logger
assert "mekf_prediction_role" in logger
assert "diagnostic_only_during_autonomous" in logger

print("V46ab PASS: Autonomous timing uses posterior measurement-relative MEKF with no prediction")
