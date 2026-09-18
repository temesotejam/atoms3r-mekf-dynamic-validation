#!/usr/bin/env python3
from pathlib import Path
import struct
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
runner = (ROOT / "src/experiment_runner.cpp").read_text(encoding="utf-8")
runner_h = (ROOT / "src/experiment_runner.h").read_text(encoding="utf-8")
log_types = (ROOT / "src/log_types.h").read_text(encoding="utf-8")
logger = (ROOT / "src/psram_logger.cpp").read_text(encoding="utf-8")
config = (ROOT / "src/config.h").read_text(encoding="utf-8")
converter = (ROOT / "tools/convert_rwlog_to_csv.py").read_text(encoding="utf-8")

assert "v46aa_control_upright_zero_20260918" in config
assert "IMU_POLL_PERIOD_US = 1000UL" in config
assert "BMI270_GYRO_ODR_HZ = 400" in config
assert "BMI270_ACCEL_ODR_HZ = 200" in config
assert "BMI270_I2C_HZ = 1000000UL" in config

for token in (
    "pitch_mekf_detector_relative_deg",
    "mekf_detector_zero_predicted_abs_deg",
    "mekf_detector_zero_sample_us",
):
    assert token in runner_h, token

measurement = runner[
    runner.index("void ExperimentRunner::beginMeasurementRun"):
    runner.index("void ExperimentRunner::beginTrial")
]
assert "status_.mekf_detector_zero_predicted_abs_deg = raw_mekf_predicted_abs_deg_" in measurement
assert "status_.pitch_mekf_detector_relative_deg" in measurement

display = runner[
    runner.index("void ExperimentRunner::updateDisplayedAngles"):
    runner.index("void ExperimentRunner::updateCurrentRollState")
]
assert "raw_mekf_predicted_abs_deg_ - status_.mekf_detector_zero_predicted_abs_deg" in display

motion = runner[
    runner.index("void ExperimentRunner::updateEnergyControlAutonomousMotion"):
    runner.index("void ExperimentRunner::updateEnergyControlAutonomousPeakTracker")
]
assert "const float detector_relative_angle_deg = status_.pitch_mekf_detector_relative_deg;" in motion
assert "energy_control_autonomous_detector_zero_angle_deg_" not in motion

# The new coordinate must not become the energy amplitude coordinate.
assert "energy_control_autonomous_gyro_relative_deg_" in motion
assert "ENERGY_CONTROL_AUTONOMOUS_GYRO_TO_VIDEO_PEAK_SCALE" in motion

assert "RWLOG_FORMAT_VERSION = 48" in logger
assert "sizeof(LogSample) == 258" in log_types
assert 'SAMPLE_FORMAT_V48 = SAMPLE_FORMAT_V47 + "hhI"' in converter

# Verify the refactor is algebraically equivalent to the previous detector:
# old = (pred_t - posterior_0) - (pred_0 - posterior_0)
# new = pred_t - pred_0
# Float32 reassociation can differ by a few ulp, so bound the maximum difference.
cpp = r"""
#include <cmath>
#include <cstdint>
#include <iostream>
int main() {
  uint32_t s=0x46aa1234u;
  float max_err=0.0f;
  unsigned nonzero=0;
  for(unsigned i=0;i<1000000;++i) {
    s=1664525u*s+1013904223u;
    const float p0 = ((int32_t)(s>>1)%4000000) * 1.0e-5f - 20.0f;
    s=1664525u*s+1013904223u;
    const float pred0 = p0 + ((int32_t)(s>>1)%20000) * 1.0e-5f - 0.1f;
    s=1664525u*s+1013904223u;
    const float pt = ((int32_t)(s>>1)%4000000) * 1.0e-5f - 20.0f;
    const float oldv = (pt-p0) - (pred0-p0);
    const float newv = pt-pred0;
    const float e = std::fabs(oldv-newv);
    if(e>0.0f) ++nonzero;
    if(e>max_err) max_err=e;
  }
  std::cout << "max_float32_reassociation_error_deg=" << max_err
            << " nonzero=" << nonzero << "\n";
  return max_err <= 4.0e-6f ? 0 : 2;
}
"""
with tempfile.TemporaryDirectory(prefix="v46aa_zero_") as d:
    p=Path(d)
    (p/"test.cpp").write_text(cpp, encoding="utf-8")
    subprocess.run(["g++","-std=c++17","-O2","-ffp-contract=off",str(p/"test.cpp"),"-o",str(p/"test")],check=True)
    subprocess.run([str(p/"test")],check=True)

print("V46aa explicit upright control-zero guards PASS")
