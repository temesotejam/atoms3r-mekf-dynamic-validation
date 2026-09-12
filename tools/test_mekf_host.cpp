#include <cassert>
#include <cmath>
#include <cstdio>
#include "../src/mekf6.hpp"

// V45 detector coordinate adapter used by ExperimentRunner v46:
// accel stays in the raw IMU frame; all gyro components are negated to match
// the legacy detector kinematic sign convention (pitch_rate = -gy).
static mekf6::Vec3 accelFilter(float ax, float ay, float az) { return {ax, ay, az}; }
static mekf6::Vec3 gyroFilter(float gx, float gy, float gz) {
  return {mekf6::degToRad(-gx), mekf6::degToRad(-gy), mekf6::degToRad(-gz)};
}
static float reportedPitch(const mekf6::Mekf6& f) { return f.eulerDeg().pitch; }

int main() {
  // 1) Static sign must match the historical detector atan2(-ax,hypot(ay,az)).
  mekf6::Mekf6 f;
  const float a10 = mekf6::degToRad(10.0f);
  auto a = accelFilter(-std::sin(a10), 0.0f, std::cos(a10));
  assert(f.initializeFromAccel(a));
  if (std::fabs(reportedPitch(f) - 10.0f) > 0.05f) return 1;

  // 2) Dynamic sign must match pitch_rate=-gy. raw gy=-90 dps for 100 ms
  // therefore advances the detector coordinate by +9 deg.
  f.reset();
  assert(f.initializeFromAccel(accelFilter(0, 0, 1)));
  for (int i = 0; i < 20; ++i) f.predict(gyroFilter(0, -90, 0), 0.005f);
  const float p = reportedPitch(f);
  std::printf("reported_pitch_after_100ms_raw_gy_-90=%.3f\n", p);
  if (!(p > 8.0f && p < 10.0f)) return 2;

  // 3) A physically consistent 45-deg sweep at 90 deg/s must remain aligned
  // with the gravity update rather than fighting it.
  f.reset();
  assert(f.initializeFromAccel(accelFilter(0, 0, 1)));
  for (int i = 1; i <= 100; ++i) {
    const float theta = mekf6::degToRad(90.0f * i * 0.005f);
    if (!f.predict(gyroFilter(0, -90, 0), 0.005f)) return 3;
    if (!f.updateAccel(accelFilter(-std::sin(theta), 0, std::cos(theta)))) return 4;
  }
  const auto sweep_diag = f.diagnostics();
  std::printf("synthetic_sweep_pitch=%.3f conf=%.3f resid=%.3f\n",
              reportedPitch(f), sweep_diag.accel_confidence,
              sweep_diag.accel_direction_residual_deg);
  if (std::fabs(reportedPitch(f) - 45.0f) > 0.5f) return 5;
  if (!sweep_diag.accel_used || sweep_diag.accel_confidence < 0.99f) return 6;

  // 4) Strong translational-acceleration contamination must be rejected.
  f.reset();
  assert(f.initializeFromAccel(accelFilter(0, 0, 1)));
  f.predict(gyroFilter(0, 0, 0), 0.005f);
  const bool used = f.updateAccel(accelFilter(0.80f, 0.0f, 1.0f));
  const auto reject_diag = f.diagnostics();
  std::printf("accel_reject used=%d conf=%.3f norm=%.3f resid=%.3f\n",
              used ? 1 : 0, reject_diag.accel_confidence,
              reject_diag.accel_norm_g, reject_diag.accel_direction_residual_deg);
  if (used || reject_diag.accel_used || reject_diag.accel_confidence >= 0.05f) return 7;

  // 5) Quaternion must stay normalized after dynamic propagation/update.
  const auto q = f.quaternion();
  const float qn = std::sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
  std::printf("q_norm=%.7f\n", qn);
  if (std::fabs(qn - 1.0f) > 1e-5f) return 8;

  std::puts("MEKF host sign/dynamics/rejection test passed");
  return 0;
}
