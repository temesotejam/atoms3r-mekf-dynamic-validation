#pragma once

#include "mekf6.hpp"

namespace appcfg {

constexpr uint32_t kSerialBaud = 115200;
constexpr uint32_t kGyroCalibrationMs = 1500;
constexpr uint32_t kGyroCalibrationTimeoutMs = 10000;
constexpr float kCalibrationMaxGyroDps = 5.0f;
constexpr float kCalibrationMaxAccelErrorG = 0.08f;
constexpr uint32_t kSerialOutputPeriodUs = 20000;

// Default physical mounting for this project:
// AtomS3R is rotated 180 degrees about its +Y axis relative to the vehicle/body
// frame. This is a proper rotation (determinant +1), not a single-axis sign flip:
//
//   body X = -sensor X
//   body Y =  sensor Y
//   body Z = -sensor Z
//
// Apply the same transform to every vector quantity expressed in the IMU frame,
// especially accelerometer and gyroscope data, before calibration or filtering.
inline mekf6::Vec3 imuToBodyY180(const mekf6::Vec3& v) {
  return {-v.x, v.y, -v.z};
}

inline mekf6::Config filterConfig() {
  mekf6::Config cfg;
  cfg.gyro_noise_std_rad_s = 0.015f;
  cfg.gyro_bias_rw_std_rad_s_sqrt_s = 0.0008f;
  cfg.accel_direction_noise_std = 0.035f;
  cfg.accel_mag_full_g = 0.08f;
  cfg.accel_mag_reject_g = 0.30f;
  cfg.accel_angle_full_deg = 6.0f;
  cfg.accel_angle_reject_deg = 22.0f;
  cfg.accel_min_confidence = 0.05f;
  cfg.min_dt_s = 0.0005f;
  cfg.max_dt_s = 0.0500f;
  return cfg;
}

}  // namespace appcfg
