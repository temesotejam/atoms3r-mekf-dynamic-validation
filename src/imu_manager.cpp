#include "imu_manager.h"

#include <math.h>
#include <M5Unified.h>
#include "config.h"

bool ImuManager::begin() {
  imu_present_ = M5.Imu.begin();
  if (!imu_present_) {
    reading_.imu_ok = false;
    last_error_ = "imu_init_failed";
    return false;
  }
  reading_.imu_ok = true;
  reading_.last_update_ms = millis();
  last_error_ = "";
  return true;
}

void ImuManager::update() {
  const uint32_t now_us = micros();
  const uint32_t period_us = Config::IMU_PERIOD_MS * 1000UL;
  if (last_due_us_ != 0 && static_cast<uint32_t>(now_us - last_due_us_) < period_us) return;
  last_due_us_ = last_due_us_ == 0 ? now_us : last_due_us_ + period_us;

  if (!imu_present_) {
    reading_.imu_ok = false;
    reading_.error_count++;
    last_error_ = "imu_not_ready";
    return;
  }
  if (!M5.Imu.update()) {
    reading_.error_count++;
    if (consecutive_errors_ < 255) consecutive_errors_++;
    if (consecutive_errors_ >= Config::IMU_ERROR_LIMIT) {
      reading_.imu_ok = false;
      last_error_ = "imu_update_false";
    }
    return;
  }

  consecutive_errors_ = 0;
  const uint32_t dt_us = prev_update_us_ == 0 ? period_us : static_cast<uint32_t>(now_us - prev_update_us_);
  prev_update_us_ = now_us;
  const auto d = M5.Imu.getImuData();
  const float gx = d.gyro.x, gy = d.gyro.y, gz = d.gyro.z;
  const float ax = d.accel.x, ay = d.accel.y, az = d.accel.z;

  reading_.ax_g = ax; reading_.ay_g = ay; reading_.az_g = az;
  reading_.gx_dps = gx; reading_.gy_dps = gy; reading_.gz_dps = gz;
  reading_.acc_norm_g = sqrtf(ax * ax + ay * ay + az * az);
  reading_.acc_norm_error_g = reading_.acc_norm_g - 1.0f;
  reading_.pitch_accel_only_deg = Config::PITCH_SIGN * atan2f(-ax, sqrtf(ay * ay + az * az)) * 57.2957795f;
  reading_.pitch_rate_dps = Config::GYRO_PITCH_RATE_SIGN * gy;
  reading_.time_since_last_pulse_ms = static_cast<uint16_t>(min<uint32_t>(65535, beta_context_time_since_last_pulse_ms_));
  reading_.update_dt_us = dt_us;
  reading_.last_update_us = now_us;
  reading_.last_update_ms = millis();
  reading_.imu_ok = true;
  last_error_ = "";
}

void ImuManager::zeroPitch() {
  // V46 attitude zero/reference handling is owned by ExperimentRunner.
}

void ImuManager::setDynamicBetaContext(bool pulse_active, uint32_t time_since_last_pulse_ms, bool pre_start_stabilize) {
  beta_context_pulse_active_ = pulse_active;
  beta_context_time_since_last_pulse_ms_ = time_since_last_pulse_ms;
  beta_context_pre_start_stabilize_ = pre_start_stabilize;
  (void)beta_context_pulse_active_;
  (void)beta_context_pre_start_stabilize_;
}

void ImuManager::forceSmoothBeta(float beta, uint8_t update_mode) {
  // Legacy API retained; the only online Madgwick comparison is managed by
  // ExperimentRunner so its beta and update timestamp exactly match the MEKF.
  reading_.beta_target = beta;
  reading_.beta_smooth = beta;
  reading_.beta_dynamic = beta;
  reading_.beta_update_mode = update_mode;
}

bool ImuManager::stale(uint32_t now_ms) const {
  if (!reading_.imu_ok) return true;
  return static_cast<uint32_t>(now_ms - reading_.last_update_ms) > Config::IMU_STALE_LIMIT_MS;
}
