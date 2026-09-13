#pragma once

#include <Arduino.h>

struct ImuReading {
  bool imu_ok = false;
  // Legacy fields retained for API compatibility. ExperimentRunner owns the
  // actual MEKF + dynamic-beta Madgwick attitude estimators in RWLOG v46.
  float pitch_deg = 0.0f;
  float pitch_fixed_beta_deg = 0.0f;
  float pitch_madgwick_beta1_deg = 0.0f;
  float pitch_step_beta_deg = 0.0f;
  float pitch_smooth_beta_deg = 0.0f;
  float pitch_dynamic_beta_deg = 0.0f;
  float pitch_accel_only_deg = 0.0f;
  float pitch_rate_dps = 0.0f;
  float ax_g = 0.0f;
  float ay_g = 0.0f;
  float az_g = 0.0f;
  float gx_dps = 0.0f;
  float gy_dps = 0.0f;
  float gz_dps = 0.0f;
  float acc_norm_g = 0.0f;
  float acc_norm_error_g = 0.0f;
  float beta_fixed = 0.0f;
  float beta_step = 0.0f;
  float beta_target = 0.0f;
  float beta_smooth = 0.0f;
  float beta_tau_s = 0.0f;
  float beta_dynamic = 0.0f;
  uint8_t beta_mode = 0;
  uint8_t beta_update_mode = 0;
  uint16_t time_since_last_pulse_ms = 65535;
  bool accel_fresh = false;
  bool gyro_fresh = false;
  uint8_t sensor_mask = 0;
  bool rate_config_ok = false;
  uint8_t bmi270_acc_conf = 0;
  uint8_t bmi270_gyr_conf = 0;
  uint32_t accel_sequence = 0;
  uint32_t gyro_sequence = 0;
  uint32_t accel_update_dt_us = 0;
  uint32_t gyro_update_dt_us = 0;
  uint32_t last_accel_update_us = 0;
  uint32_t last_gyro_update_us = 0;
  uint32_t update_dt_us = 0;
  uint32_t last_update_us = 0;
  uint32_t last_update_ms = 0;
  uint32_t error_count = 0;
};

class ImuManager {
 public:
  bool begin();
  void update();
  void zeroPitch();
  void setDynamicBetaContext(bool pulse_active, uint32_t time_since_last_pulse_ms, bool pre_start_stabilize = false);
  void forceSmoothBeta(float beta, uint8_t update_mode);

  const ImuReading& reading() const { return reading_; }
  bool ok() const { return imu_present_ && reading_.imu_ok && reading_.rate_config_ok; }
  bool stale(uint32_t now_ms) const;
  const char* lastError() const { return last_error_; }

 private:
  ImuReading reading_;
  bool imu_present_ = false;
  uint32_t last_due_us_ = 0;
  uint32_t prev_gyro_update_us_ = 0;
  uint32_t prev_accel_update_us_ = 0;
  uint8_t consecutive_errors_ = 0;
  bool beta_context_pulse_active_ = false;
  uint32_t beta_context_time_since_last_pulse_ms_ = 65535;
  bool beta_context_pre_start_stabilize_ = false;
  const char* last_error_ = "not_initialized";
};
