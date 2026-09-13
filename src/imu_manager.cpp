#include "imu_manager.h"

#include <math.h>
#include <M5Unified.h>
#include "config.h"

bool ImuManager::begin() {
  imu_present_ = M5.Imu.begin();
  if (!imu_present_) {
    reading_.imu_ok = false;
    reading_.rate_config_ok = false;
    last_error_ = "imu_init_failed";
    return false;
  }
  if (M5.Imu.getType() != m5::imu_bmi270) {
    reading_.imu_ok = false;
    reading_.rate_config_ok = false;
    last_error_ = "unexpected_imu_type_not_bmi270";
    return false;
  }
  auto* dev = M5.Imu.getImuInstancePtr(0);
  if (!dev) {
    reading_.imu_ok = false;
    reading_.rate_config_ok = false;
    last_error_ = "bmi270_instance_missing";
    return false;
  }
  constexpr uint8_t kAccConf = 0x40;
  constexpr uint8_t kGyrConf = 0x42;
  const uint8_t acc0 = dev->readRegister8(kAccConf);
  const uint8_t gyr0 = dev->readRegister8(kGyrConf);
  const uint8_t acc_target = static_cast<uint8_t>((acc0 & 0xF0u) | Config::BMI270_ACCEL_ODR_CODE);
  const uint8_t gyr_target = static_cast<uint8_t>((gyr0 & 0xF0u) | Config::BMI270_GYRO_ODR_CODE);
  const bool write_ok = dev->writeRegister8(kAccConf, acc_target) && dev->writeRegister8(kGyrConf, gyr_target);
  delay(2);
  reading_.bmi270_acc_conf = dev->readRegister8(kAccConf);
  reading_.bmi270_gyr_conf = dev->readRegister8(kGyrConf);
  reading_.rate_config_ok = write_ok &&
      ((reading_.bmi270_acc_conf & 0x0Fu) == Config::BMI270_ACCEL_ODR_CODE) &&
      ((reading_.bmi270_gyr_conf & 0x0Fu) == Config::BMI270_GYRO_ODR_CODE);
  if (!reading_.rate_config_ok) {
    reading_.imu_ok = false;
    last_error_ = "bmi270_odr_config_failed";
    return false;
  }
  reading_.imu_ok = true;
  reading_.last_update_ms = millis();
  last_error_ = "";
  return true;
}

void ImuManager::update() {
  reading_.accel_fresh = false;
  reading_.gyro_fresh = false;
  reading_.sensor_mask = 0;
  const uint32_t now_us = micros();
  if (last_due_us_ != 0 && static_cast<uint32_t>(now_us - last_due_us_) < Config::IMU_POLL_PERIOD_US) return;
  last_due_us_ = now_us;
  if (!imu_present_ || !reading_.rate_config_ok) {
    reading_.imu_ok = false;
    reading_.error_count++;
    last_error_ = "imu_not_ready";
    return;
  }
  const auto mask = M5.Imu.update();
  const uint8_t bits = static_cast<uint8_t>(mask);
  if (bits == 0) return;
  const auto d = M5.Imu.getImuData();
  const uint32_t sample_us = d.usec ? d.usec : now_us;
  const bool accel_new = bits & static_cast<uint8_t>(m5::IMU_Class::sensor_mask_accel);
  const bool gyro_new  = bits & static_cast<uint8_t>(m5::IMU_Class::sensor_mask_gyro);
  reading_.sensor_mask = bits;
  reading_.accel_fresh = accel_new;
  reading_.gyro_fresh = gyro_new;
  if (accel_new) {
    reading_.ax_g = d.accel.x; reading_.ay_g = d.accel.y; reading_.az_g = d.accel.z;
    reading_.acc_norm_g = sqrtf(reading_.ax_g*reading_.ax_g + reading_.ay_g*reading_.ay_g + reading_.az_g*reading_.az_g);
    reading_.acc_norm_error_g = reading_.acc_norm_g - 1.0f;
    reading_.pitch_accel_only_deg = Config::PITCH_SIGN * atan2f(-reading_.ax_g, sqrtf(reading_.ay_g*reading_.ay_g + reading_.az_g*reading_.az_g)) * 57.2957795f;
    reading_.accel_update_dt_us = prev_accel_update_us_ ? static_cast<uint32_t>(sample_us - prev_accel_update_us_) : 1000000UL / Config::BMI270_ACCEL_ODR_HZ;
    prev_accel_update_us_ = sample_us;
    reading_.last_accel_update_us = sample_us;
    ++reading_.accel_sequence;
  }
  if (gyro_new) {
    reading_.gx_dps = d.gyro.x; reading_.gy_dps = d.gyro.y; reading_.gz_dps = d.gyro.z;
    reading_.pitch_rate_dps = Config::GYRO_PITCH_RATE_SIGN * reading_.gy_dps;
    reading_.gyro_update_dt_us = prev_gyro_update_us_ ? static_cast<uint32_t>(sample_us - prev_gyro_update_us_) : Config::IMU_POLL_PERIOD_US;
    prev_gyro_update_us_ = sample_us;
    reading_.last_gyro_update_us = sample_us;
    ++reading_.gyro_sequence;
    reading_.update_dt_us = reading_.gyro_update_dt_us;
    reading_.last_update_us = sample_us;
    reading_.last_update_ms = millis();
  }
  reading_.time_since_last_pulse_ms = static_cast<uint16_t>(min<uint32_t>(65535, beta_context_time_since_last_pulse_ms_));
  consecutive_errors_ = 0;
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
  if (!reading_.imu_ok || !reading_.rate_config_ok || reading_.last_gyro_update_us == 0) return true;
  return static_cast<uint32_t>(now_ms - reading_.last_update_ms) > Config::IMU_STALE_LIMIT_MS;
}
