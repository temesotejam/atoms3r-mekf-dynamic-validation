#include <Arduino.h>
#include <M5Unified.h>

#include "app_config.hpp"
#include "mekf6.hpp"

namespace {

mekf6::Mekf6 filter(appcfg::filterConfig());
uint32_t last_sample_us = 0;
uint32_t last_print_us = 0;
float measured_rate_hz = 0.0f;

float vecNorm(float x, float y, float z) {
  return std::sqrt(x * x + y * y + z * z);
}

bool readImu(mekf6::Vec3& accel_g, mekf6::Vec3& gyro_dps) {
  if (!M5.Imu.update()) return false;
  const auto data = M5.Imu.getImuData();

  const mekf6::Vec3 accel_imu{data.accel.x, data.accel.y, data.accel.z};
  const mekf6::Vec3 gyro_imu{data.gyro.x, data.gyro.y, data.gyro.z};

  // Standard mounting for this project is Y180: AtomS3R is physically rotated
  // 180 degrees about +Y relative to the vehicle/body frame. Convert all IMU
  // vectors before calibration and estimation so CSV/Euler output is body-frame
  // referenced and the installed vehicle-level pose reads Roll/Pitch ~= 0 deg.
  accel_g = appcfg::imuToBodyY180(accel_imu);
  gyro_dps = appcfg::imuToBodyY180(gyro_imu);
  return true;
}

mekf6::Vec3 calibrateGyroBias() {
  Serial.println("# Gyro calibration: keep the vehicle/body still and approximately stationary.");
  mekf6::Vec3 sum{};
  uint32_t samples = 0;
  uint32_t stable_start_ms = 0;
  const uint32_t overall_start_ms = millis();

  while (millis() - overall_start_ms < appcfg::kGyroCalibrationTimeoutMs) {
    M5.update();
    mekf6::Vec3 a, g;
    if (!readImu(a, g)) {
      delay(1);
      continue;
    }

    const float anorm = vecNorm(a.x, a.y, a.z);
    const float gnorm = vecNorm(g.x, g.y, g.z);
    const bool stable = std::fabs(anorm - 1.0f) <= appcfg::kCalibrationMaxAccelErrorG &&
                        gnorm <= appcfg::kCalibrationMaxGyroDps;

    if (!stable) {
      stable_start_ms = 0;
      sum = {};
      samples = 0;
      continue;
    }

    if (stable_start_ms == 0) stable_start_ms = millis();
    sum.x += g.x;
    sum.y += g.y;
    sum.z += g.z;
    ++samples;

    if (millis() - stable_start_ms >= appcfg::kGyroCalibrationMs && samples >= 20) break;
  }

  if (samples == 0) {
    Serial.println("# WARNING: no stable samples for gyro calibration; starting with zero bias.");
    return {};
  }

  const mekf6::Vec3 bias_dps{sum.x / samples, sum.y / samples, sum.z / samples};
  Serial.printf("# Gyro bias init body-frame [dps]: %.5f, %.5f, %.5f (%lu samples)\n",
                bias_dps.x, bias_dps.y, bias_dps.z, static_cast<unsigned long>(samples));
  return {mekf6::degToRad(bias_dps.x), mekf6::degToRad(bias_dps.y), mekf6::degToRad(bias_dps.z)};
}

bool initializeAttitudeFromAccel() {
  mekf6::Vec3 sum{};
  uint32_t samples = 0;
  const uint32_t start_ms = millis();

  while (millis() - start_ms < 500) {
    M5.update();
    mekf6::Vec3 a, g;
    if (!readImu(a, g)) {
      delay(1);
      continue;
    }
    const float n = vecNorm(a.x, a.y, a.z);
    if (n > 0.7f && n < 1.3f) {
      sum.x += a.x;
      sum.y += a.y;
      sum.z += a.z;
      ++samples;
    }
  }

  if (samples == 0) return false;
  const mekf6::Vec3 avg{sum.x / samples, sum.y / samples, sum.z / samples};
  return filter.initializeFromAccel(avg);
}

void printCsvHeader() {
  Serial.println("# 6-axis MEKF: quaternion nominal state + 6-state error covariance");
  Serial.println("# Mounting: Y180 standard (body X=-IMU X, body Y=IMU Y, body Z=-IMU Z)");
  Serial.println("# NOTE: yaw has no absolute reference with gyro+accelerometer only and will drift.");
  Serial.println("t_us,rate_hz,roll_deg,pitch_deg,yaw_deg,gx_dps,gy_dps,gz_dps,bgx_dps,bgy_dps,bgz_dps,acc_norm_g,acc_mag_err_g,acc_resid_deg,acc_conf,acc_used");
}

void printCsv(uint32_t now_us, const mekf6::Vec3& gyro_dps) {
  const auto e = filter.eulerDeg();
  const auto b = filter.gyroBiasRadS();
  const auto d = filter.diagnostics();
  Serial.printf("%lu,%.2f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.5f,%.5f,%.5f,%.5f,%.5f,%.3f,%.3f,%d\n",
                static_cast<unsigned long>(now_us), measured_rate_hz,
                e.roll, e.pitch, e.yaw,
                gyro_dps.x, gyro_dps.y, gyro_dps.z,
                mekf6::radToDeg(b.x), mekf6::radToDeg(b.y), mekf6::radToDeg(b.z),
                d.accel_norm_g, d.accel_magnitude_error_g,
                d.accel_direction_residual_deg, d.accel_confidence,
                d.accel_used ? 1 : 0);
}

}  // namespace

void setup() {
  Serial.begin(appcfg::kSerialBaud);
  delay(250);

  auto cfg = M5.config();
  M5.begin(cfg);

  if (!M5.Imu.isEnabled()) {
    Serial.println("# ERROR: M5Unified could not initialize the onboard IMU.");
    Serial.println("# Check that the selected target is an AtomS3R-family device and update M5Unified.");
    while (true) delay(1000);
  }

  M5.Imu.setCalibration(0, 0, 0);
  filter.setGyroBiasRadS(calibrateGyroBias());

  if (!initializeAttitudeFromAccel()) {
    Serial.println("# WARNING: accelerometer initialization failed; using identity attitude.");
  }

  last_sample_us = micros();
  last_print_us = last_sample_us;
  printCsvHeader();
}

void loop() {
  M5.update();

  mekf6::Vec3 accel_g, gyro_dps;
  if (!readImu(accel_g, gyro_dps)) {
    delay(1);
    return;
  }

  const uint32_t now_us = micros();
  const uint32_t delta_us = now_us - last_sample_us;
  last_sample_us = now_us;
  const float dt_s = static_cast<float>(delta_us) * 1.0e-6f;

  if (dt_s > 0.0f) {
    const float instantaneous_hz = 1.0f / dt_s;
    measured_rate_hz = measured_rate_hz <= 0.0f ? instantaneous_hz
                                                 : 0.95f * measured_rate_hz + 0.05f * instantaneous_hz;
  }

  const mekf6::Vec3 gyro_rad_s{
      mekf6::degToRad(gyro_dps.x),
      mekf6::degToRad(gyro_dps.y),
      mekf6::degToRad(gyro_dps.z),
  };

  if (filter.predict(gyro_rad_s, dt_s)) {
    filter.updateAccel(accel_g);
  }

  if (now_us - last_print_us >= appcfg::kSerialOutputPeriodUs) {
    last_print_us = now_us;
    printCsv(now_us, gyro_dps);
  }
}
