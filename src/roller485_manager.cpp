#include "roller485_manager.h"

#include <Wire.h>

#include "config.h"

static constexpr uint8_t REG_OUTPUT = 0x00;
static constexpr uint8_t REG_MODE = 0x01;
static constexpr uint8_t REG_SYS_STATUS = 0x0C;
static constexpr uint8_t REG_ERROR_CODE = 0x0D;
static constexpr uint8_t REG_VIN = 0x34;
static constexpr uint8_t REG_CURRENT = 0xB0;
static constexpr uint8_t REG_CURRENT_READBACK = 0xC0;

bool Roller485Manager::begin() {
  Wire.begin(Config::I2C_SDA_PIN, Config::I2C_SCL_PIN);
  Wire.setClock(Config::I2C_HZ);
  Wire.setTimeOut(Config::I2C_TIMEOUT_MS);

  Wire.beginTransmission(Config::ROLLER_ADDR);
  const bool present = Wire.endTransmission() == 0;
  if (!present) {
    telemetry_.roller_ok = false;
    last_error_ = "roller_not_found";
    return false;
  }

  bool ok = true;
  ok &= writeU8(REG_MODE, Config::ROLLER_MODE_CURRENT);
  ok &= writeI32(REG_CURRENT, 0);
  ok &= writeU8(REG_OUTPUT, 0);
  telemetry_.roller_ok = ok;
  telemetry_.mode_raw = Config::ROLLER_MODE_CURRENT;
  telemetry_.output_raw = 0;
  command_mA_ = 0;
  last_error_ = ok ? "" : "roller_zero_failed";
  return ok;
}

void Roller485Manager::update() {
  const uint32_t now_us = micros();

  // The existing full six-register safety/status snapshot remains scheduled at
  // 20 ms. During an output pulse, this adds a separate CURRENT_READBACK-only
  // diagnostic sample. Its result is log-only and cannot affect control.
  if (command_mA_ != 0 &&
      (last_fast_current_due_us_ == 0 ||
       static_cast<uint32_t>(now_us - last_fast_current_due_us_) >=
           Config::CURRENT_AUDIT_FAST_READ_PERIOD_US)) {
    last_fast_current_due_us_ = now_us;
    readCurrentFresh(true);
  }

  const uint32_t period_us = Config::ROLLER_READ_PERIOD_MS * 1000UL;
  if (last_read_due_us_ != 0 && static_cast<uint32_t>(now_us - last_read_due_us_) < period_us) return;
  last_read_due_us_ = last_read_due_us_ == 0 ? now_us : last_read_due_us_ + period_us;

  int32_t vin_raw = 0;
  uint8_t mode = 0;
  uint8_t output = 0;
  uint8_t status = 0;
  uint8_t error = 0;

  bool ok = readCurrentFresh(command_mA_ != 0);
  ok &= readI32(REG_VIN, vin_raw);
  ok &= readU8(REG_MODE, mode);
  ok &= readU8(REG_OUTPUT, output);
  ok &= readU8(REG_SYS_STATUS, status);
  ok &= readU8(REG_ERROR_CODE, error);
  recordIo(ok);
  if (!ok) return;

  telemetry_.battery_mV = static_cast<uint16_t>(max<int32_t>(0, vin_raw * 10));
  telemetry_.mode_raw = mode;
  telemetry_.output_raw = output;
  telemetry_.status_raw = status;
  telemetry_.error_raw = error;
  telemetry_.roller_ok = error == 0;
  last_error_ = error == 0 ? "" : "roller_error_raw";
}

bool Roller485Manager::setCurrentMa(int16_t current_mA) {
  const bool was_commanded = command_mA_ != 0;
  command_mA_ = current_mA;
  const int32_t raw = static_cast<int32_t>(current_mA) * Config::ROLLER_CURRENT_RAW_PER_MA;
  bool ok = true;
  ok &= writeU8(REG_MODE, Config::ROLLER_MODE_CURRENT);
  ok &= writeI32(REG_CURRENT, raw);
  ok &= writeU8(REG_OUTPUT, current_mA == 0 ? 0 : 1);
  recordIo(ok);
  if (!ok) {
    last_error_ = "roller_current_write_failed";
    return false;
  }
  if (!was_commanded && current_mA != 0) beginCurrentAuditPulse();
  else if (was_commanded && current_mA == 0) endCurrentAuditPulse();
  return true;
}

bool Roller485Manager::stop() {
  return setCurrentMa(0);
}

uint32_t Roller485Manager::currentAgeUs(uint32_t now_us) const {
  if (telemetry_.current_sample_time_us == 0) return UINT32_MAX;
  return static_cast<uint32_t>(now_us - telemetry_.current_sample_time_us);
}

bool Roller485Manager::readCurrentFresh(bool audit_sample) {
  int32_t current_raw = 0;
  ++telemetry_.current_sequence;
  if (!readI32(REG_CURRENT_READBACK, current_raw)) {
    recordCurrentReadFailure(audit_sample);
    return false;
  }
  recordFreshCurrent(current_raw, micros(), audit_sample);
  return true;
}

void Roller485Manager::recordFreshCurrent(int32_t current_raw, uint32_t sample_time_us,
                                          bool audit_sample) {
  telemetry_.actual_current_mA = static_cast<int16_t>(
      current_raw / Config::ROLLER_CURRENT_RAW_PER_MA);
  telemetry_.current_sample_time_us = sample_time_us;
  telemetry_.current_valid = true;
  if (!audit_sample || !current_audit_active_) return;

  const int16_t current_mA = telemetry_.actual_current_mA;
  if (current_audit_has_previous_sample_) {
    const uint32_t dt_us = static_cast<uint32_t>(sample_time_us - current_audit_previous_sample_us_);
    if (dt_us > 0 && dt_us <= 50000UL && isfinite(telemetry_.q_meas_observed_mA_s)) {
      telemetry_.q_meas_observed_mA_s += 0.5f *
          (fabsf(static_cast<float>(current_audit_previous_mA_)) + fabsf(static_cast<float>(current_mA))) *
          static_cast<float>(dt_us) * 1.0e-6f;
    }
  } else {
    telemetry_.q_meas_observed_mA_s = 0.0f;
    current_audit_has_previous_sample_ = true;
  }
  current_audit_previous_mA_ = current_mA;
  current_audit_previous_sample_us_ = sample_time_us;
  if (telemetry_.current_audit_sample_count < UINT16_MAX) {
    ++telemetry_.current_audit_sample_count;
  }
  telemetry_.q_meas_observed_valid = telemetry_.current_audit_sample_count >= 2 &&
      !current_audit_read_failed_ && isfinite(telemetry_.q_meas_observed_mA_s);
}

void Roller485Manager::recordCurrentReadFailure(bool audit_sample) {
  telemetry_.current_valid = false;
  ++telemetry_.current_read_failure_count;
  if (audit_sample && current_audit_active_) {
    current_audit_read_failed_ = true;
    telemetry_.q_meas_observed_valid = false;
  }
}

void Roller485Manager::beginCurrentAuditPulse() {
  current_audit_active_ = true;
  current_audit_has_previous_sample_ = false;
  current_audit_read_failed_ = false;
  current_audit_previous_mA_ = 0;
  current_audit_previous_sample_us_ = 0;
  last_fast_current_due_us_ = 0;
  telemetry_.current_sample_time_us = 0;
  telemetry_.current_audit_sample_count = 0;
  telemetry_.q_meas_observed_mA_s = NAN;
  telemetry_.current_valid = false;
  telemetry_.q_meas_observed_valid = false;
}

void Roller485Manager::endCurrentAuditPulse() {
  current_audit_active_ = false;
}

bool Roller485Manager::writeU8(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(Config::ROLLER_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool Roller485Manager::writeI32(uint8_t reg, int32_t value) {
  uint8_t* p = reinterpret_cast<uint8_t*>(&value);
  Wire.beginTransmission(Config::ROLLER_ADDR);
  Wire.write(reg);
  Wire.write(p, 4);
  return Wire.endTransmission() == 0;
}

bool Roller485Manager::readBytes(uint8_t reg, uint8_t* buffer, size_t len) {
  Wire.beginTransmission(Config::ROLLER_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    memset(buffer, 0, len);
    return false;
  }
  const uint8_t got = Wire.requestFrom(Config::ROLLER_ADDR, static_cast<uint8_t>(len));
  if (got != len) {
    memset(buffer, 0, len);
    while (Wire.available()) Wire.read();
    return false;
  }
  for (size_t i = 0; i < len; ++i) buffer[i] = static_cast<uint8_t>(Wire.read());
  return true;
}

bool Roller485Manager::readI32(uint8_t reg, int32_t& value) {
  value = 0;
  return readBytes(reg, reinterpret_cast<uint8_t*>(&value), 4);
}

bool Roller485Manager::readU8(uint8_t reg, uint8_t& value) {
  value = 0;
  return readBytes(reg, &value, 1);
}

void Roller485Manager::recordIo(bool ok) {
  if (ok) {
    telemetry_.consecutive_errors = 0;
    telemetry_.roller_ok = true;
    return;
  }
  telemetry_.i2c_error_count++;
  if (telemetry_.consecutive_errors < 255) telemetry_.consecutive_errors++;
  telemetry_.roller_ok = false;
}
