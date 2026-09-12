#pragma once

#include <Arduino.h>

struct RollerTelemetry {
  bool roller_ok = false;
  int16_t actual_current_mA = 0;
  uint16_t battery_mV = 0;
  uint32_t i2c_error_count = 0;
  uint8_t consecutive_errors = 0;
  uint8_t mode_raw = 0;
  uint8_t output_raw = 0;
  uint8_t status_raw = 0;
  uint8_t error_raw = 0;

  // Freshness is about CURRENT_READBACK only. `current_valid` is true exactly
  // when the most recent current-read attempt succeeded; on failure the numeric
  // current remains the previous sample and must not be used as a new sample.
  uint32_t current_sample_time_us = 0;
  uint32_t current_sequence = 0;
  uint32_t current_read_failure_count = 0;
  uint16_t current_audit_sample_count = 0;
  float q_meas_observed_mA_s = NAN;
  bool current_valid = false;
  bool q_meas_observed_valid = false;
};

class Roller485Manager {
public:
  bool begin();
  void update();

  bool setCurrentMa(int16_t current_mA);
  bool stop();

  const RollerTelemetry& telemetry() const { return telemetry_; }
  uint32_t currentAgeUs(uint32_t now_us) const;
  bool ok() const { return telemetry_.roller_ok && telemetry_.consecutive_errors < 5 && telemetry_.error_raw == 0; }
  const char* lastError() const { return last_error_; }

private:
  bool writeU8(uint8_t reg, uint8_t value);
  bool writeI32(uint8_t reg, int32_t value);
  bool readBytes(uint8_t reg, uint8_t* buffer, size_t len);
  bool readI32(uint8_t reg, int32_t& value);
  bool readU8(uint8_t reg, uint8_t& value);
  bool readCurrentFresh(bool audit_sample);
  void recordFreshCurrent(int32_t current_raw, uint32_t sample_time_us, bool audit_sample);
  void recordCurrentReadFailure(bool audit_sample);
  void beginCurrentAuditPulse();
  void endCurrentAuditPulse();
  void recordIo(bool ok);

  RollerTelemetry telemetry_;
  uint32_t last_read_due_us_ = 0;
  uint32_t last_fast_current_due_us_ = 0;
  int16_t command_mA_ = 0;
  int16_t current_audit_previous_mA_ = 0;
  uint32_t current_audit_previous_sample_us_ = 0;
  bool current_audit_has_previous_sample_ = false;
  bool current_audit_active_ = false;
  bool current_audit_read_failed_ = false;
  const char* last_error_ = "not_initialized";
};
