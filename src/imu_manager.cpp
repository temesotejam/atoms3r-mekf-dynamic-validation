#include "imu_manager.h"
#include <math.h>
#include <M5Unified.h>
#include "config.h"

bool ImuManager::begin() {
  // Roller uses Arduino Wire controller 0. Do not start with a shared controller.
  internal_i2c_port_ = static_cast<int>(M5.In_I2C.getPort());
  internal_sda_ = M5.In_I2C.getSDA();
  internal_scl_ = M5.In_I2C.getSCL();
  if (internal_i2c_port_ != 1) {
    last_error_ = "imu_internal_i2c_must_be_separate_from_roller_port0";
    return false;
  }
  imu_present_ = M5.Imu.begin();
  if (!imu_present_) {
    last_error_ = "imu_init_failed";
    return false;
  }
  if (M5.Imu.getType() != m5::imu_bmi270) {
    last_error_ = "unexpected_imu_type_not_bmi270";
    return false;
  }
  auto* dev = M5.Imu.getImuInstancePtr(0);
  if (!dev) { last_error_ = "bmi270_instance_missing"; return false; }
  constexpr uint8_t kAccConf = 0x40, kGyrConf = 0x42;
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
  if (!reading_.rate_config_ok) { last_error_ = "bmi270_odr_config_failed"; return false; }
  reading_.imu_ok = true;
  reading_.last_update_ms = millis();
  capture_ = reading_;
  last_error_ = "";
  return startAcquisition();
}

bool ImuManager::startAcquisition() {
  sample_queue_ = xQueueCreateStatic(kQueueLength, sizeof(ImuReading), queue_bytes_, &queue_storage_);
  if (!sample_queue_) { last_error_ = "imu_queue_create_failed"; return false; }
  esp_timer_create_args_t args{};
  args.callback = &ImuManager::timerCallback;
  args.arg = this;
  args.dispatch_method = ESP_TIMER_TASK;
  args.name = "bmi270_poll";
  args.skip_unhandled_events = true;
  if (esp_timer_create(&args, &acquisition_timer_) != ESP_OK) {
    last_error_ = "imu_timer_create_failed";
    return false;
  }
  if (xTaskCreatePinnedToCore(&ImuManager::taskEntry, "bmi270_reader", 4096,
          this, kReaderPriority, &acquisition_task_, kReaderCore) != pdPASS) {
    esp_timer_delete(acquisition_timer_);
    acquisition_timer_ = nullptr;
    last_error_ = "imu_task_create_failed";
    return false;
  }
  if (esp_timer_start_periodic(acquisition_timer_, Config::IMU_POLL_PERIOD_US) != ESP_OK) {
    // Timer never started: the newly created task is only waiting for notification.
    vTaskDelete(acquisition_task_);
    acquisition_task_ = nullptr;
    esp_timer_delete(acquisition_timer_);
    acquisition_timer_ = nullptr;
    last_error_ = "imu_timer_start_failed";
    return false;
  }
  started_ = true;
  return true;
}

void ImuManager::timerCallback(void* arg) {
  // ESP_TIMER_TASK context: wake only; never do sensor I/O or float work here.
  auto* self = static_cast<ImuManager*>(arg);
  xTaskNotifyGive(self->acquisition_task_);
}
void ImuManager::taskEntry(void* arg) {
  static_cast<ImuManager*>(arg)->acquisitionLoop();
}
void ImuManager::acquisitionLoop() {
  const int core = xPortGetCoreID();
  const uint32_t priority = uxTaskPriorityGet(nullptr);
  portENTER_CRITICAL(&mux_);
  reader_core_ = core;
  reader_priority_ = priority;
  portEXIT_CRITICAL(&mux_);
  for (;;) {
    const uint32_t wakes = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    const uint32_t t0 = micros();
    captureSensor();
    const uint32_t elapsed = static_cast<uint32_t>(micros() - t0);
    portENTER_CRITICAL(&mux_);
    audit_.poll(elapsed, wakes);
    portEXIT_CRITICAL(&mux_);
  }
}

void ImuManager::captureSensor() {
  capture_.accel_fresh = false;
  capture_.gyro_fresh = false;
  capture_.sensor_mask = 0;
  const uint32_t now_us = micros();
  const auto mask = M5.Imu.update();
  const uint8_t bits = static_cast<uint8_t>(mask);
  if (bits == 0) return;  // No fresh value is not itself a read error.
  const auto d = M5.Imu.getImuData();
  // M5Unified's host acquisition timestamp, NOT the BMI270 hardware sensor clock.
  const uint32_t sample_us = d.usec ? d.usec : now_us;
  const bool accel_new = bits & static_cast<uint8_t>(m5::IMU_Class::sensor_mask_accel);
  const bool gyro_new = bits & static_cast<uint8_t>(m5::IMU_Class::sensor_mask_gyro);
  if ((accel_new && (!isfinite(d.accel.x) || !isfinite(d.accel.y) || !isfinite(d.accel.z))) ||
      (gyro_new && (!isfinite(d.gyro.x) || !isfinite(d.gyro.y) || !isfinite(d.gyro.z)))) {
    latchFault("imu_nonfinite_sample");
    return;
  }
  capture_.sensor_mask = bits;
  capture_.accel_fresh = accel_new;
  capture_.gyro_fresh = gyro_new;
  if (accel_new) {
    capture_.ax_g = d.accel.x; capture_.ay_g = d.accel.y; capture_.az_g = d.accel.z;
    capture_.acc_norm_g = sqrtf(capture_.ax_g* capture_.ax_g + capture_.ay_g*capture_.ay_g + capture_.az_g*capture_.az_g);
    capture_.acc_norm_error_g = capture_.acc_norm_g - 1.0f;
    capture_.pitch_accel_only_deg = Config::PITCH_SIGN * atan2f(-capture_.ax_g,
        sqrtf(capture_.ay_g*capture_.ay_g + capture_.az_g*capture_.az_g)) * 57.2957795f;
    capture_.accel_update_dt_us = prev_accel_update_us_ ? sample_us - prev_accel_update_us_ : 1000000UL / Config::BMI270_ACCEL_ODR_HZ;
    prev_accel_update_us_ = sample_us;
    capture_.last_accel_update_us = sample_us;
    ++capture_.accel_sequence;
  }
  if (gyro_new) {
    capture_.gx_dps = d.gyro.x; capture_.gy_dps = d.gyro.y; capture_.gz_dps = d.gyro.z;
    capture_.pitch_rate_dps = Config::GYRO_PITCH_RATE_SIGN * capture_.gy_dps;
    capture_.gyro_update_dt_us = prev_gyro_update_us_ ? sample_us - prev_gyro_update_us_ : 1000000UL / Config::BMI270_GYRO_ODR_HZ;
    prev_gyro_update_us_ = sample_us;
    capture_.last_gyro_update_us = sample_us;
    ++capture_.gyro_sequence;
    capture_.update_dt_us = capture_.gyro_update_dt_us;
    capture_.last_update_us = sample_us;
    capture_.last_update_ms = millis();
    capture_.imu_ok = true;
    publishSample();
  }
}

void ImuManager::latchFault(const char* reason) {
  portENTER_CRITICAL(&mux_);
  if (!fault_) fault_reason_ = reason;
  fault_ = true;
  portEXIT_CRITICAL(&mux_);
}
void ImuManager::publishSample() {
  bool sequential;
  portENTER_CRITICAL(&mux_);
  sequential = sequential_;
  latest_capture_us_ = capture_.last_gyro_update_us;
  latest_capture_ms_ = capture_.last_update_ms;
  ++total_captured_;
  audit_.sample(capture_.last_gyro_update_us, capture_.gyro_update_dt_us, capture_.gyro_sequence);
  portEXIT_CRITICAL(&mux_);
  if (xQueueSend(sample_queue_, &capture_, 0) != pdTRUE) {
    if (sequential) {
      portENTER_CRITICAL(&mux_);
      audit_.drop();
      portEXIT_CRITICAL(&mux_);
      latchFault("imu_acquisition_queue_overflow");
    } else {
      // While idle only, prefer the most recent sample (e.g. during a download).
      ImuReading discarded;
      xQueueReceive(sample_queue_, &discarded, 0);
      xQueueSend(sample_queue_, &capture_, 0);
    }
  }
  const uint32_t depth = uxQueueMessagesWaiting(sample_queue_);
  portENTER_CRITICAL(&mux_);
  if (audit_.active && depth > audit_.queue_high_water) audit_.queue_high_water = depth;
  portEXIT_CRITICAL(&mux_);
}

void ImuManager::setAcquisitionContext(bool sequential, bool measurement) {
  const uint32_t now_us = micros();
  portENTER_CRITICAL(&mux_);
  sequential_ = sequential;
  if (measurement && !audit_.active) audit_.start(now_us);
  if (!measurement && audit_.active) audit_.finish(now_us);
  portEXIT_CRITICAL(&mux_);
}
void ImuManager::update() {
  // This function runs on the Arduino thread. It is the ONLY writer of reading_.
  reading_.accel_fresh = false;
  reading_.gyro_fresh = false;
  reading_.sensor_mask = 0;
  if (!started_ || !sample_queue_) { reading_.imu_ok = false; return; }
  bool sequential, fault;
  const char* reason;
  portENTER_CRITICAL(&mux_);
  sequential = sequential_;
  fault = fault_;
  reason = fault_reason_;
  portEXIT_CRITICAL(&mux_);
  if (fault) { reading_.imu_ok = false; last_error_ = reason; return; }
  ImuReading next;
  const uint32_t depth = uxQueueMessagesWaiting(sample_queue_);
  if (xQueueReceive(sample_queue_, &next, 0) != pdTRUE) return;
  if (!sequential) {
    // Bounded drain, never an unbounded loop racing the producer.
    ImuReading newer;
    for (uint32_t i = 1; i < kQueueLength && xQueueReceive(sample_queue_, &newer, 0) == pdTRUE; ++i)
      next = newer;
  }
  const uint32_t age_us = static_cast<uint32_t>(micros() - next.last_gyro_update_us);
  const int core = xPortGetCoreID();
  const uint32_t priority = uxTaskPriorityGet(nullptr);
  portENTER_CRITICAL(&mux_);
  consumer_core_ = core;
  consumer_priority_ = priority;
  audit_.delivery(next.last_gyro_update_us, next.gyro_sequence, age_us, depth);
  portEXIT_CRITICAL(&mux_);
  if (sequential && age_us > kMaximumDeliveryAgeUs) {
    latchFault("imu_delivery_backlog_over_10ms");
    reading_.imu_ok = false;
    last_error_ = "imu_delivery_backlog_over_10ms";
    return;
  }
  reading_ = next;
  reading_.time_since_last_pulse_ms = static_cast<uint16_t>(min<uint32_t>(65535, beta_context_time_since_last_pulse_ms_));
  last_error_ = "";
}

bool ImuManager::acquisitionHealthy() const {
  portENTER_CRITICAL(&mux_);
  const bool healthy = !fault_;
  portEXIT_CRITICAL(&mux_);
  return started_ && healthy;
}
bool ImuManager::ok() const {
  return imu_present_ && reading_.imu_ok && reading_.rate_config_ok && acquisitionHealthy();
}
bool ImuManager::stale(uint32_t now_ms) const {
  portENTER_CRITICAL(&mux_);
  const uint32_t stamp = latest_capture_us_, stamp_ms = latest_capture_ms_;
  portEXIT_CRITICAL(&mux_);
  return !acquisitionHealthy() || stamp == 0 ||
      static_cast<uint32_t>(now_ms - stamp_ms) > Config::IMU_STALE_LIMIT_MS;
}
void ImuManager::zeroPitch() { }
void ImuManager::setDynamicBetaContext(bool pulse_active, uint32_t since_ms, bool pre_start) {
  beta_context_pulse_active_ = pulse_active;
  beta_context_time_since_last_pulse_ms_ = since_ms;
  beta_context_pre_start_stabilize_ = pre_start;
}
void ImuManager::forceSmoothBeta(float beta, uint8_t mode) {
  reading_.beta_target = beta; reading_.beta_smooth = beta;
  reading_.beta_dynamic = beta; reading_.beta_update_mode = mode;
}

String ImuManager::acquisitionDiagnosticsJson() const {
  bool fault;
  const char* reason;
  int reader_core, consumer_core;
  uint32_t reader_priority, consumer_priority, total;
  // Fixed-size snapshot. No String construction, sensor I/O or queue operations in the lock.
  portENTER_CRITICAL(&mux_);
  audit_snapshot_ = audit_;
  fault = fault_; reason = fault_reason_;
  reader_core = reader_core_; consumer_core = consumer_core_;
  reader_priority = reader_priority_; consumer_priority = consumer_priority_;
  total = total_captured_;
  portEXIT_CRITICAL(&mux_);
  const auto& a = audit_snapshot_;
  String json;
  json.reserve(10000);
  json = "{\"revision\":\"v46n_priority_imu_20260914\",\"firmware_version\":\"0.46.13\"";
  json += ",\"timestamp_semantics\":\"M5Unified_host_acquisition_not_sensor_clock\"";
  json += ",\"motor_controller\":\"unchanged_V46l_legacy_V7\"";
  json += ",\"reader_core\":" + String(reader_core) + ",\"reader_priority\":" + String(reader_priority);
  json += ",\"consumer_core\":" + String(consumer_core) + ",\"consumer_priority\":" + String(consumer_priority);
  json += ",\"internal_i2c_port\":" + String(internal_i2c_port_);
  json += ",\"internal_sda\":" + String(internal_sda_) + ",\"internal_scl\":" + String(internal_scl_);
  json += ",\"roller_i2c_port\":0,\"queue_capacity\":32,\"delivery_age_limit_us\":10000";
  json += ",\"started\":" + String(started_ ? "true" : "false");
  json += ",\"fault\":" + String(fault ? "true" : "false") + ",\"fault_reason\":\"" + String(reason) + "\"";
  json += ",\"total_captured_since_boot\":" + String(total);
  json += ",\"measurement_finished\":" + String(a.finished ? "true" : "false");
  json += ",\"epoch_us\":" + String(a.epoch_us) + ",\"duration_us\":" + String(a.duration_us);
  json += ",\"captured\":" + String(a.samples) + ",\"delivered\":" + String(a.delivered);
  json += ",\"polls\":" + String(a.polls) + ",\"coalesced_wakes\":" + String(a.coalesced_wakes);
  json += ",\"dt_mean_us\":" + String(a.samples ? static_cast<double>(a.dt_sum_us) / a.samples : 0.0, 3);
  json += ",\"dt_max_us\":" + String(a.dt_max_us) + ",\"poll_max_us\":" + String(a.poll_max_us);
  json += ",\"delivery_age_mean_us\":" + String(a.delivered ? static_cast<double>(a.age_sum_us) / a.delivered : 0.0, 3);
  json += ",\"delivery_age_max_us\":" + String(a.age_max_us);
  json += ",\"over_4ms\":" + String(a.over_4ms) + ",\"over_5ms\":" + String(a.over_5ms);
  json += ",\"over_10ms\":" + String(a.over_10ms) + ",\"queue_drops\":" + String(a.queue_drops);
  json += ",\"queue_high_water\":" + String(a.queue_high_water);
  json += ",\"delivery_sequence_gaps\":" + String(a.delivery_sequence_gaps);
  json += ",\"histogram_bin_us\":250,\"histogram_last_bin_lower_us\":10000";
  auto array = [&json](const char* key, const uint32_t* data, unsigned count) {
    json += ",\"" + String(key) + "\":[";
    for (unsigned i = 0; i < count; ++i) { if (i) json += ","; json += String(data[i]); }
    json += "]";
  };
  array("acquisition_dt_histogram", a.dt_bins, a.kBins);
  array("delivery_age_histogram", a.age_bins, a.kBins);
  array("samples_per_second", a.samples_per_second, a.kSeconds);
  array("max_dt_per_second_us", a.max_dt_per_second, a.kSeconds);
  array("max_delivery_age_per_second_us", a.max_age_per_second, a.kSeconds);
  json += ",\"long_gap_detail_overflow\":" + String(a.gap_overflow) + ",\"long_gaps\":[";
  for (uint32_t i = 0; i < a.gap_count; ++i) {
    if (i) json += ",";
    json += "{\"time_us\":" + String(a.gaps[i].time_us);
    json += ",\"dt_us\":" + String(a.gaps[i].dt_us) + ",\"sequence\":" + String(a.gaps[i].sequence) + "}";
  }
  json += "]}";
  return json;
}
