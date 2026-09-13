#include <Arduino.h>
#include <M5Unified.h>
#include <WebServer.h>

#include "config.h"
#include "experiment_runner.h"
#include "imu_manager.h"
#include "psram_logger.h"
#include "roller485_manager.h"
#include "upright_pose_guide.h"
#include "web_ui.h"

WebServer server(Config::HTTP_PORT);
PsramLogger logger;
ImuManager imu;
Roller485Manager roller;
ExperimentRunner runner;
WebUi web;

static uint32_t startup_guide_boot_ms = 0;
static uint32_t startup_upright_since_ms = 0;
static bool startup_guide_prompt_announced = false;
static bool startup_upright_confirmed = false;

static void displayLine(const char* line1, const char* line2 = "") {
  if (!M5.Display.width()) return;
  M5.Display.fillScreen(TFT_BLACK);
  M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
  M5.Display.setTextSize(1);
  M5.Display.setCursor(0, 4);
  M5.Display.println(line1);
  if (line2 && line2[0]) M5.Display.println(line2);
}

static void updateStartupPoseGuide() {
  // Once a measurement starts, ExperimentRunner owns the LED completely so the
  // existing START/MID/END video synchronization pattern is never modified.
  if (startup_upright_confirmed || runner.running()) return;

  const uint32_t now_ms = millis();
  if (static_cast<uint32_t>(now_ms - startup_guide_boot_ms) <
      UprightPoseGuide::GUIDE_LED_ON_AFTER_BOOT_MS) {
    return;
  }

  if (!startup_guide_prompt_announced) {
    startup_guide_prompt_announced = true;
    Serial.println("Startup guide: 10 s elapsed; LED ON until upright pose is stable");
    displayLine("Stand upright", "LED ON until stable");
  }

  // This write intentionally occurs after runner.update() while idle. During a
  // measurement runner.running()==true, so the synchronization LED is untouched.
  digitalWrite(Config::SYNC_LED_PIN, HIGH);

  const ImuReading& r = imu.reading();
  if (!UprightPoseGuide::isUprightStableSample(r)) {
    startup_upright_since_ms = 0;
    return;
  }

  if (startup_upright_since_ms == 0) startup_upright_since_ms = now_ms;
  if (static_cast<uint32_t>(now_ms - startup_upright_since_ms) <
      UprightPoseGuide::UPRIGHT_STABLE_HOLD_MS) {
    return;
  }

  startup_upright_confirmed = true;
  digitalWrite(Config::SYNC_LED_PIN, LOW);
  Serial.printf("Startup guide: upright confirmed; gravity error=%.2f deg, norm=%.3f g\n",
                UprightPoseGuide::directionErrorDeg(r), UprightPoseGuide::accelNormG(r));
  displayLine("Upright ready", "Start from Web UI");
}

void setup() {
  startup_guide_boot_ms = millis();
  Serial.begin(Config::SERIAL_BAUD);
  delay(300);
  Serial.println();
  Serial.println("AtomS3R V46 MEKF motor-driven dynamic validation");

  auto cfg = M5.config();
  cfg.serial_baudrate = 0;
  cfg.internal_imu = true;
  M5.begin(cfg);
  Serial.printf("V46 identity: board=%d imu_type=%d M5Unified=%s M5GFX=%s AHRS=%s base=%s attitude=%s\n",
                static_cast<int>(M5.getBoard()), static_cast<int>(M5.Imu.getType()),
                Config::RESOLVED_M5UNIFIED_VERSION, Config::RESOLVED_M5GFX_VERSION,
                Config::RESOLVED_ADAFRUIT_AHRS_VERSION, Config::V62_BASE_COMMIT,
                Config::ATTITUDE_VALIDATION_REVISION);
  displayLine("V46 MEKF", "V7 MOTOR VALIDATION");

  const bool psram_ok = logger.begin();
  Serial.printf("PSRAM: %s total=%u free=%u sample_capacity=%u\n", psram_ok ? "OK" : "FAILED",
                static_cast<unsigned>(logger.psramTotal()), static_cast<unsigned>(logger.psramFree()),
                static_cast<unsigned>(logger.sampleCapacity()));
  if (!psram_ok) Serial.printf("PSRAM error: %s\n", logger.lastError());

  const bool imu_ok = imu.begin();
  Serial.printf("IMU: %s\n", imu_ok ? "OK" : "FAILED");

  const bool roller_ok = roller.begin();
  roller.stop();
  Serial.printf("Roller485: %s\n", roller_ok ? "OK" : "FAILED");

  runner.begin(logger, imu, roller);
  web.begin(server, runner, imu, roller, logger);

  Serial.printf("AP SSID: %s\n", Config::AP_SSID);
  Serial.println("Open http://192.168.4.1/ and start Autonomous Energy Control V7");
  displayLine("V7 motor ready", Config::AP_SSID);
}

void loop() {
  const uint32_t loop_start_us = micros();
  M5.update();

  runner.serviceFast();
  runner.updateImuDynamicBetaContext();
  imu.update();
  roller.update();
  runner.update();
  updateStartupPoseGuide();
  web.update();

  runner.setLoopDt(static_cast<uint32_t>(micros() - loop_start_us));
}
