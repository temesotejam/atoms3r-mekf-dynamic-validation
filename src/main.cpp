#include <Arduino.h>
#include <M5Unified.h>
#include <WebServer.h>

#include "config.h"
#include "experiment_runner.h"
#include "imu_manager.h"
#include "psram_logger.h"
#include "roller485_manager.h"
#include "web_ui.h"

WebServer server(Config::HTTP_PORT);
PsramLogger logger;
ImuManager imu;
Roller485Manager roller;
ExperimentRunner runner;
WebUi web;

static void displayLine(const char* line1, const char* line2 = "") {
  if (!M5.Display.width()) return;
  M5.Display.fillScreen(TFT_BLACK);
  M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
  M5.Display.setTextSize(1);
  M5.Display.setCursor(0, 4);
  M5.Display.println(line1);
  if (line2 && line2[0]) M5.Display.println(line2);
}

void setup() {
  Serial.begin(Config::SERIAL_BAUD);
  delay(300);
  Serial.println();
  Serial.println("AtomS3CAM E2 Q shadow passive logger");

  auto cfg = M5.config();
  cfg.serial_baudrate = 0;
  cfg.internal_imu = true;
  M5.begin(cfg);
  Serial.printf("E2 Q shadow identity: board=%d imu_type=%d M5Unified=%s M5GFX=%s AHRS=%s base=%s\\n",
                static_cast<int>(M5.getBoard()), static_cast<int>(M5.Imu.getType()),
                Config::RESOLVED_M5UNIFIED_VERSION, Config::RESOLVED_M5GFX_VERSION,
                Config::RESOLVED_ADAFRUIT_AHRS_VERSION, Config::V62_BASE_COMMIT);
  displayLine("E2 Q SHADOW", "motor output OFF");

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
  Serial.println("Open http://192.168.4.1/");
  displayLine("AP ready", Config::AP_SSID);
}

void loop() {
  const uint32_t loop_start_us = micros();
  M5.update();

  runner.serviceFast();
  runner.updateImuDynamicBetaContext();
  imu.update();
  roller.update();
  runner.update();
  web.update();

  runner.setLoopDt(static_cast<uint32_t>(micros() - loop_start_us));
}
