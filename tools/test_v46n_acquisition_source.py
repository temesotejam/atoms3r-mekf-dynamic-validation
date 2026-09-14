#!/usr/bin/env python3
from pathlib import Path
import json
import subprocess

ROOT = Path(__file__).resolve().parents[1]
BASE = '50c9e049ab4c2dc73852aa6ccb2fd988ad18852c'
def text(path): return (ROOT / path).read_text(encoding='utf-8')
def baseline(path): return subprocess.check_output(['git', 'show', BASE + ':' + path], cwd=ROOT)

# Whole files, not just guard keywords: retain all motor authority and calibration.
for path in ('src/config.h', 'src/roller485_manager.cpp', 'src/roller485_manager.h',
             'src/mekf6.cpp', 'src/mekf6.hpp', 'src/upright_pose_guide.h',
             'src/log_types.h', 'platformio.ini'):
    assert (ROOT / path).read_bytes() == baseline(path), path + ' changed'

runner = text('src/experiment_runner.cpp')
old = baseline('src/experiment_runner.cpp').decode('utf-8').replace('\r\n', '\n')
anchor = 'r.gyro_sequence != timing_probe_event_.gyro_sequence_at_start) {'
replacement = 'r.gyro_sequence != timing_probe_event_.gyro_sequence_at_start &&\n        static_cast<int32_t>(r.last_gyro_update_us - timing_probe_event_.pulse_start_us) >= 0) {'
assert old.count(anchor) == 1
assert runner == old.replace(anchor, replacement, 1), 'controller changed beyond observation timestamp guard'

imu = text('src/imu_manager.cpp')
h = text('src/imu_manager.h')
main = text('src/main.cpp')
consumer = imu[imu.index('void ImuManager::update()'):imu.index('bool ImuManager::acquisitionHealthy()')]
callback = imu[imu.index('void ImuManager::timerCallback'):imu.index('void ImuManager::taskEntry')]
producer = imu[imu.index('void ImuManager::captureSensor()'):imu.index('void ImuManager::latchFault')]
assert imu.count('M5.Imu.update()') == 1
assert 'M5.Imu.update()' in producer
assert 'M5.Imu' not in consumer and 'M5.Imu' not in callback
assert 'xQueueReceive' in consumer and 'xTaskNotifyGive' in callback
assert 'portMAX_DELAY' not in consumer
assert 'kReaderCore = 1' in h and 'kReaderPriority = 6' in h
assert 'xTaskCreatePinnedToCore' in imu and 'xQueueCreateStatic' in imu
assert 'Config::IMU_POLL_PERIOD_US' in imu
assert 'internal_i2c_port_ != 1' in imu
assert 'kMaximumDeliveryAgeUs = 10000' in h
assert 'sequential && age_us > kMaximumDeliveryAgeUs' in consumer
assert 'imu_acquisition_queue_overflow' in imu
assert 'fault_ = false' not in imu, 'a latched runtime fault cannot auto-rearm'
assert 'imu_acquisition_overflow_backlog_or_stale' in main
assert main.index('checkAcquisitionHealth();') < main.index('runner.update();')
assert 'roller.startIoTask(' in main and 'roller.stop();' not in main
for forbidden in ('Serial.', 'String ', 'runner.', 'roller.', 'Wire.'):
    assert forbidden not in producer, forbidden
assert 'beta_context_' not in producer
assert 'capture_.' not in consumer
assert 'v46n_imu_acquisition' in text('src/psram_logger.cpp')
assert 'acquisitionDiagnosticsJson()' in text('src/web_ui.cpp')
manifest = json.loads(text('site/manifest.json'))
assert manifest['version'] == '0.46.13'
assert 'V46n' in manifest['name']
assert 'v46n_priority_imu_20260914' in text('site/index.html')
print('V46n exclusive ownership, bounded delivery and exact preserved control baseline PASS')
