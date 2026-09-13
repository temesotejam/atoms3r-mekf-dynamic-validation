from pathlib import Path

config = Path('src/config.h').read_text(encoding='utf-8')
main = Path('src/main.cpp').read_text(encoding='utf-8')
roller_h = Path('src/roller485_manager.h').read_text(encoding='utf-8')
roller_cpp = Path('src/roller485_manager.cpp').read_text(encoding='utf-8')
runner = Path('src/experiment_runner.cpp').read_text(encoding='utf-8')
web = Path('src/web_ui.cpp').read_text(encoding='utf-8')
manifest = Path('site/manifest.json').read_text(encoding='utf-8')

assert 'v46i_mekf_400hz_dual_core_roller_queue_20260913' in config
assert 'ROLLER_IO_TASK_CORE = 0' in config
assert 'ROLLER_IO_TASK_PRIORITY = 4' in config
assert 'ROLLER_IO_TASK_STACK_BYTES = 4096UL' in config
assert 'AtomS3R V46i MEKF dual-core motor validation' in main
assert 'roller.startIoTask(' in main
assert 'roller.update();' not in main
assert 'if (!runner.running())' in main
assert 'taskYIELD();' in main
assert 'xTaskCreatePinnedToCore' in roller_cpp
assert 'xQueueCreate(4, sizeof(RollerCommand))' in roller_cpp
assert 'xQueueSend(command_queue_' in roller_cpp
assert 'xQueueReset(command_queue_)' in roller_cpp
assert 'xTaskNotifyGive(io_task_handle_)' in roller_cpp
assert 'ulTaskNotifyTake' in roller_cpp
assert 'CURRENT_AUDIT_FAST_READ_PERIOD_US' in roller_cpp
assert 'telemetrySnapshot()' in roller_cpp
assert 'portENTER_CRITICAL(&telemetry_mux_)' in roller_cpp
assert 'Control-core API: queue a desired current; no Roller I2C is performed here.' in roller_h
assert 'roller_->telemetry()' not in runner
assert 'roller_->telemetry()' not in web
assert runner.count('telemetrySnapshot()') >= 2
assert 'roller_io_task_running' in web
assert 'roller_command_latency_max_us' in web
assert 'AtomS3R V46i MEKF Motor Validation' in manifest
assert '"version": "0.46.8"' in manifest
print('V46i dual-core task split source guards passed')
