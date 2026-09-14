from pathlib import Path

OLD_REV = "v46l_fast_solver_shadow_20260914"
NEW_REV = "v46l_fast_solver_shadow_upright_gate_fix_20260914"
OLD_VERSION = '"version": "0.46.11"'
NEW_VERSION = '"version": "0.46.12"'


def read(path: str) -> str:
    return Path(path).read_text(encoding="utf-8")


def write(path: str, text: str) -> None:
    Path(path).write_text(text, encoding="utf-8")


def replace_once(path: str, old: str, new: str) -> None:
    text = read(path)
    count = text.count(old)
    if count != 1:
        raise RuntimeError(f"{path}: expected exactly one match, got {count}: {old[:160]!r}")
    write(path, text.replace(old, new, 1))


# Identity/version: this is a V46l bug-fix build, not the V46m control switch.
replace_once(
    "src/config.h",
    f'static constexpr char ATTITUDE_VALIDATION_REVISION[] = "{OLD_REV}";',
    f'static constexpr char ATTITUDE_VALIDATION_REVISION[] = "{NEW_REV}";',
)
for path in ("src/main.cpp", "site/index.html"):
    s = read(path).replace("V46l", "V46l-r1").replace(OLD_REV, NEW_REV)
    write(path, s)
replace_once("site/manifest.json", OLD_VERSION, NEW_VERSION)
manifest = read("site/manifest.json").replace("V46l", "V46l-r1").replace(OLD_REV, NEW_REV)
write("site/manifest.json", manifest)

# Update identity-only assertions inherited from V46l.
for guard_path in (
    "tools/test_v46g_highrate_source_guards.py",
    "tools/test_v46i_task_split_source_guards.py",
    "tools/test_v46_motor_validation_source_guards.py",
    "tools/test_v46k_timing_probe_source_guards.py",
    "tools/test_v46l_fast_solver_shadow.py",
):
    s = read(guard_path)
    s = s.replace(OLD_REV, NEW_REV).replace("V46l", "V46l-r1").replace("0.46.11", "0.46.12")
    write(guard_path, s)

# ExperimentRunner receives the continuously maintained startup upright latch.
runner_h = read("src/experiment_runner.h")
api_marker = "  void setLoopDt(uint32_t dt_us) { status_.loop_dt_us = dt_us; }\n"
if runner_h.count(api_marker) != 1:
    raise RuntimeError("experiment_runner.h: setLoopDt marker changed")
runner_h = runner_h.replace(
    api_marker,
    api_marker
    + "  void setStartupUprightConfirmed(bool confirmed) { startup_upright_confirmed_ = confirmed; }\n"
    + "  bool startupUprightConfirmed() const { return startup_upright_confirmed_; }\n",
    1,
)
private_marker = "  PsramLogger::SolverShadowEvent solver_shadow_event_{};\n"
if runner_h.count(private_marker) != 1:
    raise RuntimeError("experiment_runner.h: solver shadow state marker changed")
runner_h = runner_h.replace(
    private_marker,
    "  bool startup_upright_confirmed_ = false;\n" + private_marker,
    1,
)
write("src/experiment_runner.h", runner_h)

# Start uses the 400 ms continuous guide latch rather than re-running one
# instantaneous sample gate at the HTTP POST instant.
runner = read("src/experiment_runner.cpp")
old_start_gate = '''  const ImuReading& autonomous_start_reading = imu_->reading();
  if (!UprightPoseGuide::isUprightStableSample(autonomous_start_reading)) {
    status_.last_error = "upright_pose_required_before_start";
    return false;
  }
'''
new_start_gate = '''  // The startup guide has already required 400 ms of continuous upright
  // stability. Main continuously revokes this latch if the pose becomes
  // unstable again before the HTTP start request is serviced.
  if (!startup_upright_confirmed_) {
    status_.last_error = "upright_pose_required_before_start";
    return false;
  }
'''
if runner.count(old_start_gate) != 1:
    raise RuntimeError(f"experiment_runner.cpp: instantaneous upright start gate count={runner.count(old_start_gate)}")
runner = runner.replace(old_start_gate, new_start_gate, 1)
write("src/experiment_runner.cpp", runner)

# Main guide: confirmation is no longer permanent. If the live stable-sample
# check fails after confirmation but before the run starts, revoke the latch,
# relight the LED, and require a fresh 400 ms continuous hold.
main = read("src/main.cpp")
old_head = '''static void updateStartupPoseGuide() {
  // Once a measurement starts, ExperimentRunner owns the LED completely so the
  // existing START/MID/END video synchronization pattern is never modified.
  if (startup_upright_confirmed || runner.running()) return;

  const uint32_t now_ms = millis();
'''
new_head = '''static void updateStartupPoseGuide() {
  // Once a measurement starts, ExperimentRunner owns the LED completely so the
  // existing START/MID/END video synchronization pattern is never modified.
  if (runner.running()) return;

  const uint32_t now_ms = millis();
'''
if main.count(old_head) != 1:
    raise RuntimeError("main.cpp: startup guide head marker changed")
main = main.replace(old_head, new_head, 1)

old_sample = '''  digitalWrite(Config::SYNC_LED_PIN, HIGH);

  const ImuReading& r = imu.reading();
  if (!UprightPoseGuide::isUprightStableSample(r)) {
    startup_upright_since_ms = 0;
    return;
  }

  if (startup_upright_since_ms == 0) startup_upright_since_ms = now_ms;
'''
new_sample = '''  const ImuReading& r = imu.reading();
  const bool upright_sample_ok = UprightPoseGuide::isUprightStableSample(r);

  // A previously confirmed pose remains valid only while the live sample stays
  // inside the same gravity/norm/gyro safety envelope. Any departure revokes
  // the latch before WebUi services a start request later in this loop.
  if (startup_upright_confirmed) {
    if (upright_sample_ok) {
      runner.setStartupUprightConfirmed(true);
      digitalWrite(Config::SYNC_LED_PIN, LOW);
      return;
    }
    startup_upright_confirmed = false;
    startup_upright_since_ms = 0;
    runner.setStartupUprightConfirmed(false);
    digitalWrite(Config::SYNC_LED_PIN, HIGH);
    Serial.println("Startup guide: upright confirmation revoked; pose no longer stable");
    displayLine("Stand upright", "stability lost");
    return;
  }

  digitalWrite(Config::SYNC_LED_PIN, HIGH);
  runner.setStartupUprightConfirmed(false);
  if (!upright_sample_ok) {
    startup_upright_since_ms = 0;
    return;
  }

  if (startup_upright_since_ms == 0) startup_upright_since_ms = now_ms;
'''
if main.count(old_sample) != 1:
    raise RuntimeError("main.cpp: startup guide sample marker changed")
main = main.replace(old_sample, new_sample, 1)

old_confirm = '''  startup_upright_confirmed = true;
  digitalWrite(Config::SYNC_LED_PIN, LOW);
'''
new_confirm = '''  startup_upright_confirmed = true;
  runner.setStartupUprightConfirmed(true);
  digitalWrite(Config::SYNC_LED_PIN, LOW);
'''
if main.count(old_confirm) != 1:
    raise RuntimeError("main.cpp: startup guide confirmation marker changed")
main = main.replace(old_confirm, new_confirm, 1)
write("src/main.cpp", main)

# Expose the actual gate and live diagnostics in status.json, so a future block
# says why it is blocked without guessing from the display-only READY flag.
web = read("src/web_ui.cpp")
status_marker = '  json += ",\\\"ready\\\":" + String(st.ready ? "true" : "false");\n'
if web.count(status_marker) != 1:
    raise RuntimeError("web_ui.cpp: ready status marker changed")
status_add = r'''  json += ",\"startup_upright_confirmed\":" + String(runner_->startupUprightConfirmed() ? "true" : "false");
  const ImuReading& upright_r = imu_->reading();
  json += ",\"startup_upright_direction_error_deg\":" + String(UprightPoseGuide::directionErrorDeg(upright_r), 3);
  json += ",\"startup_upright_accel_norm_g\":" + String(UprightPoseGuide::accelNormG(upright_r), 4);
  json += ",\"startup_upright_gyro_norm_dps\":" + String(UprightPoseGuide::gyroNormDps(upright_r), 3);
  json += ",\"startup_upright_max_direction_error_deg\":" + String(UprightPoseGuide::UPRIGHT_MAX_DIRECTION_ERROR_DEG, 2);
  json += ",\"startup_upright_max_gyro_norm_dps\":" + String(UprightPoseGuide::UPRIGHT_MAX_GYRO_NORM_DPS, 2);
'''
web = web.replace(status_marker, status_marker + status_add, 1)
summary_old = "| motor=${j.motor_cmd_mA||0} mA | actual=${j.roller_actual_current_mA||0} mA | remaining=${j.remaining_s||0} s`"
summary_new = "| upright=${!!j.startup_upright_confirmed} | motor=${j.motor_cmd_mA||0} mA | actual=${j.roller_actual_current_mA||0} mA | remaining=${j.remaining_s||0} s`"
if summary_old not in web:
    raise RuntimeError("web_ui.cpp: browser summary marker changed")
web = web.replace(summary_old, summary_new, 1)
write("src/web_ui.cpp", web)

# Page wording explains the revocable latch rather than promising a permanent
# LED-off state.
site = read("site/index.html")
site = site.replace(
    "機体を立てて静止させます。実測した立位重力方向に約400 ms安定するとLEDが<strong>消灯</strong>します。",
    "機体を立てて静止させます。実測した立位重力方向に約400 ms連続で安定するとLEDが<strong>消灯</strong>します。Start前に姿勢が安全範囲から外れた場合は確認が取り消され、LEDが再点灯します。",
)
write("site/index.html", site)

print("Applied V46l-r1 upright-gate consistency fix")
