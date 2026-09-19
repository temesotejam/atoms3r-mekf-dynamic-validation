#include "web_ui.h"

#include <WiFi.h>

#include "config.h"
#include "upright_pose_guide.h"
#include "run_control_worker.h"
extern RunControlWorker run_control;

static const char INDEX_HTML[] PROGMEM = R"HTML(
<!doctype html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Autonomous Energy Control V7</title><style>
body{margin:0;font-family:system-ui,sans-serif;background:#f6f8fb;color:#17202a}header{padding:14px 16px;background:#263341;color:#fff}main{padding:14px;max-width:700px;margin:auto}.card{border:1px solid #b8c2ce;background:#fff;padding:14px;border-radius:7px;margin:12px 0}.grid{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:8px}.metric{background:#f6f8fb;border-radius:5px;padding:9px}.metric b{display:block;font-size:1.18rem}.yes{color:#087d2f}.no{color:#a11d27}button,a,select,input{box-sizing:border-box;width:100%;margin-top:10px;border:1px solid #b8c2ce;background:#1769e0;color:#fff;padding:11px;border-radius:6px;font-size:16px;text-align:center;text-decoration:none}select{background:#fff;color:#17202a}button.danger{background:#c4262e;border-color:#c4262e}button:disabled,a.disabled,select:disabled{opacity:.42;pointer-events:none}small{display:block;line-height:1.45;margin:9px 0;color:#536273}</style></head><body>
<header><h1>Autonomous Energy Control V7</h1><div>V46ad / 0.46.29 / delay compensation sweep</div></header><main><p id="summary">Connecting...</p>
<div class="card"><b>起動・立位診断</b><p id="startupInfo">IMUを初期化しています</p><a href="/imu-acquisition.json">停止中のIMU診断JSON</a><p id="errorInfo"></p></div>
<div class="card"><b>Current Roll (static-calibrated display)</b><div class="grid"><div class="metric">Physical Roll Abs<b id="abs">--</b></div><div class="metric">Current Roll<b id="current">--</b></div><div class="metric">Physical Rate<b id="rate">--</b></div><div class="metric">Target / Error<b id="targetError">--</b></div><div class="metric">STATIC<b id="static">--</b></div><div class="metric">READY<b id="ready">--</b></div></div><button id="zero" onclick="zeroCurrentRoll()">ZERO Current Roll (display only)</button><label for="target">Target Current Roll</label><select id="target" onchange="setTarget()"><option value="-15">-15 deg</option><option value="-12">-12 deg</option><option value="-8">-8 deg</option><option value="-4">-4 deg</option><option value="-1.5">-1.5 deg</option><option value="0" selected>0 deg</option><option value="1.5">+1.5 deg</option><option value="4">+4 deg</option><option value="8">+8 deg</option><option value="12">+12 deg</option><option value="15">+15 deg</option></select><small id="criteria">Display-only current-roll UI. ZERO and READY never change the V0 absolute energy target or motor command.</small></div>
<div class="card"><b>Q1 direct next-peak shadow (motor OFF)</b><small>Q1 shadow remains a diagnostic. Its target does not affect the V0 motor command.</small><label for="shadowTarget">Q1 shadow target |A| (deg)</label><input id="shadowTarget" type="number" min="0" max="18" step="0.1" value="0.0" onchange="setShadowTarget()"></div>
<div class="card"><b>Passive release capture (0 mA)</b><small>Records a one-release free-decay reference. All samples remain motor/current/pulse = 0.</small><button id="passive" onclick="startPassive()">Start passive capture</button></div>
<div class="card"><b>Autonomous Energy Control V7 (actual motor)</b><small>Start upright and do not touch the machine. After the LED start signature, the firmware gives exactly one 300 mA / 100 ms strong start, waits for the first confirmed continuous-angle physical peak, then applies normal P1/Q1 Energy Control at the following continuous-filter zero-cross. Normal pulse direction matches the zero-cross physical roll-rate sign; V7 uses a bounded side-response correction only in its peak prediction and Q selection; START_KICK remains direction -1. Each physical half-cycle is peak -> zero-cross -> at most one pulse -> next peak; pulse transients are never accepted as events. The Autonomous capture is 30 s. Q_IDENT, E2, the legacy V0 path, and startup pumping are not used.</small><label for="energyTarget">Walking target peak</label><select id="energyTarget" onchange="setEnergyTarget()"><option value="8" selected>8.0 deg</option><option value="10">10.0 deg</option><option value="12">12.0 deg</option></select><label for="timingCompensation">遅延補償時間（停止中のみ変更可）</label><select id="timingCompensation" onchange="setTimingCompensation()" disabled><option value="0">0 ms（補償なし）</option><option value="3" selected>3 ms（初期値）</option><option value="6">6 ms</option><option value="9">9 ms</option></select><small id="timingInfo">設定を確認しています</small><small>粗探索：目標8°のまま、3 → 6 → 9 → 0 → 3 msを各30秒。各Run終了後、次の開始前にRWLOGを保存してください。</small><button id="energy" disabled class="danger" onclick="startEnergy()">Start autonomous energy control</button></div>
<button id="stop" class="danger" onclick="postStop()">Emergency stop</button><a id="rwlog" href="/download/rwlog" onclick="beginDownload()">Download RWLOG</a><button id="clear" onclick="postClear()">Clear log memory</button></main><script>
let downloading=false,lastStatus={},displayFrozen=false,refreshInFlight=false,timingPending=false,startPending=false,timingConfirmed=null,controlEpoch=0;const energy=document.getElementById('energy'),energyTarget=document.getElementById('energyTarget'),timingCompensation=document.getElementById('timingCompensation'),passive=document.getElementById('passive'),stop=document.getElementById('stop'),clear=document.getElementById('clear'),rwlog=document.getElementById('rwlog'),zero=document.getElementById('zero'),target=document.getElementById('target'),shadowTarget=document.getElementById('shadowTarget');
const fmt=(v,n=2)=>Number.isFinite(Number(v))?`${Number(v).toFixed(n)} deg`:'--';function lock(e,v){if(e.tagName==='A')e.classList.toggle('disabled',v);else e.disabled=v;}async function post(path){const r=await fetch(path,{method:'POST'});if(!r.ok)alert(await r.text());await refresh();return r.ok;}async function startPassive(){await post('/start-passive');}async function startEnergy(){if(startPending||timingPending||timingConfirmed===null||energy.disabled)return;const value=timingConfirmed;startPending=true;controlEpoch++;apply(lastStatus);try{const r=await fetch('/start-energy-control-autonomous?timing_ms='+encodeURIComponent(value),{method:'POST'});if(!r.ok)alert(await r.text());else{displayFrozen=true;applyFrozenState();}}catch(e){alert('開始結果を確認できません。状態の再取得を待ってください。');}finally{startPending=false;controlEpoch++;refresh();}}
async function setTimingCompensation(){if(timingPending||startPending||timingCompensation.disabled)return;const value=timingCompensation.value;timingPending=true;controlEpoch++;apply(lastStatus);try{const r=await fetch('/energy-control-autonomous/timing-compensation?ms='+encodeURIComponent(value),{method:'POST'});if(!r.ok)alert(await r.text());}catch(e){alert('設定を確認できません。状態の再取得を待ってください。');}finally{timingPending=false;timingConfirmed=null;controlEpoch++;refresh();}}async function postStop(){displayFrozen=false;await post('/stop');}async function postClear(){if(confirm('Clear the current log?'))await post('/clear');}async function zeroCurrentRoll(){await post('/current-roll/zero');}async function setTarget(){await post('/current-roll/target?deg='+encodeURIComponent(target.value));}async function setShadowTarget(){await post('/q1-shadow/target?deg='+encodeURIComponent(shadowTarget.value));}async function setEnergyTarget(){await post('/energy-control-autonomous/target?deg='+encodeURIComponent(energyTarget.value));}function beginDownload(){downloading=true;apply(lastStatus);setTimeout(()=>{downloading=false;refresh();},3000);}function mark(id,yes){const e=document.getElementById(id);e.textContent=yes?'YES':'NO';e.className=yes?'yes':'no';}
function apply(j){const b=j.startup||{};document.getElementById('startupInfo').textContent=`${b.guide_reason||'--'} | 初期化 ${b.init_attempts||0}回 | norm=${b.accel_norm_g??'--'} g | 立位ずれ=${b.direction_error_deg??'--'}° | gyro=${b.gyro_norm_dps??'--'}°/s | age=${b.sample_age_us??'--'} us | IMU=${b.imu_error||'OK'}`;document.getElementById('errorInfo').textContent=j.last_error||'';const running=!!j.running,busy=downloading||!!j.downloading||timingPending||startPending,canStart=j.state==='READY_TO_MEASURE'||j.state==='FINISHED';if(!timingPending&&!startPending){const value=j.autonomous_timing_compensation_ms;timingConfirmed=[0,3,6,9].includes(value)?value:null;if(timingConfirmed!==null)timingCompensation.value=String(value);}lock(passive,busy||running||!canStart);lock(energy,busy||running||!canStart||timingConfirmed===null);lock(timingCompensation,busy||running||!canStart||timingConfirmed===null);document.getElementById('timingInfo').textContent=timingPending?'設定中…':`次のRun: ${timingConfirmed===null?'確認待ち':timingConfirmed+' ms'} | 前回/現在のRun: ${j.autonomous_run_timing_compensation_ms??'--'} ms`;lock(energyTarget,busy||running||!canStart);lock(stop,!running);lock(clear,busy||running);lock(rwlog,busy||running||j.rwlog_downloadable!=='yes');lock(zero,running||!j.static_confirmed);lock(target,running);lock(shadowTarget,running);document.getElementById('summary').textContent=`${j.state||'UNKNOWN'} | passive=${!!j.passive_capture_mode} | energy_v6=${!!j.energy_control_autonomous_mode} phase=${j.energy_control_autonomous_phase||'IDLE'} target=${Number(j.energy_control_autonomous_target_peak_deg||0).toFixed(1)} deg | motor=${j.motor_cmd_mA||0} mA | actual=${j.roller_actual_current_mA||0} mA | remaining=${j.remaining_s||0} s`;document.getElementById('abs').textContent=fmt(j.physical_roll_abs_deg);document.getElementById('current').textContent=fmt(j.current_roll_deg);document.getElementById('rate').textContent=fmt(j.physical_roll_rate_dps,3)+'/s';document.getElementById('targetError').textContent=`${fmt(j.target_roll_deg)} / ${fmt(j.target_error_deg)}`;mark('static',!!j.static_confirmed);mark('ready',!!j.ready);document.getElementById('criteria').textContent=`STATIC: |rate| <= ${Number(j.static_rate_threshold_dps||0).toFixed(2)} deg/s for ${j.static_hold_time_ms||0} ms. READY: display-only.`;if(document.activeElement!==target){const value=String(j.target_roll_deg);if([...target.options].some(o=>o.value===value))target.value=value;}if(document.activeElement!==shadowTarget&&Number.isFinite(Number(j.q1_shadow_target_peak_abs_deg)))shadowTarget.value=Number(j.q1_shadow_target_peak_abs_deg).toFixed(1);if(document.activeElement!==energyTarget&&Number.isFinite(Number(j.energy_control_autonomous_target_peak_deg)))energyTarget.value=String(Number(j.energy_control_autonomous_target_peak_deg));}
function applyFrozenState(){[passive,energy,energyTarget,timingCompensation,clear,rwlog,zero,target,shadowTarget].forEach(x=>lock(x,true));lock(stop,false);document.getElementById('summary').textContent='MEASUREMENT RUNNING | lightweight state heartbeat';document.getElementById('timingInfo').textContent='遅延補償はRun開始時の値で固定（実使用値はRWLOGに記録）';}
async function refresh(){if(refreshInFlight)return;refreshInFlight=true;const epoch=controlEpoch;let timer;try{const controller=new AbortController();timer=setTimeout(()=>controller.abort(),1500);const r=await fetch('/status.json',{cache:'no-store',signal:controller.signal});clearTimeout(timer);if(!r.ok)throw new Error('status_failed');const status=await r.json();if(epoch!==controlEpoch)return;lastStatus=status;if(lastStatus.running){displayFrozen=true;applyFrozenState();return;}if(displayFrozen)displayFrozen=false;apply(lastStatus);}catch(e){if(!displayFrozen)[energy,energyTarget,timingCompensation,passive,stop,clear,rwlog,zero,target,shadowTarget].forEach(x=>lock(x,true));}finally{if(timer)clearTimeout(timer);refreshInFlight=false;}}setInterval(refresh,1000);refresh();
</script></body></html>
)HTML";

void WebUi::begin(WebServer& server, ExperimentRunner& runner, ImuManager& imu, Roller485Manager& roller, PsramLogger& logger) {
  server_ = &server;
  runner_ = &runner;
  imu_ = &imu;
  roller_ = &roller;
  logger_ = &logger;

  WiFi.mode(WIFI_AP);
  WiFi.softAP(Config::AP_SSID, Config::AP_PASS, Config::AP_CHANNEL);

  server_->on("/", HTTP_GET, [this]() { handleRoot(); });
  server_->on("/status.json", HTTP_GET, [this]() { handleStatus(); });
  server_->on("/imu-acquisition.json", HTTP_GET, [this]() {
    if (run_control.active() || runner_->running()) { server_->send(409, "text/plain", "read_after_run"); return; }
    server_->sendHeader("Cache-Control", "no-store");
    server_->send(200, "application/json", imu_->acquisitionDiagnosticsJson());
  });
  server_->on("/start-passive", HTTP_POST, [this]() { handleStartPassive(); });
  server_->on("/start-energy-control-v0", HTTP_POST, [this]() { handleStartEnergyControlV0(); });
  server_->on("/start-energy-control-autonomous", HTTP_POST, [this]() { handleStartEnergyControlAutonomous(); });
  server_->on("/energy-control-autonomous/target", HTTP_POST, [this]() { handleSetEnergyControlAutonomousTarget(); });
  server_->on("/energy-control-autonomous/timing-compensation", HTTP_POST, [this]() { handleSetEnergyControlAutonomousTimingCompensation(); });
  server_->on("/stop", HTTP_POST, [this]() { handleStop(); });
  server_->on("/clear", HTTP_POST, [this]() { handleClear(); });
  server_->on("/settings", HTTP_POST, [this]() { handleSettings(); });
  server_->on("/current-roll/zero", HTTP_POST, [this]() { handleCurrentRollZero(); });
  server_->on("/current-roll/target", HTTP_POST, [this]() { handleSetCurrentRollTarget(); });
  server_->on("/q1-shadow/target", HTTP_POST, [this]() { handleSetQ1ShadowTargetPeakAbs(); });
  server_->on("/download/rwlog", HTTP_GET, [this]() { handleRwLog(); });
  server_->enableDelay(false);  // Empty HTTP polls must not add sleeps to idle acquisition.
  server_->begin();
}

void WebUi::update() {
  if (server_) server_->handleClient();
}

void WebUi::handleRoot() {
  if (run_control.active()) { server_->send(409, "text/plain", "run_in_progress"); return; }
  if (run_control.active() || runner_->running()) { server_->send(409, "text/plain", "read_after_run"); return; }
  server_->sendHeader("Cache-Control", "no-store, no-cache, must-revalidate");
  server_->sendHeader("Pragma", "no-cache");
  server_->send_P(200, "text/html; charset=utf-8", INDEX_HTML);
}

void WebUi::handleStatus() {
  if (run_control.active()) {
    // Copy only immutable POD status; do not read runner/logger/imu.reading
    // while the higher-priority worker owns them. No network I/O in a lock.
    const RunControlSnapshot st = run_control.snapshot();
    char body[192];
    snprintf(body, sizeof(body),
        "{\"running\":%s,\"state\":\"%s\",\"motor_cmd_mA\":%d,\"roller_actual_current_mA\":%d,\"remaining_ms\":%lu}",
        st.running ? "true" : "false", st.state_name,
        static_cast<int>(st.motor_cmd_mA), static_cast<int>(st.actual_current_mA),
        static_cast<unsigned long>(st.remaining_ms));
    server_->send(200, "application/json", body);
    return;
  }
  server_->send(200, "application/json", statusJson());
}

void WebUi::handleStartPassive() {
  if (!run_control.ready()) { server_->send(503, "text/plain", "run_control_worker_not_ready"); return; }
  if (run_control.active()) { server_->send(409, "text/plain", "run_in_progress"); return; }
  if (logger_->downloading()) {
    server_->send(409, "text/plain", "download_in_progress");
    return;
  }
  const bool ok = runner_->startPassiveCapture();
  server_->send(ok ? 200 : 409, "text/plain", ok ? "passive_capture_started" : "start_failed");
}

void WebUi::handleStartEnergyControlV0() {
  if (!run_control.ready()) { server_->send(503, "text/plain", "run_control_worker_not_ready"); return; }
  if (run_control.active()) { server_->send(409, "text/plain", "run_in_progress"); return; }
  if (logger_->downloading()) {
    server_->send(409, "text/plain", "download_in_progress");
    return;
  }
  const bool ok = runner_->startEnergyControlV0Capture();
  server_->send(ok ? 200 : 409, "text/plain",
                ok ? "energy_control_v0_started" : runner_->status().last_error);
}

void WebUi::handleStartEnergyControlAutonomous() {
  if (run_control.active()) { server_->send(409, "text/plain", "run_in_progress"); return; }
  if (logger_->downloading()) { server_->send(409, "text/plain", "download_in_progress"); return; }
  if (!run_control.ready()) { server_->send(503, "text/plain", "run_control_worker_not_ready"); return; }
  if (server_->hasArg("timing_ms")) {
    uint32_t expected_us = 0;
    if (!autonomous_timing::parseMs(server_->arg("timing_ms").c_str(), expected_us)) {
      server_->send(400, "text/plain", "timing_ms_must_be_0_3_6_or_9"); return;
    }
    if (expected_us != runner_->energyControlAutonomousTimingCompensationUs()) {
      server_->send(409, "text/plain", "timing_selection_changed_refresh_before_start"); return;
    }
  }
  // Refresh from the idle mailbox before the unchanged physical start gate.
  // The run boundary is established by main AFTER this HTTP response returns.
  imu_->update();
  const bool ok = runner_->startEnergyControlAutonomousCapture();
  server_->send(ok ? 200 : 409, "text/plain", ok ? "energy_control_autonomous_started" : runner_->status().last_error);
}

void WebUi::handleSetEnergyControlAutonomousTimingCompensation() {
  if (run_control.active()) { server_->send(409, "text/plain", "run_in_progress"); return; }
  if (runner_->running() || logger_->downloading()) { server_->send(409, "text/plain", "busy"); return; }
  uint32_t value_us = 0;
  if (!server_->hasArg("ms") || !autonomous_timing::parseMs(server_->arg("ms").c_str(), value_us)) {
    server_->send(400, "text/plain", "timing_ms_must_be_0_3_6_or_9"); return;
  }
  const bool ok = runner_->setEnergyControlAutonomousTimingCompensation(value_us);
  server_->send(ok ? 200 : 409, "text/plain", ok ? "timing_compensation_set" : runner_->status().last_error);
}

void WebUi::handleSetEnergyControlAutonomousTarget() {
  if (run_control.active()) { server_->send(409, "text/plain", "run_in_progress"); return; }
  if (!server_->hasArg("deg")) { server_->send(400, "text/plain", "target_deg_required"); return; }
  if (runner_->running()) { server_->send(409, "text/plain", "running"); return; }
  const bool ok = runner_->setEnergyControlAutonomousTarget(server_->arg("deg").toFloat());
  server_->send(ok ? 200 : 400, "text/plain", ok ? "energy_target_set" : runner_->status().last_error);
}

void WebUi::handleStartQIdent() {
  if (run_control.active()) { server_->send(409, "text/plain", "run_in_progress"); return; }
  server_->send(409, "text/plain", "q_ident_frozen_use_energy_control_v0");
}void WebUi::handleStart() {
  if (run_control.active()) { server_->send(409, "text/plain", "run_in_progress"); return; }
  if (logger_->downloading()) {
    server_->send(409, "text/plain", "download_in_progress");
    return;
  }
  if (!server_->hasArg("trial")) {
    server_->send(400, "text/plain", "trial_required");
    return;
  }
  const uint8_t trial_number = static_cast<uint8_t>(server_->arg("trial").toInt());
  const bool ok = runner_->startSingleTrialTest(trial_number);
  server_->send(ok ? 200 : 409, "text/plain", ok ? "started" : "start_failed");
}

void WebUi::handleStartZeroCross() {
  if (run_control.active()) { server_->send(409, "text/plain", "run_in_progress"); return; }
  if (logger_->downloading()) {
    server_->send(409, "text/plain", "download_in_progress");
    return;
  }
  if (!server_->hasArg("pulse_width_ms")) {
    server_->send(400, "text/plain", "pulse_width_ms_required");
    return;
  }
  const int16_t current_mA = Config::ZERO_CROSS_OPERATING_CURRENT_MA;
  const int pulse_width_ms = server_->arg("pulse_width_ms").toInt();
  const bool pulse_ok = pulse_width_ms >= Config::ZERO_CROSS_TIME_SWEEP_MIN_PULSE_MS &&
                        pulse_width_ms <= Config::ZERO_CROSS_TIME_SWEEP_MAX_PULSE_MS;
  if (!pulse_ok) {
    server_->send(400, "text/plain", "invalid_zero_cross_condition");
    return;
  }
  const bool ok = runner_->startZeroCrossTest(static_cast<int16_t>(current_mA),
                                              static_cast<uint16_t>(pulse_width_ms));
  server_->send(ok ? 200 : 409, "text/plain", ok ? "zero_cross_started" : "start_failed");
}
void WebUi::handleStartIdentification() {
  if (run_control.active()) { server_->send(409, "text/plain", "run_in_progress"); return; }
  if (logger_->downloading()) { server_->send(409, "text/plain", "download_in_progress"); return; }
  const bool ok = runner_->startZeroCrossIdentificationTest();
  server_->send(ok ? 200 : 409, "text/plain", ok ? "validation_started" : "start_failed");
}

void WebUi::handleStartControl() {
  if (run_control.active()) { server_->send(409, "text/plain", "run_in_progress"); return; }
  if (logger_->downloading()) { server_->send(409, "text/plain", "download_in_progress"); return; }
  if (!server_->hasArg("target_peak_deg")) { server_->send(400, "text/plain", "target_peak_deg_required"); return; }
  const float target_peak_deg = server_->arg("target_peak_deg").toFloat();
  const int schedule_arg = server_->hasArg("q_probe_schedule_id") ?
      server_->arg("q_probe_schedule_id").toInt() : Config::ZERO_CROSS_CALIBRATION_Q_PROBE_SCHEDULE_A;
  if (schedule_arg < Config::ZERO_CROSS_CALIBRATION_Q_PROBE_SCHEDULE_A ||
      schedule_arg >= Config::ZERO_CROSS_CALIBRATION_Q_PROBE_SCHEDULE_COUNT) {
    server_->send(400, "text/plain", "invalid_q_probe_schedule");
    return;
  }
  const bool ok = runner_->startZeroCrossControlTest(
      target_peak_deg, static_cast<uint8_t>(schedule_arg));
  server_->send(ok ? 200 : 409, "text/plain", ok ? "control_started" : "start_failed");
}
void WebUi::handleZero() {
  if (run_control.active()) { server_->send(409, "text/plain", "run_in_progress"); return; }
  if (runner_->running()) {
    server_->send(409, "text/plain", "running");
    return;
  }
  runner_->zeroAngleNow();
  server_->send(200, "text/plain", "zeroed");
}

void WebUi::handleCurrentRollZero() {
  if (run_control.active()) { server_->send(409, "text/plain", "run_in_progress"); return; }
  const bool ok = runner_->zeroCurrentRollDisplay();
  server_->send(ok ? 200 : 409, "text/plain", ok ? "current_roll_zeroed" : runner_->status().last_error);
}

void WebUi::handleSetCurrentRollTarget() {
  if (run_control.active()) { server_->send(409, "text/plain", "run_in_progress"); return; }
  if (!server_->hasArg("deg")) {
    server_->send(400, "text/plain", "target_deg_required");
    return;
  }
  if (runner_->running()) {
    server_->send(409, "text/plain", "running");
    return;
  }
  const bool ok = runner_->setCurrentRollTarget(server_->arg("deg").toFloat());
  server_->send(ok ? 200 : 400, "text/plain", ok ? "current_roll_target_set" : runner_->status().last_error);
}

void WebUi::handleSetQ1ShadowTargetPeakAbs() {
  if (run_control.active()) { server_->send(409, "text/plain", "run_in_progress"); return; }
  if (!server_->hasArg("deg")) {
    server_->send(400, "text/plain", "target_deg_required");
    return;
  }
  if (runner_->running()) {
    server_->send(409, "text/plain", "running");
    return;
  }
  const bool ok = runner_->setQ1ShadowTargetPeakAbs(server_->arg("deg").toFloat());
  server_->send(ok ? 200 : 400, "text/plain", ok ? "q1_shadow_target_set" : runner_->status().last_error);
}

void WebUi::handleStop() {
  if (run_control.requestStop()) {
    server_->send(202, "text/plain", "stop_requested");
    return;
  }
  runner_->requestEmergencyStop("web_estop");
  server_->send(200, "text/plain", "stopped");
}

void WebUi::handleClear() {
  if (run_control.active()) { server_->send(409, "text/plain", "run_in_progress"); return; }
  if (runner_->running() || logger_->downloading()) {
    server_->send(409, "text/plain", "busy");
    return;
  }
  runner_->clearFinishedOrEstop();
  server_->send(200, "text/plain", "cleared");
}

void WebUi::handleSettings() {
  if (run_control.active()) { server_->send(409, "text/plain", "run_in_progress"); return; }
  if (runner_->running()) {
    server_->send(409, "text/plain", "running");
    return;
  }
  const int16_t current_mA = server_->hasArg("current_mA") ? static_cast<int16_t>(server_->arg("current_mA").toInt())
                                                           : Config::DEFAULT_INPUT_CURRENT_MA;
  const uint16_t pulse_width_ms =
      server_->hasArg("pulse_width_ms") ? static_cast<uint16_t>(server_->arg("pulse_width_ms").toInt())
                                        : Config::DEFAULT_PULSE_WIDTH_MS;
  const uint16_t input_interval_ms =
      server_->hasArg("input_interval_ms") ? static_cast<uint16_t>(server_->arg("input_interval_ms").toInt())
                                           : Config::DEFAULT_INPUT_INTERVAL_MS;
  runner_->setInputSettings(current_mA, pulse_width_ms, input_interval_ms);
  server_->send(200, "text/plain", "settings_applied");
}

void WebUi::handleRwLog() {
  if (run_control.active()) { server_->send(409, "text/plain", "run_in_progress"); return; }
  if (runner_->running()) {
    server_->send(409, "text/plain", "measurement_running");
    return;
  }
  logger_->streamRwLog(*server_);
}

void WebUi::appendJsonUint64(String& json, uint64_t value) {
  char buf[24];
  snprintf(buf, sizeof(buf), "%llu", static_cast<unsigned long long>(value));
  json += buf;
}

String WebUi::statusJson() const {
  const auto& st = runner_->status();
  const RollerTelemetry roller = roller_->telemetrySnapshot();
  char filename[72];
  logger_->downloadFilename(filename, sizeof(filename));
  String json;
  json.reserve(3200);
  json += "{";
  json += "\"running\":" + String(runner_->running() ? "true" : "false");
  json += ",\"downloading\":" + String(logger_->downloading() ? "true" : "false");
  json += ",\"zero_cross_mode\":" + String(runner_->zeroCrossMode() ? "true" : "false");
  json += ",\"identification_mode\":" + String(runner_->identificationMode() ? "true" : "false");
  json += ",\"q_run_mode\":\"" + String(runner_->qRunModeName()) + "\"";
  json += ",\"passive_capture_mode\":" + String(runner_->passiveCaptureMode() ? "true" : "false");
  json += ",\"q_ident_mode\":" + String(runner_->qIdentMode() ? "true" : "false");
  json += ",\"energy_control_v0_mode\":" + String(runner_->energyControlV0Mode() ? "true" : "false");
  json += ",\"energy_control_autonomous_mode\":" + String(runner_->energyControlAutonomousMode() ? "true" : "false");
  json += ",\"energy_control_autonomous_target_peak_deg\":" + String(runner_->energyControlAutonomousTargetPeakDeg(), 2);
  json += ",\"autonomous_timing_compensation_ms\":" + String(runner_->energyControlAutonomousTimingCompensationUs() / 1000);
  json += ",\"autonomous_run_timing_compensation_ms\":" + String(runner_->energyControlAutonomousRunTimingCompensationUs() / 1000);
  json += ",\"energy_control_autonomous_phase\":\"" + String(runner_->energyControlAutonomousPhaseName()) + "\"";
  json += ",\"energy_control_v0_target_peak_deg\":" + String(Config::ENERGY_CONTROL_V0_TARGET_PEAK_DEG, 2);
  json += ",\"q_ident_armed\":" + String(runner_->qIdentArmed() ? "true" : "false");
  json += ",\"q_ident_run_schedule_id\":" + String(runner_->qIdentRunScheduleId());
  json += ",\"q_ident_plus_occurrences\":" + String(runner_->qIdentPlusOccurrenceCount());
  json += ",\"q_ident_minus_occurrences\":" + String(runner_->qIdentMinusOccurrenceCount());
  json += ",\"passive_static_window_s\":" + String(static_cast<float>(Config::PASSIVE_STATIC_WINDOW_MS) / 1000.0f, 1);
  json += ",\"q_probe_schedule_id\":" + String(runner_->qProbeScheduleId());
  json += ",\"q_probe_schedule_name\":\"" + String(runner_->qProbeScheduleName()) + "\"";
  json += ",\"q_control_target_peak_deg\":" + String(runner_->controlTargetPeakDeg(), 2);
  json += ",\"zero_cross_fixed_current_mA\":" + String(runner_->zeroCrossFixedCurrentMa());
  json += ",\"state\":\"" + String(runner_->stateName()) + "\"";
  json += ",\"state_id\":" + String(static_cast<uint8_t>(st.state));
  json += ",\"boot_elapsed_s\":" + String(static_cast<float>(st.boot_elapsed_ms) / 1000.0f, 1);
  json += ",\"measure_elapsed_s\":" + String(static_cast<float>(st.measure_elapsed_ms) / 1000.0f, 1);
  json += ",\"remaining_s\":" + String(static_cast<float>(st.remaining_ms) / 1000.0f, 1);
  json += ",\"trial_index\":" + String(st.trial_index);
  json += ",\"trial_count\":" + String(st.trial_count);
  json += ",\"trial_elapsed_s\":" + String(static_cast<float>(st.trial_elapsed_ms) / 1000.0f, 1);
  json += ",\"trial_duration_s\":" + String(static_cast<float>(st.trial_duration_ms) / 1000.0f, 1);
  json += ",\"pulse_id\":" + String(st.pulse_id);
  json += ",\"pulse_active\":" + String(st.pulse_active ? "true" : "false");
  json += ",\"pulse_direction\":" + String(st.pulse_direction);
  json += ",\"current_mA_setting\":" + String(st.current_mA_setting);
  json += ",\"pulse_width_ms_setting\":" + String(st.pulse_width_ms_setting);
  json += ",\"input_interval_ms\":" + String(st.input_interval_ms);
  json += ",\"predicted_beta_min\":" + String(st.predicted_beta_min, 5);
  json += ",\"beta_hold_after_input_ms\":" + String(st.beta_hold_after_input_ms_setting);
  json += ",\"beta_recovery_tau_s_setting\":" + String(st.beta_recovery_tau_s_setting, 3);
  json += ",\"beta_model_vbat_mV\":" + String(st.beta_model_vbat_mV);
  json += ",\"predicted_i_goal_mA\":" + String(st.predicted_i_goal_mA);
  json += ",\"predicted_peak_current_mA\":" + String(st.predicted_peak_current_mA);
  json += ",\"beta_model_vbat_status\":" + String(st.beta_model_vbat_status);
  json += ",\"led_state\":" + String(st.led_state ? "true" : "false");
  json += ",\"sync_event_id\":" + String(st.sync_event_id);
  json += ",\"led_sync_pattern_id\":\"" + String(Config::LED_SYNC_PATTERN_ID) + "\"";
  json += ",\"gyro_bias_x_dps\":" + String(st.gyro_bias_x_dps, 5);
  json += ",\"gyro_bias_y_dps\":" + String(st.gyro_bias_y_dps, 5);
  json += ",\"gyro_bias_z_dps\":" + String(st.gyro_bias_z_dps, 5);
  json += ",\"attitude_filter_adopted\":\"MEKF\"";
  json += ",\"pitch_mekf_control_deg\":" + String(st.pitch_mekf_deg, 3);
  json += ",\"pitch_mekf_abs_deg\":" + String(st.pitch_mekf_abs_deg, 3);
  json += ",\"pitch_mekf_predicted_abs_deg\":" + String(st.pitch_mekf_predicted_abs_deg, 3);
  json += ",\"pitch_mekf_detector_relative_deg\":" + String(st.pitch_mekf_detector_relative_deg, 3);
  json += ",\"mekf_detector_zero_predicted_abs_deg\":" + String(st.mekf_detector_zero_predicted_abs_deg, 3);
  json += ",\"mekf_detector_zero_sample_us\":" + String(st.mekf_detector_zero_sample_us);
  json += ",\"mekf_prediction_horizon_us\":" + String(st.mekf_prediction_horizon_us);
  json += ",\"pitch_madgwick_dynamic_abs_deg\":" + String(st.pitch_madgwick_dynamic_abs_deg, 3);
  json += ",\"mekf_accel_confidence\":" + String(st.mekf_accel_confidence, 4);
  json += ",\"mekf_accel_residual_deg\":" + String(st.mekf_accel_residual_deg, 3);
  json += ",\"mekf_accel_used\":" + String(st.mekf_accel_used ? "true" : "false");
  json += ",\"pitch_madgwick_beta1_raw_deg\":" + String(st.pitch_madgwick_beta1_raw_deg, 3);
  json += ",\"pitch_madgwick_dynamic_raw_deg\":" + String(st.pitch_madgwick_dynamic_raw_deg, 3);
  json += ",\"pitch_madgwick_beta1_bias_deg\":" + String(st.pitch_madgwick_beta1_bias_deg, 3);
  json += ",\"pitch_madgwick_dynamic_bias_deg\":" + String(st.pitch_madgwick_dynamic_bias_deg, 3);
  for (uint8_t i = 0; i < Config::DYNAMIC_BETA_COUNT; ++i) {
    json += ",\"pitch_beta_series_" + String(i) + "\":" + String(st.pitch_dynamic_beta_deg[i], 3);
    json += ",\"beta_applied_series_" + String(i) + "\":" + String(st.beta_smooth_series[i], 5);
  }
  json += ",\"pitch_gyro_raw_deg\":" + String(st.pitch_gyro_raw_deg, 3);
  json += ",\"pitch_gyro_bias_corrected_deg\":" + String(st.pitch_gyro_bias_corrected_deg, 3);
  json += ",\"pitch_accel_only_deg\":" + String(st.pitch_accel_only_deg, 3);
  json += ",\"gyro_pitch_rate_dps\":" + String(st.gyro_pitch_rate_dps, 4);
  json += ",\"beta_target\":" + String(st.beta_target, 5);
  json += ",\"beta_smooth\":" + String(st.beta_smooth, 5);
  json += ",\"ax_g\":" + String(st.ax_g, 4);
  json += ",\"ay_g\":" + String(st.ay_g, 4);
  json += ",\"az_g\":" + String(st.az_g, 4);
  json += ",\"gx_dps\":" + String(st.gx_dps, 4);
  json += ",\"gy_dps\":" + String(st.gy_dps, 4);
  json += ",\"gz_dps\":" + String(st.gz_dps, 4);
  json += ",\"acc_norm_g\":" + String(st.acc_norm_g, 4);
  json += ",\"physical_roll_candidate_deg\":" + String(st.physical_roll_candidate_deg, 3);
  json += ",\"physical_roll_abs_deg\":" + String(st.physical_roll_abs_deg, 3);
  json += ",\"current_roll_deg\":" + String(st.current_roll_deg, 3);
  json += ",\"physical_roll_rate_raw_dps\":" + String(st.physical_roll_rate_raw_dps, 4);
  json += ",\"physical_roll_rate_dps\":" + String(st.physical_roll_rate_dps, 4);
  json += ",\"display_zero_offset_deg\":" + String(st.display_zero_offset_deg, 3);
  json += ",\"target_roll_deg\":" + String(st.target_roll_deg, 3);
  json += ",\"q1_shadow_target_peak_abs_deg\":" + String(runner_->q1ShadowTargetPeakAbsDeg(), 3);
  json += ",\"q1_shadow_active_target_peak_abs_deg\":" + String(runner_->q1ShadowActiveTargetPeakAbsDeg(), 3);  json += ",\"target_error_deg\":" + String(st.target_error_deg, 3);
  json += ",\"static_confirmed\":" + String(st.static_confirmed ? "true" : "false");
  json += ",\"ready\":" + String(st.ready ? "true" : "false");
  json += ",\"static_rate_threshold_dps\":" + String(Config::STATIC_RATE_THRESHOLD_DPS, 3);
  json += ",\"static_hold_time_ms\":" + String(Config::STATIC_HOLD_TIME_MS);
  json += ",\"target_tolerance_deg\":" + String(Config::TARGET_TOLERANCE_DEG, 3);
  json += ",\"motor_cmd_mA\":" + String(st.motor_cmd_mA);
  json += ",\"sample_count\":" + String(logger_->sampleCount());
  json += ",\"psram_usage_percent\":" + String(logger_->usagePercent());
  json += ",\"log_capacity\":" + String(logger_->sampleCapacity());
  json += ",\"rwlog_downloadable\":\"" + String(logger_->rwlogDownloadable() ? "yes" : "no") + "\"";
  json += ",\"download_filename\":\"" + String(filename) + "\"";
  json += ",\"run_id\":" + String(logger_->currentRunId());
  json += ",\"run_start_us\":";
  appendJsonUint64(json, logger_->runStartUs());
  json += ",\"last_measurement_done\":\"" + String(logger_->lastMeasurementDone() ? "yes" : "no") + "\"";
  json += ",\"calibration_sample_count\":" + String(st.calibration_sample_count);
  json += ",\"startup\":" + imu_->startupDiagnosticsJson();
  json += ",\"imu_ok\":" + String(imu_->ok() ? "true" : "false");
  json += ",\"roller_ok\":" + String(roller_->ok() ? "true" : "false");
  json += ",\"roller_actual_current_mA\":" + String(roller.actual_current_mA);
  json += ",\"roller_io_task_running\":" + String(roller.io_task_running ? "true" : "false");
  json += ",\"roller_io_task_ready\":" + String(roller.io_task_ready ? "true" : "false");
  json += ",\"roller_io_task_init_failed\":" + String(roller.io_task_init_failed ? "true" : "false");
  json += ",\"roller_io_init_attempt_count\":" + String(roller.io_init_attempt_count);
  json += ",\"roller_io_recovery_count\":" + String(roller.io_recovery_count);
  json += ",\"roller_command_latency_us\":" + String(roller.last_command_latency_us);
  json += ",\"roller_command_latency_max_us\":" + String(roller.max_command_latency_us);
  json += ",\"battery_mV\":" + String(roller.battery_mV);
  json += ",\"loop_dt_us\":" + String(st.loop_dt_us);
  json += ",\"log_dt_us\":" + String(st.log_dt_us);
  json += ",\"imu_dt_us\":" + String(imu_->reading().update_dt_us);
  json += ",\"last_error\":\"" + String(st.last_error && st.last_error[0] ? st.last_error : logger_->lastError()) + "\"";
  json += "}";

  // Arduino String renders non-finite floats as `nan`/`inf`, which is not valid
  // JSON. V46 intentionally uses NaN for comparison series that are disabled
  // during the autonomous run, so normalize those tokens before sending the
  // status document. This lets the browser keep polling while its visible
  // display remains frozen by design, then resume immediately at FINISHED.
  json.replace(":-Infinity", ":null");
  json.replace(":Infinity", ":null");
  json.replace(":-inf", ":null");
  json.replace(":inf", ":null");
  json.replace(":NaN", ":null");
  json.replace(":nan", ":null");
  return json;
}
