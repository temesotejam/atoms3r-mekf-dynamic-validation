#!/usr/bin/env python3
"""Exercise firmware selection, HTTP guards, projection and saved metadata on host."""
from pathlib import Path
import json
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
runner = (ROOT / 'src/experiment_runner.cpp').read_text()
web = (ROOT / 'src/web_ui.cpp').read_text()
logger = (ROOT / 'src/psram_logger.cpp').read_text()

def method(text, signature):
    start = text.index(signature)
    brace = text.index('{', start)
    depth, end = 1, brace + 1
    while depth:
        depth += (text[end] == '{') - (text[end] == '}')
        end += 1
    return text[start:end]

start = method(runner, 'bool ExperimentRunner::startEnergyControlAutonomousCapture()')
assert start.count('autonomous_timing_.captureRun();') == 1
assert start.index('upright_pose_required_before_start') < start.index('autonomous_timing_.captureRun();') < start.index('beginStartSync(')
assert runner.count('autonomous_timing_.captureRun();') == 1
sync = method(runner, 'void ExperimentRunner::beginStartSync(')
assert 'energy_control_autonomous_mode_ ? autonomous_timing_.runUs() : 0' in sync
assert 'autonomous_timing_.selectedUs()' not in runner
assert 'autonomous_timing_compensation_us_ = 0;' in method(logger, 'void PsramLogger::clear(')
assert 'autonomous_timing_.selectedUs()' not in logger
# Only the completed/saved run's snapshot may feed its metadata.
logger_start = method(logger, 'void PsramLogger::startRun(')
assignment = re.search(r'  autonomous_timing_compensation_us_ = .*?;', logger_start).group()
assert 'if (downloading_) return;' in logger_start
assert logger_start.index('if (downloading_) return;') < logger_start.index(assignment)
metadata = '\n'.join(line for line in logger.splitlines() if any('\\"'+key+'\\"' in line for key in ('autonomous_timing_compensation_us','autonomous_control_prediction_enabled','autonomous_timing_prediction_formula')))
assert metadata.count('json +=') == 3
assert 'Config::ENERGY_CONTROL_AUTONOMOUS_TIMING_COMPENSATION_US' not in metadata
projection = runner.split('  // V46ac delay compensation begin\n', 1)[1].split('  // V46ac delay compensation end', 1)[0]
state_enum = re.search(r'enum class ExperimentState.*?\n};', (ROOT/'src/log_types.h').read_text(), re.S).group()
source = r'''
#include "autonomous_timing_compensation.h"
#include <cassert>
#include <cmath>
#include <iostream>
#include <map>
#include <string>
using std::isfinite;
struct String : std::string {
 using std::string::string;
 explicit String(uint32_t v):std::string(std::to_string(v)){}
};
namespace Config { constexpr float MEKF_GYRO_Y_SCALE = 1.018f; }
STATE_ENUM
struct Logger {
 bool downloading_ = false;
 bool energy_control_autonomous_mode_ = true;
 uint32_t autonomous_timing_compensation_us_ = 0;
 bool downloading() const {return downloading_;}
 void snapshot(bool energy_control_autonomous_mode, uint32_t autonomous_timing_compensation_us) {
   energy_control_autonomous_mode_ = energy_control_autonomous_mode;
   SNAPSHOT
 }
 std::string metadata() const { std::string json="{";
   METADATA
   json.back()='}';return json;
 }
};
struct ImuReading {float gy_dps=100;};
struct Imu {int reads=0;void update(){++reads;}};
struct RunControl {bool active_=false,ready_=true;bool active()const{return active_;}bool ready()const{return ready_;}} run_control;
struct ExperimentRunner {
 struct Status {
   ExperimentState state=ExperimentState::READY_TO_MEASURE;
   const char* last_error="";
   float pitch_mekf_measurement_relative_deg=1.5f,mekf_bias_y_dps=2;
   float pitch_mekf_detector_relative_deg=0,pitch_mekf_deg=0;
 } status_;
 Logger* logger_;
 autonomous_timing::Selection autonomous_timing_{3000};
 bool energy_control_autonomous_mode_=true;
 int starts=0;
 bool running()const{return status_.state==ExperimentState::START_SYNC||status_.state==ExperimentState::RUNNING_BATCH_SWEEP||status_.state==ExperimentState::TRIAL_REST||status_.state==ExperimentState::END_SYNC;}
 const Status& status()const{return status_;}
 bool setEnergyControlAutonomousTimingCompensation(uint32_t value_us);
 uint32_t energyControlAutonomousTimingCompensationUs()const{return autonomous_timing_.selectedUs();}
 bool startEnergyControlAutonomousCapture(){++starts;return true;}
 void project(const ImuReading& r){ PROJECTION }
};
SETTER
struct Server {
 std::map<std::string,std::string> args;
 int response=0;
 bool hasArg(const char* k){return args.count(k)!=0;}
 std::string arg(const char* k){return args.at(k);}
 void send(int code,const char*,const std::string&){response=code;}
};
struct WebUi {
 Server* server_; ExperimentRunner* runner_; Logger* logger_; Imu* imu_;
 void handleSetEnergyControlAutonomousTimingCompensation();
 void handleStartEnergyControlAutonomous();
};
HTTP_SETTER
HTTP_START
int main(){
 Logger log; ExperimentRunner r; r.logger_=&log;Server s;Imu imu;WebUi ui{&s,&r,&log,&imu};
 assert(r.autonomous_timing_.selectedUs()==3000 && r.autonomous_timing_.runUs()==3000);
 for(int state=0;state<=8;++state){
   r.status_.state=static_cast<ExperimentState>(state);
   for(uint32_t us:{0u,3000u,6000u,9000u}){
     const uint32_t previous=r.autonomous_timing_.selectedUs();
     const bool allowed=state==2||state==4;
     assert(r.setEnergyControlAutonomousTimingCompensation(us)==allowed);
     assert(r.autonomous_timing_.selectedUs()==(allowed?us:previous));
   }
 }
 r.status_.state=ExperimentState::FINISHED;
 log.downloading_=true;assert(!r.setEnergyControlAutonomousTimingCompensation(0));log.downloading_=false;
 for(uint32_t invalid:{1u,3u,2999u,3001u,12000u,0xffffffffu})assert(!r.setEnergyControlAutonomousTimingCompensation(invalid));
 uint32_t parsed=12345;assert(!autonomous_timing::parseMs(nullptr,parsed)&&parsed==12345);
 for(const char* invalid:{"","3.0","-3","12","03","NaN"," 3","3 ","3junk","3000"}){
   s.args={{"ms",invalid}};ui.handleSetEnergyControlAutonomousTimingCompensation();assert(s.response==400);
 }
 s.args.clear();ui.handleSetEnergyControlAutonomousTimingCompensation();assert(s.response==400);
 // Ownership guard must run before any runner/logger access.
 run_control.active_=true;WebUi detached{&s,nullptr,nullptr,nullptr};
 detached.handleSetEnergyControlAutonomousTimingCompensation();assert(s.response==409);
 detached.handleStartEnergyControlAutonomous();assert(s.response==409);run_control.active_=false;
 for(uint32_t ms:{3u,6u,9u,0u,3u}){
   s.args={{"ms",std::to_string(ms)}};ui.handleSetEnergyControlAutonomousTimingCompensation();assert(s.response==200);
   r.autonomous_timing_.captureRun();log.snapshot(true,r.autonomous_timing_.runUs());
   r.project(ImuReading{});
   assert(std::abs(r.status_.pitch_mekf_detector_relative_deg-(1.5f+98*Config::MEKF_GYRO_Y_SCALE*ms*0.001f))<1e-6f);
   assert(r.status_.pitch_mekf_measurement_relative_deg==1.5f);
   const std::string saved=log.metadata();
   assert(r.setEnergyControlAutonomousTimingCompensation(ms==9?0:9000));
   assert(r.autonomous_timing_.runUs()==ms*1000 && log.metadata()==saved);
   std::cout<<saved<<'\n';
 }
 assert(r.setEnergyControlAutonomousTimingCompensation(0));r.autonomous_timing_.captureRun();
 r.project(ImuReading{NAN});assert(r.status_.pitch_mekf_detector_relative_deg==1.5f);
 assert(r.setEnergyControlAutonomousTimingCompensation(3000));r.autonomous_timing_.captureRun();
 r.project(ImuReading{NAN});assert(std::isnan(r.status_.pitch_mekf_detector_relative_deg));
 r.energy_control_autonomous_mode_=false;r.project(ImuReading{});assert(r.status_.pitch_mekf_detector_relative_deg==1.5f);
 s.args={{"timing_ms","6"}};ui.handleStartEnergyControlAutonomous();assert(s.response==409&&r.starts==0&&imu.reads==0);
 s.args={{"timing_ms","garbage"}};ui.handleStartEnergyControlAutonomous();assert(s.response==400&&r.starts==0);
 s.args={{"timing_ms","3"}};ui.handleStartEnergyControlAutonomous();assert(s.response==200&&r.starts==1&&imu.reads==1);
 log.snapshot(false,9000);std::cout<<log.metadata()<<'\n';
}
'''
for key, value in {
 'STATE_ENUM':state_enum,'SNAPSHOT':assignment,'METADATA':metadata,'PROJECTION':projection,
 'SETTER':method(runner,'bool ExperimentRunner::setEnergyControlAutonomousTimingCompensation('),
 'HTTP_SETTER':method(web,'void WebUi::handleSetEnergyControlAutonomousTimingCompensation('),
 'HTTP_START':method(web,'void WebUi::handleStartEnergyControlAutonomous('),
}.items():
    source=re.sub(r"\b"+key+r"\b", lambda _: value, source)
with tempfile.TemporaryDirectory() as d:
    cpp=Path(d)/'test.cpp';exe=Path(d)/'test';cpp.write_text(source)
    subprocess.run(['g++','-std=c++17','-O2','-Wall','-Wextra','-Werror','-I'+str(ROOT/'src'),str(cpp),'-o',str(exe)],check=True)
    rows=[json.loads(line) for line in subprocess.check_output([str(exe)],text=True).splitlines()]
    for row,ms in zip(rows,[3,6,9,0,3,0]):
        assert row['autonomous_timing_compensation_us']==ms*1000,row
        assert row['autonomous_control_prediction_enabled']==(ms>0),row
        assert 'autonomous_timing_compensation_us*1e-6' in row['autonomous_timing_prediction_formula']
subprocess.run(['node',str(ROOT/'tools/test_v46ad_timing_ui.js')],check=True)
print('V46ad PASS: 0/3/6/9 ms; stopped-state guards; run/metadata immutability; zero bypass; HTTP ownership/start handshake; UI races')
