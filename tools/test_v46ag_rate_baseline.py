#!/usr/bin/env python3
"""Exercise the V46ah direct baseline, retained gates and V46ah event layout."""
from pathlib import Path
import subprocess
import tempfile
import csv
import json
import re
from convert_rwlog_to_csv import write_energy_control_autonomous_events
ROOT=Path(__file__).resolve().parents[1]
runner=(ROOT/'src/experiment_runner.cpp').read_text()
logger=(ROOT/'src/psram_logger.cpp').read_text()
converter=(ROOT/'tools/convert_rwlog_to_csv.py').read_text()
start=runner.index('const auto baseline = rate_baseline::evaluate')
assign=runner.index('event.free_next_peak_amplitude_deg = baseline.adjusted_deg;',start)
energy=runner.index('event.passive_energy_j = energyControlPotentialJ',assign)
assert start<assign<energy
assert 'autonomous_timing_.runUs()' in runner[start:assign]
for key in ['p1_free_peak_before_rate_deg','rate_baseline_peak_deg','rate_baseline_correction_deg','rate_baseline_reason']:
    assert key in runner and key in logger and key in converter,key
assert 'detail += ",\\"rate_baseline_reason\\":"' in logger
assert 'RWLOG_FORMAT_VERSION = 51' in logger
assert 'v46ah_rate_baseline_100pct_no_cap_20260919' in logger
assert 'rate_baseline_delta_cap_enabled' in logger
metadata_parts=[]
for line in logger.splitlines():
    match=re.fullmatch(r'\s*json \+= ("(?:[^"\\]|\\.)*");',line)
    if match:
        value=json.loads(match.group(1))
        if value.startswith('"rate_baseline_'):metadata_parts.append(value)
metadata=json.loads('{'+''.join(metadata_parts).rstrip(',')+'}')
assert metadata['rate_baseline_revision']=='v46ah_rate_baseline_100pct_no_cap_20260919'
assert metadata['rate_baseline_blend']==1.0
assert metadata['rate_baseline_delta_cap_enabled'] is False
assert metadata['rate_baseline_delta_cap_deg'] is None
with tempfile.TemporaryDirectory() as d:
    event=dict(p1_free_peak_before_rate_deg=7.5, rate_baseline_peak_deg=8.3,
               rate_baseline_correction_deg=0.8, rate_baseline_reason=0)
    assert write_energy_control_autonomous_events(
        {'energy_control_autonomous_zero_cross_events':[event,{}]},Path(d))==(0,2)
    with (Path(d)/'energy_control_autonomous_zero_cross_events.csv').open() as f:
        rows=list(csv.DictReader(f))
    for key,value in event.items():
        assert float(rows[0][key])==value
        assert rows[1][key]==''
code=r'''
#include <cassert>
#include <cmath>
#include <iostream>
#include "rate_baseline_correction.h"
using namespace rate_baseline;
void near(float a,float b){assert(std::fabs(a-b)<1e-5f);}
int main(){
  unsigned cases=0;
  for(int side:{-1,1}) {
    const float prev=side>0?9.f:8.f;
    auto a=evaluate(8.f,65.f,side,prev,8.f,10000,3000);
    assert(a.reason==APPLIED);near(a.adjusted_deg,a.p1_deg+a.correction_deg);
    assert(a.adjusted_deg==a.rate_deg);
    near(a.correction_deg,a.rate_deg-a.p1_deg);++cases;
    for(unsigned ms:{0u,9999u}){auto r=evaluate(8,65,side,prev,8,ms,3000);
      assert(r.reason==WARMUP);near(r.adjusted_deg,8);near(r.correction_deg,0);++cases;}
    for(unsigned delay:{0u,6000u,9000u}){auto r=evaluate(8,65,side,prev,8,20000,delay);
      assert(r.reason==OTHER_SETTINGS);near(r.adjusted_deg,8);++cases;}
    for(float target:{10.f,12.f}){auto r=evaluate(8,65,side,prev,target,20000,3000);
      assert(r.reason==OTHER_SETTINGS);near(r.adjusted_deg,8);++cases;}
    for(float rate:{0.f,100.f}){auto r=evaluate(8,rate,side,prev,8,20000,3000);
      assert(r.reason==OUTSIDE_STATE);near(r.adjusted_deg,8);++cases;}
    for(float peak:{0.f,12.f}){auto r=evaluate(8,65,side,peak,8,20000,3000);
      assert(r.reason==OUTSIDE_STATE);near(r.adjusted_deg,8);++cases;}
    for(float p1:{0.f,20.f}){auto r=evaluate(p1,65,side,prev,8,20000,3000);
      assert(r.reason==APPLIED);assert(std::fabs(r.correction_deg)>.5f);
      assert(r.adjusted_deg==r.rate_deg);assert(r.adjusted_deg>=0);++cases;}
    const float peak_lo=side>0?8.3f:6.7f,peak_hi=side>0?9.8f:9.f;
    const float rate_lo=side>0?61.5f:59.f,rate_hi=side>0?72.f:70.f;
    for(float peak:{peak_lo,peak_hi})for(float rate:{rate_lo,rate_hi}){
      auto r=evaluate(8,rate,side,peak,8,10000,3000);
      assert(r.reason==APPLIED);assert(r.adjusted_deg==r.rate_deg);++cases;}
    for(float peak:{std::nextafter(peak_lo,-INFINITY),std::nextafter(peak_hi,INFINITY)}){
      auto r=evaluate(8,65,side,peak,8,10000,3000);
      assert(r.reason==OUTSIDE_STATE);near(r.adjusted_deg,8);++cases;}
    for(float rate:{std::nextafter(rate_lo,-INFINITY),std::nextafter(rate_hi,INFINITY)}){
      auto r=evaluate(8,rate,side,prev,8,10000,3000);
      assert(r.reason==OUTSIDE_STATE);near(r.adjusted_deg,8);++cases;}
    for(float rate:{NAN,INFINITY,-1.f}){auto r=evaluate(8,rate,side,prev,8,20000,3000);
      assert(r.reason==NONFINITE);near(r.adjusted_deg,8);++cases;}
  }
  auto r=evaluate(NAN,65,1,9,8,20000,3000);
  assert(r.reason==NONFINITE && std::isnan(r.adjusted_deg)); // existing fail-closed path sees NaN
  assert(evaluate(8,65,0,9,8,20000,3000).reason==NONFINITE);
  for(int side:{-1,1})for(float p1=0;p1<=15;p1+=.25f)for(float rate=59;rate<=72;rate+=.25f){
    auto x=evaluate(p1,rate,side,side>0?9:8,8,20000,3000);
    if(x.reason==APPLIED){
      assert(x.adjusted_deg==x.rate_deg);
      near(x.correction_deg,x.rate_deg-p1);
    }else{assert(x.reason==OUTSIDE_STATE);near(x.adjusted_deg,p1);near(x.correction_deg,0);}
    assert(x.adjusted_deg>=0);++cases;
  }
  // Frozen recorded cases from the two independent V46ah measurements.
  for(auto x:{evaluate(7.85594f,67.1845f,-1,8.58057f,8,11044,3000),
              evaluate(8.55661f,68.9249f,1,9.31899f,8,10605,3000)}){
    assert(x.reason==APPLIED);assert(x.adjusted_deg==x.rate_deg);++cases;
  }
  near(evaluate(7.85594f,67.1845f,-1,8.58057f,8,11044,3000).adjusted_deg,8.685486f);
  std::cout<<"V46ah production baseline correction: "<<cases
           <<" cases; both sides, inclusive state boundaries, 10s/3ms/8deg gates, fallback, finite handling, direct rate model with corrections beyond 0.5deg PASS\n";
}
'''
with tempfile.TemporaryDirectory() as d:
    p=Path(d);(p/'test.cpp').write_text(code)
    subprocess.run(['g++','-std=c++17','-O2','-Wall','-Wextra','-Werror','-I'+str(ROOT/'src'),str(p/'test.cpp'),'-o',str(p/'test')],check=True)
    subprocess.run([str(p/'test')],check=True)
