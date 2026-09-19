// Run the firmware's actual embedded script with a delayed network and minimal DOM.
const fs=require('fs'),vm=require('vm'),assert=require('assert'),path=require('path');
const source=fs.readFileSync(path.join(__dirname,'../src/web_ui.cpp'),'utf8').match(/<script>([\s\S]*?)<\/script>/)[1];
const elements=new Map(),requests=[];
const get=id=>{if(!elements.has(id))elements.set(id,{tagName:'BUTTON',disabled:false,value:'3',options:[],classList:{toggle(){}}});return elements.get(id);};
const context=vm.createContext({document:{getElementById:get,activeElement:null},fetch:(url,options)=>new Promise(resolve=>requests.push({url,options,resolve})),setTimeout:()=>1,clearTimeout(){},setInterval(){},AbortController,alert(){},confirm:()=>true,console});
const run=code=>vm.runInContext(code,context);
const tick=async()=>{for(let i=0;i<10;i++)await Promise.resolve();};
const ready=ms=>({state:'FINISHED',running:false,autonomous_timing_compensation_ms:ms,autonomous_run_timing_compensation_ms:3});
async function reply(req,data,ok=true){req.resolve({ok,json:async()=>data,text:async()=>String(data)});await tick();}
(async()=>{
 vm.runInContext(source,context);
 await reply(requests.shift(),ready(3));assert(!get('energy').disabled);assert.strictEqual(get('timingCompensation').value,'3');
 // A stale status reply overlaps selection. It must not restore the old value or unlock Start.
 run('refresh()');const stale=requests.shift();
 get('timingCompensation').value='6';run('setTimingCompensation()');const setting=requests.shift();
 assert(setting.url.endsWith('ms=6'));assert(get('energy').disabled);assert(get('timingCompensation').disabled);
 run('startEnergy()');assert.strictEqual(requests.length,0);
 await reply(stale,ready(3));assert(get('energy').disabled);
 await reply(setting,'ok');await reply(requests.shift(),ready(6));assert(!get('energy').disabled);assert.strictEqual(get('timingCompensation').value,'6');
 run('startEnergy()');const start=requests.shift();assert(start.url.endsWith('timing_ms=6'));assert(get('timingCompensation').disabled);
 run('startEnergy()');assert.strictEqual(requests.length,0);
 await reply(start,'ok');assert(get('energy').disabled);
 await reply(requests.shift(),{running:true,state:'START_SYNC'});assert(get('timingCompensation').disabled);
 run('refresh()');await reply(requests.shift(),ready(6));
 // 0 is a valid value, not an unset/falsy setting. Failed writes require status confirmation.
 get('timingCompensation').value='0';run('setTimingCompensation()');await reply(requests.shift(),'busy',false);
 assert(get('energy').disabled);await reply(requests.shift(),ready(6));assert.strictEqual(get('timingCompensation').value,'6');
 get('timingCompensation').value='0';run('setTimingCompensation()');await reply(requests.shift(),'ok');await reply(requests.shift(),ready(0));
 run('startEnergy()');assert(requests.shift().url.endsWith('timing_ms=0'));
 console.log('V46ad UI PASS: delayed setting/stale status/start locking, rejected writes, 0 ms');
})().catch(error=>{console.error(error);process.exitCode=1;});
