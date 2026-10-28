"use strict";
const $=id=>document.getElementById(id), num=id=>Number($(id).value), array=id=>JSON.parse($(id).value);
const fmt=x=>Number.isFinite(x)?Number(x).toPrecision(6):String(x);
const colors=['#127e91','#d55a2e','#5e7b45','#8865a7'];
function showError(error){$('error').hidden=false;$('error').textContent=error.message;}
async function api(path,data){
  const response=await fetch('/api/'+path,data===undefined?{cache:'no-store'}:{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(data),cache:'no-store'});
  const result=await response.json();
  if(!response.ok||result.error)throw Error((result.error||response.statusText)+(result.hint?'\n'+result.hint:''));
  return result;
}
function action(id,fn){$(id).onclick=async()=>{try{$('error').hidden=true;await fn();}catch(error){showError(error);}};}
document.querySelectorAll('.tab').forEach(button=>button.onclick=()=>{
  document.querySelectorAll('.tab,.page').forEach(e=>e.classList.remove('active'));
  button.classList.add('active');$(button.dataset.page).classList.add('active');$('error').hidden=true;
});
function table(id,headings,rows){
  const target=$(id),output=document.createElement('table'),head=document.createElement('tr');
  headings.forEach(text=>{const cell=document.createElement('th');cell.textContent=text;head.append(cell);});output.append(head);
  rows.forEach(values=>{const row=document.createElement('tr');values.forEach(value=>{const cell=document.createElement('td');cell.textContent=typeof value==='number'?fmt(value):value;row.append(cell);});output.append(row);});
  target.replaceChildren(output);
}
function armPlot(points,lengths){
  const canvas=$('armCanvas'),c=canvas.getContext('2d'),w=canvas.width,h=canvas.height;
  c.clearRect(0,0,w,h);const reach=Math.max(.01,lengths.reduce((a,b)=>a+b,0)),s=Math.min(w,h)*.43/reach,ox=w/2,oy=h/2;
  c.strokeStyle='#dce1d6';c.lineWidth=1;
  for(let i=-5;i<=5;i++){const d=i*reach/5*s;c.beginPath();c.moveTo(ox+d,0);c.lineTo(ox+d,h);c.stroke();c.beginPath();c.moveTo(0,oy+d);c.lineTo(w,oy+d);c.stroke();}
  c.strokeStyle='#a8b5ad';c.beginPath();c.moveTo(0,oy);c.lineTo(w,oy);c.moveTo(ox,0);c.lineTo(ox,h);c.stroke();
  c.fillStyle='#7e8a80';c.font='15px monospace';c.fillText('x →',w-55,oy-12);c.fillText('y ↑',ox+12,22);
  if(!points.length)return;
  const screen=points.map(([x,y])=>[ox+x*s,oy-y*s]);
  c.lineWidth=10;c.lineCap='round';c.strokeStyle=colors[0];c.beginPath();screen.forEach(([x,y],i)=>i?c.lineTo(x,y):c.moveTo(x,y));c.stroke();
  screen.forEach(([px,py],i)=>{c.fillStyle=i===0?'#20343c':'#fffefa';c.strokeStyle=colors[0];c.lineWidth=4;c.beginPath();c.arc(px,py,i===0?12:9,0,Math.PI*2);c.fill();c.stroke();});
}
let mode='stopped',viewGeneration=0,controlGeneration=0,stream=null,retryTimer=null,frameHandle=null,pendingFrame=null,lastTableAt=0;
let liveMethodDirty=false;
function selectMode(next){
  viewGeneration++;mode=next;
  if(stream){stream.close();stream=null;}
  clearTimeout(retryTimer);retryTimer=null;
  if(frameHandle!==null){cancelAnimationFrame(frameHandle);frameHandle=null;}
  pendingFrame=null;lastTableAt=0;
  $('startLive').disabled=next==='live';$('stopLive').disabled=next!=='live';
  return viewGeneration;
}
function queueFrame(data,generation,badge='LIVE'){
  if(generation!==viewGeneration)return;
  pendingFrame={data,generation,badge};
  if(frameHandle===null)frameHandle=requestAnimationFrame(drawFrame);
}
function drawFrame(now){
  frameHandle=null;const frame=pendingFrame;pendingFrame=null;
  if(!frame||frame.generation!==viewGeneration)return;
  const {data:r,badge}=frame;
  armPlot(r.points,r.parameters.lengths);$('armBadge').textContent=badge;
  $('endPosition').textContent=r.points.at(-1).map(fmt).join(', ');
  $('orientation').textContent=r.orientation===undefined?'—':fmt(r.orientation)+' rad';
  const stamp=r.state.header?.stamp;
  $('simulationTime').textContent=stamp?fmt(stamp.sec+stamp.nanosec/1e9)+' s':'—';
  if(r.settings&&typeof r.settings.method==='string'){
    const method=r.settings.method,selector=$('liveMethod');$('runtimeMethod').textContent=method.toUpperCase();
    if(!liveMethodDirty||selector.value===method){
      if(Array.from(selector.options).some(option=>option.value===method))selector.value=method;
      liveMethodDirty=false;
    }
  }
  if(now-lastTableAt>=120||!lastTableAt){
    table('armReadout',['Joint','Position','Velocity','Effort'],r.state.position.map((q,i)=>[r.state.name[i],q,r.state.velocity[i],r.state.effort[i]]));
    lastTableAt=now;
  }
}
function connectStream(generation){
  if(generation!==viewGeneration||mode!=='live')return;
  $('liveStatus').textContent='Connecting to the running arm…';$('armBadge').textContent='CONNECTING';
  const source=new EventSource('/api/stream');stream=source;
  function retry(){
    source.close();
    if(generation!==viewGeneration||mode!=='live'||stream!==source)return;
    stream=null;$('armBadge').textContent='WAITING';
    $('liveStatus').textContent='Waiting for make run. Reconnecting automatically…';
    retryTimer=setTimeout(()=>connectStream(generation),1500);
  }
  source.onmessage=event=>{
    if(generation!==viewGeneration||mode!=='live'||stream!==source)return;
    try{queueFrame(JSON.parse(event.data),generation);$('liveStatus').textContent='Live playback connected.';}catch(_error){retry();}
  };
  source.addEventListener('stream_error',retry);source.onerror=retry;
}
function startLive(){const generation=selectMode('live');connectStream(generation);}
function stopLive(){selectMode('stopped');$('armBadge').textContent='VIEW STOPPED';$('liveStatus').textContent='View stopped. The running simulation continues independently.';}
action('startLive',startLive);action('stopLive',stopLive);
$('links').onchange=()=>{$('lengths').value=$('links').value==='2'?'[1,1]':'[1,1,0.6]';$('phi').disabled=$('links').value==='2';};$('phi').disabled=true;
action('solve',async()=>{
  const lengths=array('lengths'),target=[num('tx'),num('ty')],phi=num('phi');
  if(lengths.length!==num('links'))throw Error('Link count and length array must agree.');
  const generation=selectMode('source');$('armBadge').textContent='SOURCE';$('liveStatus').textContent='Showing a computed IK pose.';
  const r=await api('ik',{lengths,target,phi});if(generation!==viewGeneration)return;
  armPlot(r.points,lengths);$('endPosition').textContent=r.position.map(fmt).join(', ');$('orientation').textContent=fmt(r.orientation)+' rad';$('simulationTime').textContent='—';
  table('armReadout',['Joint','Position / rad'],r.joints.map((q,i)=>['Joint '+(i+1),q]));
});
action('snapshot',async()=>{
  const generation=selectMode('snapshot');const r=await api('state');if(generation!==viewGeneration)return;
  queueFrame(r,generation,'SNAPSHOT');$('liveStatus').textContent='Single state displayed. Start live view to continue playback.';
});
async function sendLive(service,args){
  const result=await api('live',{service,args});
  if(result.result===false)throw Error(result.status||'Live service rejected the request.');
  return result;
}
async function controlSequence(commands){
  if(mode!=='live')startLive();
  const generation=viewGeneration,operation=++controlGeneration;
  const current=()=>operation===controlGeneration;
  for(const [service,args] of commands){if(!current())return;await sendLive(service,args);}
  if(current()&&generation===viewGeneration&&mode==='live')$('liveStatus').textContent='Command sent. Live playback continues.';
}
action('freeSwing',()=>controlSequence([
  ['/arm_sim/pause',{data:true}],['/arm_sim/reset',{}],['/pid_controller/enable',{data:false}],['/arm_sim/pause',{data:false}]
]));
$('liveMethod').onchange=()=>{liveMethodDirty=true;};
action('applyLiveMethod',()=>{const method=$('liveMethod').value;liveMethodDirty=true;return controlSequence([['/arm_sim/set_integrator',{method}]]);});
action('servo',()=>controlSequence([['/ik_action/send_goal',{x:num('tx'),y:num('ty'),phi:num('phi')}],['/pid_controller/enable',{data:true}],['/arm_sim/pause',{data:false}]]));
action('pause',()=>controlSequence([['/arm_sim/pause',{data:true}]]));
action('resume',()=>controlSequence([['/arm_sim/pause',{data:false}]]));
action('enable',()=>controlSequence([['/pid_controller/enable',{data:true}]]));
action('reset',()=>controlSequence([['/arm_sim/reset',{}]]));
let traceSeries=[],traceTime=0,traceDuration=0,tracePlaying=false,traceLastWall=null,traceFrame=null,traceGeneration=0,traceTableAt=0;
function sampleIndex(data,time){let i=0;while(i+1<data.times.length&&data.times[i+1]<=time)i++;return i;}
function tracePlot(){
  const series=traceSeries;if(!series.length)return;
  const canvas=$('traceCanvas'),c=canvas.getContext('2d'),w=canvas.width,h=canvas.height,p={l:80,r:30,t:40,b:65};c.clearRect(0,0,w,h);
  const all=series.flatMap(s=>s.data.positions),magnitude=Math.max(...all.map(Math.abs))||1,scaled=all.map(value=>value/magnitude);
  let lo=Math.min(...scaled),hi=Math.max(...scaled),pad=(hi-lo)*.08||.08;lo-=pad;hi+=pad;
  const duration=traceDuration>0?traceDuration:1,xx=t=>p.l+t/duration*(w-p.l-p.r),yy=y=>h-p.b-(y-lo)/(hi-lo)*(h-p.t-p.b);
  c.font='14px monospace';c.lineWidth=1;
  for(let i=0;i<=5;i++){const scaledValue=lo+(hi-lo)*i/5,value=scaledValue*magnitude,y=yy(scaledValue);c.strokeStyle='#dce1d6';c.beginPath();c.moveTo(p.l,y);c.lineTo(w-p.r,y);c.stroke();c.fillStyle='#697879';c.fillText(Math.abs(value)>1e6||Math.abs(value)<.001?fmt(value):value.toFixed(2),8,y+5);const t=duration*i/5,x=xx(t);c.fillText(duration<.001?fmt(t):t.toFixed(2),x-15,h-30);}
  c.fillText('t / seconds',w-145,h-8);c.fillText('position',12,20);
  series.forEach((s,index)=>{
    const end=sampleIndex(s.data,traceTime);c.strokeStyle=colors[index%colors.length];c.fillStyle=c.strokeStyle;c.lineWidth=3;c.beginPath();
    for(let i=0;i<=end;i++){const x=xx(s.data.times[i]),y=yy(s.data.positions[i]/magnitude);i?c.lineTo(x,y):c.moveTo(x,y);}c.stroke();
    c.beginPath();c.arc(xx(s.data.times[end]),yy(s.data.positions[end]/magnitude),5,0,Math.PI*2);c.fill();
  });
}
function traceReadout(now=performance.now(),force=false){
  if(!traceSeries.length)return;
  const data=traceSeries.at(-1).data,index=sampleIndex(data,traceTime);
  $('finalPosition').textContent=fmt(data.positions[index]);$('finalVelocity').textContent=fmt(data.velocities[index]);
  $('traceTime').textContent=fmt(traceTime)+' / '+fmt(traceDuration)+' s';$('traceSeek').value=String(traceTime);
  $('traceToggle').textContent=tracePlaying?'Pause playback':'Play playback';
  if(force||now-traceTableAt>=150){table('integrationReadout',['Method','Sample time','Position','Velocity'],traceSeries.map(s=>{const i=sampleIndex(s.data,traceTime);return [s.method,s.data.times[i],s.data.positions[i],s.data.velocities[i]];}));traceTableAt=now;}
}
function traceTick(now){
  traceFrame=null;if(!tracePlaying)return;
  if(traceLastWall!==null)traceTime+=(now-traceLastWall)/1000*num('traceSpeed');traceLastWall=now;
  if(traceTime>=traceDuration){if($('traceLoop').checked&&traceDuration>0)traceTime%=traceDuration;else{traceTime=traceDuration;tracePlaying=false;}}
  tracePlot();traceReadout(now);if(tracePlaying)traceFrame=requestAnimationFrame(traceTick);
}
function playTrace(){if(!traceSeries.length)return;if(traceTime>=traceDuration)traceTime=0;tracePlaying=true;traceLastWall=null;if(traceFrame===null)traceFrame=requestAnimationFrame(traceTick);traceReadout(performance.now(),true);}
function pauseTrace(){tracePlaying=false;traceLastWall=null;if(traceFrame!==null)cancelAnimationFrame(traceFrame);traceFrame=null;traceReadout(performance.now(),true);}
action('traceToggle',()=>tracePlaying?pauseTrace():playTrace());action('traceRestart',()=>{traceTime=0;playTrace();});
$('traceSeek').oninput=()=>{traceTime=num('traceSeek');traceLastWall=null;tracePlot();traceReadout(performance.now(),true);};
$('traceSpeed').onchange=()=>{traceLastWall=null;};document.addEventListener('visibilitychange',()=>{traceLastWall=null;});
$('forcing').onchange=()=>{if($('forcing').value!=='custom')$('expression').value=$('forcing').value;};
async function integrate(all){
  const request={function:$('expression').value,x0:num('x0'),xdot0:num('v0'),dt:num('dt'),steps:num('steps')};
  const methods=all?['euler','midpoint','verlet','rk4']:[$('method').value],series=[],generation=++traceGeneration;
  pauseTrace();$('integrate').disabled=true;$('runAll').disabled=true;
  try{
    for(const method of methods){const data=await api('integrate',{...request,integrator:method});if(generation!==traceGeneration)return;series.push({method,data});}
    traceSeries=series;traceTime=0;traceDuration=Math.max(...series.map(s=>s.data.times.at(-1)));$('traceSeek').max=String(traceDuration);$('traceSeek').step='any';
    $('traceSeek').disabled=false;$('traceToggle').disabled=false;$('traceRestart').disabled=false;
    $('traceLegend').replaceChildren();series.forEach((s,i)=>{const item=document.createElement('span'),swatch=document.createElement('i');swatch.style.background=colors[i];item.append(swatch,document.createTextNode(s.method));$('traceLegend').append(item);});
    tracePlot();playTrace();
  }finally{if(generation===traceGeneration){$('integrate').disabled=false;$('runAll').disabled=false;}}
}
action('integrate',()=>integrate(false));action('runAll',()=>integrate(true));
function barPlot(id,values,labels){
  const canvas=$(id),c=canvas.getContext('2d'),w=canvas.width,h=canvas.height,mid=w/2,max=Math.max(1,...values.map(Math.abs)),spacing=(h-60)/values.length;
  c.clearRect(0,0,w,h);c.strokeStyle='#a8b5ad';c.lineWidth=1;c.beginPath();c.moveTo(mid,20);c.lineTo(mid,h-20);c.stroke();c.font='17px monospace';
  values.forEach((value,i)=>{const y=30+i*spacing+spacing/2,width=value/max*(w*.35);c.fillStyle=colors[i%colors.length];c.fillRect(Math.min(mid,mid+width),y-12,Math.abs(width),24);c.fillStyle='#20343c';c.fillText(labels[i],18,y-20);c.fillText(fmt(value),18,y+8);});
}
action('pid',async()=>{const r=await api('pid',{gains:[num('kp'),num('ki'),num('kd')],state:[num('pidq'),num('pidv'),num('pidtarget'),num('pidtv'),num('piddt')]});$('pidEffort').textContent=fmt(r.effort);$('pidIntegral').textContent=fmt(r.integral);barPlot('pidCanvas',[r.effort],['Effort']);});
action('dynamics',async()=>{const r=await api('dynamics',{q:array('dq'),velocity:array('dv'),torque:array('du'),lengths:array('dl'),masses:array('dm'),gravity:num('gravity')});barPlot('dynamicsCanvas',r.acceleration,r.acceleration.map((_,i)=>'Joint '+(i+1)+' acceleration'));table('dynamicsReadout',['Joint','Acceleration','Coriolis load','Gravity load'],r.acceleration.map((a,i)=>['Joint '+(i+1),a,r.coriolis[i],r.gravity[i]]));table('massMatrix',r.mass.map((_,i)=>'Column '+(i+1)),r.mass);});
window.addEventListener('pagehide',()=>{selectMode('stopped');pauseTrace();});
armPlot([],[1,1]);startLive();
