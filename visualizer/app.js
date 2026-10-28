const $=id=>document.getElementById(id), num=id=>Number($(id).value), array=id=>JSON.parse($(id).value);
const fmt=x=>Number.isFinite(x)?Number(x).toPrecision(6):String(x);
const colors=['#127e91','#d55a2e','#5e7b45','#8865a7'];
async function api(path,data){
  $('error').hidden=true;
  const response=await fetch('/api/'+path,data===undefined?{}:{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(data)});
  const result=await response.json();
  if(!response.ok||result.error)throw Error((result.error||response.statusText)+(result.hint?'\n'+result.hint:''));
  return result;
}
function action(id,fn){$(id).onclick=async()=>{try{await fn()}catch(error){$('error').hidden=false;$('error').textContent=error.message;}}}
document.querySelectorAll('.tab').forEach(button=>button.onclick=()=>{document.querySelectorAll('.tab,.page').forEach(e=>e.classList.remove('active'));button.classList.add('active');$(button.dataset.page).classList.add('active');$('error').hidden=true;});
function table(id,headings,rows){
  const target=$(id),table=document.createElement('table'),head=document.createElement('tr');
  headings.forEach(text=>{const cell=document.createElement('th');cell.textContent=text;head.append(cell);});table.append(head);
  rows.forEach(values=>{const row=document.createElement('tr');values.forEach(value=>{const cell=document.createElement('td');cell.textContent=typeof value==='number'?fmt(value):value;row.append(cell);});table.append(row);});
  target.replaceChildren(table);
}
function armPlot(points,lengths){
  const canvas=$('armCanvas'),c=canvas.getContext('2d'),w=canvas.width,h=canvas.height;
  c.clearRect(0,0,w,h);const reach=Math.max(.01,lengths.reduce((a,b)=>a+b,0)),s=Math.min(w,h)*.4/reach,ox=w/2,oy=h/2;
  c.strokeStyle='#dce1d6';c.lineWidth=1;
  for(let i=-5;i<=5;i++){const d=i*reach/5*s;c.beginPath();c.moveTo(ox+d,0);c.lineTo(ox+d,h);c.stroke();c.beginPath();c.moveTo(0,oy+d);c.lineTo(w,oy+d);c.stroke();}
  c.strokeStyle='#a8b5ad';c.beginPath();c.moveTo(0,oy);c.lineTo(w,oy);c.moveTo(ox,0);c.lineTo(ox,h);c.stroke();
  c.fillStyle='#7e8a80';c.font='15px monospace';c.fillText('x →',w-55,oy-12);c.fillText('y ↑',ox+12,22);
  if(!points.length)return;
  const screen=points.map(([x,y])=>[ox+x*s,oy-y*s]);
  c.lineWidth=10;c.lineCap='round';c.strokeStyle='#127e91';c.beginPath();screen.forEach(([x,y],i)=>i?c.lineTo(x,y):c.moveTo(x,y));c.stroke();
  screen.forEach(([px,py],i)=>{c.fillStyle=i===0?'#20343c':'#fffefa';c.strokeStyle='#127e91';c.lineWidth=4;c.beginPath();c.arc(px,py,i===0?12:9,0,Math.PI*2);c.fill();c.stroke();});
}
$('links').onchange=()=>{$('lengths').value=$('links').value==='2'?'[1,1]':'[1,1,0.6]';$('phi').disabled=$('links').value==='2';};$('phi').disabled=true;
action('solve',async()=>{const lengths=array('lengths'),target=[num('tx'),num('ty')];if(lengths.length!==num('links'))throw Error('Link count and length array must agree.');const r=await api('ik',{lengths,target,phi:num('phi')});armPlot(r.points,lengths);$('armBadge').textContent='SOURCE';$('endPosition').textContent=r.position.map(fmt).join(', ');$('orientation').textContent=fmt(r.orientation)+' rad';table('armReadout',['Joint','Position / rad'],r.joints.map((q,i)=>['Joint '+(i+1),q]));});
action('snapshot',async()=>{const r=await api('state');armPlot(r.points,r.parameters.lengths);$('armBadge').textContent='LIVE SNAPSHOT';$('endPosition').textContent=r.points.at(-1).map(fmt).join(', ');$('orientation').textContent='—';table('armReadout',['Joint','Position','Velocity','Effort'],r.state.position.map((q,i)=>[r.state.name[i],q,r.state.velocity[i],r.state.effort[i]]));$('liveStatus').textContent='Snapshot received. Select Read live state to refresh.';});
async function live(service,args){const r=await api('live',{service,args});if(r.result===false)throw Error(r.status||'Live service rejected the request.');$('liveStatus').textContent='Command sent.';return r;}
action('servo',()=>live('/ik_action/send_goal',{x:num('tx'),y:num('ty'),phi:num('phi')}));
action('pause',()=>live('/arm_sim/pause',{data:true}));action('resume',()=>live('/arm_sim/pause',{data:false}));action('enable',()=>live('/pid_controller/enable',{data:true}));action('reset',()=>live('/arm_sim/reset',{}));
function tracePlot(series){
  const canvas=$('traceCanvas'),c=canvas.getContext('2d'),w=canvas.width,h=canvas.height,p={l:80,r:30,t:40,b:65};c.clearRect(0,0,w,h);
  const all=series.flatMap(s=>s.data.positions);let lo=Math.min(...all),hi=Math.max(...all),pad=(hi-lo)*.08||1;lo-=pad;hi+=pad;
  const duration=Math.max(...series.map(s=>s.data.times.at(-1)));const xx=t=>p.l+t/duration*(w-p.l-p.r),yy=y=>h-p.b-(y-lo)/(hi-lo)*(h-p.t-p.b);
  c.font='14px monospace';c.lineWidth=1;
  for(let i=0;i<=5;i++){const value=lo+(hi-lo)*i/5,y=yy(value);c.strokeStyle='#dce1d6';c.beginPath();c.moveTo(p.l,y);c.lineTo(w-p.r,y);c.stroke();c.fillStyle='#697879';c.fillText(value.toFixed(2),8,y+5);const t=duration*i/5,x=xx(t);c.fillText(t.toFixed(2),x-15,h-30);}
  c.fillText('t / seconds',w-145,h-8);c.fillText('position',12,20);
  series.forEach((s,index)=>{c.strokeStyle=colors[index%colors.length];c.lineWidth=3;c.beginPath();s.data.positions.forEach((v,i)=>i?c.lineTo(xx(s.data.times[i]),yy(v)):c.moveTo(xx(s.data.times[i]),yy(v)));c.stroke();});
  $('traceLegend').replaceChildren();series.forEach((s,i)=>{const item=document.createElement('span'),swatch=document.createElement('i');swatch.style.background=colors[i];item.append(swatch,document.createTextNode(s.method));$('traceLegend').append(item);});
}
$('forcing').onchange=()=>{if($('forcing').value!=='custom')$('expression').value=$('forcing').value;};
async function integrate(all){const methods=all?['euler','midpoint','verlet','rk4']:[$('method').value],series=[];for(const method of methods){const data=await api('integrate',{function:$('expression').value,x0:num('x0'),xdot0:num('v0'),dt:num('dt'),steps:num('steps'),integrator:method});series.push({method,data});}tracePlot(series);const data=series.at(-1).data;$('finalPosition').textContent=fmt(data.positions.at(-1));$('finalVelocity').textContent=fmt(data.velocities.at(-1));table('integrationReadout',['Method','Final time','Position','Velocity'],series.map(s=>[s.method,s.data.times.at(-1),s.data.positions.at(-1),s.data.velocities.at(-1)]));}
action('integrate',()=>integrate(false));action('runAll',()=>integrate(true));
function barPlot(id,values,labels){
  const canvas=$(id),c=canvas.getContext('2d'),w=canvas.width,h=canvas.height,mid=w/2,max=Math.max(1,...values.map(Math.abs)),spacing=(h-60)/values.length;
  c.clearRect(0,0,w,h);c.strokeStyle='#a8b5ad';c.lineWidth=1;c.beginPath();c.moveTo(mid,20);c.lineTo(mid,h-20);c.stroke();c.font='17px monospace';
  values.forEach((value,i)=>{const y=30+i*spacing+spacing/2,width=value/max*(w*.35);c.fillStyle=colors[i%colors.length];c.fillRect(Math.min(mid,mid+width),y-12,Math.abs(width),24);c.fillStyle='#20343c';c.fillText(labels[i],18,y-20);c.fillText(fmt(value),18,y+8);});
}
action('pid',async()=>{const r=await api('pid',{gains:[num('kp'),num('ki'),num('kd')],state:[num('pidq'),num('pidv'),num('pidtarget'),num('pidtv'),num('piddt')]});$('pidEffort').textContent=fmt(r.effort);$('pidIntegral').textContent=fmt(r.integral);barPlot('pidCanvas',[r.effort],['Effort']);});
action('dynamics',async()=>{const r=await api('dynamics',{q:array('dq'),velocity:array('dv'),torque:array('du'),lengths:array('dl'),masses:array('dm'),gravity:num('gravity')});barPlot('dynamicsCanvas',r.acceleration,r.acceleration.map((_,i)=>'Joint '+(i+1)+' acceleration'));table('dynamicsReadout',['Joint','Acceleration','Coriolis load','Gravity load'],r.acceleration.map((a,i)=>['Joint '+(i+1),a,r.coriolis[i],r.gravity[i]]));table('massMatrix',r.mass.map((_,i)=>'Column '+(i+1)),r.mass);});
armPlot([],[1,1]);
