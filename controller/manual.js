const $ = id => document.getElementById(id);
const dashboard = new URL(location.href); dashboard.port='8002'; dashboard.pathname='/'; $('dashboard').href=dashboard.href;
let enabled=false, held=false, axes=[0,0], pending=false, pointer=null, keys=new Set();
async function send(action, data={}) {
  const r=await fetch('/api/'+action,{method:'POST',headers:{'Content-Type':'application/json','X-Kenny-Control':'manual'},body:JSON.stringify(data),signal:AbortSignal.timeout(1500)});
  const result=await r.json();if(!r.ok)throw Error(result.detail||'Request rejected');return result;
}
function center(){axes=[0,0];$('knob').style.transform='translate(0px,0px)';}
function release(){held=false;pointer=null;keys.clear();center();send('release').catch(()=>{});}
async function stop(){release();enabled=false;try{await send('stop');$('message').textContent='Stopped';}catch(e){$('message').textContent=e.message;}}
$('enable').onclick=async()=>{try{await send('enable');enabled=true;$('message').textContent='Enabled · hold joystick to drive the real robot';}catch(e){$('message').textContent=e.message;}};
$('clearFault').onclick=async()=>{release();enabled=false;$('clearFault').disabled=true;try{const r=await fetch('/api/clear-fault',{method:'POST',headers:{'X-Kenny-Control':'manual'},signal:AbortSignal.timeout(8000)});const s=await r.json();if(!r.ok)throw Error(s.detail);$('message').textContent='Fault cleared · click Enable manual drive, then hold joystick';}catch(e){$('message').textContent=e.message;}finally{$('clearFault').disabled=false;}};
$('stop').onclick=stop;
$('scale').oninput=()=>$('scaleValue').textContent=$('scale').value+'%';
function updatePointer(e){const r=$('pad').getBoundingClientRect(),radius=r.width*.35;let x=(e.clientX-r.left-r.width/2)/radius,y=(e.clientY-r.top-r.height/2)/radius;const length=Math.hypot(x,y);if(length>1){x/=length;y/=length;}axes=[Math.abs(y)<.12?0:-y,Math.abs(x)<.12?0:-x];$('knob').style.transform=`translate(${x*radius}px,${y*radius}px)`;}
$('pad').onpointerdown=e=>{if(!enabled){$('message').textContent='Click Enable manual drive first. If a motor fault is shown, clear it while stopped.';return;}e.preventDefault();held=true;pointer=e.pointerId;$('pad').setPointerCapture(e.pointerId);updatePointer(e);};
$('pad').onpointermove=e=>{if(held&&pointer===e.pointerId)updatePointer(e);};
for(const name of ['pointerup','pointercancel','lostpointercapture'])$('pad').addEventListener(name,release);
$('pad').onkeydown=e=>{if(!enabled||!e.key.startsWith('Arrow'))return;e.preventDefault();keys.add(e.key);held=true;axes=[Number(keys.has('ArrowUp'))-Number(keys.has('ArrowDown')),Number(keys.has('ArrowLeft'))-Number(keys.has('ArrowRight'))];};
$('pad').onkeyup=e=>{if(e.key.startsWith('Arrow')){e.preventDefault();keys.delete(e.key);if(!keys.size)release();else axes=[Number(keys.has('ArrowUp'))-Number(keys.has('ArrowDown')),Number(keys.has('ArrowLeft'))-Number(keys.has('ArrowRight'))];}};
$('pad').onblur=release;window.addEventListener('blur',release);document.addEventListener('visibilitychange',()=>{if(document.hidden)release();});document.addEventListener('keydown',e=>{if(e.code==='Space'&&!['INPUT','BUTTON'].includes(e.target.tagName)){e.preventDefault();stop();}});
setInterval(async()=>{if(!enabled||!held||pending)return;pending=true;try{await send('drive',{forward:axes[0],turn:axes[1],power:Number($('scale').value)/100});}catch(e){release();enabled=false;$('message').textContent=e.message;}finally{pending=false;}},75);
const fmt=v=>Number.isFinite(v)?v.toFixed(2):'—';
async function poll(){try{const r=await fetch('/api/status',{signal:AbortSignal.timeout(1500)});if(!r.ok)throw Error('Joystick status unavailable');const s=await r.json();enabled=s.enabled;const m=s.monitor||{};const blocked=s.reason||s.output_reason;$('drive').textContent=blocked?'Blocked':enabled?(s.held?'Joystick held':'Enabled · idle'):'Stopped';$('message').textContent=blocked||m.gate||'Waiting for measured telemetry';$('sensors').textContent='LiDAR nearest: '+fmt(m.lidar_minimum_m)+' m · closest depth surface: '+fmt(m.depth_minimum_m)+' m · valid depth '+fmt((m.depth_valid_fraction||0)*100)+'% · central view '+fmt((m.central_depth_valid_fraction||0)*100)+'%';$('markers').textContent='Visible ArUco: '+(m.visible_markers?.join(', ')||'none');const p=m.pose_estimate;$('pose').textContent=p?.available?'Pose '+(p.live?'live':'stale')+': '+p.pose.map(fmt).join(', ')+' · '+p.source:'Map pose not established; manual movement is not verified arrival at ID 11';$('motors').textContent=JSON.stringify({pwm:m.motors?.pwm,rpm:m.motors?.rpm,counts:m.motors?.counts,faults:m.motors?.faults,bridge:m.motors?.bridge},null,2);}catch(e){release();enabled=false;$('message').textContent='Connection lost · commands stopped';}finally{setTimeout(poll,300);}}
poll();
