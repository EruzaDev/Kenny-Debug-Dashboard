const $ = id => document.getElementById(id);
const esc = value => String(value ?? '—').replace(/[&<>"']/g, c => ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
const fmt = (value, places = 2) => Number.isFinite(value) ? value.toFixed(places) : '—';
const table = (rows, headings = []) => '<table>' + (headings.length ? '<thead><tr>' + headings.map(v => '<th>' + esc(v) + '</th>').join('') + '</tr></thead>' : '') + '<tbody>' + rows.map(row => '<tr>' + row.map(v => '<td>' + esc(v) + '</td>').join('') + '</tr>').join('') + '</tbody></table>';
let state = null, mapData = null, mapImage = null, mapKey = '', scanData = null, page = 'overview', view = 'map';
let lastSuccess = 0, lastDetails = 0, mapRequest = 0, zoom = 1, pan = [0, 0], drag = null;
const editorUrl = new URL(location.href);
editorUrl.port = '8001'; editorUrl.pathname = '/'; editorUrl.search = ''; editorUrl.hash = '';
$('editor').href = $('editorStandalone').href = editorUrl.href;
$('ws').textContent = 'ws://' + editorUrl.host + '/api/live';
const fresh = (name, limit = 2) => state?.packets[name] && state.packets[name].age_s <= limit;
const visibleIds = () => new Set((state?.aruco?.seen || []).filter(v => v.last_seen_age_s < 2).map(v => v.id));

function setPage(next) {
  page = next;
  if (location.hash !== '#' + next) history.replaceState(null, '', '#' + next);
  for (const id of ['overview', 'diagnostics', 'editorPage']) $(id).hidden = id !== next;
  document.querySelectorAll('.tab').forEach(tab => tab.classList.toggle('active', tab.dataset.page === next));
  if (next === 'diagnostics') renderDetails(true);
  if (next === 'overview') drawMap();
  if (next === 'editorPage') $('editorPanel').open = true;
}
document.querySelectorAll('.tab').forEach(tab => tab.onclick = () => setPage(tab.dataset.page));
if (location.hash === '#diagnostics' || location.hash === '#editorPage') setPage(location.hash.slice(1));
window.addEventListener('hashchange', () => {
  const next = location.hash.slice(1);
  if (['overview', 'diagnostics', 'editorPage'].includes(next)) setPage(next);
});
$('editorPanel').addEventListener('toggle', () => {
  if ($('editorPanel').open && !$('editorFrame').src) $('editorFrame').src = editorUrl.href;
});
document.querySelectorAll('.diagnostic').forEach(section => section.addEventListener('toggle', () => renderDetails(true)));
async function refreshMotorPorts() {
  try {
    const response = await fetch('/api/motor-ports', {signal: AbortSignal.timeout(3000)});
    if (!response.ok) throw Error('Could not list motor ports');
    const data = await response.json();
    for (const [id, selected] of [['leftMotorPort', data.left], ['rightMotorPort', data.right]]) {
      $(id).replaceChildren(...data.ports.map(port => {
        const option = new Option(port, port);
        option.selected = port === selected;
        return option;
      }));
    }
    $('motorPortState').textContent = data.ports.length < 2 ? 'Connect both ESP32s to select motor sides.' : 'Select the physical left and right boards, then save.';
  } catch (error) { $('motorPortState').textContent = error.message; }
}
$('motorPortsDetails').addEventListener('toggle', () => { if ($('motorPortsDetails').open) refreshMotorPorts(); });
$('refreshMotorPorts').onclick = refreshMotorPorts;
$('saveMotorPorts').onclick = async () => {
  $('saveMotorPorts').disabled = true;
  $('motorPortState').textContent = 'Verifying motor firmware and saving sides…';
  try {
    const response = await fetch('/api/motor-ports', {method:'POST', headers:{'X-Kenny-Control':'operations','Content-Type':'application/json'}, body:JSON.stringify({left:$('leftMotorPort').value,right:$('rightMotorPort').value}), signal:AbortSignal.timeout(30000)});
    const data = await response.json();
    if (!response.ok) throw Error(data.detail || 'Could not save motor sides');
    $('motorPortState').textContent = 'Saved: left ' + data.left + ' · right ' + data.right + '. STOP latched; motor bridge connecting in Robot mode.';
  } catch (error) { $('motorPortState').textContent = error.message; }
  finally { $('saveMotorPorts').disabled = false; }
};

async function requestControl(action, raised = false) {
  const headers = {'X-Kenny-Control': 'operations'};
  if (action === 'pulse/forward') headers['X-Kenny-Floor-Pulse'] = 'confirmed';
  if (raised) headers['X-Kenny-Raised-Wheels'] = 'confirmed';
  const response = await fetch('/api/' + action, {method: 'POST', headers, signal: AbortSignal.timeout(3000)});
  const data = await response.json();
  if (!response.ok) throw Error(data.detail || 'Request rejected');
  render(data);
  return data;
}
async function command(action) {
  $('message').textContent = action === 'stop' ? 'Sending STOP…' : 'Requesting reset…';
  try {
    await requestControl(action);
    $('message').textContent = action === 'stop' ? 'Stop requested. Physical stopping is shown above.' : 'Stop reset; pending missions may resume.';
  } catch (error) {
    $('message').textContent = 'Command failed: ' + error.message + (action === 'stop' ? '. Use the physical motor-power stop if needed.' : '');
  }
}
$('stop').onclick = () => command('stop');
$('reset').onclick = () => { if (confirm('Allow motion again? A pending mission may resume.')) command('reset'); };
async function retrySensor(sensor) {
  const label = {lidar:'LiDAR',camera:'RGB-D camera',imu:'MPU6050'}[sensor];
  $('retryLidar').disabled = $('retryCamera').disabled = $('retryImu').disabled = true;
  $('sensorRetryState').textContent = 'Retrying ' + label + '…';
  try {
    const response = await fetch('/api/retry/' + sensor, {method: 'POST', headers: {'X-Kenny-Control': 'operations'}, signal: AbortSignal.timeout(8000)});
    const data = await response.json();
    if (!response.ok) throw Error(data.detail || 'Request rejected');
    render(data);
    $('sensorRetryState').textContent = label + ' retry requested. Waiting for live data; plug in the device if disconnected. STOP remains latched.';
  } catch (error) {
    $('sensorRetryState').textContent = label + ' retry failed: ' + error.message;
  } finally {
    $('retryLidar').disabled = $('retryCamera').disabled = $('retryImu').disabled = false;
  }
}
$('retryLidar').onclick = () => retrySensor('lidar');
$('retryCamera').onclick = () => retrySensor('camera');
$('retryImu').onclick = () => retrySensor('imu');
let lidarOrientationLoaded = false;
async function refreshLidarOrientation() {
  try {
    const response = await fetch('/api/lidar/orientation', {signal: AbortSignal.timeout(2000)});
    if (!response.ok) throw Error('LiDAR calibration unavailable');
    const data = await response.json();
    if (!lidarOrientationLoaded) {
      $('lidarYaw').value = String(data.yaw_deg);
      $('lidarInvert').checked = data.inverted;
      lidarOrientationLoaded = true;
    }
    const reading = name => data.sectors[name] ? fmt(data.sectors[name].minimum_m) + ' m (' + data.sectors[name].samples + ' rays)' : 'no measured rays';
    $('lidarSectors').textContent = data.scan_age_s == null ? 'Waiting for fresh scan after restart…' :
      'Kenny front ' + reading('front') + ' · left ' + reading('left') + ' · right ' + reading('right') + ' · rear ' + reading('rear');
  } catch (error) { $('lidarSectors').textContent = error.message; }
}
$('lidarApply').onclick = async () => {
  $('lidarApply').disabled = true;
  $('lidarOrientationState').textContent = 'Applying orientation; STOP will be latched…';
  try {
    const response = await fetch('/api/lidar/orientation', {method:'POST',
      headers: {'Content-Type':'application/json', 'X-Kenny-Control':'operations'},
      body: JSON.stringify({yaw_deg:Number($('lidarYaw').value),inverted:$('lidarInvert').checked}),
      signal: AbortSignal.timeout(45000)});
    const data = await response.json();
    if (!response.ok) throw Error(data.detail || 'Orientation update failed');
    $('lidarOrientationState').textContent = data.changed ? 'LiDAR orientation applied. Wait for fresh sectors; verify the known object before resetting STOP.' : 'Orientation unchanged; STOP latched.';
    await refreshLidarOrientation();
  } catch (error) { $('lidarOrientationState').textContent = error.message; }
  finally { $('lidarApply').disabled = false; }
};
refreshLidarOrientation();
setInterval(refreshLidarOrientation, 3000);
$('forward75').onclick = async () => {
  $('forward75').disabled = true;
  $('forwardPulseResult').textContent = 'Sending 75% forward pulse to the real ESP32…';
  try { await requestControl('pulse/forward'); }
  catch (error) { $('forwardPulseResult').textContent = 'Pulse refused: ' + error.message; }
};
$('supervisedRun').onclick = async () => {
  $('supervisedRun').disabled = true;
  $('supervisedRunState').textContent = 'Requesting 15-second supervised Robot run…';
  try {
    const result = await requestControl('supervised/start');
    $('supervisedRunState').textContent = 'Supervised drive active for ' + fmt(result.supervised_remaining_s, 1) + ' s; STOP available above.';
  } catch(error) { $('supervisedRunState').textContent = 'Run refused: ' + error.message; }
};
async function deploymentAction(action) {
  $('robotMode').disabled = $('simulationMode').disabled = $('controllerMode').disabled = $('syncMap').disabled = true;
  $('deploymentState').textContent = 'Updating deployment…';
  try {
    const response = await fetch('/api/' + action, {method: 'POST', headers: {'X-Kenny-Control': 'operations'}, signal: AbortSignal.timeout(65000)});
    const data = await response.json();
    if (!response.ok) throw Error(data.detail || 'Request rejected');
    render(data); mapKey = ''; await fetchMap(true);
    $('deploymentState').textContent = action === 'mode/controller' ? 'Continuous two-ESP32 speed joystick started. Open joystick; raw encoder counts are in Diagnostics. STOP remains latched.' : action === 'mode/robot' ? 'Robot selected. Waiting for ArUco localization and live sensors; STOP remains latched.' : action === 'mode/simulation' ? 'Simulation selected. Connect the API receiver in Map creator to run simulated requests.' : 'Uploaded Render map synchronized.';
  } catch (error) { $('deploymentState').textContent = error.message; }
  finally { $('robotMode').disabled = $('simulationMode').disabled = $('controllerMode').disabled = $('syncMap').disabled = false; }
}
$('robotMode').onclick = () => deploymentAction('mode/robot');
$('simulationMode').onclick = () => deploymentAction('mode/simulation');
$('controllerMode').onclick = () => deploymentAction('mode/controller');
$('syncMap').onclick = () => deploymentAction('map/sync');
async function applyRobotDrive() {
  $('applyDriveMode').disabled = true;
  $('driveModeState').textContent = 'Changing robot drive mode; STOP will be latched…';
  try {
    const response = await fetch('/api/drive-mode/' + $('robotDriveMode').value,
      {method:'POST',headers:{'X-Kenny-Control':'operations'},signal:AbortSignal.timeout(45000)});
    const data = await response.json();
    if (!response.ok) throw Error(data.detail || 'Drive mode change failed');
    render(data);
    $('driveModeState').textContent = 'Robot drive set to ' + data.deployment.drive_mode + '. Wait for fresh controller/motor telemetry before resetting STOP.';
  } catch (error) { $('driveModeState').textContent = error.message; }
  finally { $('applyDriveMode').disabled = false; }
}
$('applyDriveMode').onclick = applyRobotDrive;
async function bench(boost) {
  const text = boost ? 'RAISED WHEELS ONLY: 75% PWM for at most 0.5 seconds.' : 'RAISED WHEELS ONLY: low-speed forward for at most 3 seconds.';
  if (!confirm(text + ' All wheels clear of the floor, motor power connected, physical stop reachable?')) return;
  try { await requestControl(boost ? 'bench75' : 'bench', true); $('message').textContent = 'Bounded wheel test started.'; }
  catch (error) { $('message').textContent = 'Wheel test refused: ' + error.message; }
}
$('bench').onclick = () => bench(false); $('bench75').onclick = () => bench(true);
async function calibrationAction(action, payload, raised = false) {
  const headers = {'X-Kenny-Control':'operations', 'Content-Type':'application/json'};
  if (raised) headers['X-Kenny-Raised-Wheels'] = 'confirmed';
  $('calState').textContent = 'Waiting for ' + action + '…';
  try {
    const response = await fetch('/api/calibration/' + action, {method:'POST',headers,
      body:JSON.stringify(payload),signal:AbortSignal.timeout(25000)});
    const result = await response.json();
    if (!response.ok) throw Error(result.detail || 'Calibration command refused');
    render(result);
    $('calState').textContent = action === 'connect' ? 'Motor bridge starting. Wait for fresh disarmed counts before capturing.' :
      action === 'pulse' ? 'Bounded raised-wheel PWM test started; waiting for disarmed counts.' :
      action === 'hand/start' ? 'Starting counts captured. Rotate only the selected wheel forward, then capture finish.' :
      action === 'hand/finish' ? 'Revolution signal measured. Verify signal OK and invalid transitions before PWM.' :
      'Ground measurement saved; download the JSON and send it for review.';
  } catch (error) { $('calState').textContent = 'Calibration refused: ' + error.message; }
}
$('calConnect').onclick = () => calibrationAction('connect', {});
$('calStart').onclick = () => calibrationAction('hand/start', {wheel:Number($('calWheel').value)});
$('calFinish').onclick = () => calibrationAction('hand/finish', {wheel:Number($('calWheel').value),revolutions:Number($('calTurns').value)});
$('calPulse').onclick = () => {
  if (confirm('RAISED WHEELS ONLY: apply 75% PWM to ONE selected wheel for at most 0.45 seconds? Confirm wheels clear and physical motor-power stop accessible.'))
    calibrationAction('pulse', {wheel:Number($('calWheel').value)}, true);
};
$('calGround').onclick = () => calibrationAction('ground/measure', {distance_m:Number($('calDistance').value)});
$('calExport').onclick = () => download(JSON.stringify({calibration:state?.wheel_calibration,
  latest_pulse:state?.bench_result, motors:state?.packets?.motors?.data},null,2),'kenny-wheel-calibration.json');
document.addEventListener('keydown', event => {
  if (event.code === 'Space' && !['INPUT','TEXTAREA','BUTTON','SELECT','A'].includes(event.target.tagName) && !event.target.isContentEditable) {
    event.preventDefault(); command('stop');
  }
});

function download(content, name, type = 'application/json') {
  const url = URL.createObjectURL(new Blob([content], {type}));
  const link = document.createElement('a'); link.href = url; link.download = name; link.click();
  setTimeout(() => URL.revokeObjectURL(url), 1000);
}
$('report').onclick = async () => {
  try {
    const response = await fetch('/api/report'); if (!response.ok) throw Error(response.status);
    download(JSON.stringify(await response.json(), null, 2), 'kenny-diagnostics-' + new Date().toISOString().replace(/[:.]/g, '-') + '.json');
  } catch (error) { $('message').textContent = 'Report failed: ' + error.message; }
};
$('arucoExport').onclick = () => {
  const rows = [['Time','ID','Event','In selected map','Camera range m','Pose status'], ...(state?.aruco?.events || []).map(v => [new Date(v.time * 1000).toISOString(), v.id, v.event, v.registered, v.range_m ?? '', v.pose_status])];
  download(rows.map(row => row.map(v => '"' + String(v ?? '').replace(/"/g, '""') + '"').join(',')).join('\n'), 'kenny-aruco-scans.csv', 'text/csv');
};
$('repeatScans').onchange = () => renderAruco();
$('requestExport').onclick = () => download(JSON.stringify(state?.request_events || [], null, 2), 'kenny-request-process.json');
$('manualMarker').onchange = () => { $('manualRequest').disabled = !$('manualMarker').value; };
$('manualRequest').onclick = async () => {
  const marker = Number($('manualMarker').value);
  if (!$('manualMarker').value) return;
  $('manualRequest').disabled = true;
  $('manualRequestState').textContent = 'Sending manual ArUco ' + marker + ' request…';
  try {
    const response = await fetch('/api/request/manual', {method:'POST',
      headers:{'Content-Type':'application/json','X-Kenny-Control':'operations'},
      body:JSON.stringify({marker_id:marker,request_id:crypto.randomUUID()}),signal:AbortSignal.timeout(20000)});
    const data = await response.json();
    if (!response.ok) throw Error(data.detail || 'Request failed');
    $('manualRequestState').textContent = 'ArUco ' + marker + ' request ' + data.request.status + ' · ' + data.request.id.slice(0,8);
  } catch(error) { $('manualRequestState').textContent = 'Manual request failed: ' + error.message; }
  finally { $('manualRequest').disabled = false; }
};
async function cancelQueuedJob(button) {
  const id = button.dataset.cancelJob;
  button.disabled = true;
  $('queueCancelState').textContent = 'Cancelling queued request ' + id.slice(0,8) + '…';
  try {
    const response = await fetch('/api/request/cancel', {method:'POST',
      headers:{'Content-Type':'application/json','X-Kenny-Control':'operations'},
      body:JSON.stringify({job_id:id}),signal:AbortSignal.timeout(20000)});
    const data = await response.json();
    if (!response.ok) throw Error(data.detail || 'Cancellation failed');
    $('queueCancelState').textContent = 'ArUco ' + data.request.marker_id + ' request ' + id.slice(0,8) + ' cancelled on Render.';
    const latest = await fetch('/api/status', {signal:AbortSignal.timeout(3000)});
    if (latest.ok) render(await latest.json());
  } catch(error) { $('queueCancelState').textContent = 'Cancellation refused: ' + error.message; button.disabled = false; }
}

function renderPoseAndRequests() {
  const estimate = state.pose_estimate || {}, queue = state.packets.requests;
  $('poseSource').textContent = estimate.available ? (estimate.live ? estimate.source : 'Last known · stale') : 'Unavailable';
  $('poseCoordinates').textContent = estimate.available ? 'x ' + fmt(estimate.pose[0]) + ' m · y ' + fmt(estimate.pose[1]) + ' m · heading ' + fmt(estimate.pose[2]*180/Math.PI, 1) + '°' : 'Kenny position not established';
  const detections = state.packets.aruco?.data.detections || [];
  const depth = detections.filter(d => Number.isFinite(d.depth_range_m)).map(d => 'ID ' + d.id + ': depth ' + fmt(d.depth_range_m) + ' m (' + d.depth_samples + ' samples)' + (d.marker_size_warning ? ' · marker size mismatch: depth suggests ' + fmt(d.depth_inferred_size_m*100,1) + ' cm' : ''));
  $('poseDetails').textContent = (estimate.available ? 'Estimate age ' + fmt(estimate.age_s, 2) + ' s' + (estimate.uncertainty_m != null ? ' · uncertainty ' + fmt(estimate.uncertainty_m) + ' m' : '') + (estimate.live ? '' : ' · not a current position') : estimate.reason || 'Waiting for localization') + (depth.length ? ' · ' + depth.join(' · ') : ' · Depth-supported marker position pending');
  const jobs = queue?.data.jobs || [];
  $('requestState').textContent = queue ? (queue.age_s > 10 ? 'Queue status stale · ' : '') + (jobs.length ? jobs.map(j => 'ArUco ' + j.marker_id + ' · ' + j.status + ' · ' + j.id.slice(0,8)).join(' | ') : 'No pending requests') + ' · ' + (state.packets.controller?.data.status || (state.deployment?.mode === 'simulation' ? 'Navigation paused in simulation/controller debug; requests stay queued' : 'Navigation starting; STOP remains latched')) : 'Waiting for cloud queue status…';
  $('queueList').innerHTML = !queue || queue.age_s > 10 ? '<p>Cloud queue unavailable or stale. Waiting for a fresh check…</p>' : jobs.length ? jobs.map(j =>
    '<div class="toolbar"><strong>ArUco ' + esc(j.marker_id) + '</strong><span>' + esc(j.status) + ' · ' + esc(j.id.slice(0,8)) + '</span>' +
    (j.status === 'queued' ? '<button data-cancel-job="' + esc(j.id) + '">Cancel queued</button>' : '<span class="muted">Active trip</span>') + '</div>').join('') : '<p>No queued or running requests.</p>';
  $('queueList').querySelectorAll('[data-cancel-job]').forEach(button => button.onclick = () => cancelQueuedJob(button));
  const events = (state.request_events || []).slice(-100).reverse();
  $('requestLog').innerHTML = events.length ? table(events.map(e => [new Date(e.time*1000).toLocaleTimeString(), e.marker_id == null ? '—' : 'ID ' + e.marker_id, e.stage.replaceAll('_',' '), e.detail, e.job_id?.slice(0,8)]), ['Time','Destination','Stage','Process','Request']) : '<p class="muted">No request events recorded yet.</p>';
}

function health(id, value, status = 'good') { $(id).textContent = value; $(id).className = status; }
function render(s) {
  state = s; lastSuccess = Date.now();
  $('connection').textContent = 'Live · ' + new Date(s.time * 1000).toLocaleTimeString();
  $('connection').className = 'connection good';
  const p = s.packets, c = p.controller?.data, m = p.motors?.data, o = p.odom?.data;
  $('forward75').disabled = !s.emergency_stop || !(s.physical_stop_confirmed || s.debug_stop_confirmed && m?.bridge?.mode === 'dual_side_robot') || !fresh('motors', .5) || !!m?.faults || !m?.ready || s.topics.some(t => t.name === '/kenny/controller/status' && t.publishers > 0);
  $('supervisedRun').disabled = !s.emergency_stop || !s.motor_stop_confirmed || !fresh('controller') || !fresh('motors', .5) || s.deployment?.mode !== 'robot' || s.deployment?.drive_mode !== 'velocity';
  if (s.supervised_remaining_s > 0) $('supervisedRunState').textContent = 'Supervised drive · ' + fmt(s.supervised_remaining_s, 1) + ' s remaining; STOP available above.';
  if (s.ground_pulse && s.bench_running) $('forwardPulseResult').textContent = 'Real ESP32: 75% forward pulse active';
   else if (s.ground_pulse && s.bench_result) $('forwardPulseResult').textContent = 'Encoder change LF/RF/LR/RR: ' + (s.bench_result.delta_LF_RF_LR_RR?.join(' / ') || 'awaiting feedback') + (s.bench_result.stopping_confirmed ? ' · stopping confirmed by odometry' : s.bench_result.disarmed_confirmed ? ' · firmware disarmed; verify wheel movement visually' : ' · waiting for feedback');
  const depthObservation = c?.depth_observation?.measured_points > 0 ? c.depth_observation : null, imuStatus = p.imu_status?.data;
  const imuPaused = s.deployment?.imu_enabled === false;
  $('retryImu').textContent = imuPaused ? 'Resume MPU6050' : 'Retry MPU6050';
  $('nearestObstacle').textContent = depthObservation ? depthObservation.nearest_obstacle_m != null ? 'Nearest above-floor obstacle: ' + fmt(depthObservation.nearest_obstacle_m) + ' m' : 'No above-floor obstacle in measured depth points' : 'Waiting for measured depth';
  $('depthObstacleDetails').textContent = depthObservation ? 'Closest surface (including floor): ' + fmt(depthObservation.closest_surface_m) + ' m · floor coverage ' + fmt(depthObservation.floor_coverage*100,0) + '% · depth coverage ' + fmt(depthObservation.depth_coverage*100,0) + '%' + (depthObservation.too_close_points ? ' · ' + depthObservation.too_close_points + ' returns below calibrated minimum range' : '') : '';
  $('imuDetails').textContent = imuPaused ? 'MPU6050 intentionally paused · pose uses ArUco, LiDAR and wheel encoders' : imuStatus ? imuStatus.connected ? 'IMU connected on I²C bus ' + imuStatus.bus + ' at ' + imuStatus.address : 'IMU unavailable: ' + imuStatus.error + ' · automatic reconnect enabled; check power/SDA/SCL wiring' : '';
  const debug = p.host?.data.controller_debug_active;
  $('deploymentMode').textContent = debug ? 'Continuous speed test · two ESP32s' : s.deployment?.mode === 'robot' ? 'Robot mode' : 'Simulation mode';
  if (document.activeElement !== $('robotDriveMode')) $('robotDriveMode').value = s.deployment?.drive_mode || 'pwm';
  $('applyDriveMode').disabled = s.deployment?.mode !== 'robot';
  $('controllerMode').classList.toggle('selected', !!debug);
  $('debugJoystick').hidden = !debug;
  $('robotMode').classList.toggle('selected', s.deployment?.mode === 'robot' && !debug);
  $('simulationMode').classList.toggle('selected', s.deployment?.mode !== 'robot' && !debug);
  $('stopState').textContent = s.bench_running ? s.ground_pulse ? 'Real robot · 75% forward pulse' : 'Raised-wheel test running' : s.emergency_stop ? 'Stopped' : debug ? 'Controller debug permitted · raised wheels' : 'Motion permitted';
  $('stopDetail').textContent = s.bench_running ? (s.bench_duty ? '75% PWM · ESP32 command watchdog 0.2 seconds' : 'Low-speed test · deadline 3 seconds') : s.emergency_stop ? (s.motor_stop_confirmed ? 'Both ESP32s disarmed · PWM zero' : s.physical_stop_confirmed ? 'Confirmed by fresh wheel odometry' : s.debug_stop_confirmed ? 'Two ESP32s disarmed; raw encoder counts stationary' : 'Waiting for ESP32 stop acknowledgement') : c?.guard || c?.reason || 'Sensor interlocks remain active';
  $('banner').className = 'banner' + (s.bench_running || s.emergency_stop && !s.physical_stop_confirmed && !s.motor_stop_confirmed ? ' warning' : m?.faults ? ' danger' : '');
  $('reset').disabled = !s.emergency_stop || !(s.physical_stop_confirmed && fresh('controller') || s.motor_stop_confirmed && fresh('controller') || s.debug_stop_confirmed && m?.bridge?.mode === 'dual_side_debug') || !fresh('motors', .5);
  $('bench').disabled = !s.emergency_stop || !(s.physical_stop_confirmed || s.debug_stop_confirmed && m?.bridge?.mode === 'dual_side_robot') || !fresh('motors', .5) || !m?.ready || !m?.encodersEnabled || s.topics.some(t => t.name === '/kenny/controller/status' && t.publishers > 0);
  $('bench75').disabled = $('bench').disabled || !(typeof m?.benchActive === 'boolean' || m?.bridge?.mode === 'dual_side_robot' && m?.watchdog === 200);
  const calibrated = s.wheel_calibration || {hand:{},pulses:{},ground:null}, wheel = Number($('calWheel').value), wheelName = ['LF','RF','LR','RR'][wheel];
  $('calLive').textContent = fresh('motors', .5) && m?.bridge?.mode === 'dual_side_robot' ?
    ['LF','RF','LR','RR'].map((name,i) => name+' '+m.counts[i]+' transitions · '+fmt(m.rpm?.[i],1)+' RPM'+
      (m.encoderInvalidTransitions?.[i] ? ' · '+m.encoderInvalidTransitions[i]+' invalid' : '')).join(' | ') :
    'No fresh two-board encoder telemetry. Connect the motor bridge while STOP is latched.';
  $('calStart').disabled = $('calFinish').disabled = !s.emergency_stop || !s.motor_stop_confirmed;
  $('calPulse').disabled = !s.emergency_stop || !s.motor_stop_confirmed || !calibrated.hand?.[wheelName]?.signal_ok || !!s.bench_running ||
    s.topics.some(t => t.name === '/kenny/controller/status' && t.publishers > 0);
  $('calGround').disabled = !s.emergency_stop || !s.motor_stop_confirmed || !!s.bench_running || !s.ground_pulse || !s.bench_result?.disarmed_confirmed;
  $('calResults').textContent = JSON.stringify({hand:calibrated.hand, pulses:calibrated.pulses,
    ground:calibrated.ground},null,2);
  health('rgbHealth', fresh('/camera/color/image_raw') ? 'Live' : 'Unavailable', fresh('/camera/color/image_raw') ? 'good' : 'bad');
  health('depthHealth', fresh('/camera/depth/image_raw') ? 'Live' : 'Unavailable', fresh('/camera/depth/image_raw') ? 'good' : 'bad');
  health('lidarHealth', fresh('/scan') ? 'Live' : 'Unavailable', fresh('/scan') ? 'good' : 'bad');
  health('motorHealth', fresh('motors', .5) && m?.bridge?.connected ? m?.faults ? 'Fault ' + m.faults : m?.bridge?.mode === 'dual_side_debug' ? 'Two ESP32s · raw counts' : 'Connected' : 'Unavailable', fresh('motors', .5) && m?.bridge?.connected && !m?.faults ? 'good' : 'bad');
  health('imuHealth', imuPaused ? 'Paused' : imuStatus?.connected === false ? 'I²C error' : fresh('/imu/data_raw') ? 'Live' : 'Unavailable', imuPaused ? 'neutral' : fresh('/imu/data_raw') && imuStatus?.connected !== false ? 'good' : 'bad');
  const detector = p.aruco?.data, count = (detector?.detections || []).length;
  const readiness = [];
  if (debug) readiness.push('Two-ESP32 controller debug · raw Hall counts only · raised wheels · no calibrated odometry');
  else if (!fresh('controller')) readiness.push('Select Robot to start navigation');
  if (c?.mode === 'manual') readiness.push('Manual joystick active on port 8003 · autonomous missions paused');
  if (c && c.mode !== 'manual' && !c.roaming && c.uncertainty_m == null) readiness.push('Localization uncertainty unavailable');
  else if (!c?.roaming && c?.uncertainty_m > .35) readiness.push('Localization uncertainty too high: ' + fmt(c.uncertainty_m) + ' m (limit 0.35 m)');
  if (!c?.roaming && !detector?.robot_pose && !s.pose_estimate?.live) readiness.push(detector?.localization_error || 'Waiting for a surveyed ArUco pose with calibrated camera mount');
  else if (detector?.robot_pose) readiness.push('ArUco position: ' + detector.robot_pose.pose.map(v => fmt(v)).join(', ') + ' (x, y, yaw radians)');
  if (!fresh('/camera/depth/points', .5)) readiness.push('Depth point cloud missing/stale');
  if (c?.roaming) readiness.push('Local measured-clearance roaming · cloud missions paused · '+(c.roam_reason || 'waiting'));
  else if (c?.drive_mode === 'pwm') readiness.push('Autonomous direct PWM · 75% duty · pose-checked drive/turn steps');
   else if (c?.commissioning && c?.drive_mode === 'velocity') readiness.push('Continuous wheel-speed commissioning · maximum requested 0.05 m/s · measured-clearance checks active');
   else if (c?.commissioning) readiness.push('Commissioning · maximum requested 0.05 m/s · bounded peripheral gaps permitted');
  if (c?.roaming && c.roam_clearance?.scan_history) {
    const h=c.roam_clearance.scan_history;
    readiness.push('Recent measured scans: '+h.samples+' · '+h.retained_bins+' temporarily retained bins');
    if (c.roam_clearance.corridor_width_m != null) readiness.push('Travel corridor '+fmt(c.roam_clearance.corridor_width_m)+' m wide × '+fmt(c.roam_clearance.corridor_length_m)+' m long');
    if (c.roam_clearance.missing_forward_lidar_bins?.length) readiness.push('Unknown forward bins: '+c.roam_clearance.missing_forward_lidar_bins.join(', '));
  }
  if (c?.sensors?.floor && c.sensors.floor.valid_fraction < 1) readiness.push('Measured floor coverage: ' + fmt(c.sensors.floor.valid_fraction*100,0) + '%' + (c.commissioning ? '' : ' · incomplete'));
  if (c?.sensors?.depth && c.sensors.depth.valid_fraction < 1) readiness.push('Measured depth coverage: ' + fmt(c.sensors.depth.valid_fraction*100,0) + '%' + (c.commissioning ? '' : ' · incomplete'));
  if (s.deployment?.terrain_profile === 'flat_floor') readiness.push('Flat-floor profile · RGB-D floor checks active · no downward sensors');
  else if (!fresh('/kenny/downward', .5)) readiness.push('Downward measurements missing/stale');
  for (const [frame, status] of Object.entries(p.calibration?.data || {})) if (frame !== 'imu_link' && status !== 'Available') readiness.push(frame + ': ' + status);
  if (c?.last_tf_error && !c.roaming) readiness.push(c.last_tf_error);
  if (detector?.robot_pose?.pose && Number.isFinite(detector.observed_at) &&
      s.time-detector.observed_at < 1 && c?.pose) {
    const disagreement = Math.hypot(detector.robot_pose.pose[0]-c.pose[0],detector.robot_pose.pose[1]-c.pose[1]);
    if (disagreement > .35) readiness.push('ArUco and SLAM map positions differ by ' + fmt(disagreement) + ' m · verify map alignment before navigation');
  }
  $('navigationReadiness').textContent = readiness.length ? readiness.join(' · ') : 'Measured inputs available. Reset stop to serve pending requests.';
  health('arucoHealth', fresh('aruco') ? detector.state === 'Scanning' ? count ? count + ' visible' : 'Scanning' : detector.state : 'Not running', fresh('aruco') && detector.state === 'Scanning' ? 'good' : 'neutral');
  const host = p.host?.data, memory = parseFloat(host?.memory?.MemAvailable);
  $('piBrief').textContent = host ? fmt(host.temperature_c, 0) + '°C · ' + (Number.isFinite(memory) ? fmt(memory / 1e6, 1) + ' GB free' : '—') : '';
  $('missionState').textContent = c ? c.guard || c.status || (c.ready ? 'Ready' : c.reason) : 'Monitoring · navigation not started';
  $('motionBrief').textContent = fresh('odom', .5) ? fmt(o.v) + ' m/s · ' + fmt(o.w) + ' rad/s' : 'Waiting for odometry';
  $('goalBrief').textContent = c?.marker_id != null ? 'ArUco ' + c.marker_id : 'No active trip';
  renderVision(); renderAruco(); renderPoseAndRequests();
  if (page === 'overview') drawMap();
  if (page === 'diagnostics') renderDetails();
}

function renderVision() {
  if (!state) return;
  const p = state.packets;
  for (const name of ['color','depth']) {
    const packet = p['/camera/' + name + '/image_raw'], meta = p['camera_preview/' + name];
    const live = packet && packet.age_s < 2 && meta && meta.age_s < 2 && !meta.data.error;
    $(name + 'Preview').hidden = !live; $(name + 'Empty').hidden = live;
    $(name + 'Empty').textContent = meta?.data.error || 'Camera unavailable or stale';
    if (p.host?.data.camera_usb?.some(d => d.interfaces < 3)) $(name + 'Empty').textContent = 'USB not ready · reconnect the camera';
    $(name + 'State').textContent = live ? (name === 'color' ? 'Live RGB · scanned IDs highlighted' : 'Near = red · far = blue · black = unknown depth, not confirmed clear floor') : '';
    if (live && (name !== 'depth' || $('depthPanel').open) && page === 'overview') $(name + 'Preview').src = '/api/camera/' + name + '.jpg?t=' + state.time;
    if (name === 'depth') $('depthBrief').textContent = live ? 'Closest surface ' + fmt(meta.data.minimum_m) + ' m' : 'Waiting';
  }
  const packet = p.aruco?.data;
  const overlay = $('arucoOverlay');
  if (!fresh('aruco') || !packet || !Number.isFinite(packet.observed_at) || state.time - packet.observed_at > 1 || !packet.width || !packet.height) { overlay.replaceChildren(); return; }
  overlay.setAttribute('viewBox', '0 0 ' + packet.width + ' ' + packet.height);
  overlay.innerHTML = (packet.detections || []).filter(d => d.corners?.length === 4 && d.corners.every(p => p.length === 2 && p.every(Number.isFinite))).map(d => {
    const color = d.registered ? '#70ffd0' : '#ffd383';
    return '<polygon points="' + d.corners.map(p => p.join(',')).join(' ') + '" fill="none" stroke="' + color + '" stroke-width="3"/><text x="' + d.corners[0][0] + '" y="' + Math.max(16, d.corners[0][1] - 6) + '" fill="' + color + '" stroke="#09151c" stroke-width=".5" font-size="20" font-family="system-ui">ID ' + esc(d.id) + '</text>';
  }).join('');
}

function renderAruco() {
  if (!state) return;
  const p = state.packets.aruco, d = p?.data;
  const visible = (state.aruco?.seen || []).filter(v => v.last_seen_age_s < 2 && fresh('aruco'));
  $('visibleMarkers').innerHTML = visible.map(v => '<span class="marker-chip' + (v.registered ? '' : ' unknown') + '">ID ' + esc(v.id) + (v.range_m != null ? ' · ' + fmt(v.range_m) + ' m' : '') + (v.depth_range_m != null ? ' · measured depth' : ' · RGB estimate') + (v.registered ? '' : ' · not in map') + '</span>').join('');
  $('arucoStatus').textContent = !fresh('aruco') ? 'Detector not running or disconnected' : d.state === 'Scanning' ? visible.length ? 'Decoded camera sightings · mapped IDs highlighted on the saved map' : 'Scanning live RGB · ' + (d.detection_diagnostics?.rejected_candidates || 0) + ' candidates rejected' + (d.detection_diagnostics?.unconfirmed_ids?.length ? ' · ambiguous IDs ' + d.detection_diagnostics.unconfirmed_ids.join(', ') : '') + ' · ensure the entire border is visible, well lit, and matches ' + (d.dictionary || 'the map dictionary') : d.state;
  $('dictionary').textContent = d?.dictionary || state.map_summary?.dictionary || '';
  const events = [...(state.aruco?.events || [])].reverse().filter(v => $('repeatScans').checked || v.event !== 'Still visible').slice(0, 12);
  $('arucoLog').innerHTML = events.length ? table(events.map(v => [new Date(v.time * 1000).toLocaleTimeString(), v.id, v.event, v.range_m != null ? '~' + fmt(v.range_m) + ' m' : 'ID only', v.depth_range_m != null ? 'Measured depth · ' + v.depth_samples + ' points' : v.depth_status || 'RGB only', v.camera_position_m ? v.camera_position_m.map(n => fmt(n)).join(', ') : '—', v.pose_published ? 'Map pose' : 'Camera scan']), ['Time','ID','Scan','Range','Depth support','Camera XYZ m','Position']) : '<div class="empty-log">No ArUco sightings recorded yet. New IDs and reacquired markers appear here; repeat scans are grouped.</div>';
}

function renderDetails(force = false) {
  if (!state || page !== 'diagnostics' || !force && Date.now() - lastDetails < 2000) return;
  lastDetails = Date.now();
  const s = state, p = s.packets, c = p.controller?.data, m = p.motors?.data, o = p.odom?.data, cmd = p.command?.data;
  if ($('motionDetails').open) {
    $('motion').innerHTML = table([['Command',fmt(cmd?.v),fmt(cmd?.w)],['Measured',fmt(o?.v),fmt(o?.w)]], ['Motion','Forward m/s','Turn rad/s']);
   $('encoderCounts').innerHTML = m?.bridge?.mode === 'dual_side_debug' ? '<h3>Two ESP32s · raw Hall encoder transitions · 660 counts/revolution</h3>' + table(['LF','RF','LR','RR'].map((name,i) => [name,m.counts?.[i],m.encoderRate?.[i],m.encoderInvalidTransitions?.[i]]), ['Wheel','Count','Transitions/s','Invalid transitions']) + '<p class="muted">Turn each raised wheel by hand and watch its count change. Wheel directions still need physical verification; debug mode does not publish odometry.</p>' : '';
   $('motors').textContent = ''; $('motors').appendChild(Object.assign(document.createElement('pre'), {textContent: JSON.stringify(m || {detail:'Motor status unavailable'}, null, 2)})); drawMotion(s.history);
  }
  if ($('sensorDetails').open) {
    $('controller').innerHTML = c ? table([['Readiness',c.ready ? 'Ready' : c.reason],['Status',c.status],['Guard',c.guard || 'None'],['Goal',JSON.stringify(c.goal)],['Route points',c.route_points],['Uncertainty m',fmt(c.uncertainty_m)],['Inference ms',fmt(c.inference_ms)]]) : '<p class="muted">Navigation controller is not started. Raw sensor streams are shown in Overview.</p>';
    $('sensors').innerHTML = table(Object.entries(p).filter(([name]) => name.startsWith('/') || ['tf','aruco'].includes(name)).map(([name,v]) => [name,fmt(v.age_s),JSON.stringify(v.data)]), ['Input','Received age s','Details']);
  }
  if ($('eventDetails').open) $('events').innerHTML = table([...s.events].reverse().slice(0,100).map(v => [new Date(v.time * 1000).toLocaleTimeString(),v.level,v.source,v.message]), ['Time','Level','Source','Event']);
  if ($('hostDetails').open) {
    const h = p.host?.data;
    $('host').innerHTML = h ? table([['Temperature °C',fmt(h.temperature_c,1)],['Load 1/5/15 min',h.load_1_5_15?.map(v => fmt(v)).join(' / ')],['Disk free GB',h.disk_free_gb],['Memory',JSON.stringify(h.memory)],['Astra USB',JSON.stringify(h.camera_usb)]]) : 'Waiting for Pi diagnostics';
    $('serviceLogs').textContent = h?.service_logs || 'No service logs yet';
  }
  if ($('topicDetails').open) {
    $('topics').innerHTML = table(s.topics.map(v => [v.name,v.types.join(', '),v.publishers,v.subscribers]), ['Topic','Type','Publishers','Subscribers']);
    $('raw').textContent = JSON.stringify({...s, history:'Included in report', events:'Included in report'}, null, 2);
  }
  if ($('benchDetails').open) $('benchResult').textContent = s.bench_running ? 'Test running · STOP available above' : s.bench_result ? JSON.stringify(s.bench_result, null, 2) : 'No powered test in this dashboard session';
}

async function fetchMap(force = false) {
  if (!state) return;
  const source = $('mapSource').value, summary = state.map_summary || {};
  const key = state.session_id + ':' + source + ':' + summary.source + ':' + summary.revision;
  if (!force && key === mapKey) return;
  const id = ++mapRequest;
  const response = await fetch('/api/map' + (source === 'prior' ? '?source=prior' : ''), {signal:AbortSignal.timeout(4000)});
  if (!response.ok) throw Error('Map unavailable');
  const data = await response.json(); if (id !== mapRequest) return;
  mapData = data; mapKey = key; mapImage = null;
  const selectedMarker = $('manualMarker').value;
  $('manualMarker').innerHTML = '<option value="">Select a mapped ArUco…</option>' +
    (data.markers || []).filter(m => Number.isInteger(m.id)).map(m => '<option value="'+esc(m.id)+'">ArUco ID '+esc(m.id)+'</option>').join('');
  if ([...$('manualMarker').options].some(o => o.value === selectedMarker)) $('manualMarker').value = selectedMarker;
  $('manualRequest').disabled = !$('manualMarker').value;
  if (data.grid) {
    const g = data.grid; mapImage = document.createElement('canvas'); mapImage.width = g.width; mapImage.height = g.height;
    const ctx = mapImage.getContext('2d'), image = ctx.createImageData(g.width, g.height);
    for (let y = 0; y < g.height; y++) for (let x = 0; x < g.width; x++) {
      const value = g.data[y * g.width + x], index = ((g.height - 1 - y) * g.width + x) * 4;
      const color = value < 0 ? [68,81,99] : value >= 65 ? [27,38,53] : [183,198,216];
      image.data.set([...color,255], index);
    }
    ctx.putImageData(image, 0, 0);
  }
  drawMap();
}
function canvas(id) {
  const element = $(id), ratio = Math.min(devicePixelRatio || 1, 2), width = element.clientWidth, height = element.clientHeight;
  element.width = width * ratio; element.height = height * ratio;
  const ctx = element.getContext('2d'); ctx.scale(ratio,ratio); return [ctx,width,height];
}
function emptyCanvas(ctx, text, width, height) { ctx.fillStyle = '#a1b2c8'; ctx.font = '13px system-ui'; ctx.textAlign = 'center'; ctx.fillText(text, width/2, height/2); ctx.textAlign = 'left'; }
function drawMap() {
  if (page !== 'overview') return;
  const [ctx,w,h] = canvas('map');
  if (w <= 35 || h <= 35) return;
  $('mapSource').disabled = view === 'scan'; $('markerLabels').disabled = view === 'scan'; $('mapLegend').hidden = view === 'scan';
  if (view === 'scan') { drawScan(ctx,w,h); return; }
  $('mapName').textContent = mapData?.name || 'Selected environment';
  $('mapSourceBadge').textContent = mapData?.source === 'live_refined' ? 'Live refined map' : mapData?.source === 'saved_prior' ? 'Saved map' : 'Waiting';
  if (!mapData?.grid || !mapImage) { emptyCanvas(ctx,'No saved or live map available',w,h); $('mapStatus').textContent = 'Open a map in the map creator or start live mapping.'; return; }
  const g = mapData.grid, scale = Math.min((w-35)/g.width,(h-35)/g.height) * zoom;
  const ox = (w-g.width*scale)/2 + pan[0], oy = (h-g.height*scale)/2 + pan[1];
  ctx.imageSmoothingEnabled = false; ctx.drawImage(mapImage,ox,oy,g.width*scale,g.height*scale);
  const point = (x,y) => { const dx=x-g.origin[0],dy=y-g.origin[1],a=g.origin[2]; return [ox+(Math.cos(a)*dx+Math.sin(a)*dy)/g.resolution*scale,oy+(g.height-(-Math.sin(a)*dx+Math.cos(a)*dy)/g.resolution)*scale]; };
  for (const obstacle of mapData.obstacles || []) {
    const corners = [[obstacle.x,obstacle.y],[obstacle.x+obstacle.width,obstacle.y],[obstacle.x+obstacle.width,obstacle.y+obstacle.depth],[obstacle.x,obstacle.y+obstacle.depth]].map(p => point(...p));
    ctx.fillStyle = ({wall:'#536579',chair:'#5d8785',bag:'#967458',prohibited:'#b86a79'})[obstacle.kind] || '#6c8098';
    ctx.strokeStyle = '#28364a'; ctx.lineWidth = 1; ctx.beginPath(); corners.forEach((p,i) => i ? ctx.lineTo(...p) : ctx.moveTo(...p)); ctx.closePath(); ctx.fill(); ctx.stroke();
  }
  const ids = visibleIds(); ctx.font = '11px system-ui';
  for (const marker of mapData.markers || []) {
    const [x,y] = point(marker.x,marker.y), seen = ids.has(marker.id), goal = marker.id === mapData.destination_marker_id;
    ctx.fillStyle = seen ? '#21a476' : goal ? '#df8549' : '#f1c75a'; ctx.strokeStyle = '#213047'; ctx.lineWidth = 1;
    ctx.beginPath(); ctx.rect(x-5,y-5,10,10); ctx.fill(); ctx.stroke();
    if ($('markerLabels').checked) { ctx.fillStyle = '#122033'; ctx.fillText(String(marker.id),x+8,y+4); }
  }
  const route = state?.packets.route;
  if (route?.age_s < 3 && route.data.frame === 'map') { ctx.strokeStyle='#3789ee'; ctx.lineWidth=2; ctx.beginPath(); route.data.points.forEach(([x,y],i) => { const p=point(x,y); i?ctx.lineTo(...p):ctx.moveTo(...p); }); ctx.stroke(); }
   const estimate = state?.pose_estimate, pose = estimate?.pose;
   const located = estimate?.available && estimate.live;
   if (estimate?.available) {
     const [x,y] = point(...pose), a = pose[2]-g.origin[2], color = located ? '#00d89a' : '#98a5b8';
     const uncertainty = estimate.uncertainty_m;
     if (Number.isFinite(uncertainty) && uncertainty > 0) { ctx.fillStyle=located?'#00d89a22':'#98a5b822';ctx.strokeStyle=color;ctx.setLineDash([4,4]);ctx.beginPath();ctx.arc(x,y,uncertainty/g.resolution*scale,0,Math.PI*2);ctx.fill();ctx.stroke();ctx.setLineDash([]); }
     ctx.save();ctx.translate(x,y);ctx.rotate(-a);ctx.fillStyle=color;ctx.strokeStyle='#0d1521';ctx.lineWidth=2;
     ctx.beginPath();ctx.moveTo(12,0);ctx.lineTo(-8,-8);ctx.lineTo(-5,0);ctx.lineTo(-8,8);ctx.closePath();ctx.fill();ctx.stroke();ctx.restore();
     ctx.fillStyle=color;ctx.font='bold 12px system-ui';ctx.fillText(located?'Kenny':'Kenny · stale',x+14,y-10);
   }
   $('mapStatus').textContent = fmt(g.width*g.resolution,1)+' × '+fmt(g.height*g.resolution,1)+' m · '+(mapData.source==='saved_prior'?'Saved walls, obstacles and landmarks':'Live occupancy; landmark locations from selected export')+' · '+(located?estimate.source+' pose shown':estimate?.available?'Last known pose · stale':'Robot position not available yet');
}
function drawScan(ctx,w,h) {
  $('mapName').textContent = 'Live LiDAR'; $('mapSourceBadge').textContent = scanData?.frame || 'laser_frame';
  if (!scanData || scanData.age_s > 2 || !fresh('/scan')) { emptyCanvas(ctx,'Waiting for fresh LiDAR scans',w,h); $('mapStatus').textContent='LiDAR scan unavailable/stale'; return; }
  const radius = Math.min(scanData.range_max || 8,8), scale = Math.min(w,h)*.44/radius*zoom, cx=w/2+pan[0],cy=h/2+pan[1];
  ctx.font='11px system-ui'; ctx.strokeStyle='#2c3d54'; ctx.fillStyle='#71839c';
  for (const r of [1,2,4,8]) if (r<=radius) { ctx.beginPath(); ctx.arc(cx,cy,r*scale,0,2*Math.PI); ctx.stroke(); ctx.fillText(r+' m',cx+4,cy-r*scale+13); }
  ctx.strokeStyle='#415978'; ctx.beginPath(); ctx.moveTo(cx-10,cy);ctx.lineTo(cx+10,cy);ctx.moveTo(cx,cy-10);ctx.lineTo(cx,cy+10);ctx.stroke();
  ctx.fillStyle='#72c7f3'; let valid=0;
  scanData.ranges.forEach((r,i) => { if (!Number.isFinite(r)) return; valid++;const a=scanData.angle_min+i*scanData.angle_increment,x=cx+Math.cos(a)*r*scale,y=cy-Math.sin(a)*r*scale;ctx.fillRect(x-1.5,y-1.5,3,3); });
  $('mapStatus').textContent=valid+' valid returns · sensor coordinates ('+scanData.frame+') · not projected into the world map';
}
function setView(next) { view=next; zoom=1;pan=[0,0];$('showMap').classList.toggle('selected',next==='map');$('showScan').classList.toggle('selected',next==='scan');drawMap(); }
$('showMap').onclick=()=>setView('map');$('showScan').onclick=()=>setView('scan');
$('fitMap').onclick=()=>{zoom=1;pan=[0,0];drawMap();};$('markerLabels').onchange=drawMap;
$('mapSource').onchange=()=>{zoom=1;pan=[0,0];fetchMap(true).catch(error=>{$('mapStatus').textContent=error.message;});};
$('map').addEventListener('wheel',event=>{event.preventDefault();zoom=Math.max(.4,Math.min(12,zoom*Math.exp(-event.deltaY*.001)));drawMap();},{passive:false});
$('map').addEventListener('pointerdown',event=>{drag=[event.clientX,event.clientY,...pan];$('map').setPointerCapture(event.pointerId);$('map').classList.add('dragging');});
$('map').addEventListener('pointermove',event=>{if(drag){pan=[drag[2]+event.clientX-drag[0],drag[3]+event.clientY-drag[1]];drawMap();}});
for(const name of ['pointerup','pointercancel','lostpointercapture'])$('map').addEventListener(name,()=>{drag=null;$('map').classList.remove('dragging');});
function drawMotion(history) {
  const [ctx,w,h]=canvas('movement');ctx.strokeStyle='#384f6c';ctx.beginPath();ctx.moveTo(0,h/2);ctx.lineTo(w,h/2);ctx.stroke();
  for(const [key,color]of[['command','#72b6ff'],['measured','#64d4a9']]){ctx.strokeStyle=color;ctx.beginPath();let start=true;history.forEach((s,i)=>{const v=key==='command'?s.controller?.command?.[0]:s.odom?.v;if(!Number.isFinite(v)){start=true;return;}const x=i*w/Math.max(119,history.length-1),y=h/2-v*h/.4;start?ctx.moveTo(x,y):ctx.lineTo(x,y);start=false;});ctx.stroke();}
}
function disconnected() {
  $('connection').textContent='Disconnected';$('connection').className='connection bad';$('reset').disabled=$('bench').disabled=$('bench75').disabled=$('forward75').disabled=true;
  $('stopState').textContent='Connection lost';$('stopDetail').textContent='STOP cannot be confirmed. Use the physical motor-power stop if needed.';$('banner').className='banner danger';
  for(const name of ['color','depth']){$(name+'Preview').hidden=true;$(name+'Empty').hidden=false;$(name+'Empty').textContent='Connection lost';}$('arucoOverlay').replaceChildren();
}
async function poll() {
  try {
    const response=await fetch('/api/status'+(page==='diagnostics'?'?detail=1':''),{signal:AbortSignal.timeout(3000)});if(!response.ok)throw Error(response.status);
    render(await response.json());await fetchMap();
    if(view==='scan'&&page==='overview'){const r=await fetch('/api/scan',{signal:AbortSignal.timeout(2000)});if(r.ok){scanData=await r.json();drawMap();}}
  } catch(error) { if(Date.now()-lastSuccess>3000)disconnected(); }
  finally { setTimeout(poll,750); }
}
setInterval(()=>{if(lastSuccess&&Date.now()-lastSuccess>3500)disconnected();},750);
window.addEventListener('resize',()=>{drawMap();if(page==='diagnostics')renderDetails(true);});
poll();
