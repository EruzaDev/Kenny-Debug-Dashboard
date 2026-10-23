"""Local operations UI and fail-closed stop heartbeat. Access over SSH on 8002."""
import argparse
from collections import deque
import glob
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import logging
from logging.handlers import RotatingFileHandler
import math
import os
from pathlib import Path
from urllib.error import HTTPError
import shutil
import subprocess
import threading
import time
import uuid
from controller.wait_camera import devices as camera_usb_devices
from controller.dashboard_data import ArucoHistory


def retry_sensor(sensor):
    """Queue a restart of a fixed sensor unit, including a failed/stopped unit."""
    units = {'lidar': 'kenny-lidar.service', 'camera': 'kenny-sensors.service', 'imu': 'kenny-imu.service'}
    if sensor not in units:
        raise ValueError('Unknown sensor')
    unit = units[sensor]
    try:
        actions = [['reset-failed', unit]]
        if sensor == 'imu':
            actions.append(['enable', unit])
        actions.append(['restart', '--no-block', unit])
        for args in actions:
            result = subprocess.run(['systemctl', '--user', *args],
                                    capture_output=True, text=True, timeout=2.)
            if result.returncode:
                raise ValueError(result.stderr.strip() or f'Cannot restart {unit}')
    except (OSError, subprocess.TimeoutExpired) as error:
        raise ValueError(f'Sensor retry failed: {error}') from error


def debug_service(action):
    unit = 'kenny-controller-debug.service'
    result = subprocess.run(['systemctl', '--user', action, unit], capture_output=True, text=True, timeout=20)
    if result.returncode:
        raise ValueError(result.stderr.strip() or f'Cannot {action} {unit}')


MOTOR_PORTS_FILE = Path(__file__).resolve().parents[1] / 'data/motor-ports.env'


def motor_port_options():
    return sorted(p for p in glob.glob('/dev/serial/by-path/platform-*-usb-*-port0')
                  if Path(p).exists())


def motor_port_settings():
    import shlex
    values = {}
    if MOTOR_PORTS_FILE.exists():
        for line in MOTOR_PORTS_FILE.read_text().splitlines():
            parts = shlex.split(line)
            if len(parts) == 2 and parts[0] == 'export' and '=' in parts[1]:
                key, value = parts[1].split('=', 1)
                if key in ('KENNY_LEFT_MOTOR_PORT', 'KENNY_RIGHT_MOTOR_PORT'):
                    values[key] = value
    return {'ports': motor_port_options(), 'left': values.get('KENNY_LEFT_MOTOR_PORT'),
            'right': values.get('KENNY_RIGHT_MOTOR_PORT')}


def assign_motor_ports(left, right):
    from concurrent.futures import ThreadPoolExecutor
    from controller.deployment import service
    from controller.dual_motors import Board
    options = motor_port_options()
    if not isinstance(left, str) or not isinstance(right, str) or left == right or left not in options or right not in options:
        raise ValueError('Select two different connected USB ports')
    current = motor_port_settings()
    units = ('kenny-motors.service', 'kenny-controller-debug.service')
    running = {unit: subprocess.run(['systemctl', '--user', 'is-active', '--quiet', unit], timeout=2).returncode == 0
               for unit in units}
    if current['left'] == left and current['right'] == right:
        return current
    for unit in units:
        if running[unit]:
            service('stop', unit)
    # A firmware handshake prevents accidentally assigning the LiDAR's CP2102.
    def verify(path):
        board = Board(path)
        try:
            if not board.protocol.startswith('KENNY_SIDE_V3 '):
                raise ValueError(f'V3 motor firmware not found on {path}')
        finally:
            board.close()
    with ThreadPoolExecutor(max_workers=2) as workers:
        list(workers.map(verify, (left, right)))
    temporary = MOTOR_PORTS_FILE.with_suffix('.env.tmp')
    temporary.write_text(f'export KENNY_LEFT_MOTOR_PORT={left}\nexport KENNY_RIGHT_MOTOR_PORT={right}\n')
    os.replace(temporary, MOTOR_PORTS_FILE)
    for unit in units:
        if running[unit]:
            service('start', unit)
    return motor_port_settings()


def submit_manual_request(store, marker_id, request_id, cloud=None):
    if type(marker_id) is not int or type(request_id) is not str:
        raise ValueError('Choose a mapped ArUco ID and provide a request ID')
    try:
        request_id = str(uuid.UUID(request_id))
    except ValueError as error:
        raise ValueError('Invalid manual request ID') from error
    with store.lock:
        environment = store.prior or {}
        if not any(m.get('id') == marker_id for m in environment.get('markers', [])):
            raise ValueError('Choose an ArUco ID from the selected map')
        dictionary = environment['dictionary']
    if cloud is None:
        from controller.deployment import api
        cloud = api()
    job = cloud.post('/api/calls', {'request_id': request_id, 'marker_id': marker_id,
                                     'dictionary': dictionary, 'caller': 'operations-dashboard'})
    if job.get('id') and job.get('marker_id') == marker_id:
        store.request_event({'time': time.time(), 'stage': 'manual_request',
            'detail': f"Manual ArUco {marker_id} request {job.get('status', 'submitted')}",
            'job_id': job['id'], 'marker_id': marker_id})
    return job


def cancel_queued_request(store, job_id, cloud=None):
    if type(job_id) is not str:
        raise ValueError('Choose a queued request ID')
    try:
        job_id = str(uuid.UUID(job_id))
    except ValueError as error:
        raise ValueError('Invalid queued request ID') from error
    if cloud is None:
        from controller.deployment import api
        cloud = api()
    try:
        job = cloud.post('/api/pi/cancel', {'job_id': job_id})
    except HTTPError as error:
        if error.code == 404:
            try:
                detail = json.loads(error.read(16384)).get('detail')
            except (ValueError, OSError):
                detail = None
            raise ValueError('Cloud queue cancellation endpoint is not deployed yet' if detail == 'Not Found'
                             else detail or 'Queued request not found') from error
        raise
    if job.get('id') != job_id or job.get('status') != 'cancelled':
        raise ValueError('Cloud did not confirm queued request cancellation')
    with store.lock:
        previous = store.packets.get('requests', {}).get('data', {})
        if 'jobs' in previous:
            store.update('requests', {**previous, 'jobs': [j for j in previous['jobs'] if j.get('id') != job_id],
                                      'polled_at': time.time()})
        store.request_event({'time': time.time(), 'stage': 'cancelled',
            'detail': f"Queued ArUco {job['marker_id']} request cancelled on Render",
            'job_id': job_id, 'marker_id': job['marker_id']})
    return job


class Diagnostics:
    def __init__(self, log_path, environment=None):
        self.lock = threading.RLock()
        self.packets = {}
        self.events = deque(maxlen=300)
        self.samples = deque(maxlen=120)
        self.request_events = deque(maxlen=200)
        self.observed_queue = {}
        self.last_pose = None
        self.estop = True
        self.startup = None
        self.bench_until = 0.
        self.bench_duty = 0.
        self.bench_wheel = None
        self.bench_before = None
        self.bench_result = None
        self.calibration = {'hand': {}, 'pulses': {}, 'ground': None}
        self.hand_baseline = None
        self.ground_pulse = False
        self.supervised_until = 0.
        self.stop_at = time.monotonic()
        self.stopped_samples = 0
        self.debug_stopped_samples = 0
        self.last_odom = None
        self.graph = []
        self.grid = None
        self.prior = environment
        self.aruco = ArucoHistory()
        self.scan = None
        self.images = {}
        self.revision = 0
        self.deployment = {'mode': 'simulation', 'map_sha256': None}
        self.deployment_lock = threading.Lock()
        self.last_heartbeat = time.monotonic()
        self.session_id = time.time_ns()
        self.logger = logging.getLogger('kenny.operations')
        self.logger.setLevel(logging.INFO)
        log_path.parent.mkdir(parents=True, exist_ok=True)
        # Restore recent scan events, but never restore them as live sightings.
        try:
            with log_path.open('rb') as previous_log:
                previous_log.seek(max(0, log_path.stat().st_size - 256_000))
                for line in previous_log.read().splitlines():
                    if b'"source": "request"' in line:
                        try:
                            restored = json.loads(json.loads(line)['message'])
                            if isinstance(restored, dict) and 'stage' in restored:
                                self.request_events.append(restored)
                        except (ValueError, KeyError, TypeError):
                            pass
                    if b'"source": "aruco"' not in line:
                        continue
                    try:
                        event = json.loads(json.loads(line)['message'])
                        if isinstance(event, dict) and type(event.get('id')) is int and 0 <= event['id'] < 1000 and math.isfinite(event.get('time', math.nan)):
                            self.aruco.events.append(event)
                    except (ValueError, KeyError, TypeError):
                        continue
        except OSError:
            pass
        handler = RotatingFileHandler(log_path, maxBytes=5_000_000, backupCount=4)
        handler.setFormatter(logging.Formatter('%(message)s'))
        self.logger.addHandler(handler)
        self.event('dashboard', 'Dashboard started; emergency stop latched')

    def event(self, source, message, level='INFO'):
        with self.lock:
            event = {'time': time.time(), 'source': source, 'level': level, 'message': message}
            self.events.append(event)
            self.logger.info(json.dumps(event, allow_nan=False))

    def update(self, name, value):
        with self.lock:
            previous_packet = self.packets.get(name)
            if name == 'odom' and previous_packet and time.monotonic() - previous_packet['received'] > .5:
                self.stopped_samples = 0
            previous = self.packets.get(name, {}).get('data')
            self.packets[name] = {'received': time.monotonic(), 'data': value}
            if name == 'motors' and value.get('bridge', {}).get('mode') in ('dual_side_debug', 'dual_side_robot'):
                still = (value.get('ready') and not value.get('armed') and len(value.get('counts', [])) == 4
                         and previous and previous.get('boot') == value.get('boot')
                         and previous.get('counts') == value.get('counts'))
                self.debug_stopped_samples = self.debug_stopped_samples + 1 if still else 0
                if self.estop and self.debug_stopped_samples >= 3 and self.bench_result and not self.bench_result.get('disarmed_confirmed'):
                    after = list(value['counts'])
                    self.bench_result.update(after=after, disarmed_confirmed=True,
                        delta_LF_RF_LR_RR=[(int(a)-int(b)+2**31) % 2**32-2**31
                                            for a,b in zip(after,self.bench_result['before'])])
                    if self.bench_result['wheel'] is not None:
                        name = ('LF', 'RF', 'LR', 'RR')[self.bench_result['wheel']]
                        wheel = self.bench_result['wheel']
                        change = self.bench_result['delta_LF_RF_LR_RR']
                        hand = self.calibration['hand'].get(name)
                        self.bench_result['estimated_revolutions'] = (round(change[wheel] / hand['cpr'], 3)
                            if hand and hand['cpr'] > 0 else None)
                        self.bench_result['pwm_signal_ok'] = (change[wheel] > 0 and
                            all(abs(v) <= max(5, abs(change[wheel]) * .02)
                                for i, v in enumerate(change) if i != wheel))
                        self.calibration['pulses'][name] = dict(self.bench_result)
            if name == 'controller':
                keys = ('reason', 'guard', 'status', 'marker_id', 'emergency_stop')
                if previous is None or any(previous.get(k) != value.get(k) for k in keys):
                    self.event(name, json.dumps({k: value.get(k) for k in keys}))
                    mission = value.get('mission') or {}
                    if mission.get('job_id') and (previous is None or previous.get('guard') != value.get('guard')):
                        self.request_event({'time': time.time(), 'stage': 'guard' if value.get('guard') else 'controller',
                            'detail': value.get('guard') or value.get('status') or value.get('reason'),
                            'job_id': mission['job_id'], 'marker_id': value.get('marker_id') or mission.get('marker_id')})
            if name == 'mission_event':
                self.request_event(value)
            if name == 'aruco':
                for event in self.aruco.update(value):
                    self.event('aruco', json.dumps(event))
            if name == 'odom':
                stamp = value['stamp']
                if stamp != self.last_odom:
                    self.last_odom = stamp
                    stationary = abs(value['v']) < .01 and abs(value['w']) < .02
                    self.stopped_samples = self.stopped_samples + 1 if stationary and value['stamp_age_s'] <= .5 else 0
                    if self.estop and self.stopped_samples >= 3 and self.bench_result and not self.bench_result.get('stopping_confirmed'):
                        motor = self.packets.get('motors')
                        if motor and time.monotonic() - motor['received'] <= .5 and len(motor['data'].get('counts', [])) == 4:
                            after = list(motor['data']['counts'])
                            self.bench_result.update(after=after, stopping_confirmed=True,
                                delta_LF_RF_LR_RR=[(int(a) - int(b) + 2**31) % 2**32 - 2**31
                                    for a, b in zip(after, self.bench_result['before'])])
                            self.event('bench', 'Stopping confirmed; final encoder result: ' + json.dumps(self.bench_result))

    def stop(self, persist=True):
        with self.lock:
            self.estop = True
            if persist and self.startup is not None:
                self.startup.stop()
            if self.bench_until:
                after = self.packets.get('motors', {}).get('data', {}).get('counts')
                self.bench_result = {'before': self.bench_before, 'after': after,
                    'wheel': self.bench_wheel, 'duty': self.bench_duty,
                    'after_at_command_stop': after, 'stopping_confirmed': False,
                    'delta_LF_RF_LR_RR': [int(a) - int(b) for a, b in zip(after, self.bench_before)] if after else None}
                self.event('bench', 'Wheel test ended: ' + json.dumps(self.bench_result))
                self.bench_until = 0.
                self.bench_wheel = None
            self.estop = True
            self.stop_at = time.monotonic()
            self.stopped_samples = 0
            self.debug_stopped_samples = 0
            self.supervised_until = 0.
            self.event('operator', 'EMERGENCY STOP requested', 'WARN')

    def request_event(self, event):
        if not isinstance(event.get('detail'), str) or not isinstance(event.get('stage'), str):
            return
        self.request_events.append(event)
        self.event('request', json.dumps(event, allow_nan=False))

    def observe_queue(self, jobs):
        """Show pending cloud requests even when navigation is intentionally paused."""
        if not isinstance(jobs, list):
            raise ValueError('Invalid cloud queue')
        with self.lock:
            for job in jobs:
                if (not isinstance(job, dict) or not isinstance(job.get('id'), str)
                        or type(job.get('marker_id')) is not int or not isinstance(job.get('status'), str)):
                    raise ValueError('Invalid cloud request')
            visible = [{'id': j['id'], 'marker_id': j['marker_id'], 'status': j['status']} for j in jobs]
            self.update('requests', {'jobs': visible, 'polled_at': time.time(), 'source': 'cloud_queue_observer'})
            for job in visible:
                if self.observed_queue.get(job['id']) == job['status']:
                    continue
                self.observed_queue[job['id']] = job['status']
                self.request_event({'time': time.time(), 'stage': 'received' if job['status'] == 'queued' else 'queue_update',
                    'detail': f"ArUco {job['marker_id']} request {job['status']} on Render; navigation paused",
                    'job_id': job['id'], 'marker_id': job['marker_id']})
            if len(self.observed_queue) > 500:
                self.observed_queue = {j['id']: j['status'] for j in visible}

    def pose_estimate(self):
        now = time.monotonic()
        candidates = []
        for name, source in (('tf', 'SLAM + encoders'), ('controller', 'Controller map pose'), ('aruco', 'Surveyed ArUco')):
            packet = self.packets.get(name)
            if not packet:
                continue
            value = packet['data']
            if name=='tf':source=value.get('pose_source',source)
            if name == 'aruco':
                estimate = value.get('robot_pose') or {}
                pose = estimate.get('pose')
                observed_at = value.get('observed_at')
                age = time.time() - observed_at if type(observed_at) in (int, float) and math.isfinite(observed_at) else math.inf
                marker_ids = estimate.get('marker_ids', [])
                uncertainty = None
            else:
                pose = value.get('pose')
                sample_age = value.get('stamp_age_s') if name == 'tf' else value.get('ages_s', {}).get('pose')
                age = now - packet['received'] + sample_age if sample_age is not None else math.inf
                marker_ids = []
                uncertainty = value.get('uncertainty_m') if name == 'controller' else self.packets.get('controller', {}).get('data', {}).get('uncertainty_m')
            if isinstance(pose, (list, tuple)) and len(pose) == 3 and all(type(v) in (int, float) and math.isfinite(v) for v in pose) and -.1 <= age <= .5:
                candidates.append({'pose': list(pose), 'source': source, 'observed': now - age,
                                   'marker_ids': marker_ids, 'uncertainty_m': uncertainty})
        if candidates:
            self.last_pose = candidates[0]
        result = {'available': self.last_pose is not None, 'live': bool(candidates)}
        if self.last_pose:
            result.update({k: v for k, v in self.last_pose.items() if k != 'observed'},
                          age_s=round(max(0., now - self.last_pose['observed']), 3))
        aruco = self.packets.get('aruco', {}).get('data', {})
        controller = self.packets.get('controller', {}).get('data', {})
        result['reason'] = aruco.get('localization_error') or controller.get('last_tf_error') or 'Waiting for surveyed ArUco localization or live map TF'
        return result

    def reset(self, bench=False):
        with self.lock:
            if not self.estop:
                raise ValueError('Stop must be latched before reset')
            now = time.monotonic()
            dual_mode = self.packets.get('motors', {}).get('data', {}).get('bridge', {}).get('mode')
            debug = dual_mode == 'dual_side_debug'
            dual_robot = dual_mode == 'dual_side_robot'
            for name in ('controller', 'motors', 'odom'):
                if (debug and name != 'motors') or (dual_robot and name == 'odom') or (name == 'controller' and bench):
                    continue
                packet = self.packets.get(name)
                limit = 1.5 if name in ('odom', 'motors') else 3.
                if packet is None or now - packet['received'] > limit:
                    raise ValueError(f'Cannot reset: {name} unavailable/stale')
            motor = self.packets['motors']['data']
            if not motor.get('bridge', {}).get('connected') or motor.get('faults') or motor.get('estop'):
                raise ValueError('Cannot reset: motor bridge disconnected or firmware fault/estop active')
            if dual_robot or debug:
                if not self.motor_disarmed(motor):
                    raise ValueError('Cannot reset: both ESP32s must acknowledge STOP with zero PWM')
            elif self.stopped_samples < 3:
                raise ValueError('Cannot reset: three fresh stationary encoder samples required')
            if not debug and ((not bench and not self.packets['controller']['data'].get('emergency_stop')) or not motor['bridge'].get('stop_reason')):
                raise ValueError('Cannot reset: stop has not been acknowledged by both controller and motor bridge')
            if self.startup is not None:
                self.startup.released()
            self.estop = False
            self.event('operator', 'Stop reset for bounded raised-wheel test' if bench else 'Stop reset; pending missions may resume')

    @staticmethod
    def motor_disarmed(motor):
        return (motor.get('ready') is True and motor.get('armed') is False
                and motor.get('bridge', {}).get('connected') is True
                and bool(motor['bridge'].get('stop_reason'))
                and isinstance(motor.get('pwm'), list) and len(motor['pwm']) == 4
                and all(value == 0 for value in motor['pwm']))

    def startup_tick(self, now):
        with self.lock:
            if self.startup is None or not self.estop:
                return
            controller = self.packets.get('controller', {})
            motors = self.packets.get('motors', {})
            odom = self.packets.get('odom', {})
            healthy = (now-controller.get('received', -math.inf) < 1.5
                       and controller.get('data', {}).get('ready_without_stop') is True
                       and now-motors.get('received', -math.inf) < .5
                       and now-odom.get('received', -math.inf) < .5
                       and motors.get('data', {}).get('bridge', {}).get('mode') == 'dual_side_robot'
                       and self.motor_disarmed(motors.get('data', {})))
            enabled = self.deployment.get('auto_start') is True and self.deployment.get('mode') == 'robot'
            if self.startup.ready(now, enabled, healthy):
                try:
                    self.reset()
                    self.event('startup', 'Startup checks passed; robot ready for queued missions')
                except ValueError as error:
                    self.startup.since = None
                    self.startup.reason = str(error)

    def start_supervised(self):
        with self.lock:
            if self.deployment.get('mode') != 'robot' or self.deployment.get('drive_mode') != 'velocity':
                raise ValueError('Supervised run requires continuous Robot mode')
            for name, limit in (('motors', .5), ('controller', 2.), ('/scan', .5),
                                ('/camera/depth/image_raw', .5), ('camera_preview/depth', 1.)):
                packet = self.packets.get(name)
                if packet is None or time.monotonic()-packet['received'] > limit:
                    raise ValueError(f'Supervised run requires fresh {name}')
            scan = self.packets['/scan']['data'].get('minimum_m')
            depth = self.packets['camera_preview/depth']['data'].get('minimum_m')
            if scan is None or depth is None or min(scan, depth) < .3:
                raise ValueError('Measured clearance under 0.30 m; supervised run blocked')
            self.reset()
            self.supervised_until = time.monotonic() + 15.
            self.event('operator', 'Supervised continuous run enabled for 15 s; live 0.30 m cutoff active', 'WARN')

    def expire_supervised(self, now=None):
        now = time.monotonic() if now is None else now
        with self.lock:
            if not self.supervised_until or now < self.supervised_until:
                return False
            self.stop()
            self.event('operator', 'Supervised run window ended; STOP relatched')
            return True

    def start_bench(self, boost=False, ground=False, wheel=None):
        with self.lock:
            if wheel is not None and (type(wheel) is not int or wheel not in range(4) or not boost or ground):
                raise ValueError('Select one raised wheel for a bounded 75% PWM pulse')
            if ground and not boost:
                raise ValueError('Ground commissioning supports only the bounded 75% pulse')
            motor = self.packets.get('motors', {}).get('data', {})
            if (not motor.get('encodersEnabled') or not motor.get('ready') or motor.get('cpr', 0) <= 0
                    or len(motor.get('counts', [])) != 4 or any(type(v) is not int for v in motor['counts'])):
                raise ValueError('Bench test requires ready hardware and four real encoder counters')
            dual_watchdog = (motor.get('bridge', {}).get('mode') == 'dual_side_robot'
                             and motor.get('watchdog') == 200)
            if boost and 'benchActive' not in motor and not dual_watchdog:
                raise ValueError('Bounded PWM pulse requires bench firmware or two ESP32s with a 200 ms watchdog')
            if ground:
                for name in ('/scan', '/camera/depth/image_raw'):
                    packet = self.packets.get(name)
                    if packet is None or time.monotonic()-packet['received'] > .5:
                        raise ValueError('Ground pulse requires fresh LiDAR and depth measurements')
                minimum = self.packets['/scan']['data'].get('minimum_m')
                if minimum is not None and minimum < .3:
                    raise ValueError('Measured LiDAR obstacle is within 0.30 m; pulse blocked')
            self.reset(bench=True)
            self.bench_before = list(motor['counts'])
            self.bench_result = None
            self.bench_duty = .75 if boost else 0.
            self.bench_wheel = wheel
            self.ground_pulse = ground
            self.bench_until = time.monotonic() + (.45 if boost else 3.)
            self.event('bench', f'Raised wheel {("LF", "RF", "LR", "RR")[wheel]}: 75% PWM, host deadline 0.45 s, ESP32 command watchdog 0.2 s' if wheel is not None else
                'Real robot forward pulse: 75% PWM, host deadline 0.45 s, ESP32 command watchdog 0.2 s' if ground else
                'Raised-wheel forward pulse: 75% PWM, host deadline 0.45 s, ESP32 command watchdog 0.2 s' if boost else
                'Raised-wheel forward test: 0.05 m/s, maximum 3 seconds', 'WARN')

    def calibration_counts(self):
        packet = self.packets.get('motors')
        if (packet is None or time.monotonic() - packet['received'] > .5 or
                packet['data'].get('bridge', {}).get('mode') != 'dual_side_robot' or
                not self.estop or not self.motor_disarmed(packet['data']) or
                len(packet['data'].get('counts', [])) != 4):
            raise ValueError('Calibration needs fresh disarmed two-board motor telemetry and STOP latched')
        return packet['data']

    def calibration_hand(self, wheel, revolutions=None):
        from controller.encoder_calibration import delta
        if type(wheel) is not int or wheel not in range(4):
            raise ValueError('Select LF, RF, LR or RR')
        with self.lock:
            motor = self.calibration_counts()
            if revolutions is None:
                self.hand_baseline = (wheel, list(motor['counts']), motor.get('encoderInvalidTransitions'))
                return {'wheel': wheel, 'baseline': self.hand_baseline[1]}
            if type(revolutions) not in (float, int) or not math.isfinite(revolutions) or not 0 < revolutions <= 20:
                raise ValueError('Specify the measured number of forward wheel revolutions (0–20)')
            if self.hand_baseline is None or self.hand_baseline[0] != wheel:
                raise ValueError('Capture the starting counts for this wheel first')
            _, before, invalid_before = self.hand_baseline
            changes = [delta(a, b) for a, b in zip(motor['counts'], before)]
            invalid_after = motor.get('encoderInvalidTransitions')
            invalid = ([delta(a, b) for a, b in zip(invalid_after, invalid_before)]
                       if invalid_before is not None and invalid_after is not None else None)
            result = {'wheel': wheel, 'revolutions': revolutions, 'counts_delta': changes,
                      'invalid_delta': invalid, 'cpr': round(changes[wheel] / revolutions, 2),
                      'signal_ok': changes[wheel] > 0 and
                          all(abs(v) <= max(5, abs(changes[wheel]) * .02) for i, v in enumerate(changes) if i != wheel)
                          and (invalid is None or invalid[wheel] == 0)}
            self.calibration['hand'][('LF', 'RF', 'LR', 'RR')[wheel]] = result
            self.hand_baseline = None
            return result

    def calibration_ground(self, distance):
        if type(distance) not in (float, int) or not math.isfinite(distance) or not 0 < distance <= 2:
            raise ValueError('Enter measured forward chassis travel in meters (0–2)')
        with self.lock:
            self.calibration_counts()
            result = self.bench_result
            if (not self.ground_pulse or not result or not result.get('disarmed_confirmed') or
                    result.get('wheel') is not None or result.get('duty') != .75 or
                    not result.get('delta_LF_RF_LR_RR')):
                raise ValueError('Complete a ground pulse and wait for disarmed encoder feedback first')
            radii = {}
            for i, name in enumerate(('LF', 'RF', 'LR', 'RR')):
                hand = self.calibration['hand'].get(name)
                counts = result['delta_LF_RF_LR_RR'][i]
                radii[name] = (round(distance * hand['cpr'] / (2 * math.pi * counts), 5)
                               if hand and hand['signal_ok'] and counts > 0 else None)
            self.calibration['ground'] = {'distance_m': distance,
                                          'counts_delta': result['delta_LF_RF_LR_RR'],
                                          'radius_estimate_m': radii,
                                          'note': 'Short PWM pulses include acceleration and coast; repeat a measured longer manual roll for precise radius.'}
            return self.calibration['ground']

    def snapshot(self, compact=False):
        with self.lock:
            now = time.monotonic()
            odom = self.packets.get('odom')
            confirmed = bool(self.estop and self.stopped_samples >= 3 and odom and now - odom['received'] <= .5)
            motor = self.packets.get('motors')
            motor_stop_confirmed = bool(self.estop and motor and now - motor['received'] <= .5
                                        and motor['data'].get('bridge', {}).get('mode') == 'dual_side_robot'
                                        and self.motor_disarmed(motor['data']))
            debug_confirmed = bool(self.estop and motor and now - motor['received'] <= .5 and
                                   (motor['data'].get('bridge', {}).get('mode') == 'dual_side_debug' and self.motor_disarmed(motor['data'])
                                    or motor['data'].get('bridge', {}).get('mode') == 'dual_side_robot' and self.debug_stopped_samples >= 3))
            result = {'time': time.time(), 'session_id': self.session_id, 'emergency_stop': self.estop, 'physical_stop_confirmed': confirmed,
                     'supervised_remaining_s': round(max(0., self.supervised_until-now), 2),
                     'debug_stop_confirmed': debug_confirmed, 'motor_stop_confirmed': motor_stop_confirmed,
                    'bench_running': bool(self.bench_until), 'bench_result': self.bench_result,
                     'bench_duty': self.bench_duty,
                     'bench_wheel': self.bench_wheel,
                     'wheel_calibration': self.calibration,
                    'ground_pulse': self.ground_pulse,
                    'packets': {k: {'age_s': round(now - v['received'], 3), 'data': v['data']} for k, v in self.packets.items()},
                    'events': list(self.events), 'history': list(self.samples), 'topics': self.graph,
                    'aruco': self.aruco.snapshot(), 'map_summary': self.map_summary(),
                    'map_revision': self.revision, 'deployment': dict(self.deployment)}
            result.update(pose_estimate=self.pose_estimate(), request_events=list(self.request_events))
            if compact:
                result.update(events=[], history=[], topics=[t for t in self.graph if t['name'] == '/kenny/controller/status'])
                host = result['packets'].get('host')
                if host:
                    host['data'] = {k: v for k, v in host['data'].items() if k != 'service_logs'}
            return result

    def map_packet(self, prior=False):
        environment = self.prior or {}
        live = self.grid is not None and not prior
        grid = self.grid if live else environment.get('grid')
        return {'revision': self.revision, 'grid': grid,
                'source': 'live_refined' if live else 'saved_prior' if grid else 'unavailable',
                'name': environment.get('name', 'Kenny map'), 'dictionary': environment.get('dictionary'),
                'markers': environment.get('markers', []), 'destination_marker_id': environment.get('destination_marker_id'),
                'obstacles': [o for o in environment.get('obstacles', []) if not live or o['kind'] == 'prohibited'],
                'goal': environment.get('goal'), 'frame': 'map'}

    def map_summary(self):
        packet = self.map_packet()
        return {k: packet[k] for k in ('revision', 'source', 'name', 'dictionary')} | {
            'available': packet['grid'] is not None, 'marker_count': len(packet['markers'])}


def make_node(store):
    import rclpy
    from rclpy.node import Node
    from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
    from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
    from std_msgs.msg import Bool, String, Float32MultiArray, Float32, Int8
    from geometry_msgs.msg import TwistStamped
    from nav_msgs.msg import Odometry, OccupancyGrid, Path as Route
    from sensor_msgs.msg import LaserScan, PointCloud2, Image, CameraInfo, Imu
    from rcl_interfaces.msg import Log
    from visualization_msgs.msg import MarkerArray
    from tf2_ros import Buffer, TransformListener, TransformException

    class Dashboard(Node):
        def __init__(self):
            super().__init__('kenny_operations_dashboard')
            self.heartbeat_callbacks = MutuallyExclusiveCallbackGroup()
            qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.stop_pub = self.create_publisher(Bool, '/kenny/emergency_stop', qos)
            self.bench_command_pub = self.create_publisher(TwistStamped, '/kenny/cmd_vel', 10)
            self.bench_allow_pub = self.create_publisher(Bool, '/kenny/motion_allowed', 10)
            self.supervised_pub = self.create_publisher(Bool, '/kenny/supervised_clearance', 10)
            self.bench_pwm_pub = self.create_publisher(Float32, '/kenny/bench_pwm', 10)
            self.calibration_wheel_pub = self.create_publisher(Int8, '/kenny/calibration/wheel', 10)
            self.buffer = Buffer()
            self.listener = TransformListener(self.buffer, self)
            self.image_jobs = {}
            self.image_last = {}
            threading.Thread(target=self.image_worker, daemon=True).start()
            for name, topic in [('controller', '/kenny/controller/status'), ('motors', '/kenny/motors/status'), ('aruco', '/kenny/aruco/status'),
                                ('mission_event', '/kenny/mission/events'), ('requests', '/kenny/requests/status'), ('imu_status', '/kenny/imu/status')]:
                self.create_subscription(String, topic, lambda msg, n=name: self.json_packet(n, msg), 10)
            self.create_subscription(String, '/kenny/editor/placement_reference',lambda msg:self.json_packet('placement_reference',msg),10)
            self.create_subscription(Odometry, '/odom', self.odom, qos_profile_sensor_data)
            self.create_subscription(TwistStamped, '/kenny/cmd_vel', lambda m: store.update('command',
                {'v': m.twist.linear.x, 'w': m.twist.angular.z, 'stamp_age_s': self.age(m.header.stamp)}), 10)
            self.create_subscription(Bool, '/kenny/motion_allowed', lambda m: store.update('permission', {'allowed': m.data}), 10)
            self.create_subscription(OccupancyGrid, '/kenny/map', self.map, qos)
            self.create_subscription(Route, '/kenny/route', lambda m: store.update('route',
                {'frame': m.header.frame_id, 'points': [[p.pose.position.x, p.pose.position.y] for p in m.poses]}), 1)
            self.create_subscription(Log, '/rosout', lambda m: store.event(m.name, m.msg, str(m.level)), 100)
            for cls, topic in [(LaserScan, '/scan'), (PointCloud2, '/camera/depth/points'),
                               (Image, '/camera/color/image_raw'), (Image, '/camera/depth/image_raw'),
                               (CameraInfo, '/camera/color/camera_info'), (CameraInfo, '/camera/depth/camera_info'),
                                (Float32MultiArray, '/kenny/downward'), (MarkerArray, '/kenny/marker_observations'), (Imu, '/imu/data_raw')]:
                self.create_subscription(cls, topic, lambda m, t=topic: self.sensor_packet(t, m), qos_profile_sensor_data)
            self.create_timer(.1, self.heartbeat, callback_group=self.heartbeat_callbacks)
            self.create_timer(1., self.inspect)

        def age(self, stamp):
            return self.get_clock().now().nanoseconds / 1e9 - stamp.sec - stamp.nanosec / 1e9

        def sensor_packet(self, topic, msg):
            store.update(topic, self.summary(msg))
            if isinstance(msg, LaserScan):
                stride = max(1, math.ceil(len(msg.ranges) / 720))
                with store.lock:
                    store.scan = {'received': time.monotonic(), 'frame': msg.header.frame_id,
                        'angle_min': msg.angle_min, 'angle_increment': msg.angle_increment * stride,
                        'range_max': msg.range_max, 'stamp_age_s': self.age(msg.header.stamp),
                        'ranges': [float(v) if math.isfinite(v) and msg.range_min <= v <= msg.range_max else None
                                   for v in msg.ranges[::stride]]}
            if isinstance(msg, Image):
                name = 'depth' if '/depth/' in topic else 'color'
                now = time.monotonic()
                if now - self.image_last.get(name, 0) >= .5 and -.1 <= self.age(msg.header.stamp) <= .5:
                    self.image_last[name] = now
                    with store.lock:
                        self.image_jobs[name] = (msg, now)

        def image_worker(self):
            # Conversion stays off the stop-heartbeat/sensor executor.
            from controller.camera_preview import preview
            while rclpy.ok():
                with store.lock:
                    jobs, self.image_jobs = self.image_jobs, {}
                for name, (msg, received) in jobs.items():
                    try:
                        jpeg, metadata = preview(msg, depth=name == 'depth')
                        with store.lock:
                            store.images[name] = (received, jpeg)
                        store.update('camera_preview/' + name, metadata)
                    except (ValueError, TypeError, BufferError) as error:
                        store.update('camera_preview/' + name, {'error': str(error)})
                time.sleep(.05)

        def summary(self, msg):
            result = {}
            if hasattr(msg, 'header'):
                result.update(frame=msg.header.frame_id, stamp_age_s=self.age(msg.header.stamp))
            if hasattr(msg, 'ranges'):
                good = [r for r in msg.ranges if math.isfinite(r) and msg.range_min <= r <= msg.range_max]
                result.update(samples=len(msg.ranges), valid=len(good), minimum_m=min(good) if good else None)
            if hasattr(msg, 'width'):
                result.update(width=msg.width, height=msg.height)
            if hasattr(msg, 'markers'):
                result['ids'] = [m.id for m in msg.markers]
            if isinstance(msg, Float32MultiArray):
                result['values'] = [v if math.isfinite(v) else None for v in msg.data]
            return result

        def json_packet(self, name, msg):
            try:
                value = json.loads(msg.data)
                if not isinstance(value, dict):
                    raise ValueError('Expected object')
                json.dumps(value, allow_nan=False)
                store.update(name, value)
            except (ValueError, TypeError) as error:
                store.event(name, f'Invalid status JSON: {error}', 'ERROR')

        def odom(self, msg):
            values = [msg.twist.twist.linear.x, msg.twist.twist.angular.z]
            age = self.age(msg.header.stamp)
            if not all(math.isfinite(v) for v in values) or not -.1 <= age <= .5:
                return
            if msg.header.frame_id != 'odom' or msg.child_frame_id != 'base_link':
                return
            store.update('odom', {'v': values[0], 'w': values[1], 'stamp_age_s': age,
                                 'stamp': [msg.header.stamp.sec, msg.header.stamp.nanosec]})

        def map(self, msg):
            if msg.header.frame_id != 'map' or msg.info.width * msg.info.height > 4_000_000:
                return
            p, q = msg.info.origin.position, msg.info.origin.orientation
            with store.lock:
                store.grid = {'width': msg.info.width, 'height': msg.info.height, 'resolution': msg.info.resolution,
                              'origin': [p.x, p.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))], 'data': list(msg.data)}
                store.revision += 1
            store.update('map', {'frame': msg.header.frame_id, 'stamp_age_s': self.age(msg.header.stamp)})

        def heartbeat(self):
            with store.lock:
                now = time.monotonic()
                bench = bool(store.bench_until)
                if bench and now >= store.bench_until:
                    store.stop()
                store.expire_supervised(now)
                if bench and store.ground_pulse:
                    for topic in ('/scan', '/camera/depth/image_raw'):
                        packet = store.packets.get(topic)
                        if packet is None or now-packet['received'] > .5:
                            store.stop()
                    minimum = store.packets.get('/scan', {}).get('data', {}).get('minimum_m')
                    depth_minimum = store.packets.get('camera_preview/depth', {}).get('data', {}).get('minimum_m')
                    if any(v is not None and v < .3 for v in (minimum, depth_minimum)):
                        store.stop()
                if now - store.last_heartbeat > .5:
                    store.stop()
                    store.event('dashboard', 'Heartbeat delay; stop latched', 'ERROR')
                if not store.estop:
                    bridge = store.packets.get('motors', {}).get('data', {}).get('bridge', {})
                    if bridge.get('motion_inhibited'):
                        store.stop()
                        store.event('motors', 'Motor command delivery rejected; STOP latched: ' + json.dumps(bridge.get('rejection')), 'ERROR')
                    debug = bridge.get('mode') == 'dual_side_debug'
                    for name, limit in ([('motors', 1.5)] if store.bench_until or debug else [('controller', 3.), ('motors', 1.5)]):
                        packet = store.packets.get(name)
                        if packet is None or now - packet['received'] > limit:
                            store.stop()
                            store.event('dashboard', f'{name} disconnected; stop latched', 'ERROR')
                            break
                    if debug and not bridge.get('connected'):
                        store.stop()
                store.last_heartbeat = now
                store.startup_tick(now)
                self.stop_pub.publish(Bool(data=store.estop))
                self.supervised_pub.publish(Bool(data=bool(store.supervised_until > now and not store.estop)))
                if bench:
                    command = TwistStamped()
                    command.header.stamp = self.get_clock().now().to_msg()
                    command.header.frame_id = 'base_link'
                    command.twist.linear.x = .05 if store.bench_until and not store.estop and not store.bench_duty else 0.
                    if store.bench_until and not store.estop and store.bench_duty:
                        self.calibration_wheel_pub.publish(Int8(data=store.bench_wheel if store.bench_wheel is not None else -1))
                        if store.bench_wheel is None:
                            self.bench_pwm_pub.publish(Float32(data=store.bench_duty))
                    self.bench_command_pub.publish(command)
                    self.bench_allow_pub.publish(Bool(data=bool(store.bench_until and not store.estop)))

        def start_bench(self, boost=False, ground=False, wheel=None):
            if self.count_publishers('/kenny/controller/status') or self.count_publishers('/kenny/cmd_vel') > 1:
                raise ValueError('Stop the navigation/other command publisher before a raised-wheel bench test')
            store.start_bench(boost=boost, ground=ground, wheel=wheel)

        def inspect(self):
            calibration = {}
            for frame in ('laser_frame', 'camera_color_optical_frame', 'camera_depth_optical_frame', 'imu_link'):
                try:
                    self.buffer.lookup_transform('base_link', frame, rclpy.time.Time())
                    calibration[frame] = 'Available'
                except TransformException:
                    calibration[frame] = 'Missing base_link transform'
            store.update('calibration', calibration)
            controller=store.packets.get('controller',{})
            reference=store.packets.get('placement_reference',{})
            reference_data=reference.get('data',{})
            operator_reference=bool(reference_data.get('active') and time.monotonic()-reference.get('received',0)<2 and
                reference_data.get('map_sha256')==store.deployment.get('map_sha256'))
            has_reference=(controller.get('data',{}).get('pose') is not None and time.monotonic()-controller.get('received',0)<2) or operator_reference
            try:
                if not has_reference:
                    raise TransformException('No active localization reference for the current map; cached map TF is not a fresh pose')
                tf = self.buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
                p, q = tf.transform.translation, tf.transform.rotation
                store.update('tf', {'pose': [p.x, p.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))],
                                    'stamp_age_s': self.age(tf.header.stamp),
                                    'pose_source':'Operator placement + LiDAR SLAM + encoders' if operator_reference else 'SLAM + encoders'})
            except TransformException as error:
                store.update('tf', {'error': str(error)})
            with store.lock:
                store.graph = [{'name': name, 'types': types, 'publishers': self.count_publishers(name),
                                'subscribers': self.count_subscribers(name)} for name, types in self.get_topic_names_and_types()]
                sample = {'time': time.time(), 'controller': store.packets.get('controller', {}).get('data'),
                          'motors': store.packets.get('motors', {}).get('data'), 'odom': store.packets.get('odom', {}).get('data')}
                store.samples.append(sample)
                store.logger.info(json.dumps({'sample': sample}, allow_nan=False))

    return Dashboard()


def make_handler(store, node):
    class Handler(BaseHTTPRequestHandler):
        def respond(self, data, status=200, content_type='application/json'):
            body = data if isinstance(data, bytes) else json.dumps(data, allow_nan=False).encode()
            self.send_response(status)
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path == '/':
                return self.respond(Path(__file__).with_name('dashboard.html').read_bytes(), content_type='text/html; charset=utf-8')
            if self.path in ('/dashboard.js', '/dashboard.css'):
                return self.respond(Path(__file__).with_name(self.path[1:]).read_bytes(),
                    content_type='text/javascript; charset=utf-8' if self.path.endswith('.js') else 'text/css; charset=utf-8')
            if self.path.split('?')[0] in ('/api/camera/color.jpg', '/api/camera/depth.jpg'):
                name = self.path.split('/')[3].split('.')[0]
                with store.lock:
                    packet = store.images.get(name)
                    if packet is None or time.monotonic() - packet[0] > 2.:
                        return self.respond({'detail': 'Camera frame unavailable/stale'}, 503)
                    return self.respond(packet[1], content_type='image/jpeg')
            if self.path == '/api/report':
                with store.lock:
                    return self.respond({**store.snapshot(), 'refined_grid': store.grid, 'map': store.map_packet(), 'latest_scan': store.scan})
            if self.path == '/api/lidar/orientation':
                from controller.lidar_orientation import settings, sectors
                with store.lock:
                    scan = store.scan if store.scan and time.monotonic()-store.scan['received'] <= .5 else None
                    configured = settings()
                    return self.respond({**configured, 'scan_age_s': time.monotonic()-scan['received'] if scan else None,
                                         'sectors': sectors(scan, configured['yaw_deg'])})
            if self.path == '/api/motor-ports':
                return self.respond(motor_port_settings())
            if self.path in ('/api/status', '/api/status?detail=1'):
                return self.respond(store.snapshot(compact=self.path == '/api/status'))
            if self.path in ('/api/map', '/api/map?source=prior'):
                with store.lock:
                    return self.respond(store.map_packet(prior='source=prior' in self.path))
            if self.path == '/api/scan':
                with store.lock:
                    return self.respond(None if store.scan is None else {k: v for k, v in store.scan.items() if k != 'received'} | {
                        'age_s': round(time.monotonic() - store.scan['received'], 3)})
            self.respond({'detail': 'Not found'}, 404)

        def do_POST(self):
            origin = self.headers.get('Origin')
            if origin != 'http://' + self.headers.get('Host', '') or self.headers.get('X-Kenny-Control') != 'operations':
                return self.respond({'detail': 'Same-origin operations request required'}, 403)
            try:
                if self.path == '/api/stop':
                    store.stop()
                elif self.path == '/api/reset':
                    store.reset()
                elif self.path == '/api/supervised/start':
                    store.start_supervised()
                elif self.path == '/api/calibration/connect':
                    from controller.deployment import service
                    store.stop()
                    node.heartbeat()
                    with store.deployment_lock:
                        if store.deployment.get('mode') != 'robot':
                            raise ValueError('Select Robot mode first')
                        service('stop', 'kenny-navigation.service')
                        debug_service('stop')
                        service('start', 'kenny-motors.service')
                        store.event('calibration', 'Two-board motor bridge started with STOP latched; navigation paused')
                elif self.path == '/api/motor-ports':
                    size = int(self.headers.get('Content-Length', 0))
                    if not 1 <= size <= 512:
                        raise ValueError('Invalid motor port request size')
                    choice = json.loads(self.rfile.read(size))
                    if not isinstance(choice, dict):
                        raise ValueError('Expected motor port selection')
                    store.stop()
                    node.heartbeat()
                    with store.deployment_lock:
                        from controller.deployment import service
                        if subprocess.run(['systemctl', '--user', 'is-active', '--quiet', 'kenny-navigation.service'],
                                          timeout=2).returncode == 0:
                            service('stop', 'kenny-navigation.service')
                        result = assign_motor_ports(choice.get('left'), choice.get('right'))
                        if store.deployment.get('mode') == 'robot' and subprocess.run(
                                ['systemctl', '--user', 'is-active', '--quiet', 'kenny-controller-debug.service'],
                                timeout=2).returncode != 0:
                            subprocess.run(['systemctl', '--user', 'reset-failed', 'kenny-motors.service'],
                                           capture_output=True, timeout=2)
                            service('start', 'kenny-motors.service')
                        store.event('motors', 'Left/right ESP32 USB assignment updated; STOP latched')
                    return self.respond(result)
                elif self.path in ('/api/calibration/hand/start', '/api/calibration/hand/finish',
                                   '/api/calibration/pulse', '/api/calibration/ground/measure'):
                    size = int(self.headers.get('Content-Length', 0))
                    if not 1 <= size <= 256:
                        raise ValueError('Invalid calibration request size')
                    choice = json.loads(self.rfile.read(size))
                    if not isinstance(choice, dict):
                        raise ValueError('Expected a calibration object')
                    if self.path == '/api/calibration/hand/start':
                        store.calibration_hand(choice.get('wheel'))
                    elif self.path == '/api/calibration/hand/finish':
                        store.calibration_hand(choice.get('wheel'), choice.get('revolutions'))
                    elif self.path == '/api/calibration/pulse':
                        if self.headers.get('X-Kenny-Raised-Wheels') != 'confirmed':
                            raise ValueError('Confirm all wheels raised and physical stop accessible')
                        wheel = choice.get('wheel')
                        if type(wheel) is not int or wheel not in range(4):
                            raise ValueError('Select a wheel')
                        hand = store.calibration['hand'].get(('LF', 'RF', 'LR', 'RR')[wheel])
                        if not hand or not hand['signal_ok']:
                            raise ValueError('Verify this wheel encoder by hand before powered PWM test')
                        node.start_bench(boost=True, wheel=wheel)
                    else:
                        store.calibration_ground(choice.get('distance_m'))
                elif self.path == '/api/request/manual':
                    size = int(self.headers.get('Content-Length', 0))
                    if not 1 <= size <= 256:
                        raise ValueError('Invalid manual request size')
                    request = json.loads(self.rfile.read(size))
                    if not isinstance(request, dict):
                        raise ValueError('Expected a manual request object')
                    return self.respond({'request': submit_manual_request(store, request.get('marker_id'),
                                                                          request.get('request_id'))})
                elif self.path == '/api/request/cancel':
                    size = int(self.headers.get('Content-Length', 0))
                    if not 1 <= size <= 256:
                        raise ValueError('Invalid queue cancellation size')
                    request = json.loads(self.rfile.read(size))
                    if not isinstance(request, dict):
                        raise ValueError('Expected a queued request object')
                    return self.respond({'request': cancel_queued_request(store, request.get('job_id'))})
                elif self.path == '/api/lidar/orientation':
                    from controller.deployment import service
                    from controller.lidar_orientation import set_settings, settings
                    size = int(self.headers.get('Content-Length', 0))
                    if not 1 <= size <= 128:
                        raise ValueError('Invalid LiDAR orientation request size')
                    choice = json.loads(self.rfile.read(size))
                    if not isinstance(choice, dict):
                        raise ValueError('Invalid LiDAR orientation request')
                    store.stop()
                    node.heartbeat()
                    with store.deployment_lock:
                        current = settings()
                        changed = choice != current
                        if changed:
                            running = subprocess.run(['systemctl', '--user', 'is-active', 'kenny-navigation.service'],
                                capture_output=True, timeout=2.).returncode == 0
                            service('stop', 'kenny-navigation.service')
                            service('stop', 'kenny-lidar.service')
                            service('stop', 'kenny-sensor-mounts.service')
                            set_settings(choice.get('yaw_deg'), choice.get('inverted'))
                            node.buffer.clear()
                            with store.lock:
                                store.scan = None
                                store.packets.pop('/scan', None)
                            service('start', 'kenny-sensor-mounts.service')
                            service('start', 'kenny-lidar.service')
                            if running:
                                service('start', 'kenny-navigation.service')
                            store.event('lidar', f"Orientation updated: yaw {choice['yaw_deg']}°, inverted {choice['inverted']}; STOP latched")
                    return self.respond({'orientation': settings(), 'changed': changed, 'emergency_stop': store.estop})
                elif self.path in ('/api/drive-mode/pwm', '/api/drive-mode/velocity'):
                    from controller.deployment import service, state, STATE
                    from pi.receiver import save
                    mode = self.path.rsplit('/', 1)[1]
                    store.stop()
                    node.heartbeat()
                    with store.deployment_lock:
                        current = state()
                        if current['mode'] != 'robot':
                            raise ValueError('Select Robot mode before changing its drive mode')
                        service('stop', 'kenny-navigation.service')
                        service('stop', 'kenny-motors.service')
                        debug_service('stop')
                        current['drive_mode'] = mode
                        save(STATE, current)
                        store.deployment = current
                        service('start', 'kenny-motors.service')
                        service('start', 'kenny-navigation.service')
                        store.event('deployment', f'Robot drive mode {mode} selected; STOP latched')
                elif self.path in ('/api/mode/simulation', '/api/mode/robot', '/api/mode/controller', '/api/map/sync'):
                    from controller.deployment import switch_mode, sync_map
                    store.stop()
                    node.heartbeat()
                    with store.deployment_lock:
                        if self.path == '/api/mode/controller':
                            from controller.deployment import service
                            if store.deployment.get('mode') == 'robot':
                                # Preserve ownership of the running cloud job; STOP
                                # and pause navigation while the debug joystick owns USB.
                                service('stop', 'kenny-navigation.service')
                            service('stop', 'kenny-motors.service')
                            debug_service('start')
                            store.event('deployment', 'Two-ESP32 controller debugging selected; STOP latched')
                        elif self.path == '/api/mode/robot':
                            debug_service('stop')
                            try:
                                if store.deployment.get('mode') == 'robot':
                                    from controller.deployment import service
                                    service('start', 'kenny-motors.service')
                                    service('start', 'kenny-navigation.service')
                                else:
                                    store.deployment = switch_mode(store, 'robot')
                            except Exception:
                                debug_service('start')
                                raise
                        else:
                            debug_service('stop')
                            store.deployment = sync_map(store) if self.path == '/api/map/sync' else switch_mode(store, 'simulation')
                elif self.path in ('/api/retry/lidar', '/api/retry/camera', '/api/retry/imu'):
                    sensor = self.path.rsplit('/', 1)[1]
                    store.stop()
                    node.heartbeat()
                    retry_sensor(sensor)
                    if sensor == 'imu':
                        from controller.deployment import state, STATE
                        from pi.receiver import save
                        with store.deployment_lock:
                            current = state()
                            current['imu_enabled'] = True
                            save(STATE, current)
                            store.deployment = current
                    store.event('sensors', f'{sensor} reconnect requested; waiting for live data', 'INFO')
                elif self.path in ('/api/bench', '/api/bench75'):
                    if self.headers.get('X-Kenny-Raised-Wheels') != 'confirmed':
                        raise ValueError('Raised wheels and accessible motor-power stop must be confirmed')
                    node.start_bench(boost=self.path == '/api/bench75')
                elif self.path == '/api/pulse/forward':
                    if self.headers.get('X-Kenny-Floor-Pulse') != 'confirmed':
                        raise ValueError('Explicit operator ground-pulse request required')
                    node.start_bench(boost=True, ground=True)
                else:
                    return self.respond({'detail': 'Not found'}, 404)
                node.heartbeat()
                self.respond(store.snapshot())
            except (ValueError, OSError, TimeoutError, subprocess.TimeoutExpired) as error:
                self.respond({'detail': str(error)}, 409)
            self.close_connection = True

        def log_message(self, *args):
            pass
    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=8002)
    parser.add_argument('--log', type=Path, default=Path(__file__).resolve().parents[1] / 'data/operations.jsonl')
    parser.add_argument('--environment', type=Path, default=Path.home() / 'Downloads/kenny-environment/environment.json')
    args = parser.parse_args()
    import rclpy
    rclpy.init()
    environment = None
    map_error = None
    try:
        from kenny_rl.environment import validate_environment
        environment = validate_environment(json.loads(args.environment.read_text()))
    except (OSError, ValueError, KeyError, TypeError) as error:
        map_error = str(error)
    store = Diagnostics(args.log, environment)
    from controller.deployment import state as deployment_state, sync_map, MAP
    store.deployment = deployment_state()
    from controller.startup import StartupRelease
    store.startup = StartupRelease(args.log.parent / 'startup-state.json')
    if MAP.exists():
        from kenny_rl.environment import validate_environment
        store.prior = validate_environment(json.loads(MAP.read_text()))
    if map_error:
        store.event('map', 'Cannot load selected saved map: ' + map_error, 'ERROR')
    node = make_node(store)
    server = ThreadingHTTPServer(('127.0.0.1', args.port), make_handler(store, node))
    server.daemon_threads = True
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    finished = threading.Event()

    def cloud_map_worker():
        previous_error = None
        while not finished.is_set():
            try:
                with store.deployment_lock:
                    # A remote map revision stops Robot mode, but controller debug
                    # owns the USB boards for a local joystick test. Defer map
                    # synchronization until that test ends so a periodic poll
                    # cannot re-latch the operations STOP mid-gesture.
                    debug = subprocess.run(['systemctl', '--user', 'is-active', '--quiet',
                                            'kenny-controller-debug.service'], timeout=2.)
                    if debug.returncode != 0:
                        store.deployment = sync_map(store)
                previous_error = None
            except (OSError, ValueError, TimeoutError, subprocess.TimeoutExpired) as error:
                if str(error) != previous_error:
                    store.event('map', 'Render map sync: ' + str(error), 'ERROR')
                    previous_error = str(error)
            finished.wait(30.)

    threading.Thread(target=cloud_map_worker, daemon=True).start()

    def cloud_queue_worker():
        from controller.deployment import api
        previous_error = None
        while not finished.is_set():
            try:
                with store.lock:
                    requests = store.packets.get('requests')
                    paused = (store.deployment.get('mode') == 'simulation' or requests is None
                              or time.monotonic()-requests['received'] > 5.)
                if paused:
                    store.observe_queue(api().get('/api/pi/queue')['jobs'])
                previous_error = None
            except (OSError, ValueError, KeyError, TypeError, TimeoutError) as error:
                if str(error) != previous_error:
                    store.event('request', 'Cloud queue check failed: ' + str(error), 'ERROR')
                    previous_error = str(error)
            finished.wait(5.)

    threading.Thread(target=cloud_queue_worker, daemon=True).start()

    def host_diagnostics():
        while not finished.is_set():
            try:
                result = subprocess.run(['journalctl', '--user', '-u', 'kenny-sensors', '-u', 'kenny-operations',
                    '-u', 'kenny-live-map', '-u', 'kenny-motors', '-u', 'kenny-controller-debug', '-u', 'kenny-lidar', '-u', 'kenny-aruco', '-u', 'kenny-bringup', '--lines=80', '--no-pager', '--output=short-iso'],
                    capture_output=True, text=True, timeout=2.)
                service_logs = '\n'.join('[controller debug password hidden]' if 'Password:' in line else line
                                         for line in result.stdout[-24000:].splitlines())
                debug = subprocess.run(['systemctl', '--user', 'is-active', 'kenny-controller-debug.service'],
                    capture_output=True, text=True, timeout=2.)
                memory = {}
                for line in Path('/proc/meminfo').read_text().splitlines():
                    key, value = line.split(':', 1)
                    if key in ('MemTotal', 'MemAvailable', 'SwapTotal', 'SwapFree'):
                        memory[key] = value.strip()
                temperature = Path('/sys/class/thermal/thermal_zone0/temp')
                store.update('host', {'load_1_5_15': list(os.getloadavg()), 'memory': memory,
                    'camera_usb': camera_usb_devices(), 'controller_debug_active': debug.stdout.strip() == 'active',
                    'disk_free_gb': round(shutil.disk_usage(args.log.parent).free / 1e9, 2),
                    'temperature_c': float(temperature.read_text()) / 1000 if temperature.exists() else None,
                    'service_logs': service_logs})
            except (OSError, ValueError, subprocess.TimeoutExpired) as error:
                store.update('host', {'error': str(error)})
            finished.wait(5.)

    threading.Thread(target=host_diagnostics, daemon=True).start()
    print(f'Kenny operations: http://127.0.0.1:{args.port} (stop latched)', flush=True)
    from rclpy.executors import ExternalShutdownException
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        finished.set()
        store.stop(persist=False)
        if rclpy.ok():
            node.heartbeat()
        server.shutdown()
        server.server_close()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
