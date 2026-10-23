"""Local deployment state, Render map synchronization and fixed service controls."""
import fcntl
import json
import os
from pathlib import Path
import subprocess

from controller.missions import CloudAPI
from pi.receiver import save

ROOT = Path(__file__).resolve().parents[1]
STATE = ROOT / 'data/deployment.json'
MAP = ROOT / 'data/cloud-environment.json'
MODEL = ROOT.parent / 'Kenny---Automotive-self-driving-trash-can/releases/dynamic-v3-seed2/model/best.zip'


def state():
    current = json.loads(STATE.read_text()) if STATE.exists() else {'mode': 'simulation', 'map_sha256': None}
    current.setdefault('terrain_profile', 'flat_floor')
    current.setdefault('imu_enabled', True)
    return current


def api():
    return CloudAPI(os.environ.get('KENNY_API_URL', 'https://kenny-api.onrender.com'),
                    os.environ.get('KENNY_PI_TOKEN') or os.environ.get('KENNY_DISPATCH_TOKEN', ''))


def service(action, unit):
    result = subprocess.run(['systemctl', '--user', action, unit], capture_output=True, text=True, timeout=20)
    if result.returncode:
        raise ValueError(result.stderr.strip() or f'{action} {unit} failed')


def sync_map(store):
    from kenny_rl.environment import validate_environment
    packet = api().get('/api/map')
    environment = validate_environment(packet['environment'])
    current = state()
    if current.get('map_sha256') == packet['sha256'] and MAP.exists():
        return current
    if current['mode'] == 'robot':
        if not store.estop:
            store.stop()
        status = subprocess.run(['systemctl', '--user', 'is-active', 'kenny-navigation.service'],
                                capture_output=True, text=True, timeout=20)
        if status.stdout.strip() not in ('inactive', 'failed'):
            raise ValueError('Render map changed. Switch to Simulation, sync the map, then switch to Robot to relocalize.')
    save(MAP, environment)
    current.update(map_sha256=packet['sha256'], map_name=environment.get('name'), environment=str(MAP), model=str(MODEL))
    save(STATE, current)
    with store.lock:
        store.prior = environment
        store.grid = None
        store.last_pose = None
        for name in ('tf', 'controller', 'aruco'):
            store.packets.pop(name, None)
        store.revision += 1
    service('restart', 'kenny-aruco.service')
    store.event('map', 'Loaded uploaded Render map: ' + environment.get('name', 'Kenny map'))
    return current


def switch_mode(store, mode):
    if mode not in ('simulation', 'robot'):
        raise ValueError('Unknown deployment mode')
    # The simulator holds this same lock throughout claim/run/result upload.
    lock_path = ROOT.parent / 'kenny-map-creator/data/simulation-receiver.lock'
    lock_path.parent.mkdir(parents=True, exist_ok=True)
    with lock_path.open('w') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as error:
            raise ValueError('A simulation request is running; wait for it to finish before switching') from error
        current = state()
        journal = ROOT / 'data/main-controller.json'
        if journal.exists() and json.loads(journal.read_text()).get('phase') in ('running', 'switching', 'claiming'):
            raise ValueError('Finish or resolve the current robot request before switching modes')
        if mode == 'robot':
            if current['mode'] != 'robot':
                current = sync_map(store)
            if not MODEL.is_file():
                raise ValueError('Navigation checkpoint is missing')
            jobs = api().get('/api/pi/queue')['jobs']
            if any(j.get('status') == 'running' for j in jobs):
                raise ValueError('A cloud request is already running; resolve it before switching')
        service('stop', 'kenny-navigation.service')
        if mode == 'simulation':
            service('stop', 'kenny-motors.service')
        current['mode'] = mode
        if mode == 'robot':
            # Process queued calls on this robot instead of local roam-only commissioning.
            current['roam'] = False
            current['target_marker'] = -1
        save(STATE, current)
        if mode == 'robot':
            try:
                service('start', 'kenny-motors.service')
                service('start', 'kenny-navigation.service')
            except ValueError:
                service('stop', 'kenny-motors.service')
                current['mode'] = 'simulation'
                save(STATE, current)
                raise
        with store.lock:
            store.grid = None
            store.last_pose = None
            store.packets.pop('controller', None)
            store.packets.pop('tf', None)
            store.packets.pop('route', None)
            store.revision += 1
        store.event('deployment', mode.title() + ' mode selected; STOP remains latched')
        return current


def main():
    current = state()
    if current['mode'] != 'robot':
        raise SystemExit('Robot mode is not selected')
    os.execvp('ros2', ['ros2', 'launch', str(ROOT / 'controller/bringup.launch.py'),
        'environment:=' + str(MAP), 'model:=' + str(MODEL), 'auto_marker_start:=true', 'flat_floor:=true',
        'commissioning:=' + str(current.get('commissioning', False)).lower(),
        'drive_mode:=' + str(current.get('drive_mode', 'velocity')),
        'roam:=' + str(current.get('roam', False)).lower(),
        'max_speed:=' + str(current.get('max_speed', .1)),
        'target_marker:=' + str(current.get('target_marker', -1)),
        'api:=' + api().url, 'dashboard:=false', 'editor_bridge:=false', 'markers:=false', 'motors:=false'])


if __name__ == '__main__':
    main()
