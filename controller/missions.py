"""Durable cloud scheduling around a resident navigation driver."""
import fcntl
import json
from pathlib import Path
import time
from urllib.request import Request, urlopen

from pi.receiver import API, save
from .planning import nearest


class CloudAPI(API):
    def get(self, path):
        headers = {'Authorization': f'Bearer {self.token}'} if self.token else {}
        with urlopen(Request(self.url + path, headers=headers), timeout=15) as response:
            return json.load(response)


class Missions:
    def __init__(self, api, driver, state_path, worker='kenny-main', margin=.25, timeout=1800, target_marker=None, resume_stopped=False):
        self.api, self.driver, self.path = api, driver, Path(state_path)
        self.worker, self.margin, self.timeout = worker, margin, timeout
        self.target_marker = target_marker
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.lock = self.path.with_suffix('.lock').open('w')
        fcntl.flock(self.lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        self.state = json.loads(self.path.read_text()) if self.path.exists() else None
        if self.state is not None:
            self.worker = self.state['worker']
        if self.state and (self.state['phase'] == 'switching' or
                           self.state['phase'] == 'running' and not resume_stopped):
            self.lock.close()
            raise RuntimeError(f'Interrupted physical mission in {self.path}; stop the robot and resolve the recorded API job before removing this journal')
        self.started = None
        self.observed_jobs = {}
        self.last_wait = None
        if self.state and self.state['phase'] == 'running':
            try:
                gate = getattr(driver, 'stop_gate', None)
                if gate is None or not gate.reason():
                    raise RuntimeError('Dashboard STOP must be latched before resuming an interrupted mission')
                job = self.state['job']
                running = next((j for j in api.get('/api/pi/queue')['jobs'] if j['id'] == job['id']), None)
                if running is None or running['status'] != 'running' or running['worker'] != self.worker:
                    raise RuntimeError('Cloud no longer confirms ownership of the interrupted mission')
                self.started = time.monotonic()
                driver.set_goal(running)
                self.state['job'] = running
                self.event('resumed', 'Interrupted request resumed with dashboard STOP latched', running)
            except Exception:
                self.lock.close()
                raise

    def event(self, stage, detail, job=None):
        callback = getattr(self.driver, 'mission_event', None)
        if callback:
            callback({'time': time.time(), 'stage': stage, 'detail': detail,
                      'job_id': job.get('id') if job else None,
                      'marker_id': job.get('marker_id') if job else None})

    def observe(self, jobs):
        callback = getattr(self.driver, 'mission_queue', None)
        if callback:
            callback(jobs)
        for job in jobs:
            key = (job['id'], job['status'])
            if self.observed_jobs.get(job['id']) == key:
                continue
            self.observed_jobs[job['id']] = key
            self.event('received' if job['status'] == 'queued' else 'queue_update',
                       f"Request for ArUco {job['marker_id']}: {job['status']}", job)
        if len(self.observed_jobs) > 500:
            self.observed_jobs = {job['id']: (job['id'], job['status']) for job in jobs}

    def close(self):
        self.lock.close()

    def write(self):
        save(self.path, self.state)

    def step(self, now=None):
        now = time.monotonic() if now is None else now
        if self.state and self.state['phase'] == 'finished':
            self.api.post(f"/api/calls/{self.state['job']['id']}/result",
                          {'worker': self.state['worker'], **self.state['outcome']})
            self.event('result_uploaded', self.state['outcome']['detail'], self.state['job'])
            self.path.unlink()
            self.state = None
            return 'Result delivered; waiting for calls'
        if self.state and self.state['phase'] == 'claiming':
            self.event('claiming', 'Claiming selected request on Render', self.state['job'])
            response = self.api.post('/api/pi/claim', {'worker': self.worker, 'job_id': self.state['job']['id']})
            if response['job'] is None:
                self.path.unlink()
                self.state = None
                return 'Another receiver owns the running trip'
            self.activate(response['job'], now)
            return 'Selected call claimed'
        if self.state and self.state['phase'] == 'switching':
            self.event('switching', 'Switching to nearer request after confirmed stopping', self.state['next_job'])
            response = self.api.post('/api/pi/switch', {'worker': self.worker,
                'current_job_id': self.state['job']['id'], 'next_job_id': self.state['next_job']['id']})
            self.activate(response['job'], now)
            return 'Switched to nearer call; previous call remains queued'
        jobs = self.api.get('/api/pi/queue')['jobs']
        self.observe(jobs)
        if self.state and self.state['phase'] == 'running' and self.driver.arrived():
            self.finish('succeeded', 'Near requested ArUco with fresh measured depth or localized pose; robot stopped')
            return 'Destination reached'
        if self.target_marker is not None:
            jobs = [job for job in jobs if job['marker_id'] == self.target_marker]
        snapshot = self.driver.snapshot()
        if not snapshot['ready']:
            waiting = (snapshot['reason'], tuple(j['id'] for j in jobs))
            if waiting != self.last_wait:
                for job in jobs:
                    self.event('waiting', 'Navigation blocked: ' + snapshot['reason'], job)
                self.last_wait = waiting
            return f"Waiting: {snapshot['reason']}"
        self.last_wait = None
        if self.state is None:
            selected = nearest(jobs, snapshot['pose'], self.driver.destination)
            if selected is None:
                return 'Waiting for a registered ArUco call'
            self.state = {'phase': 'claiming', 'job': selected, 'worker': self.worker}
            self.write()
            self.event('selected', 'Selected nearest registered destination', selected)
            self.step(now)
            return f"Navigating to ArUco ID {selected['marker_id']}"
        current = self.state['job']
        if now - self.started > self.timeout:
            self.finish('failed', 'Trip timed out; robot stopped')
            return 'Trip timed out'
        selected = nearest(jobs, snapshot['pose'], self.driver.destination, current, self.margin)
        if selected is not None:
            # Stop and confirm before changing server ownership. HTTP retries
            # cannot accidentally resurrect the old physical navigation goal.
            self.driver.stop()
            self.state = {**self.state, 'phase': 'switching', 'next_job': selected}
            self.write()
            return self.step(now)
        return f"Navigating to ArUco ID {current['marker_id']}"

    def activate(self, job, now):
        self.state = {'phase': 'running', 'job': job, 'worker': self.worker}
        self.write()
        self.started = now
        self.driver.set_goal(job)
        self.event('goal_set', 'Request claimed; navigation goal assigned', job)

    def finish(self, status, detail):
        self.driver.stop()
        self.event(status, detail, self.state['job'])
        self.state = {**self.state, 'phase': 'finished', 'outcome': {'status': status, 'detail': detail}}
        self.write()
