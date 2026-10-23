"""Open the local dashboard once the desktop and HTTP service are ready."""
import argparse
import fcntl
import json
import os
from pathlib import Path
import subprocess
import time
from urllib.error import URLError
from urllib.request import urlopen

URL = 'http://localhost:8002'


def wait_ready(timeout):
    deadline = time.monotonic() + timeout
    while True:
        try:
            with urlopen(URL + '/api/status', timeout=2) as response:
                data = json.load(response)
            if isinstance(data, dict) and isinstance(data.get('emergency_stop'), bool):
                return True
        except (OSError, URLError, ValueError):
            pass
        if time.monotonic() >= deadline:
            return False
        time.sleep(1.)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check', action='store_true', help='Check dashboard readiness without opening a browser')
    parser.add_argument('--timeout', type=float, default=120.)
    args = parser.parse_args()
    if args.check:
        ready = wait_ready(args.timeout)
        print('Dashboard ready: ' + URL if ready else 'Dashboard readiness timed out')
        return 0 if ready else 1
    if not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
        print('Dashboard browser needs a graphical desktop session', flush=True)
        return 1
    runtime = Path(os.environ.get('XDG_RUNTIME_DIR', '/run/user/' + str(os.getuid())))
    with (runtime / 'kenny-dashboard-browser.lock').open('a') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            return 0
        logs = Path(__file__).resolve().parents[1] / 'data'
        logs.mkdir(parents=True, exist_ok=True)
        with (logs / 'browser-startup.log').open('a') as log:
            log.write(time.strftime('%Y-%m-%d %H:%M:%S') + ' Waiting for dashboard\n')
            log.flush()
            if not wait_ready(args.timeout):
                log.write('Dashboard readiness timed out; browser was not launched\n')
                return 1
            child = subprocess.Popen(['/usr/bin/xdg-open', URL], stdin=subprocess.DEVNULL,
                stdout=log, stderr=log, start_new_session=True)
            log.write(f'Opening dashboard browser: {URL}\n')
            log.flush()
            # Keep the desktop launcher alive while xdg-open owns its browser
            # process, so desktop-session cleanup cannot kill a detached child.
            returncode = child.wait()
            if returncode:
                log.write(f'Browser launcher failed: exit {returncode}\n')
                return 1
            log.write('Browser launcher completed successfully\n')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
