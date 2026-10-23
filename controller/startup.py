"""One readiness-gated startup release per OS boot; stops survive restarts."""
import json
from pathlib import Path

from pi.receiver import save


class StartupRelease:
    def __init__(self, path, boot_id=None):
        self.path = Path(path)
        self.boot_id = boot_id or Path('/proc/sys/kernel/random/boot_id').read_text().strip()
        self.since = None
        self.reason = 'Waiting for startup readiness'
        try:
            self.state = json.loads(self.path.read_text()) if self.path.exists() else {}
            if not isinstance(self.state, dict):
                raise ValueError('Invalid startup state')
        except (OSError, ValueError):
            self.state = {'inhibited': True}
            self.reason = 'Startup state unreadable; manual reset required'

    def stop(self):
        self.since = None
        self.state['inhibited'] = True
        self.reason = 'Stop/fault latched; manual reset required'
        save(self.path, self.state)

    def released(self):
        self.state = {'inhibited': False, 'released_boot': self.boot_id}
        save(self.path, self.state)
        self.since = None
        self.reason = 'Startup release completed'

    def ready(self, now, enabled, healthy):
        if not enabled or self.state.get('inhibited') or self.state.get('released_boot') == self.boot_id:
            self.since = None
            return False
        if not healthy:
            self.since = None
            self.reason = 'Waiting for localized controller, sensors, and stopped motors'
            return False
        if self.since is None:
            self.since = now
        self.reason = 'Confirming stable startup readiness'
        return now-self.since >= 3.
