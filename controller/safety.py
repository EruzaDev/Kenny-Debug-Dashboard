"""Fail-closed dashboard stop heartbeat, shared by policy and motor adapter."""
import time
import threading


class StopGate:
    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.stopped = True
        self.updated = None
        self.needs_stop = True
        self.lock = threading.RLock()

    def update(self, stopped):
        with self.lock:
            if self.updated is not None and self.clock() - self.updated > .5:
                self.needs_stop = True
            if stopped:
                self.needs_stop = False
            self.stopped = bool(stopped) or self.needs_stop
            self.updated = self.clock()

    def reason(self):
        with self.lock:
            if self.updated is None or self.clock() - self.updated > .5:
                self.needs_stop = True
                self.stopped = True
                return 'Dashboard stop heartbeat unavailable/stale'
            return 'Emergency stop latched' if self.stopped else ''
