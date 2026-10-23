"""Bounded, de-duplicated camera scan history for the operations dashboard."""
from collections import deque
import time


class ArucoHistory:
    def __init__(self, wall=time.time, clock=time.monotonic):
        self.wall, self.clock = wall, clock
        self.seen = {}
        self.events = deque(maxlen=100)
        self.frame = None

    def update(self, packet):
        stamp = packet.get('observed_at')
        frame = (packet.get('sequence'), stamp)
        if stamp is None or frame == self.frame:
            return []
        self.frame = frame
        age = self.wall() - stamp
        if not -.1 <= age <= 2.:
            return []
        now = self.clock()
        added = []
        for detection in packet.get('detections', []):
            marker_id = detection.get('id')
            if type(marker_id) is not int or not 0 <= marker_id < 1000:
                continue
            previous = self.seen.get(marker_id)
            reacquired = previous is None or now - previous['last_seen'] > 2.
            record = {**detection, 'last_seen': now - max(0., age),
                      'frames_seen': (previous['frames_seen'] if previous else 0) + 1,
                      'logged_at': previous['logged_at'] if previous else -float('inf')}
            if reacquired or now - record['logged_at'] >= 5.:
                event = {**detection, 'time': stamp, 'dictionary': packet.get('dictionary'),
                         'event': 'Seen' if previous is None else 'Reacquired' if reacquired else 'Still visible'}
                self.events.append(event)
                added.append(event)
                record['logged_at'] = now
            self.seen[marker_id] = record
        return added

    def snapshot(self):
        now = self.clock()
        return {'events': list(self.events), 'seen': [
            {k: v for k, v in record.items() if k not in ('last_seen', 'logged_at')} | {
                'last_seen_age_s': round(now - record['last_seen'], 3)}
            for _, record in sorted(self.seen.items())]}
