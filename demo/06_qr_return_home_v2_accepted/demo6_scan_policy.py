#!/usr/bin/env python3
"""Pure state machine for one stable QR result followed by a 5s hold.

A confirmed decoded QR persists during the five-second observation period,
although lost camera detections continue to be shown by the 1Hz detector.
Conflicting QR payloads fail rather than silently replacing the first code.
"""
class ScanPolicy:
    def __init__(self, start, timeout, hold_time):
        self.start = float(start)
        self.timeout = float(timeout)
        self.hold_time = float(hold_time)
        self.code = ''
        self.detected_at = None
        self.failure = ''

    def push(self, text, now):
        if self.failure or not text:
            return
        if self.code and self.code != text:
            self.failure = 'conflicting QR codes detected'
        elif not self.code:
            self.code = text
            self.detected_at = float(now)

    def state(self, now):
        now = float(now)
        if self.failure:
            return 'FAIL'
        if self.detected_at is not None:
            return 'READY' if now - self.detected_at >= self.hold_time else 'HOLD'
        return 'TIMEOUT' if now - self.start >= self.timeout else 'SEARCH'

    def remaining(self, now):
        if self.detected_at is None:
            return None
        return max(0.0, self.hold_time - (float(now) - self.detected_at))
