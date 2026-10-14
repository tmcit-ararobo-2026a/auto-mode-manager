"""Deterministic contract-v1 request handling and production-based freshness."""

import math
from dataclasses import dataclass

STANDBY, READY, MOVING, TRACKING, GOAL, FAIL = range(6)
SLEEP, RELEASE, MOVE = range(3)
MAX_SEQUENCE = (1 << 64) - 1


def wrap(angle_rad):
    """Return the shortest signed angular difference."""
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def wire_angle(bearing_rad, sign, offset_deg):
    """Round ties away from zero, rejecting the unrounded range before conversion."""
    if sign not in (-1, 1) or not all(math.isfinite(x) for x in (bearing_rad, offset_deg)):
        return None
    value = sign * bearing_rad * 180 / math.pi + offset_deg
    if abs(value) > 20:
        return None
    if value >= 0:
        return math.floor(value + 0.5)
    return math.ceil(value - 0.5)


class Freshness:
    """Strict session/sequence progress with a steady deadline from production age."""

    def __init__(self, timeout_s):
        self.timeout_s = timeout_s
        self.session = self.sequence = self.stamp_ns = 0
        self.deadline_s = 0.0
        self.retired = set()

    def accept(self, session, sequence, stamp_ns, ros_ns, steady_s):
        """Accept new progress; return (accepted, session_changed)."""
        age_s = (ros_ns - stamp_ns) * 1e-9
        if session == 0 or sequence == 0 or stamp_ns <= 0 or not 0 <= age_s < self.timeout_s:
            return False, False
        if session in self.retired:
            return False, False
        changed = self.session != 0 and session != self.session
        if session == self.session and (sequence <= self.sequence or stamp_ns <= self.stamp_ns):
            return False, False
        if changed:
            self.retired.add(self.session)
        self.session, self.sequence, self.stamp_ns = session, sequence, stamp_ns
        self.deadline_s = steady_s + self.timeout_s - age_s
        return True, changed

    def fresh(self, ros_ns, steady_s):
        """Never renew a lease from reception or repeated publication."""
        return (self.session != 0 and steady_s < self.deadline_s
                and 0 <= (ros_ns - self.stamp_ns) * 1e-9 < self.timeout_s)


@dataclass
class RequestState:
    """Request state independent of ROS delivery and action transport."""

    state: int = STANDBY
    generation: int = 1
    target_id: int = 0
    latest_command: int = SLEEP
    release_start_s: float = -1.0
    last_command_s: float = -1.0
    armed: bool = False
    cancelling: bool = False
    cancel_generation: int = 0
    cancel_started_s: float = 0.0
    cancel_sleep: bool = False
    reason: str = 'Awaiting release and quality'
    overflow: bool = False

    def advance(self):
        """Stop before uint64 wrap; the runtime starts a new session after cleanup."""
        if self.generation >= MAX_SEQUENCE - 1:
            self.overflow = True
            self.state = FAIL
            self.armed = False
            self.reason = 'Generation exhausted; new session required'
            return False
        self.generation += 1
        return True

    def released(self, steady_s):
        """Require a continuous release window and a fresh last observation."""
        return (self.release_start_s >= 0 and self.last_command_s >= 0
                and steady_s - self.release_start_s >= 0.15
                and steady_s - self.last_command_s < 0.15
                and self.latest_command in (SLEEP, RELEASE))

    def cancel(self, steady_s, reason, sleep=False, fault=False):
        """Invalidate before transport cancellation, preserving the old lease generation."""
        if not self.cancelling:
            self.cancel_generation = self.generation
            self.cancel_started_s = steady_s
            self.cancelling = True
            self.advance()
        self.armed = False
        self.release_start_s = -1.0
        self.cancel_sleep = self.cancel_sleep or sleep
        if fault or self.state == FAIL:
            self.state = FAIL
        else:
            self.state = STANDBY
        self.reason = reason

    def observe(self, command, target_id, steady_s, capable):
        """Consume every Move once; latch release/cancel before low-priority results."""
        previous_command = self.latest_command
        if self.last_command_s < 0 or steady_s - self.last_command_s >= 0.15:
            self.release_start_s = -1.0
            self.armed = False
        self.last_command_s = steady_s
        self.latest_command = command
        if command in (SLEEP, RELEASE):
            if previous_command == MOVE or self.state in (MOVING, TRACKING, GOAL):
                self.cancel(steady_s, 'Operator cancellation', sleep=command == SLEEP)
            if self.release_start_s < 0:
                self.release_start_s = steady_s
            if command == SLEEP and self.state != FAIL and not self.cancelling:
                self.state = STANDBY
            return False
        if command != MOVE or target_id not in range(8):
            self.cancel(steady_s, 'Invalid command', fault=True)
            return False
        # No replacement or pending request is retained while Move is held.
        accept = (self.armed and self.state == READY and not self.cancelling and capable)
        self.armed = False
        self.release_start_s = -1.0
        if not accept:
            self.reason = 'Move consumed: release, quality or capability unavailable'
            return False
        if not self.advance():
            return False
        self.target_id = target_id
        self.state = MOVING
        if target_id == 7:
            self.state = TRACKING
        self.reason = 'New execution accepted'
        return True

    def update_ready(self, steady_s, ready):
        """Return to Ready only after cleanup, quality and observed release."""
        if self.cancelling or self.state == FAIL:
            return
        if self.state in (STANDBY, READY) and self.latest_command in (SLEEP, RELEASE):
            self.armed = ready and self.released(steady_s)
            if self.armed and self.latest_command == RELEASE:
                self.state = READY
                self.reason = 'Simulation profile ready'
            else:
                self.state = STANDBY

    def cancel_complete(self):
        """Finish ROS cleanup without arming a hand or releasing a fault latch."""
        self.cancelling = False
        self.cancel_sleep = False
        if self.state != FAIL:
            self.state = STANDBY
            self.reason = 'ROS cancellation complete; release required'

    def reset(self, steady_s, healthy):
        """Reset only after all causes, cleanup, quality and release are satisfied."""
        if self.state != FAIL:
            return False, 'No latched fault'
        if not healthy or self.cancelling or not self.released(steady_s) or self.overflow:
            return False, 'Causes, quality, cleanup and continuous release must be satisfied'
        self.state = STANDBY
        self.reason = 'Fault reset; no path or hand resumed'
        self.update_ready(steady_s, healthy)
        return True, self.reason
