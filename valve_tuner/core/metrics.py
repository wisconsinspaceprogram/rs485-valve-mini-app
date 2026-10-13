"""Step-response metrics computed from the debug stream.

A move starts whenever the reported target changes. Metrics use the position the valve
was actually at when the move started, so the first move after power-up (where the
valve can be anywhere) is still measured correctly, just over a different travel.
"""
from dataclasses import dataclass
from typing import Optional

MIN_TRAVEL = 40        # counts; smaller target changes are not scored
SETTLE_FRACTION = 0.02 # settle band = +/-2 % of travel ...
SETTLE_MIN = 5         # ... but never tighter than this many counts
MAX_MOVE_S = 6.0
HOLD_S = 0.5           # in band with PWM 0 for this long -> move finished


@dataclass
class MoveMetrics:
    index: int
    label: str
    start: int
    target: int
    travel: int
    first: bool = False
    rise_ms: Optional[float] = None        # 5 % -> 95 % of travel
    overshoot: float = 0.0                 # counts past target
    overshoot_pct: float = 0.0             # % of travel
    settle_ms: Optional[float] = None
    final_error: int = 0
    done: bool = False


def _cross(ts, ps, level):
    for i, p in enumerate(ps):
        if p >= level:
            if i == 0 or ps[i] == ps[i - 1]:
                return ts[i]
            f = (level - ps[i - 1]) / (ps[i] - ps[i - 1])
            return ts[i - 1] + f * (ts[i] - ts[i - 1])
    return None


def compute(m: MoveMetrics, t0, ts, pos):
    travel = m.target - m.start
    prog = [(p - m.start) / travel for p in pos]
    t05, t95 = _cross(ts, prog, 0.05), _cross(ts, prog, 0.95)
    m.rise_ms = (t95 - t05) * 1000 if t05 is not None and t95 is not None else None
    over = max(0.0, max(prog) - 1.0)
    m.overshoot = over * abs(travel)
    m.overshoot_pct = over * 100
    band = max(SETTLE_FRACTION * abs(travel), SETTLE_MIN)
    last_out = max((i for i, p in enumerate(pos) if abs(p - m.target) > band), default=-1)
    if last_out == len(pos) - 1:
        m.settle_ms = None
    else:
        m.settle_ms = max(0.0, (ts[last_out + 1] - t0) * 1000)
    m.final_error = pos[-1] - m.target
    return band


class MoveTracker:
    def __init__(self):
        self.moves = []
        self.version = 0
        self._prev = None
        self._cur = None
        self._count = 0

    def reset(self):
        self.__init__()

    def add(self, t, pos, target, pwm, labeler=lambda tgt: "Move"):
        prev, self._prev = self._prev, (t, pos, target)
        if prev is None:
            return
        if target != prev[2]:
            self._finish()
            if abs(target - prev[1]) >= MIN_TRAVEL:
                self._count += 1
                m = MoveMetrics(self._count, labeler(target), prev[1], target,
                                target - prev[1], first=(self._count == 1))
                # the command landed somewhere between the two samples
                self._cur = dict(m=m, t0=(prev[0] + t) / 2, ts=[prev[0]], pos=[prev[1]], in_since=None)
                self.moves.append(m)
        c = self._cur
        if c is None:
            return
        c["ts"].append(t)
        c["pos"].append(pos)
        band = compute(c["m"], c["t0"], c["ts"], c["pos"])
        self.version += 1
        if abs(pos - target) <= band and pwm == 0:
            if c["in_since"] is None:
                c["in_since"] = t
        else:
            c["in_since"] = None
        if t - c["t0"] > MAX_MOVE_S or (c["in_since"] is not None and t - c["in_since"] >= HOLD_S):
            self._finish()

    def _finish(self):
        if self._cur is not None:
            self._cur["m"].done = True
            self._cur = None
            self.version += 1
