"""Automatic Kp/Kd search.

Every candidate is scored on one OPEN and one CLOSE move, and the score is driven by the
WORSE of the two directions, so a setting that is perfect one way but overshoots the
other does not win. The search is a small pattern search around the best point so far:
it tries more/less Kd and Kp +/-1, picks the first direction to try from what the worse
side did (overshoot -> more damping first), and tightens the Kd step when nothing helps.
"""
import threading
import time
from dataclasses import dataclass
from typing import Optional

from .metrics import MoveMetrics

KP_MIN, KP_MAX = 1, 20
KD_MIN, KD_MAX = 0, 9999
GOOD_SCORE = 12.0        # stop early when the worse side is this good (roughly: within tolerance, no overshoot)
IMPROVE = 0.95           # a candidate must beat the best by 5 % to replace it (measurement noise)
MISSING = 1000.0


def move_cost(m: Optional[MoveMetrics]) -> float:
    """Lower is better. Units are roughly 'counts away from a perfect hit'."""
    if m is None:
        return MISSING
    cost = abs(m.final_error) + m.overshoot
    if m.settle_ms is None:
        # ended outside the +/-2 % band: weight the miss heavily, but keep it proportional
        # so the search can still tell "missed by 20" from "missed by 40"
        cost += 20 + 2 * abs(m.final_error)
    else:
        cost += m.settle_ms / 200        # mild preference for faster settling
    if m.timed_out and m.settle_ms is not None:
        cost += 100                      # still hunting when the clock ran out
    return cost


@dataclass
class Trial:
    n: int
    kp: int
    kd: int
    open: Optional[MoveMetrics]
    close: Optional[MoveMetrics]
    note: str = ""

    @property
    def cost_open(self):
        return move_cost(self.open)

    @property
    def cost_close(self):
        return move_cost(self.close)

    @property
    def score(self):
        worst, other = max(self.cost_open, self.cost_close), min(self.cost_open, self.cost_close)
        return worst + 0.25 * other

    @property
    def worst(self):
        return self.open if self.cost_open >= self.cost_close else self.close


class _Abort(Exception):
    pass


class AutoTuner:
    def __init__(self, link):
        self.link = link
        self._lock = threading.Lock()
        self._stop = False
        self._last_act = 0.0
        self.running = False
        self.status = "Idle"
        self.trials = []
        self.best = None          # (kp, kd, mean score)
        self.used = 0
        self.max_moves = 30
        self.gap = 3.0
        self.version = 0

    # ------------------------------------------------------------------ public
    def start(self, kp, kd, max_moves=30, gap=3.0):
        if self.running:
            return
        self._stop, self.running = False, True
        self.trials, self.best, self.used = [], None, 0
        self.max_moves, self.gap = int(max_moves), float(gap)
        threading.Thread(target=self._run, args=(int(kp), int(kd)), daemon=True).start()

    def stop(self):
        self._stop = True

    def snapshot(self):
        with self._lock:
            return dict(running=self.running, status=self.status, trials=list(self.trials),
                        best=self.best, used=self.used, max_moves=self.max_moves,
                        version=self.version)

    # ------------------------------------------------------------------ helpers
    def _status(self, text):
        with self._lock:
            self.status = text
            self.version += 1

    def _sleep(self, seconds):
        end = time.monotonic() + seconds
        while True:
            if self._stop:
                raise _Abort("stopped by user")
            if self.link.status != "connected":
                raise _Abort("valve disconnected")
            left = end - time.monotonic()
            if left <= 0:
                return
            time.sleep(min(0.05, left))

    def _left(self):
        return self.max_moves - self.used

    def _actuate(self, direction, what):
        """Send one open/close (respecting the minimum gap) and return its metrics."""
        self._sleep(max(0.0, self._last_act + self.gap - time.monotonic()))
        moves = self.link.snapshot()["moves"]
        before = moves[-1] if moves else None
        self._status(f"Actuation {self.used + 1}/{self.max_moves}: {direction} ({what})")
        (self.link.open_valve if direction == "open" else self.link.close_valve)()
        self._last_act = time.monotonic()
        self.used += 1
        while True:
            self._sleep(0.05)
            moves = self.link.snapshot()["moves"]
            new = moves[-1] if moves and moves[-1] is not before else None
            waited = time.monotonic() - self._last_act
            if new is not None and new.done:
                return new
            if (new is None and waited > 1.5) or waited > 9.0:
                return new     # valve never started moving (already there, or command lost)

    def _apply(self, kp, kd):
        for field, value in (("Kp", kp), ("Kd", kd)):
            cfg = self.link.snapshot()["config"] or {}
            if cfg.get(field) == value:
                continue
            self.link.set_param(field, value)
            end = time.monotonic() + 5.0
            while (self.link.snapshot()["config"] or {}).get(field) != value:
                if time.monotonic() > end:
                    raise _Abort(f"valve did not accept {field}={value}")
                self._sleep(0.05)

    def _evaluate(self, kp, kd, note):
        self._status(f"Setting Kp={kp}, Kd={kd} ({note})")
        self._apply(kp, kd)
        what = f"Kp={kp}, Kd={kd}"
        o = self._actuate("open", what)
        c = self._actuate("close", what)
        trial = Trial(len(self.trials) + 1, kp, kd, o, c, note)
        with self._lock:
            self.trials.append(trial)
            self._rank()
            self.version += 1
        self.link.say(f"Auto-tune trial {trial.n}: Kp={kp} Kd={kd} score={trial.score:.0f} "
                      f"(open {trial.cost_open:.0f}, close {trial.cost_close:.0f})")
        return trial

    def _rank(self):
        """Best = lowest mean score over every trial of that (Kp, Kd)."""
        groups = {}
        for t in self.trials:
            groups.setdefault((t.kp, t.kd), []).append(t.score)
        (kp, kd), scores = min(groups.items(), key=lambda kv: sum(kv[1]) / len(kv[1]))
        self.best = (kp, kd, sum(scores) / len(scores))

    @staticmethod
    def _neighbours(t, factor):
        kd_up = min(KD_MAX, int(round(max(t.kd * factor, t.kd + 50) / 10) * 10))
        kd_down = max(KD_MIN, int(round(t.kd / factor / 10) * 10))
        w = t.worst
        overshooting = w is not None and w.overshoot > 5
        if overshooting:   # brake earlier first
            order = [(t.kp, kd_up), (t.kp - 1, t.kd), (t.kp, kd_down), (t.kp + 1, t.kd)]
        else:              # stopping short / sluggish: less damping or more push first
            order = [(t.kp, kd_down), (t.kp + 1, t.kd), (t.kp, kd_up), (t.kp - 1, t.kd)]
        return [(kp, kd) for kp, kd in order if KP_MIN <= kp <= KP_MAX and (kp, kd) != (t.kp, t.kd)]

    # ------------------------------------------------------------------ search
    def _run(self, kp0, kd0):
        cfg = self.link.snapshot()["config"] or {}
        original = (cfg.get("Kp"), cfg.get("Kd"))
        try:
            if self.link.status != "connected":
                raise _Abort("no valve connected")
            if None in original:
                raise _Abort("valve settings unknown (config was never read)")
            self.link.say(f"Auto-tune started from Kp={kp0}, Kd={kd0}")
            self._actuate("close", "positioning, not scored")   # start from a known end

            best = self._evaluate(kp0, kd0, "start")
            tried = {(kp0, kd0)}
            factor, reason = 1.6, "actuation budget used"
            while True:
                if best.score <= GOOD_SCORE:
                    reason = "both directions already hit cleanly"
                    break
                if self._left() < 4:          # 2 for a trial + 2 kept for the verification run
                    break
                improved = False
                for kp, kd in self._neighbours(best, factor):
                    if (kp, kd) in tried:
                        continue
                    if self._left() < 4:
                        break
                    tried.add((kp, kd))
                    trial = self._evaluate(kp, kd, "search")
                    if trial.score < best.score * IMPROVE:
                        best, improved = trial, True
                        break
                if not improved:
                    factor = factor ** 0.5
                    if factor < 1.12:
                        reason = "no nearby setting is better"
                        break

            if self._left() >= 2:
                self._evaluate(self.best[0], self.best[1], "verify best")
            kp, kd, score = self.best
            self._status(f"Writing best values Kp={kp}, Kd={kd}")
            self._apply(kp, kd)
            msg = (f"Done ({reason}). Valve now set to Kp={kp}, Kd={kd} "
                   f"(score {score:.0f}, {self.used} actuations).")
            self._status(msg)
            self.link.say("Auto-tune: " + msg)
        except _Abort as e:
            msg = f"Auto-tune aborted: {e}."
            try:
                if self.link.status == "connected" and None not in original:
                    self._stop = False
                    self._apply(*original)
                    msg += f" Restored Kp={original[0]}, Kd={original[1]}."
            except _Abort:
                msg += " Could NOT restore the original Kp/Kd - check the settings."
            self._status(msg)
            self.link.say(msg)
        finally:
            self.running = False
            with self._lock:
                self.version += 1
