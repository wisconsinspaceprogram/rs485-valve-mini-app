"""Run the auto-tuner against the simulator (short gap so it finishes quickly)."""
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from core.autotune import AutoTuner  # noqa: E402
from core.valve_link import ValveLink  # noqa: E402

for port in ["SIM: one valve @99", "SIM: lopsided valve @99 (overshoots one way)"]:
    link = ValveLink()
    link.connect(port)
    while link.snapshot()["status"] != "connected":
        time.sleep(0.1)
    tuner = AutoTuner(link)
    tuner.start(3, 2000, max_moves=30, gap=0.6)
    while tuner.snapshot()["running"]:
        time.sleep(0.2)
    s = tuner.snapshot()
    print("\n" + port)
    for t in s["trials"]:
        f = lambda m: "none" if m is None else f"err{m.final_error:+4d} over{m.overshoot:4.0f} settle{'  --' if m.settle_ms is None else f'{m.settle_ms:4.0f}'}"
        print(f" {t.n:2d} Kp={t.kp:2d} Kd={t.kd:4d} | open {f(t.open)} | close {f(t.close)} | score {t.score:6.1f} {t.note}")
    print(" ", s["status"], "| used", s["used"])
    cfg = link.snapshot()["config"]
    assert s["used"] <= 30 and (cfg["Kp"], cfg["Kd"]) == s["best"][:2]
    link.shutdown()
print("ALL OK")
