"""Headless check of the link + metrics against the simulated valve.  Run: python tests/test_core.py"""
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from core.valve_link import ValveLink  # noqa: E402


def wait(link, cond, timeout=15):
    end = time.time() + timeout
    while time.time() < end:
        s = link.snapshot()
        if cond(s):
            return s
        time.sleep(0.05)
    raise AssertionError(f"timeout; status={s['status']} msg={s['message']} log={s['log'][-5:]}")


def idle(s):
    return not s["busy"]


def test_single():
    link = ValveLink()
    link.connect("SIM: one valve @99")
    s = wait(link, lambda s: s["status"] in ("connected", "blocked"))
    assert s["status"] == "connected" and s["address"] == 99, s["message"]
    assert s["config"]["Kp"] == 3 and s["config"]["angleClose"] == 2048
    wait(link, lambda s: s["streaming"])

    link.set_param("Kp", 4)
    wait(link, lambda s: s["config"]["Kp"] == 4)
    link.set_param("address", 14)
    s = wait(link, lambda s: s["address"] == 14 and s["config"]["address"] == 14)

    link.close_valve()
    s = wait(link, lambda s: s["moves"] and s["moves"][-1].done)
    m = s["moves"][-1]
    print("close:", m)
    assert m.label == "Close" and m.first and m.target == 2048 and m.rise_ms and m.settle_ms
    link.open_valve()
    s = wait(link, lambda s: len(s["moves"]) == 2 and s["moves"][-1].done)
    m = s["moves"][-1]
    print("open: ", m)
    assert m.label == "Open" and m.travel < -900 and 50 < m.rise_ms < 3000
    link.move_to(1500)
    s = wait(link, lambda s: len(s["moves"]) == 3 and s["moves"][-1].done)
    assert s["moves"][-1].label == "Move" and s["moves"][-1].target == 1500
    link.shutdown()
    print("\n".join(s["log"]))


def test_blocked(port, needle):
    link = ValveLink()
    link.connect(port)
    s = wait(link, lambda s: s["status"] in ("connected", "blocked"))
    print(port, "->", s["status"], "|", s["message"])
    assert s["status"] == "blocked" and needle in s["message"], s["message"]
    link.shutdown()


if __name__ == "__main__":
    test_single()
    for _ in range(4):
        test_blocked("SIM: two valves @99", "valve")
    test_blocked("SIM: two valves @12 and @99", "12, 99")
    link = ValveLink()
    link.connect("SIM: one valve @14")
    assert wait(link, lambda s: s["status"] in ("connected", "blocked"))["address"] == 14
    link.shutdown()
    print("ALL OK")
