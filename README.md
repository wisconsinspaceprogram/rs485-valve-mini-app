# Valve tuner

Standalone tuner / test app for ONE RS485 valve on a USB-RS485 adapter.

    pip install -r requirements.txt
    python main.py            # real hardware
    python main.py --sim      # adds simulated valves to the port list
    python tests/test_core.py # headless check of scan, settings and metrics against the simulator

- `core/protocol.py`   command formats and reply parsing
- `core/valve_link.py` serial worker thread: scan, verified writes, debug stream
- `core/metrics.py`    overshoot / settling / 5-95 % time per move
- `core/sim.py`        simulated valve(s) running the firmware control law
- `gui/tuner_gui.py`   Dear PyGui window
