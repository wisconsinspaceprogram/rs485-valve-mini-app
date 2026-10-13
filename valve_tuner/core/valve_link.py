"""Background serial link to a single valve on a USB-RS485 adapter.

One worker thread owns the port. The GUI queues jobs (connect, open, set Kp, ...) and
reads a snapshot of the state each frame.

Because the valve streams debug lines ~50x per second on a half-duplex bus, a command
from the PC can collide with one. So every send is timed into the gap right after a
debug line, and everything that can be verified is read back and retried.
"""
import queue
import threading
import time
from collections import deque

from . import protocol as P
from .metrics import MoveTracker
from .sim import SIM_PORTS, open_sim

DEBUG_PERIOD = 0.021      # firmware prints when tick - last_print > 20 ms
MIN_CMD_GAP = 0.006       # firmware can drop a command that follows another too closely
SCAN_TIMEOUT = 0.04       # per-address wait for a position reply
FLASH_WAIT = 0.12         # valve is stalled while it erases/writes its config page


class ValveLink:
    def __init__(self):
        self._lock = threading.Lock()
        self._jobs = queue.Queue()
        self._ser = None
        self._buf = bytearray()
        self._capture = None
        self._stop = False
        self._last_write = 0.0
        self._last_debug = 0.0
        self._line_count = 0
        self._cfg_seq = 0
        self._debug_count = 0
        self._junk_count = 0
        self._pending_cfg = {}
        self._last_t = None
        self._t_origin = time.monotonic()
        self._tracker = MoveTracker()

        # state read by the GUI (through snapshot())
        self.status = "disconnected"   # disconnected | scanning | connected | blocked
        self.message = "Not connected"
        self.progress = 0.0
        self.address = None
        self.config = None
        self.motor = None              # True/False once commanded, None = unknown
        self.samples = deque(maxlen=int(35 / DEBUG_PERIOD))
        self.log = deque(maxlen=200)
        self.log_seq = 0
        self.busy = False
        self.reported = None           # last {v,AA,PPP} value from the valve (0-200)
        self.poll_position = False     # set by the GUI checkbox
        self._last_poll = 0.0

        threading.Thread(target=self._run, daemon=True).start()

    # ------------------------------------------------------------------ public API
    def connect(self, port, baud=P.BAUD):
        self._jobs.put(lambda: self._job_connect(port, baud))

    def rescan(self):
        self._jobs.put(self._job_scan)

    def disconnect(self):
        self._jobs.put(self._job_disconnect)

    def open_valve(self):
        self._jobs.put(lambda: self._job_move(P.CMD_OPEN, None, "angleOpen"))

    def close_valve(self):
        self._jobs.put(lambda: self._job_move(P.CMD_CLOSE, None, "angleClose"))

    def move_to(self, raw):
        self._jobs.put(lambda: self._job_move(P.CMD_GOTO, int(raw), None))

    def set_param(self, field, value):
        self._jobs.put(lambda: self._job_set(field, int(value)))

    def refresh_config(self):
        self._jobs.put(lambda: self._read_config(self.address, tries=4))

    def set_motor(self, on):
        self._jobs.put(lambda: self._job_motor(on))

    def clear_history(self):
        with self._lock:
            self.samples.clear()
            self._tracker.reset()

    def shutdown(self):
        self._jobs.put(self._job_disconnect)
        self._jobs.put(lambda: setattr(self, "_stop", True))
        t_end = time.monotonic() + 1.0
        while not self._stop and time.monotonic() < t_end:
            time.sleep(0.02)

    def snapshot(self):
        with self._lock:
            return dict(status=self.status, message=self.message, progress=self.progress,
                        address=self.address, config=dict(self.config) if self.config else None,
                        motor=self.motor, samples=list(self.samples),
                        moves=list(self._tracker.moves), moves_version=self._tracker.version,
                        log=list(self.log), log_seq=self.log_seq, busy=self.busy,
                        streaming=time.monotonic() - self._last_debug < 0.3, reported=self.reported,)

    # ------------------------------------------------------------------ worker
    def _run(self):
        while not self._stop:
            try:
                job = self._jobs.get_nowait() if self._ser else self._jobs.get(timeout=0.05)
            except queue.Empty:
                job = None
            try:
                if job:
                    self.busy = True
                    job()
                elif self._ser:
                    self._pump(0.01)
                    self._poll_reported()
            except Exception as e:  # serial errors (adapter unplugged etc.)
                self._say(f"Link error: {e}")
                self._close_port()
                self._set(status="disconnected", message=f"Disconnected: {e}")
            finally:
                self.busy = False

    def _say(self, text):
        with self._lock:
            self.log.append(f"{time.strftime('%H:%M:%S')}  {text}")
            self.log_seq += 1

    def _set(self, **kw):
        with self._lock:
            for k, v in kw.items():
                setattr(self, k, v)

    def _close_port(self):
        if self._ser:
            try:
                self._ser.close()
            except Exception:
                pass
        self._ser = None
        self._set(address=None, config=None, motor=None)
        self._set(address=None, config=None, motor=None, reported=None)

    # ------------------------------------------------------------------ serial I/O
    def _pump(self, duration):
        """Read and parse incoming bytes for `duration` seconds."""
        end = time.monotonic() + duration
        while True:
            n = self._ser.in_waiting
            data = self._ser.read(n if n else 1)
            if data:
                self._feed(data)
            if time.monotonic() >= end:
                return

    def _feed(self, data):
        if self._capture is not None:
            self._capture += data
        self._buf += data
        while True:
            i = self._buf.find(b"\n")
            if i < 0:
                break
            line = bytes(self._buf[:i])
            del self._buf[:i + 1]
            self._handle_line(line)
        if len(self._buf) > 512:
            del self._buf[:-64]

    def _handle_line(self, line):
        self._line_count += 1
        item = P.classify(line)
        if item is None:
            return
        if item[0] == "debug":
            now = time.monotonic()
            self._debug_count += 1
            self._last_debug = now
            # USB adapters deliver bytes in bursts, so rebuild even timestamps from the
            # known print period and only nudge them toward the arrival time.
            arrival = now - self._t_origin
            if self._last_t is None or arrival - (self._last_t + DEBUG_PERIOD) > 0.045:
                t = arrival
            else:
                expected = self._last_t + DEBUG_PERIOD
                t = expected + 0.02 * (arrival - expected)
            self._last_t = t
            _, pos, tgt, pwm = item
            with self._lock:
                self.samples.append((t, pos, tgt, pwm))
                self._tracker.add(t, pos, tgt, pwm, self._label)
        elif item[0] == "config":
            with self._lock:
                self._pending_cfg = item[1]
            self._cfg_seq += 1
        elif item[0] == "junk":
            self._junk_count += 1
        elif item[0] == "pos":
            if item[1] == self.address:
                self._set(reported=item[2])

    def _label(self, target):
        c = self.config or {}
        if target == c.get("angleOpen"):
            return "Open"
        if target == c.get("angleClose"):
            return "Close"
        return "Move"

    def _streaming(self):
        return time.monotonic() - self._last_debug < 0.1

    def _send(self, data):
        wait = MIN_CMD_GAP - (time.monotonic() - self._last_write)
        if wait > 0:
            self._pump(wait)
        if self._streaming():
            # wait for the end of the next debug line, then talk in the quiet gap
            n, end = self._line_count, time.monotonic() + 0.05
            while self._line_count == n and time.monotonic() < end:
                self._pump(0.001)
        self._ser.write(data)
        self._ser.flush()
        self._last_write = time.monotonic()

    def _query_raw(self, data, timeout):
        """Send a position query and return the raw bytes that came back."""
        self._capture = bytearray()
        self._send(data)
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self._pump(0.003)
            if P.RE_POS.search(self._capture):
                self._pump(0.012)   # linger: a second valve would show up here
                break
        raw, self._capture = bytes(self._capture), None
        return raw

    def _poll_reported(self):
        if (self.poll_position and self.status == "connected"
                and time.monotonic() - self._last_poll > 0.25):
            self._last_poll = time.monotonic()
            self._send(P.cmd(self.address, P.CMD_POSITION))

    # ------------------------------------------------------------------ jobs
    def _job_connect(self, port, baud):
        self._close_port()
        try:
            if port in SIM_PORTS:
                self._ser = open_sim(port)
            else:
                import serial
                self._ser = serial.Serial(port, baud, timeout=0.005, rtscts=False,
                                          dsrdtr=False, xonxoff=False)
                self._ser.reset_input_buffer()
        except Exception as e:
            self._ser = None
            self._set(status="disconnected", message=f"Could not open {port}: {e}")
            self._say(f"Could not open {port}: {e}")
            return
        self._say(f"Opened {port} @ {baud}")
        self._job_scan()

    def _job_disconnect(self):
        if self._ser and self.address is not None:
            try:
                for _ in range(3):
                    self._send(P.cmd(self.address, P.CMD_DEBUG_OFF))
                self._pump(0.03)
            except Exception:
                pass
        self._close_port()
        self._set(status="disconnected", message="Not connected")
        self._say("Disconnected")

    def _quiet_bus(self):
        """Turn debug mode off on anything listening, until the bus is silent."""
        for _ in range(4):
            self._ser.write(P.cmd(P.BROADCAST, P.CMD_DEBUG_OFF))
            time.sleep(MIN_CMD_GAP)
            for a in range(100):      # old firmware may not know the broadcast address
                self._ser.write(P.cmd(a, P.CMD_DEBUG_OFF))
                self._ser.flush()
                time.sleep(0.004)
            self._last_write = time.monotonic()
            self._pump(0.05)          # drain whatever was in flight
            self._capture = bytearray()
            self._pump(0.12)
            noisy, self._capture = len(self._capture), None
            if not noisy:
                return True
        return False

    def _job_scan(self):
        if not self._ser:
            return
        with self._lock:
            self.samples.clear()
            self._tracker.reset()
        self._last_t = None
        self._set(status="scanning", message="Quieting bus...", progress=0.0,
                  address=None, config=None, motor=None)
        if not self._quiet_bus():
            self._block("Bus never went quiet. Something keeps transmitting - check wiring "
                        "and that only valves are on the bus.")
            return

        self._set(message="Scanning addresses 00-99...")
        found, collided = {}, set()
        addrs = [a for a in range(100) if a != P.BROADCAST]
        for i, a in enumerate(addrs):
            sent = P.cmd(a, P.CMD_POSITION)
            frames, junk = P.analyse_scan_reply(self._query_raw(sent, SCAN_TIMEOUT), sent)
            for fa, _ in frames:
                found[fa] = found.get(fa, 0) + 1
            if junk or len(frames) > 1:
                collided.add(a)
            self._set(progress=(i + 1) / len(addrs))

        # Hammer each responder: two valves on one address collide into garbage
        # (or answer back-to-back), which a single valve never does.
        for a in list(found):
            sent = P.cmd(a, P.CMD_POSITION)
            for _ in range(8):
                frames, junk = P.analyse_scan_reply(self._query_raw(sent, SCAN_TIMEOUT), sent)
                if junk or len(frames) != 1 or frames[0][0] != a:
                    collided.add(a)

        # Broadcast check: with one valve there is exactly one clean answer.
        sent = P.cmd(P.BROADCAST, P.CMD_POSITION)
        broadcast_bad = False
        for _ in range(4):
            frames, junk = P.analyse_scan_reply(self._query_raw(sent, SCAN_TIMEOUT), sent)
            for fa, _ in frames:
                found.setdefault(fa, 1)
            if junk or len(frames) > 1:
                broadcast_bad = True

        addrs_found = sorted(found)
        self._say(f"Scan: responders={[f'{a:02d}' for a in addrs_found]} "
                  f"collisions={[f'{a:02d}' for a in sorted(collided)]} "
                  f"broadcast={'collision' if broadcast_bad else 'clean'}")
        if len(addrs_found) > 1:
            self._block("More than one valve on the bus (addresses "
                        + ", ".join(f"{a:02d}" for a in addrs_found) + "). Connect only one.")
        elif collided:
            where = ", ".join(f"{a:02d}" for a in sorted(collided))
            self._block(f"Replies collided at address {where}: more than one valve is "
                        "probably sharing that address. Connect only one.")
        elif broadcast_bad:
            self._block("Broadcast query collided: more than one valve seems to be connected.")
        elif not addrs_found:
            self._block("No valve responded. Check power, A/B wiring and the COM port.")
        else:
            self._attach(addrs_found[0])

    def _block(self, text):
        self._set(status="blocked", message=text, progress=1.0)
        self._say(text)

    def _attach(self, addr):
        self._set(address=addr, message=f"Valve {addr:02d} found, reading config...")
        if self._read_config(addr, tries=5) is None:
            self._say("Valve did not report its config (command 31). Settings will show '?'.")
        got_debug = False
        for _ in range(5):
            self._send(P.cmd(addr, P.CMD_DEBUG_ON))
            self._pump(0.12)
            if self._streaming():
                got_debug = True
                break
        if not got_debug:
            self._say("No debug stream after command 29 - plots will stay empty.")
        else:
            # Strongest duplicate-address check: one valve prints a clean line every
            # ~21 ms. Two valves on this address give double the lines or garbled ones.
            window = 0.7
            self._pump(0.05)
            self._debug_count = self._junk_count = 0
            self._pump(window)
            lines, junk = self._debug_count, self._junk_count
            if lines > 1.35 * window / DEBUG_PERIOD or junk >= 3:
                for _ in range(4):
                    self._ser.write(P.cmd(addr, P.CMD_DEBUG_OFF))
                    self._ser.flush()
                    time.sleep(0.013)
                self._set(address=None, config=None)
                self._block(f"Two or more valves are answering at address {addr:02d} "
                            f"(debug stream had {lines} lines and {junk} garbled in {window:.1f} s; "
                            f"one valve gives ~{window / DEBUG_PERIOD:.0f}). Connect only one.")
                return
            with self._lock:
                self.samples.clear()
                self._tracker.reset()
        self._set(status="connected", progress=1.0,
                  message=f"Connected to valve {addr:02d} - the only valve on the bus")
        self._say(f"Connected to valve {addr:02d}")

    def _read_config(self, addr, tries=3):
        if not self._ser or addr is None:
            return None
        for _ in range(tries):
            seq = self._cfg_seq
            self._send(P.cmd(addr, P.CMD_CONFIG))
            end = time.monotonic() + 0.15
            while self._cfg_seq == seq and time.monotonic() < end:
                self._pump(0.004)
            if self._cfg_seq != seq and self._pending_cfg.get("address") == addr:
                self._set(config=dict(self._pending_cfg))
                return self._pending_cfg
        return None

    def _job_set(self, field, value):
        if self.status != "connected":
            return
        old = self.address
        for attempt in range(5):
            self._send(P.cmd(old, P.SET_CMD[field], value))
            self._pump(FLASH_WAIT)
            cfg = None
            for a in ([value, old] if field == "address" else [old]):
                cfg = self._read_config(a, tries=2)
                if cfg:
                    break
            if cfg and cfg[field] == value:
                if field == "address":
                    self._set(address=value,
                              message=f"Connected to valve {value:02d} - the only valve on the bus")
                self._say(f"{field} = {value} confirmed by valve"
                          + (f" (attempt {attempt + 1})" if attempt else ""))
                return
        self._say(f"FAILED to set {field} to {value}: valve never reported the new value")

    def _latest_target(self):
        with self._lock:
            return self.samples[-1][2] if self.samples else None

    def _job_move(self, num, value, cfg_key):
        if self.status != "connected":
            return
        expected = value if cfg_key is None else (self.config or {}).get(cfg_key)
        name = {P.CMD_OPEN: "Open", P.CMD_CLOSE: "Close"}.get(num, f"Move to {value}")
        packet = P.cmd(self.address, num, value)
        if expected is None or not self._streaming() or self._latest_target() == expected:
            self._send(packet)          # nothing to verify against
            self._say(f"{name} sent")
            return
        for attempt in range(5):
            self._send(packet)
            end = time.monotonic() + 0.12
            while time.monotonic() < end:
                self._pump(0.005)
                if self._latest_target() == expected:
                    self._say(f"{name} accepted" + (f" (attempt {attempt + 1})" if attempt else ""))
                    return
        self._say(f"{name} FAILED: valve target never changed to {expected}")

    def _job_motor(self, on):
        if self.status != "connected":
            return
        for _ in range(3):    # no read-back exists for this, so repeat it (harmless)
            self._send(P.cmd(self.address, P.CMD_MOTOR_ON if on else P.CMD_MOTOR_OFF))
            self._pump(0.025)
        self._set(motor=bool(on))
        self._say("Motor enable sent" if on else "Motor disable sent (not confirmable)")
