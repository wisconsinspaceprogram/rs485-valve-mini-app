"""Simulated valve(s) behind a pyserial-like object, for trying the app without hardware.

The control law mirrors the valve firmware; the motor model is a simple first-order lag.
When two valves answer at once their bytes are AND-ed together, like a bus collision.
"""
import random
import re
import time

RE_CMD = re.compile(rb"\{(\d\d),(\d\d)(?:,(\d{4}))?\}")


class SimValve:
    def __init__(self, address=99, pos=1500.0):
        self.cfg = dict(address=address, angleOpen=1024, angleClose=2048, Kp=3, Kd=1000,
                        errorTol=5, errorTolChange=1, extraVal=0)
        self.pos, self.vel = float(pos), 0.0
        self.target, self.hit, self.enabled, self.debug = 1000, 1, 1, 0
        self.last_error, self.rde = 0, 0
        self.tick, self.last_move, self.last_deriv = 0, -10000, 0
        self.last_print = random.randint(0, 20)   # valves free-run, so print phases differ
        self.pwm = 0

    def step(self):
        """Advance 1 ms. Returns a debug line or b''."""
        self.tick += 1
        adc = int(self.pos)
        error = adc - self.target
        if self.tick - self.last_deriv > 5:
            delta = error - self.last_error
            self.rde = int((self.rde * 20 + delta * 80) / 100)
            self.last_deriv, self.last_error = self.tick, error
        speed = 0
        if not self.hit:
            if abs(error) > 1000:
                speed = 1000 if error > 0 else -1000
            else:
                speed = error * self.cfg["Kp"] + int(self.rde * self.cfg["Kd"] / 1000)
        if speed > 0:
            speed = speed // 2 + 500
        elif speed < 0:
            speed = -((-speed) // 2) - 500
        speed = max(-999, min(999, speed))
        if abs(error) < self.cfg["errorTol"] and self.rde * 10 < self.cfg["errorTolChange"]:
            self.hit, speed = 1, 0
        if not self.enabled:
            speed = 0
        pwm = abs(speed)
        dt = self.tick - self.last_move
        if dt < 100 and pwm > 400:
            pwm = 400 + (pwm - 400) * dt // 100
        self.pwm = pwm
        drive = -pwm if speed > 0 else (pwm if speed < 0 else 0)   # +speed drives adc down
        self.vel += (drive / 999 * 2500 - self.vel) / 25
        self.pos = max(0.0, min(4095.0, self.pos + self.vel * 0.001))
        if self.debug and self.tick - self.last_print > 20:
            self.last_print = self.tick
            return f"{adc}, {self.target}, {pwm}, {self.rde * 10}\r\n".encode()
        return b""

    def command(self, addr, num, value):
        c = self.cfg
        if addr != c["address"] and addr != 98:
            return b""
        if 20 <= num <= 28 and value is None:
            return b""
        if num in (1, 2, 20):
            self.target = {1: c["angleOpen"], 2: c["angleClose"], 20: value}[num]
            self.hit, self.last_move = 0, self.tick
        elif 21 <= num <= 28:
            key = ["angleOpen", "angleClose", "Kp", "Kd", "errorTol", "errorTolChange",
                   "extraVal", "address"][num - 21]
            c[key] = value
        elif num == 29:
            self.debug = 1
        elif num == 30:
            self.debug = 0
        elif num == 31:
            return ("".join(f"{v}," for v in c.values()) + "\r\n").encode()
        elif num == 32:
            rng = c["angleOpen"] - c["angleClose"]
            pct = int((int(self.pos) - c["angleClose"]) * 100 / rng) if rng else 0
            return f"{{v,{c['address']:02d},{max(0, min(200, pct + 50)):03d}}}\r\n".encode()
        elif num == 40:
            self.enabled = 0
        elif num == 41:
            self.enabled = 1
        return b""


def _bits(data):
    out = []
    for byte in data:
        out += [0] + [(byte >> i) & 1 for i in range(8)] + [1]
    return out


def _merge(parts):
    """Combine simultaneous transmissions bit by bit with a random skew between senders
    (real valves are never bit-synchronous), then decode whatever a UART would see."""
    parts = [p for p in parts if p]
    if not parts:
        return b""
    if len(parts) == 1:
        return parts[0]
    streams = [[1] * random.randint(0, 40) + _bits(p) for p in parts]
    n = max(len(x) for x in streams) + 10
    line = [min(x[i] if i < len(x) else 1 for x in streams) for i in range(n)]
    out, i = bytearray(), 0
    while i < n - 9:
        if line[i] == 0:                      # start bit
            byte = sum(line[i + 1 + k] << k for k in range(8))
            if line[i + 9] == 1:              # valid stop bit, else framing error (dropped)
                out.append(byte)
            i += 10
        else:
            i += 1
    return bytes(out)


class SimSerial:
    def __init__(self, valves, timeout=0.005):
        self.valves, self.timeout = valves, timeout
        self._t0, self._ms = time.monotonic(), 0
        self._out, self._in = bytearray(), bytearray()
        self.is_open = True

    def _advance(self):
        now_ms = int((time.monotonic() - self._t0) * 1000)
        while self._ms < now_ms:
            self._ms += 1
            self._out += _merge([v.step() for v in self.valves])

    @property
    def in_waiting(self):
        self._advance()
        return len(self._out)

    def read(self, n=1):
        self._advance()
        if not self._out:
            time.sleep(self.timeout)
            self._advance()
        data = bytes(self._out[:n])
        del self._out[:n]
        return data

    def write(self, data):
        self._advance()
        self._in += data
        while True:
            m = RE_CMD.search(self._in)
            if not m:
                break
            addr, num = int(m.group(1)), int(m.group(2))
            value = int(m.group(3)) if m.group(3) else None
            del self._in[:m.end()]
            self._out += _merge([v.command(addr, num, value) for v in self.valves])
        del self._in[:-32]
        return len(data)

    def flush(self):
        pass

    def reset_input_buffer(self):
        self._out.clear()

    def close(self):
        self.is_open = False


SIM_PORTS = {
    "SIM: one valve @99": lambda: [SimValve(99)],
    "SIM: one valve @14": lambda: [SimValve(14, pos=2600)],
    "SIM: two valves @99": lambda: [SimValve(99, 1500), SimValve(99, 1900)],
    "SIM: two valves @12 and @99": lambda: [SimValve(12, 1500), SimValve(99, 1900)],
}


def open_sim(name):
    return SimSerial(SIM_PORTS[name]())
