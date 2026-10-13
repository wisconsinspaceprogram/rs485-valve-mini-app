"""Valve RS485 command formatting and reply parsing (matches the valve main.c)."""
import re

BROADCAST = 98          # every valve accepts commands sent to 98
BAUD = 115200

CONFIG_FIELDS = ["address", "angleOpen", "angleClose", "Kp", "Kd",
                 "errorTol", "errorTolChange", "extraVal"]

SET_CMD = {"angleOpen": 21, "angleClose": 22, "Kp": 23, "Kd": 24,
           "errorTol": 25, "errorTolChange": 26, "extraVal": 27, "address": 28}

CMD_OPEN, CMD_CLOSE, CMD_GOTO = 1, 2, 20
CMD_DEBUG_ON, CMD_DEBUG_OFF, CMD_CONFIG, CMD_POSITION = 29, 30, 31, 32
CMD_MOTOR_OFF, CMD_MOTOR_ON = 40, 41

# {v,AA,PPP}
RE_POS = re.compile(rb"\{v,(\d\d),(\d{3})\}")
# debug line: "adc, target, pwm, dErr*10"   (separator is comma + space)
RE_DEBUG = re.compile(rb"(?<![\d-])(-?\d+), (-?\d+), (-?\d+), (-?\d+)$")
# config line: 8 values, comma after each, no braces: "99,1024,2048,3,1000,5,1,0,"
RE_CONFIG = re.compile(rb"(?<!\d)" + rb"(\d+)," * 8 + rb"$")


def cmd(addr, num, value=None):
    if value is None:
        return f"{{{addr:02d},{num:02d}}}".encode()
    return f"{{{addr:02d},{num:02d},{value:04d}}}".encode()


def classify(line: bytes):
    """Return ('debug', pos, target, pwm) | ('config', dict) | ('pos', addr, val) | ('junk', line) | None."""
    line = line.strip()
    if not line:
        return None
    m = RE_DEBUG.search(line)
    if m:
        pos, tgt, pwm = int(m.group(1)), int(m.group(2)), int(m.group(3))
        if 0 <= pos <= 4095 and 0 <= tgt <= 9999 and 0 <= pwm <= 1000:
            return ("debug", pos, tgt, pwm)
        return ("junk", line)
    m = RE_CONFIG.search(line)
    if m:
        return ("config", dict(zip(CONFIG_FIELDS, (int(g) for g in m.groups()))))
    m = RE_POS.search(line)
    if m:
        return ("pos", int(m.group(1)), int(m.group(2)))
    return ("junk", line)


def analyse_scan_reply(raw: bytes, sent: bytes = b""):
    """Split raw bytes captured after a position query into (frames, junk).

    frames: list of (addr, value). junk: leftover bytes that are not part of a clean
    frame (bus collisions show up here). Our own echoed command is ignored.
    """
    frames = [(int(a), int(v)) for a, v in RE_POS.findall(raw)]
    rest = RE_POS.sub(b"", raw)
    if sent:
        rest = rest.replace(sent, b"")
    junk = rest.replace(b"\r", b"").replace(b"\n", b"").replace(b"\x00", b"")
    return frames, junk
