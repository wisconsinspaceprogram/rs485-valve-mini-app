import time
from collections import deque

import dearpygui.dearpygui as dpg

from core.autotune import AutoTuner
from core.sim import SIM_PORTS

link = None
tuner = None
show_sim = False
paused = False
window_s = 10.0

# field, label, min, max  (rows in the settings tables)
MAIN_SETTINGS = [
    ("Kp", "Kp", 0, 9999),
    ("Kd", "Kd", 0, 9999),
    ("angleOpen", "Open position", 0, 4095),
    ("angleClose", "Close position", 0, 4095),
    ("address", "Address", 0, 99),
]
HIDDEN_SETTINGS = [
    ("errorTol", "Error tolerance", 0, 9999),
    ("errorTolChange", "Error change tol.", 0, 9999),
]
CONTROL_TAGS = ["btn_open", "btn_close", "btn_go", "btn_motor_on", "btn_motor_off", "btn_refresh_cfg"]

GREEN, RED, YELLOW, GREY = (90, 220, 110), (240, 90, 90), (240, 200, 70), (160, 160, 160)

_shown_cfg = {}
_shown_moves = -1
_shown_log = -1
_shown_tune = -1


# ----------------------------------------------------------------------------- callbacks
def list_ports():
    ports = []
    try:
        from serial.tools import list_ports as lp
        ports = [p.device for p in lp.comports()]
    except Exception:
        pass
    return ports + (list(SIM_PORTS) if show_sim else [])


def cb_refresh_ports():
    ports = list_ports()
    dpg.configure_item("port_combo", items=ports)
    if ports and dpg.get_value("port_combo") not in ports:
        dpg.set_value("port_combo", ports[0])


def cb_connect():
    port = dpg.get_value("port_combo")
    if port:
        link.connect(port, int(dpg.get_value("baud_input")))


# Manual actuation limit: at most MAX_ACTUATIONS in any WINDOW_S seconds. Four back to
# back is fine, then the valve has to rest until the oldest one ages out of the window.
MAX_ACTUATIONS = 4
WINDOW_S = 20.0
_actuations = deque()


def _cooldown_left():
    now = time.monotonic()
    while _actuations and now - _actuations[0] >= WINDOW_S:
        _actuations.popleft()
    if len(_actuations) < MAX_ACTUATIONS:
        return 0.0
    return WINDOW_S - (now - _actuations[0])


def _actuate(action):
    """Run a manual open/close/move unless the valve needs to cool down."""
    if tuner.running:
        dpg.set_value("cooldown_text", "Auto-tune is running.\nStop it before moving the valve by hand.")
        dpg.configure_item("cooldown_popup", show=True)
        return
    if _cooldown_left() > 0:
        dpg.configure_item("cooldown_popup", show=True)
        return
    _actuations.append(time.monotonic())
    action()


def cb_go():
    _actuate(lambda: link.move_to(max(0, min(4095, int(dpg.get_value("goto_input"))))))


def cb_set(sender, app_data, user_data):
    field, lo, hi = user_data
    value = int(dpg.get_value(f"new_{field}"))
    if not lo <= value <= hi or (field == "address" and value == 98):
        why = "98 is the broadcast address" if value == 98 else f"allowed range is {lo}-{hi}"
        dpg.set_value("set_error", f"{field} = {value} not sent: {why}")
        return
    dpg.set_value("set_error", "")
    link.set_param(field, value)


def cb_window(sender, app_data):
    global window_s
    window_s = float(app_data.split()[0])


def cb_pause():
    global paused
    paused = not paused
    dpg.configure_item("btn_pause", label="Resume" if paused else "Pause")


# ----------------------------------------------------------------------------- layout
def _button_theme(color):
    with dpg.theme() as theme:
        with dpg.theme_component(dpg.mvButton):
            dpg.add_theme_color(dpg.mvThemeCol_Button, color)
            dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered, [min(255, c + 30) for c in color])
            dpg.add_theme_color(dpg.mvThemeCol_ButtonActive, [min(255, c + 50) for c in color])
    return theme


def _settings_table(rows):
    with dpg.table(header_row=True, borders_innerH=True, policy=dpg.mvTable_SizingFixedFit):
        dpg.add_table_column(label="Setting", width_fixed=True, init_width_or_weight=125)
        dpg.add_table_column(label="Valve reports", width_fixed=True, init_width_or_weight=95)
        dpg.add_table_column(label="New value", width_fixed=True, init_width_or_weight=95)
        dpg.add_table_column(label="", width_fixed=True, init_width_or_weight=50)
        for field, label, lo, hi in rows:
            with dpg.table_row():
                dpg.add_text(label)
                dpg.add_text("?", tag=f"cur_{field}", color=GREEN)
                dpg.add_input_int(tag=f"new_{field}", width=90, step=0, min_value=lo, max_value=hi,
                                  min_clamped=True, max_clamped=True, on_enter=True,
                                  callback=cb_set, user_data=(field, lo, hi))
                dpg.add_button(label="Set", tag=f"set_{field}", width=45,
                               callback=cb_set, user_data=(field, lo, hi))
                CONTROL_TAGS.append(f"set_{field}")


def cb_autotune_start():
    tuner.start(dpg.get_value("at_kp"), dpg.get_value("at_kd"),
                dpg.get_value("at_max"), dpg.get_value("at_gap"))


def _build_autotune_tab():
    dpg.add_text("Searches for the Kp / Kd that make the valve land on BOTH end points most reliably. "
                 "Each candidate is tried once opening and once closing and is judged by its worse "
                 "direction, so a setting that only overshoots one way still counts as bad.",
                 wrap=1300, color=GREY)
    with dpg.group(horizontal=True):
        dpg.add_text("Start Kp")
        dpg.add_input_int(tag="at_kp", default_value=3, width=70, step=0, min_value=1, max_value=20,
                          min_clamped=True, max_clamped=True)
        dpg.add_text("  Start Kd")
        dpg.add_input_int(tag="at_kd", default_value=2000, width=80, step=0, min_value=0,
                          max_value=9999, min_clamped=True, max_clamped=True)
        dpg.add_text("  Max actuations")
        dpg.add_input_int(tag="at_max", default_value=30, width=70, step=0, min_value=5,
                          max_value=200, min_clamped=True, max_clamped=True)
        dpg.add_text("  Seconds between actuations")
        dpg.add_input_float(tag="at_gap", default_value=3.0, width=70, step=0, format="%.1f",
                            min_value=3.0, max_value=60.0, min_clamped=True, max_clamped=True)
        dpg.add_button(label="Start auto-tune", tag="at_start", width=140, callback=cb_autotune_start)
        dpg.add_button(label="Stop", tag="at_stop", width=80, callback=lambda: tuner.stop())
    dpg.bind_item_theme("at_start", _button_theme([40, 120, 60]))
    dpg.bind_item_theme("at_stop", _button_theme([150, 50, 50]))
    dpg.add_text("Idle", tag="at_status", wrap=1300)
    dpg.add_progress_bar(tag="at_progress", width=-1, height=8)
    dpg.add_text("", tag="at_best", color=GREEN)
    with dpg.plot(height=230, width=-1):
        dpg.add_plot_legend()
        dpg.add_plot_axis(dpg.mvXAxis, label="seconds (0 = now)", tag="at_x")
        with dpg.plot_axis(dpg.mvYAxis, label="raw position", tag="at_y"):
            dpg.add_line_series([], [], label="Valve position", tag="at_s_pos")
            dpg.add_line_series([], [], label="Desired position", tag="at_s_tgt")
    dpg.set_axis_limits("at_x", -30, 0)
    with dpg.table(tag="at_table", header_row=True, borders_innerH=True, borders_outerH=True,
                   row_background=True, scrollY=True, height=300):
        for col in ["Trial", "Kp", "Kd", "Open: final error", "Open: overshoot", "Open: settling",
                    "Close: final error", "Close: overshoot", "Close: settling", "Score", "Note"]:
            dpg.add_table_column(label=col)
    dpg.add_text("Score: lower is better. It is the worse direction's (final error + overshoot, in "
                 "counts, with a penalty for not settling) plus a quarter of the better direction's. "
                 "When the run finishes the best Kp/Kd are written to the valve; Stop restores the "
                 "values the valve had before.", wrap=1300, color=GREY)


def _update_autotune(s):
    global _shown_tune
    t = tuner.snapshot()
    connected = s["status"] == "connected"
    dpg.configure_item("at_start", enabled=connected and not t["running"])
    dpg.configure_item("at_stop", enabled=t["running"])
    dpg.set_value("at_progress", t["used"] / max(1, t["max_moves"]))
    if s["samples"]:
        t_now = s["samples"][-1][0]
        vis = [p for p in s["samples"] if p[0] >= t_now - 30]
        xs = [p[0] - t_now for p in vis]
        pos, tgt = [p[1] for p in vis], [p[2] for p in vis]
        dpg.set_value("at_s_pos", [xs, pos])
        dpg.set_value("at_s_tgt", [xs, tgt])
        lo, hi = min(min(pos), min(tgt)), max(max(pos), max(tgt))
        pad = max(40, (hi - lo) * 0.08)
        dpg.set_axis_limits("at_y", lo - pad, hi + pad)
    if t["version"] == _shown_tune:
        return
    _shown_tune = t["version"]
    dpg.set_value("at_status", t["status"])
    best = t["best"]
    dpg.set_value("at_best", f"Best so far: Kp = {best[0]}, Kd = {best[1]}   (score {best[2]:.0f})"
                  if best else "")
    for row in dpg.get_item_children("at_table", 1):
        dpg.delete_item(row)
    for tr in reversed(t["trials"]):
        is_best = best is not None and (tr.kp, tr.kd) == best[:2]
        with dpg.table_row(parent="at_table"):
            dpg.add_text(str(tr.n))
            dpg.add_text(str(tr.kp), color=GREEN if is_best else (255, 255, 255))
            dpg.add_text(str(tr.kd), color=GREEN if is_best else (255, 255, 255))
            for m in (tr.open, tr.close):
                if m is None:
                    for _ in range(3):
                        dpg.add_text("no move", color=RED)
                    continue
                dpg.add_text(f"{m.final_error:+d}")
                dpg.add_text(f"{m.overshoot:.0f}", color=RED if m.overshoot_pct > 5 else (255, 255, 255))
                if m.settle_ms is None:
                    dpg.add_text("not settled", color=RED)
                else:
                    dpg.add_text(_fmt_ms(m.settle_ms))
            dpg.add_text(f"{tr.score:.0f}")
            dpg.add_text(tr.note + ("  <- best" if is_best else ""))


def build(valve_link, sim=False):
    global link, show_sim, tuner
    link, show_sim = valve_link, sim
    tuner = AutoTuner(link)

    with dpg.window(label="Cool down", tag="cooldown_popup", modal=True, show=False,
                    no_resize=True, no_collapse=True, width=380, height=130, pos=(500, 330)):
        dpg.add_text("", tag="cooldown_text")
        dpg.add_spacer(height=6)
        dpg.add_button(label="OK", width=80,
                       callback=lambda: dpg.configure_item("cooldown_popup", show=False))

    with dpg.window(tag="Main Window", no_title_bar=True, no_move=True):
        # ---- connection bar
        with dpg.group(horizontal=True):
            ports = list_ports()
            dpg.add_text("Port")
            dpg.add_combo(ports, tag="port_combo", width=230, default_value=ports[0] if ports else "")
            dpg.add_button(label="Refresh", callback=cb_refresh_ports)
            dpg.add_text("  Baud")
            dpg.add_input_int(tag="baud_input", default_value=115200, width=90, step=0)
            dpg.add_button(label="Connect & Scan", callback=cb_connect)
            dpg.add_button(label="Rescan", callback=lambda: link.rescan())
            dpg.add_button(label="Disconnect", callback=lambda: link.disconnect())
        dpg.add_text("Not connected", tag="status_text", color=GREY, wrap=1300)
        dpg.add_progress_bar(tag="scan_progress", width=-1, height=8, show=False)
        dpg.add_separator()

        with dpg.tab_bar():
            with dpg.tab(label="Tune & test"):
                with dpg.group(horizontal=True):
                    # ---- left column: live values, actuation, settings, log
                    with dpg.child_window(width=420, border=False):
                        dpg.add_text("Valve")
                        with dpg.table(header_row=False, policy=dpg.mvTable_SizingFixedFit):
                            dpg.add_table_column(width_fixed=True, init_width_or_weight=140)
                            dpg.add_table_column()
                            for tag, label in [("v_addr", "Address"), ("v_pos", "Position (raw)"),
                                               ("v_tgt", "Desired (raw)"), ("v_pwm", "PWM"),
                                               ("v_rep", "Valve-reported %"),
                                               ("v_motor", "Motor")]:
                                with dpg.table_row():
                                    dpg.add_text(label)
                                    dpg.add_text("-", tag=tag)
                        dpg.add_checkbox(label="Poll valve-reported % open (command 32)",
                                         callback=lambda s, a: setattr(link, "poll_position", a))
                        dpg.add_spacer(height=4)
                        with dpg.group(horizontal=True):
                            dpg.add_button(label="OPEN", tag="btn_open", width=200, height=45,
                                           callback=lambda: _actuate(link.open_valve))
                            dpg.add_button(label="CLOSE", tag="btn_close", width=200, height=45,
                                           callback=lambda: _actuate(link.close_valve))
                        dpg.bind_item_theme("btn_open", _button_theme([40, 120, 60]))
                        dpg.bind_item_theme("btn_close", _button_theme([150, 50, 50]))
                        with dpg.group(horizontal=True):
                            dpg.add_text("Move to raw position")
                            dpg.add_input_int(tag="goto_input", width=100, step=0, default_value=1500,
                                              min_value=0, max_value=4095, min_clamped=True,
                                              max_clamped=True, on_enter=True, callback=cb_go)
                            dpg.add_button(label="Go", tag="btn_go", width=50, callback=cb_go)
                        with dpg.group(horizontal=True):
                            dpg.add_button(label="Enable motor", tag="btn_motor_on", width=200,
                                           callback=lambda: link.set_motor(True))
                            dpg.add_button(label="Disable motor", tag="btn_motor_off", width=200,
                                           callback=lambda: link.set_motor(False))

                        dpg.add_spacer(height=6)
                        dpg.add_separator()
                        dpg.add_text("Settings (saved in the valve's flash)")
                        _settings_table(MAIN_SETTINGS)
                        dpg.add_text("Address: 12-35 for use with the ECU, 99 = unassigned", color=GREY)
                        with dpg.collapsing_header(label="Tolerances", default_open=False):
                            _settings_table(HIDDEN_SETTINGS)
                        dpg.add_button(label="Re-read settings from valve", tag="btn_refresh_cfg",
                                       callback=lambda: link.refresh_config())
                        dpg.add_text("", tag="set_error", color=RED, wrap=400)

                        dpg.add_separator()
                        dpg.add_text("Log")
                        dpg.add_input_text(tag="log_box", multiline=True, readonly=True, width=-1, height=-1)

                    # ---- right column: plots + metrics
                    with dpg.child_window(border=False):
                        with dpg.group(horizontal=True):
                            dpg.add_text("Window")
                            dpg.add_radio_button(["10 s", "30 s"], horizontal=True, callback=cb_window)
                            dpg.add_button(label="Pause", tag="btn_pause", width=80, callback=cb_pause)
                            dpg.add_button(label="Clear", width=80, callback=lambda: link.clear_history())
                        with dpg.plot(height=330, width=-1, tag="plot_pos"):
                            dpg.add_plot_legend()
                            dpg.add_plot_axis(dpg.mvXAxis, tag="x_pos", no_tick_labels=True)
                            with dpg.plot_axis(dpg.mvYAxis, label="raw position", tag="y_pos"):
                                dpg.add_line_series([], [], label="Valve position", tag="s_pos")
                                dpg.add_line_series([], [], label="Desired position", tag="s_tgt")
                        with dpg.plot(height=190, width=-1, tag="plot_pwm"):
                            dpg.add_plot_legend()
                            dpg.add_plot_axis(dpg.mvXAxis, label="seconds (0 = now)", tag="x_pwm")
                            with dpg.plot_axis(dpg.mvYAxis, label="PWM", tag="y_pwm"):
                                dpg.add_line_series([], [], label="PWM", tag="s_pwm")
                        dpg.set_axis_limits("y_pwm", 0, 1050)

                        dpg.add_text("Move metrics (newest first)")
                        with dpg.table(tag="metrics_table", header_row=True, borders_innerH=True,
                                       borders_outerH=True, row_background=True):
                            for col in ["No.", "Move", "From -> To", "Travel", "5-95 % time",
                                        "Overshoot", "Settling time", "Final error"]:
                                dpg.add_table_column(label=col)
                        dpg.add_text("Settling = time from command until position stays within +/-2 % of travel "
                                     "(min 5 counts). Each move is measured from wherever the valve actually "
                                     "started, so a first move (*) from an unknown position is still valid, "
                                     "just over a different travel.", color=GREY, wrap=900)

            with dpg.tab(label="Auto-tune"):
                _build_autotune_tab()


# ----------------------------------------------------------------------------- per-frame update
def _fmt_ms(v):
    return "-" if v is None else f"{v:.0f} ms"


def update():
    global _shown_moves, _shown_log
    s = link.snapshot()
    connected = s["status"] == "connected"

    color = {"connected": GREEN, "scanning": YELLOW, "blocked": RED}.get(s["status"], GREY)
    dpg.set_value("status_text", s["message"])
    dpg.configure_item("status_text", color=color)
    dpg.configure_item("scan_progress", show=s["status"] == "scanning")
    dpg.set_value("scan_progress", s["progress"])
    for tag in CONTROL_TAGS:
        dpg.configure_item(tag, enabled=connected)

    # live values
    last = s["samples"][-1] if s["samples"] and s["streaming"] else None
    dpg.set_value("v_addr", f"{s['address']:02d}" if s["address"] is not None else "-")
    dpg.set_value("v_pos", str(last[1]) if last else "-")
    dpg.set_value("v_tgt", str(last[2]) if last else "-")
    dpg.set_value("v_pwm", str(last[3]) if last else "-")
    rep = s["reported"]
    show_rep = connected and link.poll_position and rep is not None
    dpg.set_value("v_rep", f"{rep - 50} % open   (sent as {rep:03d})" if show_rep else "-")
    dpg.set_value("v_motor", {True: "enabled", False: "DISABLED", None: "enabled (power-up default)"}
                  [s["motor"]] if connected else "-")

    # settings: show what the valve reported; pre-fill the inputs when it changes
    cfg = s["config"] or {}
    for field, *_ in MAIN_SETTINGS + HIDDEN_SETTINGS:
        value = cfg.get(field)
        if _shown_cfg.get(field, "unset") != value:
            _shown_cfg[field] = value
            dpg.set_value(f"cur_{field}", "?" if value is None else str(value))
            if value is not None:
                dpg.set_value(f"new_{field}", value)

    # plots
    if not paused and s["samples"]:
        t_now = s["samples"][-1][0]
        vis = [p for p in s["samples"] if p[0] >= t_now - window_s]
        xs = [p[0] - t_now for p in vis]
        pos, tgt = [p[1] for p in vis], [p[2] for p in vis]
        dpg.set_value("s_pos", [xs, pos])
        dpg.set_value("s_tgt", [xs, tgt])
        dpg.set_value("s_pwm", [xs, [p[3] for p in vis]])
        lo, hi = min(min(pos), min(tgt)), max(max(pos), max(tgt))
        pad = max(40, (hi - lo) * 0.08)
        dpg.set_axis_limits("y_pos", lo - pad, hi + pad)
    elif not s["samples"] and not paused:
        for tag in ("s_pos", "s_tgt", "s_pwm"):
            dpg.set_value(tag, [[], []])
    if not paused:
        dpg.set_axis_limits("x_pos", -window_s, 0)
        dpg.set_axis_limits("x_pwm", -window_s, 0)

    # metrics table
    if s["moves_version"] != _shown_moves:
        _shown_moves = s["moves_version"]
        for row in dpg.get_item_children("metrics_table", 1):
            dpg.delete_item(row)
        for m in reversed(s["moves"][-8:]):
            with dpg.table_row(parent="metrics_table"):
                dpg.add_text(f"{m.index}{'*' if m.first else ''}")
                dpg.add_text(m.label + ("" if m.done else " ..."))
                dpg.add_text(f"{m.start} -> {m.target}")
                dpg.add_text(f"{abs(m.travel)}")
                dpg.add_text(_fmt_ms(m.rise_ms))
                dpg.add_text(f"{m.overshoot:.0f}  ({m.overshoot_pct:.1f} %)",
                             color=RED if m.overshoot_pct > 5 else (255, 255, 255))
                if m.settle_ms is None:
                    dpg.add_text("not settled" if m.done else "...", color=RED if m.done else GREY)
                else:
                    dpg.add_text(_fmt_ms(m.settle_ms))
                dpg.add_text(f"{m.final_error:+d}")

    _update_autotune(s)

    if dpg.is_item_shown("cooldown_popup") and not tuner.running:
        left = _cooldown_left()
        if left > 0:
            dpg.set_value("cooldown_text", f"Let the valve cool down!\n{MAX_ACTUATIONS} actuations in the "
                          f"last {WINDOW_S:.0f} s. Ready again in {left:.0f} s.")
        else:
            dpg.configure_item("cooldown_popup", show=False)

    if s["log_seq"] != _shown_log:
        _shown_log = s["log_seq"]
        dpg.set_value("log_box", "\n".join(reversed(s["log"])))
