import dearpygui.dearpygui as dpg

from core.sim import SIM_PORTS

link = None
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


def cb_go():
    link.move_to(max(0, min(4095, int(dpg.get_value("goto_input")))))


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


def build(valve_link, sim=False):
    global link, show_sim
    link, show_sim = valve_link, sim

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
                                   callback=lambda: link.open_valve())
                    dpg.add_button(label="CLOSE", tag="btn_close", width=200, height=45,
                                   callback=lambda: link.close_valve())
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

    if s["log_seq"] != _shown_log:
        _shown_log = s["log_seq"]
        dpg.set_value("log_box", "\n".join(reversed(s["log"])))
