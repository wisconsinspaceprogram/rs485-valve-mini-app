"""Valve tuner: connect ONE valve through a USB-RS485 adapter, tune it and test it.

    python main.py          normal use
    python main.py --sim    also list simulated valves in the port box (no hardware needed)
"""
import argparse

import dearpygui.dearpygui as dpg

from core.valve_link import ValveLink
from gui import tuner_gui


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--sim", action="store_true", help="offer simulated valves as ports")
    args = parser.parse_args()

    link = ValveLink()
    dpg.create_context()
    tuner_gui.build(link, sim=args.sim)
    dpg.create_viewport(title="Valve tuner", width=1400, height=900)
    dpg.setup_dearpygui()
    dpg.show_viewport()
    dpg.set_primary_window("Main Window", True)

    try:
        while dpg.is_dearpygui_running():
            tuner_gui.update()
            dpg.render_dearpygui_frame()
    finally:
        link.shutdown()      # turns debug mode back off on the valve
        dpg.destroy_context()


if __name__ == "__main__":
    main()
