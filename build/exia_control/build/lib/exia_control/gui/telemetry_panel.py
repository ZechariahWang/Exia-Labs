import collections
import time

import dearpygui.dearpygui as dpg


COLOR_GREEN = (100, 220, 100, 255)
COLOR_RED = (220, 80, 80, 255)
COLOR_YELLOW = (220, 200, 80, 255)
COLOR_WHITE = (220, 220, 220, 255)
COLOR_CYAN = (100, 200, 220, 255)
COLOR_DIM = (140, 140, 140, 255)

GEAR_NAMES = {0: 'R', 1: 'N', 2: 'H'}
LOG_MAX_LINES = 200


class TelemetryPanel:

    def __init__(self, on_estop=None, on_clear_estop=None):
        self._on_estop = on_estop
        self._on_clear_estop = on_clear_estop
        self._estop_theme = None
        self._clear_theme = None
        self._last_estop_state = False
        self._prev_lss = None
        self._prev_motor = None
        self._prev_gear = None
        self._prev_gps = None
        self._prev_imu = None
        self._prev_radio = None
        self._log_lines = collections.deque(maxlen=LOG_MAX_LINES)
        self._log_dirty = False

    def build(self):
        with dpg.group(horizontal=True):
            dpg.add_text("LSS:", tag="hw_lss_label")
            dpg.add_text("OFF", tag="hw_lss_val", color=COLOR_RED)
            dpg.add_spacer(width=16)
            dpg.add_text("MTR:", tag="hw_mtr_label")
            dpg.add_text("OFF", tag="hw_mtr_val", color=COLOR_RED)
            dpg.add_spacer(width=16)
            dpg.add_text("GEAR:", tag="hw_gear_label")
            dpg.add_text("N", tag="hw_gear_val", color=COLOR_WHITE)
            dpg.add_spacer(width=40)
            dpg.add_text("", tag="hw_estop", color=COLOR_RED)

        dpg.add_spacer(height=4)
        dpg.add_button(
            label="E-STOP",
            tag="btn_estop",
            width=-1,
            height=50,
            callback=lambda: self._toggle_estop(),
        )
        with dpg.theme() as self._estop_theme:
            with dpg.theme_component(dpg.mvButton):
                dpg.add_theme_color(dpg.mvThemeCol_Button, (180, 20, 20, 255))
                dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered, (220, 40, 40, 255))
                dpg.add_theme_color(dpg.mvThemeCol_ButtonActive, (140, 10, 10, 255))
                dpg.add_theme_color(dpg.mvThemeCol_Text, (255, 255, 255, 255))
                dpg.add_theme_style(dpg.mvStyleVar_FrameRounding, 6)
        with dpg.theme() as self._clear_theme:
            with dpg.theme_component(dpg.mvButton):
                dpg.add_theme_color(dpg.mvThemeCol_Button, (20, 120, 20, 255))
                dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered, (30, 160, 30, 255))
                dpg.add_theme_color(dpg.mvThemeCol_ButtonActive, (15, 90, 15, 255))
                dpg.add_theme_color(dpg.mvThemeCol_Text, (255, 255, 255, 255))
                dpg.add_theme_style(dpg.mvStyleVar_FrameRounding, 6)
        dpg.bind_item_theme("btn_estop", self._estop_theme)

        dpg.add_separator()

        with dpg.table(header_row=False, policy=dpg.mvTable_SizingStretchProp,
                        borders_innerV=True, borders_outerV=False,
                        borders_innerH=False, borders_outerH=False):
            dpg.add_table_column(init_width_or_weight=1.0)
            dpg.add_table_column(init_width_or_weight=1.0)

            with dpg.table_row():
                with dpg.table_cell():
                    with dpg.child_window(border=True, height=120, no_scrollbar=True, no_scroll_with_mouse=True):
                        dpg.add_text("Drive State", color=COLOR_YELLOW)
                        dpg.add_separator()
                        dpg.add_text("Throttle:  0%", tag="drive_throttle")
                        dpg.add_text("Brake:     0%", tag="drive_brake")
                        dpg.add_text("Steer:    +0%", tag="drive_steer")
                        dpg.add_text("Gear:      N", tag="drive_gear")

                    dpg.add_spacer(height=4)

                    with dpg.child_window(border=True, height=80, no_scrollbar=True, no_scroll_with_mouse=True):
                        dpg.add_text("Cmd Vel", color=COLOR_YELLOW)
                        dpg.add_separator()
                        dpg.add_text("Linear X:   0.000 m/s", tag="cmd_linear_x")
                        dpg.add_text("Angular Z:  0.000 rad/s", tag="cmd_angular_z")

                with dpg.table_cell():
                    with dpg.child_window(border=True, height=100, no_scrollbar=True, no_scroll_with_mouse=True):
                        dpg.add_text("IMU", color=COLOR_YELLOW)
                        dpg.add_separator()
                        dpg.add_text("Heading:  --", tag="imu_heading")
                        dpg.add_text("Roll:     --", tag="imu_roll")
                        dpg.add_text("Pitch:    --", tag="imu_pitch")

                    dpg.add_spacer(height=4)

                    with dpg.child_window(border=True, height=60, no_scrollbar=True, no_scroll_with_mouse=True):
                        dpg.add_text("GPS", color=COLOR_YELLOW)
                        dpg.add_separator()
                        dpg.add_text("Lat: --  Lon: --", tag="gps_coords")

                    dpg.add_spacer(height=4)

                    with dpg.child_window(border=True, height=58, no_scrollbar=True, no_scroll_with_mouse=True):
                        dpg.add_text("Radio", color=COLOR_YELLOW)
                        with dpg.group(horizontal=True):
                            dpg.add_text("Status:", tag="radio_label")
                            dpg.add_text("--", tag="radio_status", color=COLOR_WHITE)

        dpg.add_spacer(height=4)

        with dpg.child_window(border=True, height=-1, tag="log_window"):
            dpg.add_text("Event Log", color=COLOR_YELLOW)
            dpg.add_separator()
            dpg.add_text("", tag="log_text", wrap=0, color=COLOR_DIM)

    def _toggle_estop(self):
        if self._last_estop_state:
            if self._on_clear_estop:
                self._on_clear_estop()
        else:
            if self._on_estop:
                self._on_estop()

    def log(self, msg):
        ts = time.strftime("%H:%M:%S")
        self._log_lines.append(f"[{ts}] {msg}")
        self._log_dirty = True

    def _flush_log(self):
        if not self._log_dirty:
            return
        self._log_dirty = False
        dpg.set_value("log_text", "\n".join(self._log_lines))
        dpg.set_y_scroll("log_window", dpg.get_y_scroll_max("log_window"))

    def refresh(self, node):
        lss_ok = getattr(node, '_lss_ok', False)
        dpg.set_value("hw_lss_val", "OK" if lss_ok else "OFF")
        dpg.configure_item("hw_lss_val", color=COLOR_GREEN if lss_ok else COLOR_RED)
        if lss_ok != self._prev_lss:
            self._prev_lss = lss_ok
            self.log("LSS connected" if lss_ok else "LSS disconnected")

        motor_ok = False
        if hasattr(node, '_motor_lock'):
            with node._motor_lock:
                motor_ok = node._motor_ok
        dpg.set_value("hw_mtr_val", "OK" if motor_ok else "OFF")
        dpg.configure_item("hw_mtr_val", color=COLOR_GREEN if motor_ok else COLOR_RED)
        if motor_ok != self._prev_motor:
            self._prev_motor = motor_ok
            self.log("Steering motor attached" if motor_ok else "Steering motor detached")

        gear = getattr(node, '_gear', 1)
        gear_str = GEAR_NAMES.get(gear, '?')
        dpg.set_value("hw_gear_val", gear_str)
        if gear != self._prev_gear:
            if self._prev_gear is not None:
                self.log(f"Gear -> {gear_str}")
            self._prev_gear = gear

        estop = getattr(node, '_estop', False)
        dpg.set_value("hw_estop", "E-STOP" if estop else "")
        if estop != self._last_estop_state:
            self._last_estop_state = estop
            if estop:
                dpg.configure_item("btn_estop", label="CLEAR E-STOP")
                dpg.bind_item_theme("btn_estop", self._clear_theme)
                self.log("E-STOP activated")
            else:
                dpg.configure_item("btn_estop", label="E-STOP")
                dpg.bind_item_theme("btn_estop", self._estop_theme)
                self.log("E-STOP cleared")

        throttle_pct = int(getattr(node, '_throttle_ramp', 0.0) * 100)
        brake_pct = int(getattr(node, '_brake_ramp', 0.0) * 100)
        steer_pct = int(getattr(node, '_steer_ramp', 0.0) * 100)

        dpg.set_value("drive_throttle", f"Throttle: {throttle_pct:3d}%")
        dpg.set_value("drive_brake", f"Brake:    {brake_pct:3d}%")
        dpg.set_value("drive_steer", f"Steer:   {steer_pct:+4d}%")
        dpg.set_value("drive_gear", f"Gear:      {gear_str}")

        cmd_lx = getattr(node, '_cmd_vel_linear_x', 0.0)
        cmd_az = getattr(node, '_cmd_vel_angular_z', 0.0)
        dpg.set_value("cmd_linear_x", f"Linear X:  {cmd_lx:+7.3f} m/s")
        dpg.set_value("cmd_angular_z", f"Angular Z: {cmd_az:+7.3f} rad/s")

        imu_heading = getattr(node, '_imu_heading', None)
        imu_roll = getattr(node, '_imu_roll', None)
        imu_pitch = getattr(node, '_imu_pitch', None)
        if imu_heading is not None:
            dpg.set_value("imu_heading", f"Heading: {imu_heading:+7.1f} deg")
            dpg.set_value("imu_roll", f"Roll:    {imu_roll:+7.1f} deg")
            dpg.set_value("imu_pitch", f"Pitch:   {imu_pitch:+7.1f} deg")
        else:
            dpg.set_value("imu_heading", "Heading:  --")
            dpg.set_value("imu_roll", "Roll:     --")
            dpg.set_value("imu_pitch", "Pitch:    --")
        has_imu = imu_heading is not None
        if has_imu != self._prev_imu:
            self._prev_imu = has_imu
            if has_imu:
                self.log("IMU data received")
            else:
                self.log("IMU data lost")

        gps_lat = getattr(node, '_gps_lat', None)
        gps_lon = getattr(node, '_gps_lon', None)
        if gps_lat is not None and gps_lon is not None:
            dpg.set_value("gps_coords", f"Lat: {gps_lat:.6f}  Lon: {gps_lon:.6f}")
        else:
            dpg.set_value("gps_coords", "Lat: --  Lon: --")
        has_gps = gps_lat is not None
        if has_gps != self._prev_gps:
            self._prev_gps = has_gps
            if has_gps:
                self.log(f"GPS fix acquired ({gps_lat:.6f}, {gps_lon:.6f})")
            else:
                self.log("GPS fix lost")

        remote_active = (time.monotonic() - getattr(node, '_remote_last_time', 0.0)) < 0.3
        if getattr(node, '_remote_mode', False):
            dpg.set_value("radio_status", "REMOTE")
            dpg.configure_item("radio_status", color=COLOR_GREEN)
            radio_state = 'remote'
        elif remote_active:
            dpg.set_value("radio_status", "RADIO")
            dpg.configure_item("radio_status", color=COLOR_GREEN)
            radio_state = 'radio'
        else:
            dpg.set_value("radio_status", "LOCAL")
            dpg.configure_item("radio_status", color=COLOR_WHITE)
            radio_state = 'local'
        if radio_state != self._prev_radio:
            if self._prev_radio is not None:
                self.log(f"Input source -> {radio_state.upper()}")
            self._prev_radio = radio_state

        self._flush_log()
