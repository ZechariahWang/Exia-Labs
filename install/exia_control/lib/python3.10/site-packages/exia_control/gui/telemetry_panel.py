import collections
import time
import math

import dearpygui.dearpygui as dpg


COLOR_GREEN = (100, 220, 100, 255)
COLOR_RED = (220, 80, 80, 255)
COLOR_YELLOW = (220, 200, 80, 255)
COLOR_WHITE = (220, 220, 220, 255)

GEAR_NAMES = {0: 'R', 1: 'N', 2: 'H'}


class TelemetryPanel:

    def __init__(self):
        self._gps_trail_x = collections.deque(maxlen=500)
        self._gps_trail_y = collections.deque(maxlen=500)
        self._gps_origin = None
        self._line_series_tag = None
        self._x_axis_tag = None
        self._y_axis_tag = None

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

        with dpg.child_window(border=True, height=-1):
            dpg.add_text("GPS Map", color=COLOR_YELLOW)
            with dpg.plot(label="", height=-1, width=-1, tag="gps_plot"):
                self._x_axis_tag = dpg.add_plot_axis(dpg.mvXAxis, label="X (m)")
                self._y_axis_tag = dpg.add_plot_axis(dpg.mvYAxis, label="Y (m)")
                self._line_series_tag = dpg.add_line_series(
                    [], [], label="Trail", parent=self._y_axis_tag)

    def refresh(self, node):
        lss_ok = getattr(node, '_lss_ok', False)
        dpg.set_value("hw_lss_val", "OK" if lss_ok else "OFF")
        dpg.configure_item("hw_lss_val", color=COLOR_GREEN if lss_ok else COLOR_RED)

        motor_ok = False
        if hasattr(node, '_motor_lock'):
            with node._motor_lock:
                motor_ok = node._motor_ok
        dpg.set_value("hw_mtr_val", "OK" if motor_ok else "OFF")
        dpg.configure_item("hw_mtr_val", color=COLOR_GREEN if motor_ok else COLOR_RED)

        gear = getattr(node, '_gear', 1)
        gear_str = GEAR_NAMES.get(gear, '?')
        dpg.set_value("hw_gear_val", gear_str)

        estop = getattr(node, '_estop', False)
        dpg.set_value("hw_estop", "E-STOP" if estop else "")

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

        gps_lat = getattr(node, '_gps_lat', None)
        gps_lon = getattr(node, '_gps_lon', None)
        if gps_lat is not None and gps_lon is not None:
            dpg.set_value("gps_coords", f"Lat: {gps_lat:.6f}  Lon: {gps_lon:.6f}")
            if self._gps_origin is None:
                self._gps_origin = (gps_lat, gps_lon)
            origin_lat, origin_lon = self._gps_origin
            x = (gps_lon - origin_lon) * math.cos(math.radians(origin_lat)) * 111320.0
            y = (gps_lat - origin_lat) * 111320.0
            self._gps_trail_x.append(x)
            self._gps_trail_y.append(y)
            xs = list(self._gps_trail_x)
            ys = list(self._gps_trail_y)
            dpg.set_value(self._line_series_tag, [xs, ys])
            if len(xs) > 1:
                dpg.fit_axis_data(self._x_axis_tag)
                dpg.fit_axis_data(self._y_axis_tag)
        else:
            dpg.set_value("gps_coords", "Lat: --  Lon: --")

        remote_active = (time.monotonic() - getattr(node, '_remote_last_time', 0.0)) < 0.3
        if getattr(node, '_remote_mode', False):
            dpg.set_value("radio_status", "REMOTE")
            dpg.configure_item("radio_status", color=COLOR_GREEN)
        elif remote_active:
            dpg.set_value("radio_status", "RADIO")
            dpg.configure_item("radio_status", color=COLOR_GREEN)
        else:
            dpg.set_value("radio_status", "LOCAL")
            dpg.configure_item("radio_status", color=COLOR_WHITE)
