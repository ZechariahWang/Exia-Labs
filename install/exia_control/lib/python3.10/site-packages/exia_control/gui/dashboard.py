import dearpygui.dearpygui as dpg

from exia_control.gui.telemetry_panel import TelemetryPanel

_panel = None
_node_ref = None
_blue_status_panel = None
_blue_config_panel = None


def _coords_text_to_geojson(coords_text: str) -> dict:
    coords: list[list[float]] = []
    for line in coords_text.splitlines():
        line = line.strip()
        if not line:
            continue
        parts = [p.strip() for p in line.split(",")]
        if len(parts) != 2:
            continue
        try:
            lat = float(parts[0])
            lon = float(parts[1])
        except ValueError:
            continue
        coords.append([lon, lat])
    return {"type": "LineString", "coordinates": coords}


def create_dashboard(node, blue_state=None, blue_config_store=None, blue_service=None):
    global _panel, _node_ref, _blue_status_panel, _blue_config_panel
    _node_ref = node
    _panel = TelemetryPanel()

    has_blue = blue_state is not None and blue_config_store is not None and blue_service is not None

    if has_blue:
        from exia_control.gui.blue_status_panel import BlueStatusPanel
        from exia_control.gui.blue_config_panel import BlueConfigPanel

        def set_override_task():
            name = dpg.get_value("blue_override_name").strip() or "Manual Override"
            route_geojson = _coords_text_to_geojson(dpg.get_value("blue_override_coords"))
            try:
                blue_service.set_override_task(name, route_geojson)
                dpg.set_value("blue_override_status", "Override task set")
            except Exception as exc:
                dpg.set_value("blue_override_status", f"Failed: {exc}")

        def clear_override_task():
            blue_service.clear_override_task()
            dpg.set_value("blue_override_status", "Override cleared")

        def on_override_inputs_changed(name, coords_text):
            cfg = blue_config_store.get()
            cfg.override_task_name = (name or "").strip()
            cfg.override_task_geojson = _coords_text_to_geojson(coords_text)
            blue_config_store.update(cfg)
            blue_config_store.save()

        def set_blue_sync_enabled(enabled):
            cfg = blue_config_store.get()
            if cfg.blue_sync_enabled != enabled:
                cfg.blue_sync_enabled = enabled
                blue_config_store.update(cfg)
                blue_config_store.save()

        def on_config_applied(config):
            _blue_status_panel.apply_override_config_values(config)

        _blue_status_panel = BlueStatusPanel(
            state=blue_state,
            config=blue_config_store,
            on_set_override_task=set_override_task,
            on_clear_override_task=clear_override_task,
            on_override_inputs_changed=on_override_inputs_changed,
            on_start_resume_task=blue_service.start_resume_task,
            on_stop_cancel_task=blue_service.stop_cancel_task,
            on_set_blue_sync_enabled=set_blue_sync_enabled,
        )
        _blue_config_panel = BlueConfigPanel(
            config_store=blue_config_store,
            debounce_seconds=0.8,
            on_config_applied=on_config_applied,
        )

    dpg.create_context()

    if has_blue:
        dpg.create_viewport(title="Exia Telemetry + Blue", width=1100, height=700)
    else:
        dpg.create_viewport(title="Exia Telemetry", width=900, height=600)

    with dpg.window(tag="main_window", no_move=True, no_resize=True,
                    no_collapse=True, no_title_bar=True):
        if has_blue:
            with dpg.tab_bar(tag="main_tabs"):
                with dpg.tab(label="Telemetry"):
                    _panel.build()
                with dpg.tab(label="Blue Status"):
                    _blue_status_panel.build()
                with dpg.tab(label="Blue Config"):
                    _blue_config_panel.build()
        else:
            _panel.build()

    if has_blue:
        _blue_status_panel.apply_override_config_values(blue_config_store.get())

        theme_override_set = _create_button_theme((232, 192, 52), (246, 209, 80), (201, 163, 35))
        theme_override_clear = _create_button_theme((235, 141, 52), (247, 161, 81), (201, 116, 37))
        theme_save = _create_button_theme((80, 170, 220), (104, 187, 231), (62, 145, 192))

        if dpg.does_item_exist("blue_btn_set_override"):
            dpg.bind_item_theme("blue_btn_set_override", theme_override_set)
        if dpg.does_item_exist("blue_btn_clear_override"):
            dpg.bind_item_theme("blue_btn_clear_override", theme_override_clear)
        if dpg.does_item_exist("blue_btn_save_config"):
            dpg.bind_item_theme("blue_btn_save_config", theme_save)

    dpg.setup_dearpygui()
    dpg.set_primary_window("main_window", True)
    dpg.show_viewport()


def run_frame():
    if not dpg.is_dearpygui_running():
        return False
    if _panel is not None and _node_ref is not None:
        _panel.refresh(_node_ref)
    if _blue_status_panel is not None:
        _blue_status_panel.refresh()
    if _blue_config_panel is not None:
        _blue_config_panel.maybe_autosave()
    dpg.render_dearpygui_frame()
    return True


def stop_dashboard():
    try:
        dpg.destroy_context()
    except Exception:
        pass


def _create_button_theme(base, hover, active):
    with dpg.theme() as theme:
        with dpg.theme_component(dpg.mvButton):
            dpg.add_theme_color(dpg.mvThemeCol_Button, (*base, 255))
            dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered, (*hover, 255))
            dpg.add_theme_color(dpg.mvThemeCol_ButtonActive, (*active, 255))
            dpg.add_theme_color(dpg.mvThemeCol_Text, (20, 20, 20, 255))
    return theme
