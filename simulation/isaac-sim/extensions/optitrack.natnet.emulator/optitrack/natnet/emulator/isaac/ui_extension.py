# Copyright (c) 2024 Carnegie Mellon University
# MIT License - see LICENSE in the repository root for full text.
"""Kit extension entry: docked editor for the NatNet interface config prim.

Create/manage the ``/World/NatNetInterface`` prim. The window docks to the
bottom-right (alongside the Property panel, like Pegasus) so it's easy to find.

Sync model is explicit and user-driven via the top button row.
"""

from __future__ import annotations

import omni.ext

from .config import VALID_MODES, VALID_UP_AXES, BodyBinding, NatNetInterfaceConfig
from .manager import NatNetServerManager
from .usd_bindings import author_interface, find_interfaces, read_interface, read_world_pose

_DEFAULT_PRIM_PATH = "/World/NatNetInterface"
_LABEL_WIDTH = 140
_POS_REFRESH_PERIOD = 1.0 / 6.0  # seconds between live USD position reads

_COLOR_LIVE = 0xFF33CC33  # green: prim resolves and server is streaming
_COLOR_IDLE = 0xFFAAAAAA  # grey: prim resolves but server not running
_COLOR_LOST = 0xFF3333FF  # red: no prim / NaN


class NatNetEmulatorExtension(omni.ext.IExt):
    """Registers the Window menu entry + the docked editor panel."""

    def on_startup(self, ext_id):  # noqa: D401 - Kit lifecycle hook
        self._window = None
        self._bodies_frame = None
        self._cfg = NatNetInterfaceConfig()
        self._row_readouts = {}
        self._pos_refresh_sub = None
        self._last_pos_refresh = 0.0
        self._manager = NatNetServerManager()
        self._manager.on_startup()
        self._add_menu()
        self._subscribe_position_refresh()

    def on_shutdown(self):
        self._remove_menu()
        self._pos_refresh_sub = None
        self._row_readouts = {}
        if self._manager is not None:
            self._manager.on_shutdown()
            self._manager = None
        if self._window is not None:
            self._window.destroy()
            self._window = None

    # --- live position readout -------------------------------------------------

    def _subscribe_position_refresh(self):
        try:
            import omni.kit.app
        except Exception:  # pragma: no cover - Kit only
            return
        self._pos_refresh_sub = (
            omni.kit.app.get_app()
            .get_update_event_stream()
            .create_subscription_to_pop(self._on_pos_refresh, name="natnet_ui_pos_refresh")
        )

    def _on_pos_refresh(self, _event):
        import time

        if self._window is None or not self._window.visible or not self._row_readouts:
            return
        now = time.monotonic()
        if now - self._last_pos_refresh < _POS_REFRESH_PERIOD:
            return
        self._last_pos_refresh = now
        stage = self._get_stage()
        running = self._manager is not None and self._manager.is_running
        for idx, (status_label, pos_label) in list(self._row_readouts.items()):
            if not (0 <= idx < len(self._cfg.bodies)):
                continue
            target = self._cfg.bodies[idx].target_prim
            symbol, color, text = self._row_readout(stage, target, running)
            status_label.text = symbol
            status_label.style = {"color": color}
            pos_label.text = text
            pos_label.style = {"color": color}

    def _row_readout(self, stage, target, running):
        if not target:
            return "\u25cb", _COLOR_IDLE, "no target prim"
        prim = stage.GetPrimAtPath(target) if stage is not None else None
        pose = read_world_pose(prim) if prim is not None else None
        if pose is None:
            return "\u2717", _COLOR_LOST, "NaN (prim missing)"
        (x, y, z), _quat = pose
        text = f"{x:+.3f}, {y:+.3f}, {z:+.3f}"
        if running:
            return "\u25cf", _COLOR_LIVE, text
        return "\u25cf", _COLOR_IDLE, text

    # --- menu ------------------------------------------------------------------

    def _add_menu(self):
        try:
            import omni.kit.menu.utils as menu_utils
            from omni.kit.menu.utils import MenuItemDescription
        except Exception:  # pragma: no cover - Kit only
            return
        self._menu_entries = [
            MenuItemDescription(name="NatNet Interface", onclick_fn=self._toggle_window)
        ]
        menu_utils.add_menu_items(self._menu_entries, "Window")

    def _remove_menu(self):
        try:
            import omni.kit.menu.utils as menu_utils
        except Exception:  # pragma: no cover - Kit only
            return
        if getattr(self, "_menu_entries", None):
            menu_utils.remove_menu_items(self._menu_entries, "Window")
            self._menu_entries = None

    # --- window ----------------------------------------------------------------

    def _toggle_window(self, *_):
        import omni.ui as ui

        if self._window is None:
            self._window = ui.Window("NatNet Interface", width=400, height=600)
            self._window.frame.set_build_fn(self._build_window)
            # Dock bottom-right next to the Property panel, like Pegasus.
            self._window.deferred_dock_in("Property", ui.DockPolicy.CURRENT_WINDOW_IS_ACTIVE)
            self._window.visible = True
            return
        self._window.visible = not self._window.visible

    def _refresh(self, *_):
        if self._window is not None:
            self._window.frame.rebuild()

    def _build_window(self):
        import omni.ui as ui

        with ui.ScrollingFrame():
            with ui.VStack(spacing=6, height=0):
                ui.Label("NatNet interface", height=0, style={"font_size": 16})

                with ui.HStack(height=28, spacing=6):
                    ui.Button("Create Interface", clicked_fn=self._create_server)
                    ui.Button("Save", clicked_fn=self._save)
                    ui.Button("Load from Stage", clicked_fn=self._load_from_stage)
                    ui.Button("Print config", clicked_fn=self._print_config)

                running = self._manager is not None and self._manager.is_running
                with ui.HStack(height=28, spacing=6):
                    ui.Button(
                        "Stop Server" if running else "Start Server",
                        clicked_fn=self._toggle_server,
                    )
                    ui.Label(
                        f"Server: {'RUNNING' if running else 'stopped'}",
                        width=0,
                        style={"color": 0xFF33CC33 if running else 0xFF888888},
                    )

                ui.Label(
                    "\u26a0  Remember to save after each edit",
                    height=0,
                    word_wrap=True,
                    style={"color": 0xFF33CCFF, "font_size": 14},
                )

                ui.Label(self._status_text(), height=0, word_wrap=True)

                ui.Separator(height=6)
                self._bool_row(ui, "Server enabled", "server_enabled", self._cfg.server_enabled)
                self._bool_row(ui, "Pose noise enabled", "pose_noise_enabled", self._cfg.pose_noise_enabled)
                self._float_row(ui, "Pose noise std meters", "pose_noise_std_meters", self._cfg.pose_noise_std_meters)
                self._float_row(ui, "Pose noise rotation deg", "pose_noise_rotation_deg", self._cfg.pose_noise_rotation_deg)
                self._str_row(ui, "Server IP", "server_ip", self._cfg.server_ip)
                self._combo_row(ui, "Mode", "mode", self._cfg.mode, VALID_MODES)
                self._int_row(ui, "Command port", "command_port", self._cfg.command_port)
                self._int_row(ui, "Data port", "data_port", self._cfg.data_port)
                self._float_row(ui, "Publish rate (Hz)", "publish_rate", self._cfg.publish_rate)
                self._combo_row(ui, "Up axis", "up_axis", self._cfg.up_axis, VALID_UP_AXES)

                ui.Separator(height=6)
                ui.Label("Tracked bodies", height=0, style={"font_size": 14})
                self._bodies_frame = ui.Frame(height=0)
                self._bodies_frame.set_build_fn(self._build_bodies)
                with ui.HStack(height=0, spacing=6):
                    ui.Button("Add body (from selection)", clicked_fn=self._add_body)

    def _status_text(self):
        prim = self._find_interface()
        if prim is None:
            return "No prim on stage yet — Save or Create Server to author one."
        return f"Prim on stage: {prim.GetPath().pathString} (Save to push edits, Load to pull)"

    # --- server field rows (edit the working copy only) ------------------------

    def _bool_row(self, ui, label, key, value):
        with ui.HStack(height=0):
            ui.Label(label, width=_LABEL_WIDTH)
            cb = ui.CheckBox()
            cb.model.set_value(bool(value))
            cb.model.add_value_changed_fn(
                lambda m, k=key: self._set_cfg_field(k, m.get_value_as_bool())
            )

    def _str_row(self, ui, label, key, value):
        with ui.HStack(height=0):
            ui.Label(label, width=_LABEL_WIDTH)
            model = ui.StringField().model
            model.set_value(str(value))
            model.add_value_changed_fn(
                lambda m, k=key: self._set_cfg_field(k, m.get_value_as_string())
            )

    def _int_row(self, ui, label, key, value):
        with ui.HStack(height=0):
            ui.Label(label, width=_LABEL_WIDTH)
            model = ui.IntField().model
            model.set_value(int(value))
            model.add_value_changed_fn(
                lambda m, k=key: self._set_cfg_field(k, m.get_value_as_int())
            )

    def _float_row(self, ui, label, key, value):
        with ui.HStack(height=0):
            ui.Label(label, width=_LABEL_WIDTH)
            model = ui.FloatField().model
            model.set_value(float(value))
            model.add_value_changed_fn(
                lambda m, k=key: self._set_cfg_field(k, m.get_value_as_float())
            )

    def _combo_row(self, ui, label, key, value, choices):
        with ui.HStack(height=0):
            ui.Label(label, width=_LABEL_WIDTH)
            index = choices.index(value) if value in choices else 0
            combo = ui.ComboBox(index, *choices)
            combo.model.get_item_value_model().add_value_changed_fn(
                lambda m, k=key, c=choices: self._set_cfg_field(k, c[m.get_value_as_int()])
            )

    def _set_cfg_field(self, attr, value):
        setattr(self._cfg, attr, value)

    # --- bodies ----------------------------------------------------------------

    def _rebuild_bodies(self, *_):
        if self._bodies_frame is not None:
            self._bodies_frame.rebuild()

    def _build_bodies(self):
        import omni.ui as ui

        self._row_readouts = {}
        with ui.VStack(spacing=6, height=0):
            if not self._cfg.bodies:
                ui.Label("  (no bodies — select a prim and click Add body)", height=0)
                return
            with ui.HStack(height=0, spacing=4):
                ui.Label("Rigid body name", width=ui.Fraction(1))
                ui.Label("ID", width=40)
                ui.Label("Parent", width=50)
                ui.Label("Target prim", width=ui.Fraction(2))
                ui.Spacer(width=98)
            for idx, body in enumerate(self._cfg.bodies):
                self._build_body_row(ui, idx, body)

    def _build_body_row(self, ui, idx, body):
        with ui.VStack(height=0, spacing=2):
            with ui.HStack(height=0, spacing=4):
                name = ui.StringField(width=ui.Fraction(1)).model
                name.set_value(body.rigid_body_name)
                name.add_value_changed_fn(
                    lambda m, i=idx: self._set_body_field(i, "rigid_body_name", m.get_value_as_string())
                )

                sid = ui.IntField(width=40).model
                sid.set_value(body.streaming_id)
                sid.add_value_changed_fn(
                    lambda m, i=idx: self._set_body_field(i, "streaming_id", m.get_value_as_int())
                )

                parent = ui.IntField(width=50).model
                parent.set_value(body.parent_id)
                parent.add_value_changed_fn(
                    lambda m, i=idx: self._set_body_field(i, "parent_id", m.get_value_as_int())
                )

                target = ui.StringField(width=ui.Fraction(2), tooltip="USD path of the tracked prim").model
                target.set_value(body.target_prim)
                target.add_value_changed_fn(
                    lambda m, i=idx: self._set_body_field(i, "target_prim", m.get_value_as_string())
                )

                ui.Button("set target", width=70, clicked_fn=lambda i=idx: self._retarget_body(i))
                ui.Button("x", width=24, clicked_fn=lambda i=idx: self._remove_body_at(i))

            # Live readout: status dot + world position pulled from the USD stage.
            stage = self._get_stage()
            running = self._manager is not None and self._manager.is_running
            symbol, color, text = self._row_readout(stage, body.target_prim, running)
            with ui.HStack(height=0, spacing=6):
                ui.Spacer(width=4)
                status_label = ui.Label(symbol, width=14, style={"color": color})
                ui.Label("pos:", width=30, style={"color": _COLOR_IDLE})
                pos_label = ui.Label(text, width=ui.Fraction(1), style={"color": color})
            self._row_readouts[idx] = (status_label, pos_label)

    def _set_body_field(self, index, attr, value):
        if 0 <= index < len(self._cfg.bodies):
            setattr(self._cfg.bodies[index], attr, value)

    def _add_body(self):
        next_id = max((b.streaming_id for b in self._cfg.bodies), default=0) + 1
        target = self._selected_target_path(self._find_interface())
        name = target.rsplit("/", 1)[-1] if target else f"Body{next_id}"
        existing = {b.rigid_body_name for b in self._cfg.bodies}
        while name in existing:
            name = f"{name}_{next_id}"
        self._cfg.bodies.append(BodyBinding(rigid_body_name=name, target_prim=target, streaming_id=next_id))
        self._rebuild_bodies()

    def _remove_body_at(self, index):
        if 0 <= index < len(self._cfg.bodies):
            self._cfg.bodies.pop(index)
            self._rebuild_bodies()

    def _retarget_body(self, index):
        import carb

        path = self._selected_target_path(self._find_interface())
        if not path:
            carb.log_warn("[natnet] Select a prim in the viewport to retarget this body.")
            return
        if 0 <= index < len(self._cfg.bodies):
            self._cfg.bodies[index].target_prim = path
            self._rebuild_bodies()

    # --- stage helpers ---------------------------------------------------------

    def _get_stage(self):
        import omni.usd

        return omni.usd.get_context().get_stage()

    def _find_interface(self):
        stage = self._get_stage()
        if stage is None:
            return None
        interfaces = find_interfaces(stage)
        return interfaces[0] if interfaces else None

    def _interface_path(self):
        prim = self._find_interface()
        return prim.GetPath().pathString if prim is not None else _DEFAULT_PRIM_PATH

    def _select(self, prim_path):
        import omni.usd

        omni.usd.get_context().get_selection().set_selected_prim_paths([prim_path], True)

    def _selected_target_path(self, interface_prim):
        import omni.usd

        sel = omni.usd.get_context().get_selection().get_selected_prim_paths()
        iface_path = interface_prim.GetPath().pathString if interface_prim else None
        for path in sel:
            if path != iface_path:
                return path
        return ""

    # --- explicit sync actions -------------------------------------------------

    def _save(self):
        import carb

        stage = self._get_stage()
        if stage is None:
            carb.log_error("[natnet] No active stage.")
            return
        try:
            self._cfg.validate()
        except ValueError as exc:
            carb.log_error(f"[natnet] Not saved: {exc}")
            return
        path = self._interface_path()
        author_interface(stage, path, self._cfg)
        carb.log_info(f"[natnet] Saved interface to {path} ({len(self._cfg.bodies)} bodies).")
        self._refresh()

    def _load_from_stage(self):
        import carb

        prim = self._find_interface()
        if prim is None:
            self._cfg = NatNetInterfaceConfig()
            carb.log_warn("[natnet] No interface on stage — reset to defaults.")
        else:
            self._cfg = read_interface(prim)
            carb.log_info(f"[natnet] Loaded interface from {prim.GetPath().pathString}.")
        self._refresh()

    def _print_config(self):
        # Print whatever is authored on the stage (the source of truth).
        if self._manager is not None:
            self._manager.scan_and_print()

    def _toggle_server(self):
        # Start/stop the live server at the click of this button, regardless of the
        # serverEnabled attribute. Builds from the prim that's actually on the stage.
        import carb

        if self._manager is None:
            return
        if not self._manager.is_running:
            if self._find_interface() is None:
                carb.log_warn("[natnet] No interface on stage — Create/Save one first.")
                return
            try:
                self._manager.start_from_stage()
            except Exception as exc:  # noqa: BLE001 - surface to the user
                carb.log_error(f"[natnet] Could not start server: {exc}")
        else:
            self._manager.stop_server()
        self._refresh()

    def _create_server(self):
        import carb

        stage = self._get_stage()
        if stage is None:
            carb.log_error("[natnet] No active stage.")
            return
        prim = self._find_interface()
        if prim is None:
            try:
                self._cfg.validate()
            except ValueError as exc:
                carb.log_error(f"[natnet] Cannot create: {exc}")
                return
            author_interface(stage, _DEFAULT_PRIM_PATH, self._cfg)
            path = _DEFAULT_PRIM_PATH
            carb.log_info(f"[natnet] Created interface prim at {path}. (Server start: later commit.)")
        else:
            path = prim.GetPath().pathString
            carb.log_info(f"[natnet] Interface already exists at {path}. (Server start: later commit.)")
        self._select(path)
        self._refresh()
