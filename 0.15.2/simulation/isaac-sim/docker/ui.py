# Copyright (c) 2018-2020, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
"""This module provides user interface components for establishing and managing Nucleus connections in the Omniverse environment."""


import os
import sys
import tempfile

import carb
import carb.settings
import omni.ui as ui
import asyncio
import omni.kit.app
from omni.kit.async_engine import run_coroutine

from typing import Coroutine, Any, Callable, Union
from omni.kit.window.popup_dialog.dialog import PopupDialog
from omni.kit.window.popup_dialog.form_dialog import FormDialog, FormWidget
from .style import UI_STYLES, ICON_PATH


class ConnectorDialog(PopupDialog):
    """The main connection dialog for establishing Nucleus connections.

    This dialog provides a user interface to add new Omniverse connections with an optional name. It manages the UI elements and tasks associated with the connection process, such as displaying progress, handling cancellations, and showing alerts.
    """

    def __init__(self):
        super().__init__(width=400, title="Add Nucleus connection", ok_label="Ok", cancel_label="Cancel", modal=True)
        self._form_widget = None
        self._progress_bar = None
        self._alert_pane = None
        self._task = None
        self._build_ui()

    def _build_ui(self):
        field_defs = [
            FormDialog.FieldDef("url", "Omniverse://  ", ui.StringField, "", True),
            FormDialog.FieldDef("name", "Optional Name:  ", ui.StringField, ""),
        ]
        with self._window.frame:
            with ui.VStack(style=UI_STYLES, style_type_name_override="Dialog"):
                # OM-81873: This is a temporary solution for the authentication dialog; add the form widget in a frame
                #  so that it could be easily hidden/shown
                self._form_frame = ui.Frame()
                with self._form_frame:
                    self._form_widget = FormWidget("Add a new connection with optional name.", field_defs)
                with ui.ZStack():
                    ui.Spacer(height=60)
                    with ui.VStack():
                        ui.Spacer(height=8)
                        self._progress_bar = LoopProgressBar(puck_size=40)
                        ui.Spacer()
                    # OM-86699: Fix issue with randomly resized connector dialog, specify ZStack width with window width
                    # OM-98771: Substract padding from window width when specifying AlertPane width
                    self._alert_pane = AlertPane(width=self._window.width - 24)
                self._build_ok_cancel_buttons()
        self.hide()
        self.set_cancel_clicked_fn(lambda _: self._on_cancel_fn())

    def _on_cancel_fn(self):
        self.cancel_task()
        self.hide()

    def get_value(self, name: str) -> str:
        if self._form_widget:
            return self._form_widget.get_value(name)
        return None

    def set_value(self, name: str, value: str):
        if self._form_widget:
            field = self._form_widget.get_field(name)
            if field:
                field.model.set_value(value or "")

    def destroy(self):
        self.cancel_task()
        if self._form_widget:
            self._form_widget.destroy()
            self._form_widget = None
        if self._form_frame:
            self._form_frame = None
        self._progress_bar = None
        if self._alert_pane:
            self._alert_pane.destroy()
            self._alert_pane = None
        super().destroy()

    def __del__(self):
        self.destroy()

    async def run_cancellable_task(self, task: Coroutine) -> Any:
        """Manages running and cancelling the given task"""
        if self._task:
            self.cancel_task()
        self._task = asyncio.create_task(task)

        try:
            return await self._task
        except asyncio.CancelledError:
            carb.log_info(f"Cancelling task ... {self._task}")
            raise
        except Exception as e:
            raise
        finally:
            self._task = None

    def cancel_task(self):
        """Cancels the managed task"""
        if self._task is not None:
            self._task.cancel()
        self._task = None

    @property
    def frame(self):
        return self._window.frame

    @property
    def visible(self) -> bool:
        if self._window:
            return self._window.visible
        return False

    def show(self, name: str = None, url: str = None):
        """Shows the dialog after resetting to a default state"""
        broken_url = omni.client.break_url(url or "")
        host = broken_url.host if broken_url.scheme == "omniverse" else None
        self.set_value("url", host)
        self.set_value("name", name)
        if self._progress_bar:
            self._progress_bar.hide()
        if self._alert_pane:
            self._alert_pane.hide()
        if not self._okay_button:
            self._build_ok_cancel_buttons()
        self._okay_button.enabled = True
        self._cancel_button.enabled = True
        self._okay_button.visible = True
        if self._form_frame:
            self._form_frame.visible = True

        self._window.visible = True

        # focus fields on show
        if self._form_widget:
            self._form_widget.focus()

    def hide(self):
        self._window.visible = False
        # reset window title on hide; authentication mode might have reset the window title.
        self._window.title = self._title

    def show_waiting(self):
        if self._alert_pane:
            self._alert_pane.hide()
        if self._progress_bar:
            self._progress_bar.show()
        # Disable apply button
        if self._okay_button:
            self._okay_button.enabled = False

    def show_alert(self, msg: str = "", alert_type: int = 0):
        # hide form widget and OK button
        if self._okay_button:
            self._okay_button.visible = False
            # Disable apply button
            self._okay_button.enabled = False
        if self._form_frame:
            self._form_frame.visible = False

        if self._alert_pane:
            self._alert_pane.show(msg, alert_type=alert_type)
        if self._progress_bar:
            self._progress_bar.hide()
        self._window.visible = True

    def show_authenticate(self, name: str, url: str):
        """Shows the dialog for authentication, when the name and url is already given."""
        # hide form widget and OK button
        if self._okay_button:
            self._okay_button.visible = False
        if self._form_frame:
            self._form_frame.visible = False

        # show login info
        broken_url = omni.client.break_url(url or "")
        host = broken_url.host if broken_url.scheme == "omniverse" else None
        if self._alert_pane:
            msg = f"Connecting to ({host}) requires login authentication. Please login from your web browser."
            carb.log_warn(msg)
            #self._alert_pane.show(msg, alert_type=AlertPane.Info)

        # update window title
        self._window.title = "Login Required"
        #self._window.visible = True


class LoopProgressBar:
    """A looping progress bar for visual indication of ongoing processes.

    This class creates a graphical progress bar that loops to indicate an ongoing process without a defined end. It is commonly used in UIs to show that an action is taking place.

    Args:
        puck_size (int): Size of the moving element in the progress bar.
        period (float): The time it takes for the puck to complete one loop."""

    def __init__(self, puck_size: int = 20, period: float = 120):
        self._puck_size = puck_size
        self._period = period
        self._widget = None
        self._spacer_left = None
        self._spacer_right = None
        self._loop_task = None
        self._build_ui()

    def _build_ui(self):
        self._widget = ui.Frame(visible=False)
        with self._widget:
            with ui.ZStack(height=20):
                ui.Rectangle(style_type_name_override="ProgressBar")
                with ui.HStack():
                    self._spacer_left = ui.Spacer(width=ui.Fraction(0))
                    ui.Rectangle(width=self._puck_size, style_type_name_override="ProgressBar.Puck")
                    self._spacer_right = ui.Spacer(width=ui.Fraction(1))

    async def _inf_loop(self):
        counter = 0
        while True:
            await omni.kit.app.get_app().next_update_async()
            width = float(counter % self._period) / self._period
            # Ping-pong
            width = width * 2
            if width > 1.0:
                width = 2.0 - width
            self._spacer_left.width = ui.Fraction(width)
            self._spacer_right.width = ui.Fraction(1.0 - width)
            counter += 1

    def __del__(self):
        self.hide()
        self._widget = None

    def show(self):
        if self._widget:
            self._loop_task = asyncio.ensure_future(self._inf_loop())
            self._widget.visible = True

    def hide(self):
        if self._widget:
            if self._loop_task is not None:
                self._loop_task.cancel()
            self._loop_task = None
            self._widget.visible = False


class AlertPane:
    """A class representing an alert pane within an application dialog.

    This pane can be used to display informational messages or warnings to the user in a stylized format.

    Args:
        width: int
            The width of the alert pane in pixels."""

    Info = 0
    Warn = 1

    def __init__(self, width: int = 400):
        self._widget: ui.Frame = None
        self._frame: ui.Frame = None
        self._icon_frame: ui.Frame = None
        self._label: ui.Label = None
        self._build_ui(width=width)

    def destroy(self):
        self._label = None
        self._frame = None
        self._widget = None

    def _build_ui(self, width: int = 400):
        self._widget = ui.Frame(visible=False, height=0)
        with self._widget:
            with ui.ZStack(width=width, style=UI_STYLES):
                self._frame = ui.Frame()
                with ui.HStack(spacing=4, style_type_name_override="AlertPane.Content"):
                    self._icon_frame = ui.Frame()
                    self._label = ui.Label("", height=20, word_wrap=True, style_type_name_override="Label")

    def show(self, msg: str = "", alert_type: int = 0):
        if self._widget:
            with self._frame:
                style_name = "warn" if alert_type == AlertPane.Warn else "info"
                ui.Rectangle(style_type_name_override="AlertPane", name=style_name)
            with self._icon_frame:
                icon = f"{ICON_PATH}/warn.svg" if alert_type == AlertPane.Warn else f"{ICON_PATH}/info.svg"
                ui.ImageWithProvider(
                    icon,
                    width=16,
                    height=16,
                    fill_policy=ui.IwpFillPolicy.IWP_PRESERVE_ASPECT_FIT,
                    style_type_name_override="Image",
                )
            if self._label:
                self._label.text = msg
            self._widget.visible = True

    def hide(self):
        if self._widget:
            self._widget.visible = False


# OM-98450: Add UI for device auth flow
class DeviceAuthFlowDialog(PopupDialog):
    """A dialog for facilitating device authentication flow.

    This class manages the user interface for device authentication, providing a QR code for the user
    to scan and input a code for authentication. It handles countdowns for code expiration and
    retries for expired codes.

    Args:
        params (omni.client.AuthDeviceFlowParams): Parameters for device authentication including URL, server, and expiration.
    """

    TEMP_DIR = os.path.join(tempfile.gettempdir(), "kit_device_auth_qrcode/")

    """The main authentication dialog for device auth flow."""

    def __init__(self, params: omni.client.AuthDeviceFlowParams):
        super().__init__(width=400, title="Device Authentication", ok_label="Retry", cancel_label="Cancel", modal=True)
        self._image = None
        self._time_remaining = params.expiration
        self._server = params.server
        self._expiration_frame = None
        self._build_ui(params)
        self._count_down_task = run_coroutine(self._count_expiration_async())
        self._generate_qrcode(params.url)

    @property
    def visible(self) -> bool:
        if self._window:
            return self._window.visible
        return False

    def _build_ui(self, params: omni.client.AuthDeviceFlowParams):
        with self._window.frame:
            with ui.VStack(style=UI_STYLES, style_type_name_override="Dialog"):
                ui.Label(f"Connecting to {params.server}", alignment=ui.Alignment.CENTER)
                with ui.HStack():
                    ui.Spacer()
                    self._image = ui.Image(width=128, height=128, style_type_name_override="QrCode")
                    ui.Spacer()
                with ui.HStack():
                    ui.Label(f"Please go to ", width=0)
                    # OM-102092: Change URL to be a string field so it is selectable and could be copied
                    # TODO: Note that if the URL is too long, currently it could get clipped and will not display the
                    #  entire url; Solution could be:
                    #  1) Have both a label and a string field, and when mouse pressed, switch to string field for
                    #     selection; which is a bit hacky, since the string field height need to be dynamically adjusted
                    #  2) have omni.ui.Label support selection of text
                    ui.StringField(
                        ui.SimpleStringModel(params.url),
                        style=UI_STYLES,
                        style_type_name_override="StringField.Url",
                        read_only=True,
                    )
                ui.Label("or scan the qrcode above, and enter the code below:")
                ui.Spacer(height=2)
                ui.Label(params.code, style_type_name_override="Label.Code")
                ui.Spacer(height=2)
                self._expiration_frame = ui.Frame()
                with self._expiration_frame:
                    with ui.HStack(spacing=4):
                        ui.Spacer()
                        ui.Label("Code expires in", width=0)
                        self._time_remaining_label = ui.Label(
                            str(self._time_remaining), style_type_name_override="Label.TimeRemaining", width=0
                        )
                        ui.Label("seconds", width=0)
                        ui.Spacer()
                ui.Spacer(height=10)
                self._build_ok_cancel_buttons()
                ui.Spacer(height=30)
        self.set_cancel_clicked_fn(lambda _: self._on_cancel_fn())
        self.set_okay_clicked_fn(lambda _: self._on_retry())
        self._okay_button.visible = False

    def _on_cancel_fn(self):
        self._count_down_task.cancel()
        self.hide()

    def _on_retry(self):
        self._on_cancel()
        if not self._server.startswith("omniverse://"):
            server = "omniverse://" + self._server
        else:
            server = self._server
        omni.client.reconnect(server)

    def destroy(self):
        if self._count_down_task:
            self._count_down_task.cancel()
            self._count_down_task = None
        super().destroy()

    def __del__(self):
        self.destroy()

    def hide(self):
        self._window.visible = False
        # reset window title on hide; authentication mode might have reset the window title.
        self._window.title = self._title

    def show_expired(self):
        """Show the user when the code has expired."""
        with self._expiration_frame:
            ui.Label("Code has expired. Please try again.", style_type_name_override="Label.Expired")
        self._okay_button.visible = True

    def _generate_qrcode(self, url: str):  # pragma: no cover
        """Generates a qrcode from the given url."""
        if not os.path.exists(self.TEMP_DIR):
            os.mkdir(self.TEMP_DIR)
        # Swap out the : && / since the url we are dealing with are usually something like: "https://a.b.c/x/y",
        #  replacing these special characters with "_" here so we don't end up violating OS naming conventions
        img_name = url.replace("/", "_").replace(":", "_")
        img_path = os.path.join(self.TEMP_DIR, f"qr_{img_name}.png")

        # we could skip generating the qrcode for the same url, since the image would be the same with the same encoding
        if not os.path.exists(img_path):
            # try to use qrcode module to generate the image
            try:
                import qrcode
            except ImportError:
                carb.log_warn("Package qrcode not installed. Skipping qr code generation.")
                return

            img = qrcode.make(url)
            img.save(img_path)

        # update image source url
        self._image.source_url = img_path

    async def _count_expiration_async(self):
        """Counts down the remaining time in seconds before expiration."""
        while self._time_remaining > 0:
            await asyncio.sleep(1)
            self._time_remaining -= 1
            self._time_remaining_label.text = str(self._time_remaining)
        self.show_expired()
