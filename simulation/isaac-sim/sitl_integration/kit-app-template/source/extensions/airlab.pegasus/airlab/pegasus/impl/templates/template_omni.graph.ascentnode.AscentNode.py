"""Custom property panel layout for airlab.pegasus"""

import asyncio
import os
import shutil
import subprocess
import weakref
from functools import partial

import carb.settings
import omni.client
import omni.graph.core as og
import omni.graph.tools as ogt
import omni.graph.tools.ogn as ogn
import omni.ui as ui
from airlab.pegasus.ogn.OgnAscentNodeDatabase import OgnAscentNodeDatabase
from omni.graph.ui import OmniGraphAttributeModel, OmniGraphPropertiesWidgetBuilder
from omni.kit.property.usd.custom_layout_helper import CustomLayoutFrame, CustomLayoutGroup, CustomLayoutProperty
from omni.kit.property.usd.usd_attribute_widget import UsdPropertyUiEntry
from omni.kit.property.usd.usd_property_widget import UsdPropertiesWidgetBuilder
from omni.kit.widget.text_editor import TextEditor
from pxr import Sdf, Usd

EXT_PATH = omni.kit.app.get_app().get_extension_manager().get_extension_path_by_module("airlab.pegasus")
ICONS_PATH = f"{EXT_PATH}/icons"
FONTS_PATH = f"{EXT_PATH}/fonts"
ATTRIB_LABEL_STYLE = {"alignment": ui.Alignment.LEFT_TOP}


# ==========================================================================================
# Destroy a window after a delay. Useful for having a widget callback destroy its own
# window without crashing Kit.
#
# One drawback to this approach is that this embeds a reference to the window which may keep it
# alive if it is closed by other means (e.g. using its close icon). This can usually be overcome
# by giving the window a visibility_changed_fn which also calls this.
def _delayed_destroy_window(window: ui.Window, num_ticks=1):
    async def do_destroy(window, num_ticks):
        # The window might be destroyed by other means while we're waiting. Get a weak reference to it.
        window_ref = weakref.ref(window)
        for _ in range(num_ticks):
            await omni.kit.app.get_app().next_update_async()
        window = window_ref()
        if window is not None:
            window.destroy()

    asyncio.ensure_future(do_destroy(window, num_ticks))


class ComboBoxOption(ui.AbstractItem):
    """Provide a conversion from simple text to a StringModel to be used in ComboBox options"""

    def __init__(self, text: str):
        super().__init__()
        self.model = ui.SimpleStringModel(text)

    def destroy(self):
        self.model = None


class ComboBoxModel(ui.AbstractItemModel):
    """The underlying model of a combo box"""

    def __init__(self, option_names, option_values, current_value, on_value_changed_callback):
        super().__init__()
        self.option_names = option_names
        self.option_values = option_values
        self.on_value_changed_callback = on_value_changed_callback
        self._current_index = ui.SimpleIntModel(self.option_values.index(current_value))
        self._current_index.add_value_changed_fn(self._on_index_changed)
        self._items = [ComboBoxOption(option) for option in self.option_names]

    def destroy(self):
        ogt.destroy_property(self, "_current_index")
        ogt.destroy_property(self, "_items")

    def get_item_children(self, item):
        return self._items

    def get_item_value_model(self, item, column_id: int):
        if item is None:
            return self._current_index  # combo box expects index model on item == None
        return item.model

    def _on_index_changed(self, new_index: ui.SimpleIntModel):
        new_value = self.option_values[new_index.as_int]
        self.on_value_changed_callback(new_value)
        self._item_changed(None)


class CreateAttributePopupDialog:
    """The popup dialog for creating new dynamic attribute on the script node"""

    def __init__(self, create_new_attribute_callback, **kwargs):
        self.create_new_attribute_callback = create_new_attribute_callback
        self.all_supported_types = []
        self.all_displayed_types = []
        self.all_sdf_types = []
        self.window = None
        self.attribute_name_field = None
        self.port_type_radio_collection = None
        self.input_port_button = None
        self.output_port_button = None
        self.state_port_button = None
        self.scrolling_frame = None
        self.selected_type_button = None
        self.selected_memory_type = None
        self.selected_cuda_pointers = None
        self.error_message_label = None

        self.get_all_supported_types()
        self.build_popup_dialog()

    def get_all_supported_types(self):
        """Get a list of types that can be added to the script node"""
        # "any" types need to be manually resolved by script writers,
        # "transform" types are marked for deprecation in USD, so we don't want to support them
        self.all_supported_types = [
            attr_type
            for attr_type in ogn.supported_attribute_type_names()
            if attr_type != "any" and attr_type[:9] != "transform"
        ]
        self.all_displayed_types = self.all_supported_types
        self.all_sdf_types = [
            og.AttributeType.sdf_type_name_from_type(og.AttributeType.type_from_ogn_type_name(attr_type)) or ""
            for attr_type in self.all_supported_types
        ]

    def build_scrolling_frame(self):
        """Build the scrolling frame underneath the search bar"""

        def _on_type_selected(button):
            if self.selected_type_button is not None:
                self.selected_type_button.checked = False
            self.selected_type_button = button
            self.selected_type_button.checked = True

        self.scrolling_frame.clear()
        with self.scrolling_frame:
            with ui.VStack():
                for displayed_type in self.all_displayed_types:
                    button = ui.Button(displayed_type, height=20)
                    button.set_clicked_fn(partial(_on_type_selected, button))

    def build_popup_dialog(self):
        def filter_types_by_prefix(text):
            """Callback executed when the user presses enter in the search bar"""
            if text is None:
                self.all_displayed_types = self.all_supported_types
            else:
                text = text[0]
                self.all_displayed_types = [
                    displayed_type
                    for i, displayed_type in enumerate(self.all_supported_types)
                    if (displayed_type.startswith(text) or self.all_sdf_types[i].startswith(text))
                ]
            self.build_scrolling_frame()
            self.selected_type_button = None

        def on_create_new_attribute(weak_self):
            """Callback executed when the user creates a new dynamic attribute"""
            ref_self = weak_self()
            if not ref_self.attribute_name_field.model.get_value_as_string():
                ref_self.error_message_label.text = "Error: Attribute name cannot be empty!"
                return

            if not ref_self.attribute_name_field.model.get_value_as_string()[0].isalpha():
                ref_self.error_message_label.text = "Error: The first character of attribute name must be a letter!"
                return

            if (
                not ref_self.input_port_button.checked
                and not ref_self.output_port_button.checked
                and not ref_self.state_port_button.checked
            ):
                ref_self.error_message_label.text = "Error: You must select a port type!"
                return

            if ref_self.selected_type_button is None:
                ref_self.error_message_label.text = "Error: You must select a type for the new attribute!"
                return

            attrib_name = ref_self.attribute_name_field.model.get_value_as_string()
            attrib_port_type = og.AttributePortType.ATTRIBUTE_PORT_TYPE_INPUT
            if ref_self.output_port_button.checked:
                attrib_port_type = og.AttributePortType.ATTRIBUTE_PORT_TYPE_OUTPUT
            elif ref_self.state_port_button.checked:
                attrib_port_type = og.AttributePortType.ATTRIBUTE_PORT_TYPE_STATE
            attrib_type_name = ref_self.selected_type_button.text
            attrib_type = og.AttributeType.type_from_ogn_type_name(attrib_type_name)
            ref_self.create_new_attribute_callback(
                attrib_name,
                attrib_port_type,
                attrib_type,
                ref_self.selected_memory_type,
                ref_self.selected_cuda_pointers,
            )

            ref_self.window.visible = False

        def on_cancel_clicked(weak_self):
            ref_self = weak_self()
            ref_self.window.visible = False

        window_flags = ui.WINDOW_FLAGS_NO_RESIZE | ui.WINDOW_FLAGS_MODAL
        self.window = ui.Window(
            "Add Attribute",
            width=400,
            height=0,
            padding_x=15,
            padding_y=15,
            flags=window_flags,
        )
        input_field_width = ui.Percent(80)

        with self.window.frame:
            with ui.VStack(spacing=10):
                # Attribute name string field
                with ui.HStack(height=0):
                    ui.Label("Name")
                    self.attribute_name_field = ui.StringField(
                        width=input_field_width, height=20, identifier="_ascentnode_name"
                    )

                # Attribute port type radio button
                with ui.HStack(height=0):
                    ui.Label("Port Type")
                    self.port_type_radio_collection = ui.RadioCollection()
                    with ui.HStack(width=input_field_width, height=20):
                        self.input_port_button = ui.RadioButton(
                            text="input", radio_collection=self.port_type_radio_collection
                        )
                        self.output_port_button = ui.RadioButton(
                            text="output", radio_collection=self.port_type_radio_collection
                        )
                        self.state_port_button = ui.RadioButton(
                            text="state", radio_collection=self.port_type_radio_collection
                        )

                # Attribute type search bar
                with ui.HStack(height=0):
                    ui.Label("Data Type", alignment=ui.Alignment.LEFT_TOP)
                    with ui.VStack(width=input_field_width):
                        # Search bar
                        try:
                            from omni.kit.widget.searchfield import SearchField

                            SearchField(
                                show_tokens=False, on_search_fn=filter_types_by_prefix, subscribe_edit_changed=True
                            )
                        except ImportError:
                            # skip the search bar if the module cannot be imported
                            pass
                        # List of attribute types
                        self.scrolling_frame = ui.ScrollingFrame(
                            height=150,
                            horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_OFF,
                            vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                            style_type_name_override="TreeView",
                        )
                        self.build_scrolling_frame()

                # TODO: Uncomment this block when dynamic attributes support memory types
                # # Attribute memory type combo box
                # with ui.HStack(height=0):
                #     ui.Label("Attribute Memory Type: ")
                #     memory_type_option_names = ["CPU", "CUDA", "Any"]
                #     memory_type_option_values = [ogn.MemoryTypeValues.CPU,
                #         ogn.MemoryTypeValues.CUDA, ogn.MemoryTypeValues.ANY]
                #     self.selected_memory_type = ogn.MemoryTypeValues.CPU
                #     def _on_memory_type_selected(new_memory_type):
                #         self.selected_memory_type = new_memory_type
                #     ui.ComboBox(
                #         ComboBoxModel(
                #             memory_type_option_names,
                #             memory_type_option_values,
                #             self.selected_memory_type,
                #             _on_memory_type_selected,
                #         ),
                #         width=input_field_width,
                #     )

                # # CUDA pointers combo box
                # with ui.HStack(height=0):
                #     ui.Label("CUDA Pointers: ")
                #     cuda_pointers_option_names = ["CUDA", "CPU"]
                #     cuda_pointers_option_values = [ogn.CudaPointerValues.CUDA, ogn.CudaPointerValues.CPU]
                #     self.selected_cuda_pointers = ogn.CudaPointerValues.CUDA
                #     def _on_cuda_pointers_selected(new_cuda_pointers):
                #         self.selected_cuda_pointers = new_cuda_pointers
                #     ui.ComboBox(
                #         ComboBoxModel(
                #             cuda_pointers_option_names,
                #             cuda_pointers_option_values,
                #             self.selected_cuda_pointers,
                #             _on_cuda_pointers_selected,
                #         ),
                #         width=input_field_width,
                #     )

                # OK button to confirm selection
                with ui.HStack(height=0):
                    ui.Spacer()
                    with ui.HStack(width=input_field_width, height=20):
                        ui.Button(
                            "OK",
                            clicked_fn=lambda weak_self=weakref.ref(self): on_create_new_attribute(weak_self),
                            identifier="_ascentnode_add_ok",
                        )
                        ui.Button(
                            "Cancel",
                            clicked_fn=lambda weak_self=weakref.ref(self): on_cancel_clicked(weak_self),
                            identifier="_ascentnode_add_cancel",
                        )

                # Some empty space to display error messages if needed
                self.error_message_label = ui.Label(
                    " ", height=20, alignment=ui.Alignment.H_CENTER, style={"color": 0xFF0000FF}
                )


class ScriptTextbox(TextEditor):
    def __init__(self, script_model: OmniGraphAttributeModel):
        super().__init__(
            syntax=TextEditor.Syntax.PYTHON,
            style={"font": f"{FONTS_PATH}/DejaVuSansMono.ttf"},
            text=script_model.get_value_as_string(),
        )
        self.script_model = script_model
        self.script_model_callback_id = self.script_model.add_value_changed_fn(self._on_script_model_changed)

        self.set_edited_fn(self._on_script_edited)

    def _on_script_edited(self, text_changed: bool):
        if text_changed:
            # Don't trigger the model changed callback when script is edited
            self.script_model.remove_value_changed_fn(self.script_model_callback_id)
            # Remove the newline that TextEditor adds or else it will accumulate
            self.script_model.set_value(self.text[:-1])
            self.script_model_callback_id = self.script_model.add_value_changed_fn(self._on_script_model_changed)

    def _on_script_model_changed(self, script_model):
        self.text = script_model.get_value_as_string()  # noqa: PLW0201


class CustomLayout:
    def __init__(self, compute_node_widget):
        self.remove_attribute_menu = None
        self.code_snippets_menu = None
        self.enable = True
        self.compute_node_widget = compute_node_widget
        self.node_prim_path = self.compute_node_widget._payload[-1]
        self.node = og.Controller.node(self.node_prim_path)
        self.script_textbox_widget = None
        self.script_textbox_model = None
        self.script_textbox_resizer = None
        self.script_path_model = None
        self.script_selector_window = None
        self.external_script_editor = None
        self.external_script_editor_ui_name = None
        self.DEFAULT_SCRIPT = ""
        self.EXAMPLE_SCRIPTS = []
        self.EXAMPLE_SCRIPTS_TITLE = []
        self.add_attribute_button = None
        self.remove_attribute_button = None
        self.code_snippets_button = None
        self.use_path_model = None
        self.EXISTING_ATTRIBUTES = [
            "inputs:script",
            "inputs:scriptPath",
            "inputs:usePath",
            "inputs:execIn",
            "outputs:execOut",
            "state:omni_initialized",
            "node:type",
            "node:typeVersion",
        ]

        # Retrieve the example scripts
        cur_file_path = os.path.abspath(os.path.dirname(__file__))
        example_scripts_path = os.path.join(cur_file_path, "..", "ascentnode_example_scripts.py")

        with open(example_scripts_path, "r", encoding="utf-8") as file:
            file_contents = file.read().split("# # # DELIMITER # # #")
            for script in file_contents[1:]:
                script = script.strip()
                script_lines = script.splitlines(keepends=True)
                script_title_line = script_lines[0]
                script_title = script_title_line.strip()[9:-1]
                script_content = "".join(script_lines[1:])

                if script_title == "Default Script":
                    self.DEFAULT_SCRIPT = script_content
                else:
                    self.EXAMPLE_SCRIPTS.append(script_content)
                    self.EXAMPLE_SCRIPTS_TITLE.append(script_title)

        # Determine the external script editor
        # Check the settings
        editor = carb.settings.get_settings().get("/app/editor")
        if not editor:
            # Check the environment variable EDITOR
            editor = os.environ.get("EDITOR", None)
            if not editor:
                # Default to VSCode
                editor = "code"

        # Remove quotes from the editor name if present
        if editor[0] == '"' and editor[-1] == '"':
            editor = editor[1:-1]

        # Get the user-friendly editor name
        editor_ui_name = editor
        if editor == "code":
            editor_ui_name = "VSCode"
        elif editor == "notepad":
            editor_ui_name = "Notepad"

        # Check that the editor exists and is executable
        if not (os.path.isfile(editor) and os.access(editor, os.X_OK)):
            try:
                editor = shutil.which(editor)
            except shutil.Error:
                editor = None

        if not editor:
            # Resort to notepad on windows and gedit on linux
            if os.name == "nt":
                editor = "notepad"
                editor_ui_name = "Notepad"
            else:
                editor = "gedit"
                editor_ui_name = "gedit"

        self.external_script_editor = editor
        self.external_script_editor_ui_name = editor_ui_name

    def retrieve_existing_attributes(self):
        """Retrieve the dynamic attributes that already exist on the node"""
        all_attributes = self.node.get_attributes()
        inputs = [
            attrib
            for attrib in all_attributes
            if attrib.get_port_type() == og.AttributePortType.ATTRIBUTE_PORT_TYPE_INPUT
        ]
        outputs = [
            attrib
            for attrib in all_attributes
            if attrib.get_port_type() == og.AttributePortType.ATTRIBUTE_PORT_TYPE_OUTPUT
        ]
        states = [
            attrib
            for attrib in all_attributes
            if attrib.get_port_type() == og.AttributePortType.ATTRIBUTE_PORT_TYPE_STATE
        ]
        return (all_attributes, inputs, outputs, states)

    def _on_script_textbox_resizer_dragged(self, offset_y: ui.Length):
        self.script_textbox_resizer.offset_y = max(offset_y.value, 50)

    def _script_textbox_build_fn(self, ui_prop: UsdPropertyUiEntry, *args):
        """Build the textbox used to input custom scripts"""
        self.script_textbox_model = OmniGraphAttributeModel(
            self.compute_node_widget.stage, [self.node_prim_path.AppendProperty("inputs:script")], False, {}
        )
        if self.script_textbox_model.get_value_as_string() == "":
            self.script_textbox_model.set_value(self.DEFAULT_SCRIPT)

        with ui.ZStack():
            with ui.VStack():
                self.script_textbox_widget = ScriptTextbox(self.script_textbox_model)
                # Disable editing if the script value comes from an upstream connection
                if og.Controller.attribute("inputs:script", self.node).get_upstream_connection_count() > 0:
                    self.script_textbox_widget.read_only = True  # noqa: PLW0201
                ui.Spacer(height=12)
            # Add a draggable bar below the script textbox to resize it
            self.script_textbox_resizer = ui.Placer(offset_y=200, draggable=True, drag_axis=ui.Axis.Y)
            self.script_textbox_resizer.set_offset_y_changed_fn(self._on_script_textbox_resizer_dragged)
            with self.script_textbox_resizer:
                script_textbox_resizer_style = {
                    ":hovered": {"background_color": 0xFFB0703B},
                    ":pressed": {"background_color": 0xFFB0703B},
                }
                with ui.ZStack(height=12):
                    ui.Rectangle(style=script_textbox_resizer_style)
                    with ui.HStack(height=12):
                        ui.Label(
                            "V", alignment=ui.Alignment.CENTER, tooltip="Drag this handle to resize the script editor"
                        )

    def _reset_button_build_fn(self, *args):
        """Build button that calls the cleanup script and forces the setup script to be called on next compute"""

        def do_reset():
            """Call the user-defined cleanup function and set state:omni_initialized to false"""
            OgnAscentNodeDatabase.NODE_TYPE_CLASS.try_cleanup(self.node)

        ui.Button(
            "Reload Script",
            identifier="_ascentnode_reset",
            clicked_fn=do_reset,
            tooltip="Execute the setup script again on the next compute and recompile if necessary",
        )

    def _add_attribute_button_build_fn(self, *args):
        def create_dynamic_attribute(
            weak_self, attrib_name, attrib_port_type, attrib_type, attrib_memory_type, cuda_pointers
        ):
            self_ref = weak_self()
            if (
                attrib_name == "execOut"
                and attrib_port_type == og.AttributePortType.ATTRIBUTE_PORT_TYPE_OUTPUT
                and attrib_type.get_ogn_type_name() == "execution"
            ):
                # Unhide outputs:execOut instead of creating it
                self_ref.node.get_attribute("outputs:execOut").set_metadata(ogn.MetadataKeys.HIDDEN, None)
                return

            new_attribute = og.Controller.create_attribute(self_ref.node, attrib_name, attrib_type, attrib_port_type)
            if new_attribute is None:
                return

            if attrib_type.get_type_name() == "prim" and attrib_port_type in (
                og.AttributePortType.ATTRIBUTE_PORT_TYPE_OUTPUT,
                og.AttributePortType.ATTRIBUTE_PORT_TYPE_STATE,
            ):
                # For bundle output/state attribs, the default UI name contains the port type, so we set it here instead
                def make_ui_name(attrib_name: str):
                    parts_out = []
                    words = attrib_name.replace("_", " ").split(" ")
                    for word in words:  # noqa: PLR1702
                        if word.islower() or word.isupper():
                            parts_out += [word]
                        else:
                            # Mixed case.
                            # Lower-case followed by upper-case breaks between them. E.g. 'usdPrim' -> 'usd Prim'
                            # Upper-case followed by lower-case breaks before them. E.g: 'USDPrim' -> 'USD Prim'
                            # Combined example: abcDEFgHi -> abc DE Fg Hi
                            sub_word = ""
                            uppers = ""
                            for c in word:
                                if c.isupper():
                                    if not uppers:  # noqa: SIM102
                                        if sub_word:
                                            parts_out += [sub_word]
                                            sub_word = ""
                                    uppers += c
                                else:
                                    if len(uppers) > 1:
                                        parts_out += [uppers[:-1]]
                                    sub_word += uppers[-1:] + c
                                    uppers = ""

                            if sub_word:
                                parts_out += [sub_word]
                            elif uppers:
                                parts_out += [uppers]

                    # Title-case any words which are all lower case.
                    parts_out = [part.title() if part.islower() else part for part in parts_out]
                    return " ".join(parts_out)

                new_attribute.set_metadata(ogn.MetadataKeys.UI_NAME, make_ui_name(attrib_name))

            # TODO: Uncomment this when dynamic attributes support memory types
            # new_attribute.set_metadata(ogn.MetadataKeys.MEMORY_TYPE, attrib_memory_type)
            # new_attribute.set_metadata(ogn.MetadataKeys.CUDA_POINTERS, cuda_pointers)
            self_ref.compute_node_widget.rebuild_window()

        def on_click_add(weak_self):
            """Post the "Add Attribute" window"""
            dialog = CreateAttributePopupDialog(partial(create_dynamic_attribute, weak_self))

            def close_window():
                window = dialog.window
                dialog.window = None  # ensure we aren't still holding on to the window going in to GC
                _delayed_destroy_window(window)

            # Called when the window is hidden by the dialog buttons or the close icon
            def visibility_changed(is_visible: bool):
                if not is_visible:
                    close_window()

            dialog.window.set_visibility_changed_fn(visibility_changed)

        self.add_attribute_button = ui.Button(
            "Add Attribute...",
            identifier="_ascentnode_add_attribute",
            clicked_fn=partial(on_click_add, weakref.ref(self)),
        )

    def _remove_attribute_button_build_fn(self, *args):
        def remove_dynamic_attribute(weak_self, attrib):
            ref_self = weak_self()
            if attrib.get_name() == "outputs:execOut":
                # Hide outputs:execOut instead of removing it
                og.Controller.disconnect_all(("outputs:execOut", ref_self.node))
                attrib.set_metadata(ogn.MetadataKeys.HIDDEN, "1")
                return

            success = og.Controller.remove_attribute(attrib)
            if not success:
                return
            self.compute_node_widget.rebuild_window()

        def _remove_attribute_menu_build_fn(weak_self):
            ref_self = weak_self()
            ref_self.remove_attribute_menu = ui.Menu("Remove Attribute")
            (all_attributes, _, _, _) = ref_self.retrieve_existing_attributes()
            with ref_self.remove_attribute_menu:
                for attrib in all_attributes:
                    name = attrib.get_name()
                    # These attributes were not created by user so they are not deletable,
                    # except for outputs:execOut, which can be hidden if not already hidden
                    if not attrib.is_dynamic() and not (
                        name == "outputs:execOut" and attrib.get_metadata(ogn.MetadataKeys.HIDDEN) != "1"
                    ):
                        continue
                    # Any attribute without the inputs:/outputs:/state: prefix were not created by user so they are
                    # not deletable, except for bundle output and state attributes, which are delimited with '_'
                    if (
                        name[:7] != "inputs:"
                        and name[:8] != "outputs:"
                        and name[:6] != "state:"
                        and not (
                            attrib.get_type_name() == "bundle" and (name[:8] == "outputs_" or name[:6] == "state_")
                        )
                    ):
                        continue
                    # Otherwise we should allow user to delete this attribute
                    ui.MenuItem(name, triggered_fn=partial(remove_dynamic_attribute, weak_self, attrib))
            self.remove_attribute_menu.show()

        self.remove_attribute_button = ui.Button(
            "Remove Attribute...",
            identifier="_ascentnode_remove_attribute",
            clicked_fn=lambda weak_self=weakref.ref(self): _remove_attribute_menu_build_fn(weak_self),
        )

    def _special_control_build_fn(self, *args):
        with ui.HStack():
            UsdPropertiesWidgetBuilder._create_label("")  # noqa: PLW0212
            ui.Spacer(width=7)
            self._add_attribute_button_build_fn()
            ui.Spacer(width=8)
            self._remove_attribute_button_build_fn()

    def _get_absolute_script_path(self):
        """Get the possibly relative path in inputs:scriptPath as an absolute path"""
        script_path = og.Controller.get(og.Controller.attribute("inputs:scriptPath", self.node))

        edit_layer = self.compute_node_widget.stage.GetEditTarget().GetLayer()
        if not edit_layer.anonymous:
            script_path = omni.client.combine_urls(edit_layer.realPath, script_path).replace("\\", "/")

        return script_path

    def _show_script_selector_window(self):
        """Create and show the file browser window which is used to select the script for inputs:scriptPath"""
        try:
            from omni.kit.window.filepicker import FilePickerDialog
        except ImportError:
            # Do nothing if the module cannot be imported
            return

        def _on_click_okay(filename: str, dirname: str):
            # Get the relative path relative to the edit layer
            chosen_file = omni.client.combine_urls(dirname, filename)
            edit_layer = self.compute_node_widget.stage.GetEditTarget().GetLayer()
            if not edit_layer.anonymous:
                chosen_file = omni.client.make_relative_url(edit_layer.realPath, chosen_file)
            chosen_file = chosen_file.replace("\\", "/")

            # Set the value of inputs:scriptPath
            self.script_path_model.set_value(chosen_file)
            self.script_selector_window.hide()

        def _on_click_cancel(filename: str, dirname: str):
            self.script_selector_window.hide()

        self.script_selector_window = FilePickerDialog(
            "Select a Python script",
            click_apply_handler=_on_click_okay,
            click_cancel_handler=_on_click_cancel,
            allow_multi_selection=False,
            file_extension_options=[("*.py", "Python scripts (*.py)")],
        )

        self.script_selector_window.show(self._get_absolute_script_path())

    def _launch_external_script_editor(self):
        """Launch an external editor targeting the path specified in inputs:scriptPath"""
        # Use cmd in case the editor is a bat or cmd file
        call_command = ["cmd", "/c"] if os.name == "nt" else []
        call_command.append(self.external_script_editor)
        call_command.append(self._get_absolute_script_path())

        subprocess.Popen(call_command)  # noqa: PLR1732

    def _script_path_build_fn(self, ui_prop: UsdPropertyUiEntry, *args):
        """Build the asset attribute widget for inputs:scriptPath"""
        with ui.HStack():
            self.script_path_model = OmniGraphPropertiesWidgetBuilder.build(
                self.compute_node_widget.stage,
                ui_prop.prop_name,
                ui_prop.metadata,
                ui_prop.property_type,
                [self.node_prim_path],
                {"style": ATTRIB_LABEL_STYLE},
            )

    def get_use_path(self, stage: Usd.Stage, node_prim_path: Sdf.Path) -> bool:
        # Gets the value of the usePath input attribute
        return og.Controller.get(og.Controller.attribute("inputs:usePath", og.get_node_by_path(str(node_prim_path))))

    def _use_path_build_fn(self, ui_prop: UsdPropertyUiEntry, *args):
        # Build the boolean toggle for inputs:usePath
        ui_prop.override_display_name("Use Path")
        self.use_path_model = OmniGraphPropertiesWidgetBuilder.build(
            self.compute_node_widget.stage,
            ui_prop.prop_name,
            ui_prop.metadata,
            ui_prop.property_type,
            [self.node_prim_path],
            {"style": ATTRIB_LABEL_STYLE},
        )
        return self.use_path_model

    def apply(self, props):
        """Called by compute_node_widget to apply UI when selection changes"""

        def find_prop(name):
            try:
                return next((p for p in props if p.prop_name == name))
            except StopIteration:
                return None

        frame = CustomLayoutFrame(hide_extra=True)
        (_, inputs, outputs, states) = self.retrieve_existing_attributes()

        def _build_reset(self, *args):
            def _code_snippets_menu_build_fn(weak_self):
                """Build the code snippets popup menu"""
                self_ref = weak_self()
                self_ref.code_snippets_menu = ui.Menu("Code Snippets")
                with self_ref.code_snippets_menu:
                    for example_script_title, example_script in zip(
                        self_ref.EXAMPLE_SCRIPTS_TITLE, self_ref.EXAMPLE_SCRIPTS
                    ):
                        ui.MenuItem(
                            example_script_title,
                            triggered_fn=partial(self_ref.script_textbox_model.set_value, example_script),
                        )
                self.code_snippets_menu.show()

            with ui.HStack():
                UsdPropertiesWidgetBuilder._create_label("")  # noqa: PLW0212
                ui.Spacer(width=7)
                self._reset_button_build_fn()
                ui.Spacer(width=8)
                self.code_snippets_button = ui.Button(
                    "Code Snippets...",
                    identifier="_ascentnode_snippets",
                    clicked_fn=lambda weak_self=weakref.ref(self): _code_snippets_menu_build_fn(weak_self),
                )

        with frame:
            with CustomLayoutGroup("Add and Remove Attributes"):
                CustomLayoutProperty(None, None, self._special_control_build_fn)

            with CustomLayoutGroup("Script"):
                prop = find_prop("inputs:script")
                CustomLayoutProperty(prop.prop_name, build_fn=partial(self._script_textbox_build_fn, prop))

                CustomLayoutProperty(None, None, partial(_build_reset, self))

                prop = find_prop("inputs:usePath")
                CustomLayoutProperty(prop.prop_name, build_fn=partial(self._use_path_build_fn, prop))

                prop = find_prop("inputs:scriptPath")
                CustomLayoutProperty(prop.prop_name, build_fn=partial(self._script_path_build_fn, prop))

            with CustomLayoutGroup("Inputs"):
                for input_attrib in inputs:
                    attrib_name = input_attrib.get_name()
                    if input_attrib.is_dynamic():
                        prop = find_prop(attrib_name)
                        if prop is not None:
                            CustomLayoutProperty(prop.prop_name, attrib_name[7:])

            with CustomLayoutGroup("Outputs"):
                for output_attrib in outputs:
                    attrib_name = output_attrib.get_name()
                    if output_attrib.is_dynamic():
                        prop = find_prop(attrib_name)
                        if prop is not None:
                            CustomLayoutProperty(prop.prop_name, attrib_name[8:])

            with CustomLayoutGroup("State"):
                for state_attrib in states:
                    attrib_name = state_attrib.get_name()
                    if state_attrib.is_dynamic():
                        prop = find_prop(attrib_name)
                        if prop is not None:
                            CustomLayoutProperty(prop.prop_name, attrib_name)

        return frame.apply(props)
