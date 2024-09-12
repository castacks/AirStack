"""Tests for ascentnode which exercise the UI"""

import os
import tempfile

import carb
import omni.graph.core as og
import omni.graph.ui._impl.omnigraph_attribute_base as ogab
import omni.kit.app
import omni.kit.commands
import omni.kit.test
import omni.ui as ui
import omni.usd
from airlab.airstack._impl.extension import (
    ASCENTNODE_ENABLE_OPT_IN_SETTING,
    ASCENTNODE_OPT_IN_SETTING,
    check_for_ascentnodes,
    is_check_enabled,
    verify_ascentnode_load,
)
from omni.kit import ui_test
from omni.kit.test_suite.helpers import wait_for_window
from omni.ui.tests.test_base import OmniUiTest


class TestAscentNodeUI(OmniUiTest):
    """
    Tests for ascentnode which exercise the UI
    """

    TEST_GRAPH_PATH = "/World/TestGraph"

    # Before running each test
    async def setUp(self):
        await super().setUp()

        # Ensure we have a clean stage for the test
        await omni.usd.get_context().new_stage_async()
        # Give OG a chance to set up on the first stage update
        await omni.kit.app.get_app().next_update_async()
        self._temp_file_path = None  # A temporary file we need to clean up
        import omni.kit.window.property as p

        self._w = p.get_window()

        # The OG attribute-base UI should refresh every frame
        ogab.AUTO_REFRESH_PERIOD = 0

    async def tearDown(self):
        if (self._temp_file_path is not None) and os.path.isfile(self._temp_file_path):
            os.remove(self._temp_file_path)
        # Close the stage to avoid dangling references to the graph. (OM-84680)
        await omni.usd.get_context().close_stage_async()
        await super().tearDown()

    async def test_interaction(self):
        """
        Exercise the controls on the custom template
        """
        usd_context = omni.usd.get_context()

        keys = og.Controller.Keys
        controller = og.Controller()

        (_, (script_node,), _, _) = controller.edit(
            self.TEST_GRAPH_PATH,
            {
                keys.CREATE_NODES: [
                    ("AscentNode", "airlab.airstack.AscentNode"),
                ],
                keys.SET_VALUES: [("AscentNode.inputs:usePath", False), ("AscentNode.inputs:scriptPath", "")],
            },
        )
        ok = script_node.create_attribute(
            "outputs:out", og.Type(og.BaseDataType.INT), og.AttributePortType.ATTRIBUTE_PORT_TYPE_OUTPUT
        )
        self.assertTrue(ok)

        attr_out = script_node.get_attribute("outputs:out")
        attr_script = script_node.get_attribute("inputs:script")

        # Select the node.
        usd_context.get_selection().set_selected_prim_paths([script_node.get_prim_path()], True)

        # Wait for property panel to converge
        await ui_test.human_delay(5)
        # Set the inline script back to empty
        attr_script.set("")
        reset_button = ui_test.find("Property//Frame/**/Button[*].identifier=='_ascentnode_reset'")
        script_path_model = ui_test.find("Property//Frame/**/.identifier=='sdf_asset_inputs:scriptPath'")
        use_path_toggle = ui_test.find("Property//Frame/**/.identifier=='bool_inputs:usePath'")
        snippets_button = ui_test.find("Property//Frame/**/Button[*].identifier=='_ascentnode_snippets'")

        # write out a script and change the file path
        await use_path_toggle.click()
        await ui_test.human_delay(5)

        # NamedTemporaryFile is not very nice on Windows, need to ensure we close before node tries to read
        with tempfile.NamedTemporaryFile(mode="w+t", encoding="utf-8", delete=False) as tf:
            self._temp_file_path = tf.name
            tf.write(
                """
def compute(db: og.Database):
    db.outputs.out = 42
    return True"""
            )

        await script_path_model.input(self._temp_file_path)
        await omni.kit.app.get_app().next_update_async()

        # verify it has now computed, because `input` above will trigger the widget `end_edit`
        self.assertEqual(attr_out.get(), 42)

        # change the script, verify it doesn't take effect until reset is pressed
        with open(self._temp_file_path, mode="w+t", encoding="utf-8") as tf:
            tf.write(
                """
def compute(db: og.Database):
    db.outputs.out = 1000
    return True"""
            )

        await omni.kit.app.get_app().next_update_async()
        # verify it has now computed
        self.assertEqual(attr_out.get(), 42)

        await reset_button.click()
        await ui_test.human_delay(1)
        # Verify the script now computed with the new script
        self.assertEqual(attr_out.get(), 1000)

        # Switch it back to inline script
        attr_script.set(
            """
def compute(db):
    db.outputs.out = 1
"""
        )
        await ui_test.human_delay(1)
        await use_path_toggle.click()
        await reset_button.click()
        await ui_test.human_delay(1)
        self.assertEqual(attr_out.get(), 1)

        # Switch it back to external script
        await use_path_toggle.click()
        await ui_test.human_delay(1)
        self.assertEqual(attr_out.get(), 1000)

        # Now add an attribute using the dialog
        await ui_test.find("Property//Frame/**/Button[*].identifier=='_ascentnode_add_attribute'").click()
        await wait_for_window("Add Attribute")
        await ui_test.find("Add Attribute//Frame/**/StringField[*].identifier=='_ascentnode_name'").input("test_attrib")
        # Find the only string field without an identifier, that is the search field
        await ui_test.find("Add Attribute//Frame/**/StringField[*].identifier!='_ascentnode_name'").input("int64")
        await ui_test.find("Add Attribute//Frame/**/Button[*].text=='int64'").click()

        await ui_test.human_delay(1)

        await ui_test.find("Add Attribute//Frame/**/Button[*].identifier=='_ascentnode_add_ok'").click()
        await ui_test.human_delay(3)

        # Check the attribute was actually added
        attr_test = script_node.get_attribute("inputs:test_attrib")
        self.assertEqual(attr_test.get_resolved_type(), og.Type(og.BaseDataType.INT64, 1, 0))

        # Show the remove menu, remove the item we added

        await ui_test.find("Property//Frame/**/Button[*].identifier=='_ascentnode_remove_attribute'").click()
        await ui_test.human_delay(1)
        menu = ui.Menu.get_current()
        test_item = next((item for item in ui.Inspector.get_children(menu) if item.text == "inputs:test_attrib"))
        test_item.call_triggered_fn()
        await ui_test.human_delay(1)
        self.assertFalse(script_node.get_attribute_exists("inputs:test_attrib"))
        menu.hide()

        # Clear the current script
        attr_script.set("")
        await ui_test.human_delay(1)

        # Show the snippets menu
        await snippets_button.click()
        await ui_test.human_delay(1)
        menu = ui.Menu.get_current()
        # select the first one, verify the script was changed
        ui.Inspector.get_children(menu)[0].call_triggered_fn()
        await ui_test.human_delay(1)
        self.assertTrue("def compute(db)" in attr_script.get())

    # --------------------------------------------------------------------------------------------------------------
    async def test_setting_interaction(self):
        """Test interactions with the user regarding the setting to enable the script nodes"""
        settings = carb.settings.get_settings()

        def _get_settings() -> tuple[bool, bool]:
            return (
                bool(settings.get(ASCENTNODE_ENABLE_OPT_IN_SETTING)),
                bool(settings.get(ASCENTNODE_OPT_IN_SETTING)),
            )

        def _set_settings(enable_opt_in: bool, opt_in: bool):
            settings.set(ASCENTNODE_ENABLE_OPT_IN_SETTING, enable_opt_in)
            settings.set(ASCENTNODE_OPT_IN_SETTING, opt_in)

        (orig_enable_opt_in, orig_opt_in) = _get_settings()

        try:
            # Check the combinations of settings to see if the "check_enabled" state is correct for each one
            settings.destroy_item(ASCENTNODE_ENABLE_OPT_IN_SETTING)
            self.assertTrue(is_check_enabled())
            settings.set(ASCENTNODE_ENABLE_OPT_IN_SETTING, False)
            self.assertFalse(is_check_enabled())
            settings.set(ASCENTNODE_ENABLE_OPT_IN_SETTING, True)
            self.assertTrue(is_check_enabled())

            verify_ascentnode_load([])
            no_button_ref = ui_test.find('Warning//Frame/**/Button[*].text=="No"')
            yes_button_ref = ui_test.find('Warning//Frame/**/Button[*].text=="Yes"')
            if no_button_ref:
                button = no_button_ref.widget
                button.call_clicked_fn()
            (no_enable_opt_in, no_opt_in) = _get_settings()
            self.assertTrue(no_enable_opt_in)
            self.assertFalse(no_opt_in)

            if yes_button_ref:
                button = yes_button_ref.widget
                button.call_clicked_fn()
            (yes_enable_opt_in, yes_opt_in) = _get_settings()
            self.assertTrue(yes_enable_opt_in)
            self.assertTrue(yes_opt_in)

            # Check with no graph first to catch simple cases
            check_for_ascentnodes()

            # Turn the opt-in off to check for the dialog
            _set_settings(True, False)
            check_for_ascentnodes()
            if yes_button_ref:
                button = yes_button_ref.widget
                button.call_clicked_fn()

            # Final case is correct enabling when a graph with script nodes is present
            _set_settings(True, False)
            _ = og.Controller.edit(
                "/TestGraph",
                {
                    og.Controller.Keys.CREATE_NODES: ("Script", "airlab.airstack.AscentNode"),
                },
            )
            check_for_ascentnodes()
            if yes_button_ref:
                button = yes_button_ref.widget
                button.call_clicked_fn()

        finally:
            _set_settings(orig_enable_opt_in, orig_opt_in)
