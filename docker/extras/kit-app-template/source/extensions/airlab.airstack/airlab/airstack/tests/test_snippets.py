"""Basic tests of the hardcoded snippets delivered to the user"""

import ast
import re
import sys
from io import StringIO

import omni.graph.core as og
import omni.graph.core.tests as ogts
import omni.usd
from airlab.airstack._impl.parse_examples import extract_snippets
from airlab.airstack.ogn.OgnAscentNodeDatabase import OgnAscentNodeDatabase
from pxr import Usd


# ==============================================================================================================
class TestAscentNodeSnippets(ogts.OmniGraphTestCase):

    # The leading underscore methods are tests for individual code snippets. Each requires a different configuration
    # and setup so its easiest to keep them separated like this, and makes it easier to add new snippets.
    # --------------------------------------------------------------------------------------------------------------
    async def _test_default_script(self, graph: og.Graph, script_node: og.Node):
        """Test the snippet with the title 'Default Script'. Only need to check that it set the output execution state.
        Args:
            graph: The graph in which the node lives
            script_node: The node based on the snippet to be tested
        """
        await og.Controller.evaluate(graph)
        exec_out = script_node.get_attribute("outputs:execOut")
        self.assertTrue(exec_out.is_valid())
        self.assertEqual(og.ExecutionAttributeState.ENABLED, og.Controller.get(exec_out))

    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    async def _test_compute_count(self, graph: og.Graph, script_node: og.Node):
        """Test the snippet with the title 'Compute Count' by evaluating a few times and checking the count.
        Args:
            graph: The graph in which the node lives
            script_node: The node based on the snippet to be tested
        """
        for _ in range(3):
            await og.Controller.evaluate(graph)
        out_attr = script_node.get_attribute("outputs:my_output_attribute")
        self.assertTrue(out_attr.is_valid())
        self.assertLessEqual(3, og.Controller.get(out_attr))

    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    async def _test_fibonacci(self, graph: og.Graph, script_node: og.Node):
        """Test the snippet with the title 'Fibonacci'.
        Args:
            graph: The graph in which the node lives
            script_node: The node based on the snippet to be tested
        """
        for _ in range(6):
            await og.Controller.evaluate(graph)
        out_attr = script_node.get_attribute("outputs:my_output_attribute")
        self.assertTrue(out_attr.is_valid())
        # The sixth Fibonacci number is 8, corresponding to six evaluations
        self.assertEqual(8, og.Controller.get(out_attr))

    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    async def _test_controller(self, graph: og.Graph):
        """Test the snippet with the title 'Controller'.
        Args:
            graph: The graph in which the node lives
            script_node: The node based on the snippet to be tested
        """
        await og.Controller.evaluate(graph)
        await og.Controller.evaluate(graph)
        cube_count = 0
        for prim in Usd.PrimRange(omni.usd.get_context().get_stage().GetPrimAtPath("/World")):
            if str(prim.GetPrimPath()).startswith("Cube"):
                cube_count += 1

    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    async def _test_warp_deformer(self, graph: og.Graph, script_node: og.Node):
        """Test the snippet with the title 'Sine Deformer With Warp'.
        Args:
            graph: The graph in which the node lives
            script_node: The node based on the snippet to be tested
        """
        in_points_attr = script_node.get_attribute("inputs:points")
        time_attr = script_node.get_attribute("inputs:time")
        out_points_attr = script_node.get_attribute("outputs:points")
        og.Controller.set(in_points_attr, [[1.0, 1.0, 1.0], [2.0, 2.0, 2.0]])
        og.Controller.set(time_attr, 2.0)
        await og.Controller.evaluate()
        result = og.Controller.get(out_points_attr)
        self.assertEqual(1.0, result[0][0])
        self.assertAlmostEqual(9.632094, result[0][1], places=5)
        self.assertEqual(1.0, result[0][2])
        self.assertEqual(2.0, result[1][0])
        self.assertAlmostEqual(10.084964, result[1][1], places=5)
        self.assertEqual(2.0, result[1][2])

    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    async def _test_callbacks(self, graph: og.Graph, script_node: og.Node):
        """Test the snippet with the title 'Value Changed Callbacks'.
        Args:
            graph: The graph in which the node lives
            script_node: The node based on the snippet to be tested
        """
        # Capture printed messages so that execution of the callback can be tested
        output = StringIO()
        sys.stdout = output
        # Set the value twice to ensure the callback is triggered
        og.Controller.set(script_node.get_attribute("inputs:my_input_attribute"), 3)
        await omni.kit.app.get_app().next_update_async()
        og.Controller.set(script_node.get_attribute("inputs:my_input_attribute"), 4)
        await omni.kit.app.get_app().next_update_async()
        # Wait a tick so that the callback gets a chance to be processed
        await omni.kit.app.get_app().next_update_async()
        sys.stdout = sys.__stdout__

        self.assertTrue("Setting up the value changed callback" in output.getvalue())
        self.assertTrue("inputs:my_input_attribute" in output.getvalue())

    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    async def _test_compute_timer(self, graph: og.Graph, script_node: og.Node):
        """Test the snippet with the title 'Compute Timer'.
        Args:
            graph: The graph in which the node lives
            script_node: The node based on the snippet to be tested
        """
        await og.Controller.evaluate(graph)
        state_info = OgnAscentNodeDatabase.get_internal_state(script_node, script_node.get_graph_instance_id())
        self.assertTrue(state_info.elapsed > 0.09)

    # --------------------------------------------------------------------------------------------------------------
    async def test_extracted_snippets(self):
        """Extract the snippets from the example scripts and make script nodes from them to ensure they are legal"""

        # Expressions to extract attribute descriptions from the code strings
        re_inputs = re.compile(r"\s*#\s*(inputs:[^\(]+)\(([^\)]+)\)\s*(.*)")
        re_outputs = re.compile(r"\s*#\s*(outputs:[^\(]+)\(([^\)]+)\)\s*(.*)")
        re_state = re.compile(r"\s*#\s*(state:[^\(]+)\(([^\)]+)\)\s*(.*)")

        # Read the snippet file and extract all of the individual strings
        snippets = extract_snippets()

        controller = og.Controller()
        (graph, _, _, _) = controller.edit("/TestGraph", {})

        for index, (title, content) in enumerate(snippets, 1):
            # For the Warp sample first check to make sure that omni.warp is accessible before running the test
            if (
                title == "Sine Deformer With Warp"
                and not omni.kit.app.get_app_interface().get_extension_manager().is_extension_enabled("omni.warp")
            ):  # pragma: no cover  Warp should always be present
                continue
            script_node = controller.create_node((f"AscentNode{index}", graph), "airlab.airstack.AscentNode")
            self.assertIsNotNone(script_node)
            self.assertTrue(script_node.is_valid())

            # Empty titles indicate parsing failure
            self.assertTrue(title, f"No title in snippet {index}")
            self.assertTrue(content, f"No content in snippet {index}")

            # The first thing that can be confirmed is that the string is valid Python by using the AST to parse it.
            try:
                ast.parse(content)
            except SyntaxError as error:  # pragma: no cover  Firewall
                self.fail(f"Failed to parse content in snippet {index} with '{error}'")

            # The attribute descriptions can be extracted from the string and added to the script node to confirm
            # that they are legal.
            for code_line in content.split("\n"):
                input_match = re_inputs.match(code_line)
                if input_match:
                    (name, attr_type, description) = input_match.groups()
                    self.assertTrue(description, f"Found empty description on attribute {name} at snippet {index}")
                    input_attr = controller.create_attribute(script_node, name, attr_type, og.AttributePortType.INPUT)
                    self.assertIsNotNone(input_attr)
                    self.assertTrue(input_attr.is_valid())
                    continue

                output_match = re_outputs.match(code_line)
                if output_match:
                    (name, attr_type, description) = output_match.groups()
                    self.assertTrue(description, f"Found empty description on attribute {name} at snippet {index}")
                    output_attr = controller.create_attribute(script_node, name, attr_type, og.AttributePortType.OUTPUT)
                    self.assertIsNotNone(output_attr)
                    self.assertTrue(output_attr.is_valid())
                    continue

                state_match = re_state.match(code_line)
                if state_match:
                    (name, attr_type, description) = state_match.groups()
                    self.assertTrue(description, f"Found empty description on attribute {name} at snippet {index}")
                    state_attr = controller.create_attribute(script_node, name, attr_type, og.AttributePortType.STATE)
                    self.assertIsNotNone(state_attr)
                    self.assertTrue(state_attr.is_valid())
                    continue

            # Now that the attributes are present, apply the script to the node
            script_attr = script_node.get_attribute("inputs:script")
            og.Controller.set(script_attr, content)

            # Resetting the node ensures everything starts in a stable state
            OgnAscentNodeDatabase.NODE_TYPE_CLASS.try_cleanup(script_node)

            # Snippet-specific testing. This relies on knowledge of how the specific snippet works. If any new
            # snippets are added without tests this will fail and a new test should be added for it.
            match title:
                case "Default Script":
                    await self._test_default_script(graph, script_node)
                case "Compute Count":
                    await self._test_compute_count(graph, script_node)
                case "Fibonacci":
                    await self._test_fibonacci(graph, script_node)
                case "Controller":
                    await self._test_controller(graph)
                case "Sine Deformer With Warp":
                    await self._test_warp_deformer(graph, script_node)
                case "Value Changed Callbacks":
                    await self._test_callbacks(graph, script_node)
                case "Compute Timer":
                    await self._test_compute_timer(graph, script_node)
                case _:
                    self.fail(f"Test needs to be written for snippet '{title}'")
