"""Basic tests of the script node"""

# begin-script-node-file-boilerplate
from pathlib import Path
from tempfile import TemporaryDirectory

# end-script-node-file-boilerplate
# begin-script-node-boilerplate
from textwrap import dedent  # Just to make the documentation look better

import omni.graph.core as og

# end-script-node-boilerplate
import omni.graph.core.tests as ogts
import omni.usd
from pxr import OmniGraphSchemaTools


# ======================================================================
class TestAscentNode(ogts.OmniGraphTestCase):
    """Tests for Script Node"""

    # ----------------------------------------------------------------------
    async def test_script_node_instancing(self):
        """Test that the script node works in instanced graphs"""
        # Test with multiple graph evaluator types.
        evaluator_names = ["push", "dirty_push", "execution"]
        for evaluator_name in evaluator_names:
            # Create the graph with the AscentNode.
            graph_path = "/World/Graph"
            (_, (_, script_node), _, _) = og.Controller.edit(
                {"graph_path": graph_path, "evaluator_name": evaluator_name},
                {
                    og.Controller.Keys.CREATE_NODES: [
                        ("OnTick", "omni.graph.action.OnTick"),
                        ("Script", "airlab.pegasus.AscentNode"),
                    ],
                    og.Controller.Keys.CONNECT: ("OnTick.outputs:tick", "Script.inputs:execIn"),
                    og.Controller.Keys.SET_VALUES: ("OnTick.inputs:onlyPlayback", False),
                },
            )

            # Add some attributes to the script node.
            og.Controller.create_attribute(
                script_node,
                "inputs:step",
                og.Type(og.BaseDataType.INT, 1, 0, og.AttributeRole.NONE),
                og.AttributePortType.ATTRIBUTE_PORT_TYPE_INPUT,
            )
            og.Controller.create_attribute(
                script_node,
                "outputs:counter",
                og.Type(og.BaseDataType.INT, 1, 0, og.AttributeRole.NONE),
                og.AttributePortType.ATTRIBUTE_PORT_TYPE_OUTPUT,
            )
            og.Controller.create_attribute(
                script_node,
                "outputs:graph_target_name",
                og.Type(og.BaseDataType.TOKEN, 1, 0, og.AttributeRole.NONE),
                og.AttributePortType.ATTRIBUTE_PORT_TYPE_OUTPUT,
            )
            script_node.get_attribute("inputs:step").set(2)

            # Simple script inside AscentNode that increments a counter and writes out
            # the graph target name.
            script_string = "def compute(db):\n"
            script_string += "    db.outputs.counter += db.inputs.step\n"
            script_string += "    db.outputs.graph_target_name = db.abi_context.get_graph_target()\n"

            # Instance the graph 5 times.
            num_prims = 5
            prim_paths = [None] * num_prims
            stage = omni.usd.get_context().get_stage()
            for i in range(num_prims):
                prim_paths[i] = f"/World/Prim_{i}"
                stage.DefinePrim(prim_paths[i])
                OmniGraphSchemaTools.applyOmniGraphAPI(stage, prim_paths[i], graph_path)

            # Set the script attribute on the script node in the instanced graph.
            script_node.get_attribute("inputs:script").set(script_string)

            # Evaluate and validate results.
            await omni.kit.app.get_app().next_update_async()
            await omni.kit.app.get_app().next_update_async()
            await omni.kit.app.get_app().next_update_async()
            for i in range(num_prims):
                if evaluator_name == "dirty_push":
                    self.assertEqual(script_node.get_attribute("outputs:counter").get(instance=i), 2)
                else:
                    self.assertEqual(script_node.get_attribute("outputs:counter").get(instance=i), 6)
                self.assertEqual(script_node.get_attribute("outputs:graph_target_name").get(instance=i), prim_paths[i])

            # Delete the graph and prims.
            for i in range(num_prims):
                stage.RemovePrim(prim_paths[i])
            stage.RemovePrim(graph_path)
            await omni.kit.app.get_app().next_update_async()

    # ----------------------------------------------------------------------
    async def test_compute_creates_dynamic_attrib_legacy(self):
        """Test that the script node can create dynamic attribs within its compute"""
        controller = og.Controller()
        keys = og.Controller.Keys

        script = """
attribute_exists = db.node.get_attribute_exists("inputs:multiplier")
if attribute_exists != True:
    db.node.create_attribute("inputs:multiplier", og.Type(og.BaseDataType.DOUBLE))
db.outputs.data = db.inputs.data * db.inputs.multiplier"""

        (
            graph,
            (on_impulse_node, script_node),
            _,
            _,
        ) = controller.edit(
            {"graph_path": "/TestGraph", "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    ("OnImpulse", "omni.graph.action.OnImpulseEvent"),
                    ("Script", "airlab.pegasus.AscentNode"),
                ],
                keys.CONNECT: ("OnImpulse.outputs:execOut", "Script.inputs:execIn"),
                keys.SET_VALUES: [
                    ("OnImpulse.inputs:onlyPlayback", False),
                    ("Script.inputs:script", script),
                ],
            },
        )

        script_node.create_attribute(
            "inputs:data", og.Type(og.BaseDataType.DOUBLE), og.AttributePortType.ATTRIBUTE_PORT_TYPE_INPUT
        )
        script_node.create_attribute(
            "outputs:data", og.Type(og.BaseDataType.DOUBLE), og.AttributePortType.ATTRIBUTE_PORT_TYPE_OUTPUT
        )
        controller.edit(
            "/TestGraph",
            {
                keys.SET_VALUES: [
                    ("Script.inputs:data", 42),
                    ("Script.outputs:data", 0),
                ],
            },
        )
        await controller.evaluate(graph)

        # trigger graph evaluation once so the compute runs
        og.Controller.set(controller.attribute("state:enableImpulse", on_impulse_node), True)
        await controller.evaluate(graph)
        val = og.Controller.get(controller.attribute("outputs:data", script_node))
        self.assertEqual(val, 0)

        # set value on the dynamic attrib and check compute
        og.Controller.set(controller.attribute("inputs:multiplier", script_node), 2.0)
        og.Controller.set(controller.attribute("state:enableImpulse", on_impulse_node), True)
        await controller.evaluate(graph)
        val = og.Controller.get(controller.attribute("outputs:data", script_node))
        self.assertEqual(val, 84)

    # ----------------------------------------------------------------------
    async def test_use_loaded_dynamic_attrib(self):
        """Test that the script node can use a dynamic attrib loaded from USD"""
        await ogts.load_test_file("TestAscentNode.usda", use_caller_subdirectory=True)

        controller = og.Controller()
        val = og.Controller.get(controller.attribute("outputs:data", "/World/ActionGraph/script_node"))
        self.assertEqual(val, 0)
        # trigger graph evaluation once so the compute runs
        og.Controller.set(controller.attribute("state:enableImpulse", "/World/ActionGraph/on_impulse_event"), True)
        await controller.evaluate()
        val = og.Controller.get(controller.attribute("outputs:data", "/World/ActionGraph/script_node"))
        self.assertEqual(val, 84)
        as_int = og.Controller.get(controller.attribute("outputs:asInt", "/World/ActionGraph/script_node"))
        as_float = og.Controller.get(controller.attribute("state:asFloat", "/World/ActionGraph/script_node"))
        self.assertEqual(as_int, 84)
        self.assertEqual(as_float, 84.0)

    # ----------------------------------------------------------------------
    async def test_simple_scripts_legacy(self):
        """Test that some simple scripts work as intended"""
        controller = og.Controller()
        keys = og.Controller.Keys

        (
            graph,
            (script_node,),
            _,
            _,
        ) = controller.edit(
            "/TestGraph",
            {
                keys.CREATE_NODES: ("Script", "airlab.pegasus.AscentNode"),
            },
        )

        script_node.create_attribute(
            "inputs:my_input_attribute", og.Type(og.BaseDataType.INT), og.AttributePortType.ATTRIBUTE_PORT_TYPE_INPUT
        )
        script_node.create_attribute(
            "outputs:my_output_attribute", og.Type(og.BaseDataType.INT), og.AttributePortType.ATTRIBUTE_PORT_TYPE_OUTPUT
        )
        output_controller = og.Controller(og.Controller.attribute("outputs:my_output_attribute", script_node))

        controller.edit(
            "/TestGraph",
            {
                keys.SET_VALUES: ("Script.inputs:script", "db.outputs.my_output_attribute = 123"),
            },
        )
        await controller.evaluate(graph)
        self.assertEqual(output_controller.get(), 123)

        controller.edit(
            "/TestGraph",
            {
                keys.SET_VALUES: [
                    ("Script.inputs:script", "db.outputs.my_output_attribute = -db.inputs.my_input_attribute"),
                    ("Script.inputs:my_input_attribute", 1234),
                    ("Script.state:omni_initialized", False),
                ]
            },
        )
        await controller.evaluate(graph)
        self.assertEqual(output_controller.get(), -1234)

        controller.edit(
            "/TestGraph",
            {
                keys.SET_VALUES: [
                    (
                        "Script.inputs:script",
                        "db.outputs.my_output_attribute = db.inputs.my_input_attribute * db.inputs.my_input_attribute",
                    ),
                    ("Script.inputs:my_input_attribute", -12),
                    ("Script.state:omni_initialized", False),
                ]
            },
        )
        await controller.evaluate(graph)
        self.assertEqual(output_controller.get(), 144)

    # ----------------------------------------------------------------------
    async def test_internal_state_keeps_persistent_info_legacy(self):
        """Test that the script node can keep persistent information using internal state"""

        script = """
if (not hasattr(db.per_instance_state, 'num1')):
    db.per_instance_state.num1 = 0
    db.per_instance_state.num2 = 1
else:
    sum = db.per_instance_state.num1 + db.per_instance_state.num2
    db.per_instance_state.num1 = db.per_instance_state.num2
    db.per_instance_state.num2 = sum
db.outputs.data = db.per_instance_state.num1"""

        controller = og.Controller()
        keys = og.Controller.Keys

        (
            graph,
            (_, script_node),
            _,
            _,
        ) = controller.edit(
            {"graph_path": "/TestGraph", "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    ("OnTick", "omni.graph.action.OnTick"),
                    ("Script", "airlab.pegasus.AscentNode"),
                ],
                keys.CONNECT: ("OnTick.outputs:tick", "Script.inputs:execIn"),
                keys.SET_VALUES: [
                    ("OnTick.inputs:onlyPlayback", False),
                    ("Script.inputs:script", script),
                ],
            },
        )

        script_node.create_attribute(
            "outputs:data", og.Type(og.BaseDataType.INT), og.AttributePortType.ATTRIBUTE_PORT_TYPE_OUTPUT
        )
        output_controller = og.Controller(og.Controller.attribute("outputs:data", script_node))

        # Check that the script node produces the Fibonacci numbers
        await controller.evaluate(graph)
        self.assertEqual(output_controller.get(), 0)
        await controller.evaluate(graph)
        self.assertEqual(output_controller.get(), 1)
        await controller.evaluate(graph)
        self.assertEqual(output_controller.get(), 1)
        await controller.evaluate(graph)
        self.assertEqual(output_controller.get(), 2)
        await controller.evaluate(graph)
        self.assertEqual(output_controller.get(), 3)
        await controller.evaluate(graph)
        self.assertEqual(output_controller.get(), 5)
        await controller.evaluate(graph)
        self.assertEqual(output_controller.get(), 8)
        await controller.evaluate(graph)
        self.assertEqual(output_controller.get(), 13)
        await controller.evaluate(graph)
        self.assertEqual(output_controller.get(), 21)
        await controller.evaluate(graph)
        self.assertEqual(output_controller.get(), 34)
        await controller.evaluate(graph)
        self.assertEqual(output_controller.get(), 55)

    # ----------------------------------------------------------------------
    async def test_script_global_scope(self):
        """Test that variables, functions, and classes defined outside of the user-defined callbacks are visible"""

        controller = og.Controller()
        keys = og.Controller.Keys

        (
            graph,
            (script_node,),
            _,
            _,
        ) = controller.edit(
            "/TestGraph",
            {
                keys.CREATE_NODES: ("Script", "airlab.pegasus.AscentNode"),
            },
        )

        script_node.create_attribute(
            "outputs:output_a", og.Type(og.BaseDataType.INT), og.AttributePortType.ATTRIBUTE_PORT_TYPE_OUTPUT
        )
        script_node.create_attribute(
            "outputs:output_b", og.Type(og.BaseDataType.TOKEN), og.AttributePortType.ATTRIBUTE_PORT_TYPE_OUTPUT
        )
        output_a_controller = og.Controller(og.Controller.attribute("outputs:output_a", script_node))
        output_b_controller = og.Controller(og.Controller.attribute("outputs:output_b", script_node))

        script_test_constants = """
MY_CONSTANT_A = 123
MY_CONSTANT_B = 'foo'

def setup(db):
    db.outputs.output_a = MY_CONSTANT_A

def compute(db):
    db.outputs.output_b = MY_CONSTANT_B"""

        controller.edit(
            "/TestGraph",
            {
                keys.SET_VALUES: ("Script.inputs:script", script_test_constants),
            },
        )
        await controller.evaluate(graph)
        self.assertEqual(output_a_controller.get(), 123)
        self.assertEqual(output_b_controller.get(), "foo")
        await controller.evaluate(graph)
        self.assertEqual(output_b_controller.get(), "foo")

        script_test_variables = """
my_variable = 123

def setup(db):
    global my_variable
    my_variable = 234
    db.outputs.output_a = my_variable

def compute(db):
    global my_variable
    db.outputs.output_b = f'{my_variable}'
    my_variable += 1"""

        controller.edit(
            "/TestGraph",
            {
                keys.SET_VALUES: [
                    ("Script.inputs:script", script_test_variables),
                    ("Script.state:omni_initialized", False),
                ],
            },
        )
        await controller.evaluate(graph)
        self.assertEqual(output_a_controller.get(), 234)
        self.assertEqual(output_b_controller.get(), "234")
        await controller.evaluate(graph)
        self.assertEqual(output_b_controller.get(), "235")

        script_test_functions = """
def my_function_a():
    return 123

my_variable_b = 'foo'

def my_function_b():
    return my_variable_b

def setup(db):
    db.outputs.output_a = my_function_a()

def compute(db):
    db.outputs.output_b = my_function_b()
    global my_variable_b
    my_variable_b = 'bar'"""

        controller.edit(
            "/TestGraph",
            {
                keys.SET_VALUES: [
                    ("Script.inputs:script", script_test_functions),
                    ("Script.state:omni_initialized", False),
                ]
            },
        )
        await controller.evaluate(graph)
        self.assertEqual(output_a_controller.get(), 123)
        self.assertEqual(output_b_controller.get(), "foo")
        await controller.evaluate(graph)
        self.assertEqual(output_b_controller.get(), "bar")

        script_test_imports = """
import inspect
import math

my_lambda = lambda x: x
code_len = len(inspect.getsource(my_lambda))

def setup(db):
    db.outputs.output_a = code_len

def compute(db):
    db.outputs.output_b = f'{math.pi:.2f}'"""

        controller.edit(
            "/TestGraph",
            {
                keys.SET_VALUES: [
                    ("Script.inputs:script", script_test_imports),
                    ("Script.state:omni_initialized", False),
                ]
            },
        )
        await controller.evaluate(graph)
        self.assertEqual(output_a_controller.get(), 24)
        self.assertEqual(output_b_controller.get(), "3.14")
        await controller.evaluate(graph)
        self.assertEqual(output_b_controller.get(), "3.14")

        script_test_classes = """
class MyClass:
    def __init__(self, value):
        self.value = value

    def get_value(self):
        return self.value

    @staticmethod
    def get_num():
        return 123

my_variable = MyClass('foo')

def setup(db):
    db.outputs.output_a = MyClass.get_num()

def compute(db):
    db.outputs.output_b = my_variable.get_value()
    my_variable.value = 'bar'"""

        controller.edit(
            "/TestGraph",
            {
                keys.SET_VALUES: [
                    ("Script.inputs:script", script_test_classes),
                    ("Script.state:omni_initialized", False),
                ]
            },
        )
        await controller.evaluate(graph)
        self.assertEqual(output_a_controller.get(), 123)
        self.assertEqual(output_b_controller.get(), "foo")
        await controller.evaluate(graph)
        self.assertEqual(output_b_controller.get(), "bar")

    # --------------------------------------------------------------------------------------------------------------
    async def test_script_node_cleanup_failure(self):
        """Check the exception handling when the cleanup fails"""
        controller = og.Controller()
        keys = og.Controller.Keys

        (
            graph,
            (script_node,),
            _,
            _,
        ) = controller.edit(
            "/TestGraph",
            {
                keys.CREATE_NODES: ("Script", "airlab.pegasus.AscentNode"),
            },
        )

        script_test_exception = """
def cleanup(db):
    raise AttributeError

def compute(db):
    pass"""

        controller.edit(
            "/TestGraph",
            {
                keys.SET_VALUES: ("Script.inputs:script", script_test_exception),
            },
        )
        await controller.evaluate(graph)
        controller.set(script_node.get_attribute("state:omni_initialized"), False)
        await controller.evaluate(graph)

    # --------------------------------------------------------------------------------------------------------------
    async def test_script_node_edge_cases(self):
        """Check the parts of the code that are handling rare or uncommon error conditions"""
        controller = og.Controller()
        keys = og.Controller.Keys

        (
            graph,
            (script_node,),
            _,
            _,
        ) = controller.edit(
            "/TestGraph",
            {
                keys.CREATE_NODES: ("Script", "airlab.pegasus.AscentNode"),
            },
        )
        use_path_attr = script_node.get_attribute("inputs:usePath")
        script_path_attr = script_node.get_attribute("inputs:scriptPath")
        controller.set(use_path_attr, True)

        # Test when path script is empty
        await controller.evaluate()

        script_raise = """
def compute(db):
    raise og.OmniGraphError"""

        with TemporaryDirectory() as test_directory:
            tmp_file_path = Path(test_directory) / "sample_ascentnode_script.py"

            # Test when script file not found
            controller.set(script_path_attr, str(tmp_file_path))
            await controller.evaluate(graph)

            # Test when compute raises an exception
            with open(tmp_file_path, "w", encoding="utf-8") as script_fd:
                script_fd.write(script_raise)

            controller.set(script_path_attr, str(tmp_file_path))
            await controller.evaluate(graph)

            # Test when compute script is empty
            with open(tmp_file_path, "w", encoding="utf-8") as script_fd:
                script_fd.write("")
            await controller.evaluate(graph)

    # --------------------------------------------------------------------------------------------------------------
    async def test_script_node_documentation(self):
        '''This test is set up to be code that runs independently as a test itself, and is also formatted so that
        the script node documentation can extract the code for literal inclusion. Using this approach we can
        guarantee that the code we show in our documentation is always functional.

        The example script used is as follows (repeated a few times so that each section of the documentation will have
        a unique instance of it so that it can be self-contained).

        # begin-script-node-script
        """Set the output to True if the first input is larger than the second input"""

        def compute(db: og.Database):
            db.outputs.greater = db.inputs.first_input > db.inputs.second_input
            # You always return True to indicate that the compute succeeded
            return True
        """
        # end-script-node-script
        '''
        # begin-script-node-creation
        # Note the extra parameter to the graph to ensure it is an action graph.
        (_, (script_node,), _, _) = og.Controller.edit(
            {"graph_path": "/TestGraph", "evaluator_name": "execution"},
            {og.Controller.Keys.CREATE_NODES: ("AscentNode", "airlab.pegasus.AscentNode")},
        )
        # After this call "script_node" contains the og.Node that was created in a brand new OmniGraph.
        # end-script-node-creation

        # Helper to add a tick event so that the script node can be tested in operation
        _ = og.Controller.edit(
            "/TestGraph",
            {
                og.Controller.Keys.CREATE_NODES: [("OnTick", "omni.graph.action.OnTick")],
                og.Controller.Keys.SET_VALUES: [("OnTick.inputs:onlyPlayback", False)],
                og.Controller.Keys.CONNECT: [("OnTick.outputs:tick", "/TestGraph/AscentNode.inputs:execIn")],
            },
        )

        # begin-script-node-set-to-file
        script_text = '''
        """Set the output to True if the first input is larger than the second input"""

        def compute(db: og.Database):
            db.outputs.greater = db.inputs.first_input > db.inputs.second_input
            # You always return True to indicate that the compute succeeded
            return True
        '''
        with TemporaryDirectory() as test_directory:
            tmp_file_path = Path(test_directory) / "sample_ascentnode_script.py"
            with open(tmp_file_path, "w", encoding="utf-8") as script_fd:
                script_fd.write(dedent(script_text))
            use_path_attr = script_node.get_attribute("inputs:usePath")
            script_path_attr = script_node.get_attribute("inputs:scriptPath")
            og.Controller.set(use_path_attr, True)
            og.Controller.set(script_path_attr, str(tmp_file_path))
            # end-script-node-set-to-file
            self.assertEqual(og.Controller.get(script_path_attr), str(tmp_file_path))

            # Since there is only one node this attribute addition only needs to happen once. In the docs it will
            # be inserted in multiple locations to ensure that each section is complete unto itself.
            # begin-script-node-add-attributes
            # In the script you have accessed the database variables "db.inputs.first_input", "db.inputs.second_input",
            # and "db.outputs.greater". These all correspond to attributes of the named port type (input or output)
            # that must now be added to the node in order for it to function correctly.
            first_input = og.Controller.create_attribute(script_node, "first_input", "int", og.AttributePortType.INPUT)
            second_input = og.Controller.create_attribute(
                script_node, "second_input", "int", og.AttributePortType.INPUT
            )
            output = og.Controller.create_attribute(script_node, "greater", "bool", og.AttributePortType.OUTPUT)
            # end-script-node-add-attributes

            # Do some basic validity checks to make sure the script node is working correctly.
            self.assertTrue(first_input.is_valid())
            self.assertTrue(second_input.is_valid())
            self.assertTrue(output.is_valid())
            og.Controller.set(first_input, 3)
            og.Controller.set(second_input, 4)
            await og.Controller.evaluate()
            self.assertFalse(og.Controller.get(output))

            og.Controller.set(first_input, 4)
            og.Controller.set(second_input, 3)
            await og.Controller.evaluate()
            self.assertTrue(og.Controller.get(output))

            # For the next test
            og.Controller.set(use_path_attr, False)

        # The text is duplicated because it is included in two different sections of the documentation so in the
        # interests of having everything in one place the complete script is put into both locations.
        # begin-script-node-set-to-script
        script_text = '''
        """Set the output to True if the first input is larger than the second input"""

        def compute(db: og.Database):
            db.outputs.greater = db.inputs.first_input > db.inputs.second_input
            # You always return True to indicate that the compute succeeded
            return True
        '''
        script_attr = script_node.get_attribute("inputs:script")
        og.Controller.set(script_attr, dedent(script_text))
        # end-script-node-set-to-script

        # Do some basic validity checks to make sure the script node is working correctly.
        self.assertEqual(og.Controller.get(script_path_attr), str(tmp_file_path))
        og.Controller.set(first_input, 3)
        og.Controller.set(second_input, 4)
        await og.Controller.evaluate()
        self.assertFalse(og.Controller.get(output))

        og.Controller.set(first_input, 4)
        og.Controller.set(second_input, 3)
        await og.Controller.evaluate()
        self.assertTrue(og.Controller.get(output))
