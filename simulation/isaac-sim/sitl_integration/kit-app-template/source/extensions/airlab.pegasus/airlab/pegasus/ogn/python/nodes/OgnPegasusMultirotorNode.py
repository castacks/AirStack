import inspect
import os
import traceback

import omni.client
import omni.graph.core as og
import omni.graph.tools.ogn as ogn
import omni.usd
from airlab.pegasus.ogn.OgnPegasusMultirotorNodeDatabase import OgnPegasusMultirotorNodeDatabase


class OgnPegasusMultirotorNodeState:
    def __init__(self):
        self.node_initialized: bool = False  # Flag used to check if the per-instance node state is initialized.

class OgnPegasusMultirotorNode:
    @staticmethod
    def internal_state():
        return OgnPegasusMultirotorNodeState()

    @staticmethod
    def _is_initialized(node: og.Node) -> bool:
        return og.Controller.get(node.get_attribute("state:omni_initialized"))

    @staticmethod
    def _set_initialized(node: og.Node, init: bool):
        return og.Controller.set(node.get_attribute("state:omni_initialized"), init)

    @staticmethod
    def initialize(context, node: og.Node):
        state = OgnPegasusMultirotorNodeDatabase.shared_internal_state(node)
        state.node_initialized = True

        OgnPegasusMultirotorNode._set_initialized(node, False)

    @staticmethod
    def release(node: og.Node):
        state = OgnPegasusMultirotorNodeDatabase.shared_internal_state(node)

        # Same logic as when the reset button is pressed
        OgnPegasusMultirotorNode.try_cleanup(node)

    @staticmethod
    def release_instance(node, graph_instance_id):
        """Overrides the release_instance method so that any per-script cleanup can happen before the per-node data
        is deleted.
        """
        # Same logic as when the reset button is pressed
        OgnPegasusMultirotorNode.try_cleanup(node)

    @staticmethod
    def try_cleanup(node: og.Node):
        # Skip if not setup in the fist place or already cleaned up
        if not OgnPegasusMultirotorNode._is_initialized(node):
            return

        state = OgnPegasusMultirotorNodeDatabase.shared_internal_state(node)

        # Call the user-defined cleanup function
        if state.code.cleanup_fn is not None:
            # Get the database object
            per_node_data = OgnPegasusMultirotorNodeDatabase.PER_NODE_DATA[node.node_id()]
            db = per_node_data.get("_db")

            try:
                db.inputs._setting_locked = True  # noqa: PLW0212
                state.code.cleanup_fn(db)
            except Exception:  # pylint: disable=broad-except
                OgnPegasusMultirotorNode._print_stacktrace(db)
            finally:
                db.inputs._setting_locked = False  # noqa: PLW0212
        OgnPegasusMultirotorNode._set_initialized(node, False)

    @staticmethod
    def _print_stacktrace(db: OgnPegasusMultirotorNodeDatabase):
        stacktrace = traceback.format_exc().splitlines(keepends=True)
        stacktrace_iter = iter(stacktrace)
        stacktrace_output = ""

        for stacktrace_line in stacktrace_iter:
            if "OgnPegasusMultirotorNode.py" in stacktrace_line:
                # The stack trace shows that the exception originates from this file
                # Removing this useless information from the stack trace
                next(stacktrace_iter, None)
            else:
                stacktrace_output += stacktrace_line

        db.log_error(stacktrace_output)

    @staticmethod
    def compute(db) -> bool:
        #print('compute 2')
        return True
        # Note that we initialize this node's OgnPegasusMultirotorNodeState in the OgnPegasusMultirotorNode initialize
        # method. While this works for non-instanced workflows, if we try to instance an OmniGraph
        # that contains a PegasusMultirotorNode we run into issues, mainly because the instanced PegasusMultirotorNode
        # will NOT have an initialized OgnPegasusMultirotorNodeState (since the instanced node's initialize()
        # method was never actually executed). To account for this, in the compute method we'll
        # simply call the OgnPegasusMultirotorNode initialize() if said method was never called.
        # if not db.shared_state.node_initialized:
        #     OgnPegasusMultirotorNode.initialize(db.abi_context, db.abi_node)

        # use_path = db.inputs.usePath
        # cur_script: str = ""  # The script contents
        # tempfile_path = db.shared_state.tempfile_path  # The path to the script file to be compiled/executed
        # initialized = db.state.omni_initialized
        # if use_path:
        #     script_path = db.inputs.scriptPath
        #     if not script_path:
        #         return True
        # else:
        #     # Use inputs:script for the script and the temporary file for the script path
        #     cur_script = db.inputs.script
        #     if not cur_script:
        #         return True
        # if use_path != db.shared_state.use_path:
        #     initialized = False
        #     db.shared_state.use_path = use_path
        # try:
        #     # Compile / Execute the script if necessary
        #     if not initialized:
        #         db.state.omni_initialized = True
        #         db.shared_state.code = UserCode()
        #         db.shared_state.script = None
        #         try:
        #             if use_path:
        #                 cur_script = OgnPegasusMultirotorNode._read_script_file(script_path)
        #                 db.shared_state.script_path = script_path
        #             # If the script content has changed we need to re-compile
        #             if db.shared_state.script != cur_script:
        #                 with open(tempfile_path, "w", encoding="utf-8") as tf:
        #                     tf.write(cur_script)
        #                 db.shared_state.code.code_object = compile(cur_script, tempfile_path, "exec")
        #                 db.shared_state.script = cur_script
        #         except Exception as ex:  # pylint: disable=broad-except
        #             # No need for a callstack for an i/o or compilation error
        #             db.log_error(str(ex))
        #             return False
        #         # Execute the script inside a context manager that captures the names defined in it
        #         with ScriptContextSaver(db.shared_state.code.script_context):
        #             exec(db.shared_state.code.code_object)  # noqa: PLW0122

        #         # Extract the user-defined setup, compute, and cleanup functions
        #         db.shared_state.code.compute_fn = db.shared_state.code.script_context.get("compute")
        #         if not callable(db.shared_state.code.compute_fn):
        #             db.shared_state.code.compute_fn = None

        #         if db.shared_state.code.compute_fn is None:
        #             # Assume the script is legacy, so execute on every compute
        #             db.log_warning("compute(db) not defined in user script, running in legacy mode")
        #             db.shared_state.code.compute_fn = OgnPegasusMultirotorNode._legacy_compute
        #             return True

        #         db.shared_state.code.setup_fn = db.shared_state.code.script_context.get("setup")
        #         if not callable(db.shared_state.code.setup_fn):
        #             db.shared_state.code.setup_fn = None

        #         db.shared_state.code.cleanup_fn = db.shared_state.code.script_context.get("cleanup")
        #         if not callable(db.shared_state.code.cleanup_fn):
        #             db.shared_state.code.cleanup_fn = None

        #         # Inject script-global names into the function globals
        #         if db.shared_state.code.compute_fn is not None:
        #             db.shared_state.code.compute_fn.__globals__.update(db.shared_state.code.script_context)

        #         if db.shared_state.code.setup_fn is not None:
        #             db.shared_state.code.setup_fn.__globals__.update(db.shared_state.code.script_context)

        #         if db.shared_state.code.cleanup_fn is not None:
        #             db.shared_state.code.cleanup_fn.__globals__.update(db.shared_state.code.script_context)

        #         # Call the user-defined setup function
        #         if db.shared_state.code.setup_fn is not None:
        #             db.shared_state.code.setup_fn(db)

        #     # ------------------------------------------------------------------------------------
        #     # Call the user-defined compute function
        #     if db.shared_state.code.compute_fn is not None:
        #         db.shared_state.code.compute_fn(db)

        #     # Set outputs:execOut if not hidden
        #     if db.node.get_attribute("outputs:execOut").get_metadata(ogn.MetadataKeys.HIDDEN) != "1":
        #         db.outputs.execOut = og.ExecutionAttributeState.ENABLED
        # except Exception:  # pylint: disable=broad-except
        #     OgnPegasusMultirotorNode._print_stacktrace(db)
        #     return False
        # return True
