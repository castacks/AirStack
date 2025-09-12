import inspect
import os
import tempfile
import traceback

import omni.client
import omni.graph.core as og
import omni.graph.tools.ogn as ogn
import omni.usd
from airlab.pegasus.ogn.OgnAscentNodeDatabase import OgnAscentNodeDatabase


# A hacky context manager that captures local variable name declarations and saves them in a dict
class ScriptContextSaver:
    def __init__(self, script_context: dict):
        self.script_context = script_context
        self.local_names = None

    def __enter__(self):
        caller_frame = inspect.currentframe().f_back
        self.local_names = set(caller_frame.f_locals)
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        caller_frame = inspect.currentframe().f_back
        caller_locals = caller_frame.f_locals
        for name in caller_locals:
            if name not in self.local_names:
                self.script_context[name] = caller_locals[name]


class UserCode:
    """The cached data associated with a user script"""

    def __init__(self):
        self.code_object = None  # The compiled code object
        self.setup_fn = None  # setup()
        self.cleanup_fn = None  # cleanup()
        self.compute_fn = None  # compute()
        self.script_context = {}  # namespace for the executed code


class OgnAscentNodeState:
    def __init__(self):
        self.code = UserCode()  # The cached code data
        self.tempfile_path: str = None  # Name of the temporary file for storing the script
        self.script_path: str = None  # The last value of inputs:scriptPath
        self.script: str = None  # The last value of inputs:script
        self.use_path: bool = None  # The last value of inputs:usePath
        self.node_initialized: bool = False  # Flag used to check if the per-instance node state is initialized.


class OgnAscentNode:
    @staticmethod
    def internal_state():
        return OgnAscentNodeState()

    @staticmethod
    def _is_initialized(node: og.Node) -> bool:
        return og.Controller.get(node.get_attribute("state:omni_initialized"))

    @staticmethod
    def _set_initialized(node: og.Node, init: bool):
        return og.Controller.set(node.get_attribute("state:omni_initialized"), init)

    @staticmethod
    def initialize(context, node: og.Node):
        state = OgnAscentNodeDatabase.shared_internal_state(node)
        state.node_initialized = True

        # Create a temporary file for storing the script
        with tempfile.NamedTemporaryFile(suffix=".py", delete=False) as tf:
            state.tempfile_path = tf.name

        OgnAscentNode._set_initialized(node, False)

    @staticmethod
    def release(node: og.Node):
        state = OgnAscentNodeDatabase.shared_internal_state(node)

        # Same logic as when the reset button is pressed
        OgnAscentNode.try_cleanup(node)

        # Delete the temporary file for storing the script
        if state.tempfile_path is not None and os.path.exists(state.tempfile_path):
            os.remove(state.tempfile_path)

    @staticmethod
    def release_instance(node, graph_instance_id):
        """Overrides the release_instance method so that any per-script cleanup can happen before the per-node data
        is deleted.
        """
        # Same logic as when the reset button is pressed
        OgnAscentNode.try_cleanup(node)

    @staticmethod
    def try_cleanup(node: og.Node):
        # Skip if not setup in the fist place or already cleaned up
        if not OgnAscentNode._is_initialized(node):
            return

        state = OgnAscentNodeDatabase.shared_internal_state(node)

        # Call the user-defined cleanup function
        if state.code.cleanup_fn is not None:
            # Get the database object
            per_node_data = OgnAscentNodeDatabase.PER_NODE_DATA[node.node_id()]
            db = per_node_data.get("_db")

            try:
                db.inputs._setting_locked = True  # noqa: PLW0212
                state.code.cleanup_fn(db)
            except Exception:  # pylint: disable=broad-except
                OgnAscentNode._print_stacktrace(db)
            finally:
                db.inputs._setting_locked = False  # noqa: PLW0212
        OgnAscentNode._set_initialized(node, False)

    @staticmethod
    def _print_stacktrace(db: OgnAscentNodeDatabase):
        stacktrace = traceback.format_exc().splitlines(keepends=True)
        stacktrace_iter = iter(stacktrace)
        stacktrace_output = ""

        for stacktrace_line in stacktrace_iter:
            if "OgnAscentNode.py" in stacktrace_line:
                # The stack trace shows that the exception originates from this file
                # Removing this useless information from the stack trace
                next(stacktrace_iter, None)
            else:
                stacktrace_output += stacktrace_line

        db.log_error(stacktrace_output)

    @staticmethod
    def _read_script_file(file_path: str) -> str:
        """Reads the given file and returns the contents"""
        # Get the absolute path from the possibly relative path in inputs:scriptPath with the edit layer
        edit_layer = omni.usd.get_context().get_stage().GetEditTarget().GetLayer()
        if not edit_layer.anonymous:
            file_path = omni.client.combine_urls(edit_layer.realPath, file_path).replace("\\", "/")
        del edit_layer

        # Try to read the script at the specified path
        result, _, content = omni.client.read_file(file_path)

        if result != omni.client.Result.OK:
            raise RuntimeError(f"Could not open/read the script at '{file_path}': error code: {result}")

        script_bytes = memoryview(content).tobytes()
        if len(script_bytes) < 2:
            return ""
        cur_script = script_bytes.decode("utf-8")
        return cur_script

    @staticmethod
    def _legacy_compute(db: OgnAscentNodeDatabase):
        # Legacy compute we just exec the whole script every compute
        with ScriptContextSaver(db.shared_state.code.script_context):
            exec(db.shared_state.code.code_object)  # noqa: PLW0122

    @staticmethod
    def compute(db) -> bool:
        #print('compute 2')
        return True
        # Note that we initialize this node's OgnAscentNodeState in the OgnAscentNode initialize
        # method. While this works for non-instanced workflows, if we try to instance an OmniGraph
        # that contains a AscentNode we run into issues, mainly because the instanced AscentNode
        # will NOT have an initialized OgnAscentNodeState (since the instanced node's initialize()
        # method was never actually executed). To account for this, in the compute method we'll
        # simply call the OgnAscentNode initialize() if said method was never called.
        if not db.shared_state.node_initialized:
            OgnAscentNode.initialize(db.abi_context, db.abi_node)

        use_path = db.inputs.usePath
        cur_script: str = ""  # The script contents
        tempfile_path = db.shared_state.tempfile_path  # The path to the script file to be compiled/executed
        initialized = db.state.omni_initialized
        if use_path:
            script_path = db.inputs.scriptPath
            if not script_path:
                return True
        else:
            # Use inputs:script for the script and the temporary file for the script path
            cur_script = db.inputs.script
            if not cur_script:
                return True
        if use_path != db.shared_state.use_path:
            initialized = False
            db.shared_state.use_path = use_path
        try:
            # Compile / Execute the script if necessary
            if not initialized:
                db.state.omni_initialized = True
                db.shared_state.code = UserCode()
                db.shared_state.script = None
                try:
                    if use_path:
                        cur_script = OgnAscentNode._read_script_file(script_path)
                        db.shared_state.script_path = script_path
                    # If the script content has changed we need to re-compile
                    if db.shared_state.script != cur_script:
                        with open(tempfile_path, "w", encoding="utf-8") as tf:
                            tf.write(cur_script)
                        db.shared_state.code.code_object = compile(cur_script, tempfile_path, "exec")
                        db.shared_state.script = cur_script
                except Exception as ex:  # pylint: disable=broad-except
                    # No need for a callstack for an i/o or compilation error
                    db.log_error(str(ex))
                    return False
                # Execute the script inside a context manager that captures the names defined in it
                with ScriptContextSaver(db.shared_state.code.script_context):
                    exec(db.shared_state.code.code_object)  # noqa: PLW0122

                # Extract the user-defined setup, compute, and cleanup functions
                db.shared_state.code.compute_fn = db.shared_state.code.script_context.get("compute")
                if not callable(db.shared_state.code.compute_fn):
                    db.shared_state.code.compute_fn = None

                if db.shared_state.code.compute_fn is None:
                    # Assume the script is legacy, so execute on every compute
                    db.log_warning("compute(db) not defined in user script, running in legacy mode")
                    db.shared_state.code.compute_fn = OgnAscentNode._legacy_compute
                    return True

                db.shared_state.code.setup_fn = db.shared_state.code.script_context.get("setup")
                if not callable(db.shared_state.code.setup_fn):
                    db.shared_state.code.setup_fn = None

                db.shared_state.code.cleanup_fn = db.shared_state.code.script_context.get("cleanup")
                if not callable(db.shared_state.code.cleanup_fn):
                    db.shared_state.code.cleanup_fn = None

                # Inject script-global names into the function globals
                if db.shared_state.code.compute_fn is not None:
                    db.shared_state.code.compute_fn.__globals__.update(db.shared_state.code.script_context)

                if db.shared_state.code.setup_fn is not None:
                    db.shared_state.code.setup_fn.__globals__.update(db.shared_state.code.script_context)

                if db.shared_state.code.cleanup_fn is not None:
                    db.shared_state.code.cleanup_fn.__globals__.update(db.shared_state.code.script_context)

                # Call the user-defined setup function
                if db.shared_state.code.setup_fn is not None:
                    db.shared_state.code.setup_fn(db)

            # ------------------------------------------------------------------------------------
            # Call the user-defined compute function
            if db.shared_state.code.compute_fn is not None:
                db.shared_state.code.compute_fn(db)

            # Set outputs:execOut if not hidden
            if db.node.get_attribute("outputs:execOut").get_metadata(ogn.MetadataKeys.HIDDEN) != "1":
                db.outputs.execOut = og.ExecutionAttributeState.ENABLED
        except Exception:  # pylint: disable=broad-except
            OgnAscentNode._print_stacktrace(db)
            return False
        return True
