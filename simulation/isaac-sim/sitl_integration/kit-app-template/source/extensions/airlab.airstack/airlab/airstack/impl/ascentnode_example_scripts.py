# This file contains the set of example scripts for the script node.
# User can click on the Code Snippets button in the UI to display these scripts.
# To add new example scripts to the script node,
# simply add the delimiter to the bottom of this file, followed by the new script.

# Declare og to suppress linter warnings about undefined variables
og = None


# # # DELIMITER # # #
Title = "Default Script"
# This script is executed the first time the script node computes, or the next time
# it computes after this script is modified or the 'Reset' button is pressed.
#
# The following callback functions may be defined in this script:
#     setup(db): Called immediately after this script is executed
#     compute(db): Called every time the node computes (should always be defined)
#     cleanup(db): Called when the node is deleted or the reset button is pressed
# Available variables:
#    db: og.Database The node interface - attributes are exposed in a namespace like db.inputs.foo and db.outputs.bar.
#                    Use db.log_error, db.log_warning to report problems in the compute function.
#    og: The omni.graph.core module


def setup(db: og.Database):
    pass


def cleanup(db: og.Database):
    pass


def compute(db: og.Database):
    return True


# # # DELIMITER # # #
Title = "Compute Count"


# In this example, we retrieve the number of times this script node has been computed
# and assign it to Output Data so that downstream nodes can use this information.
# Add these attributes first:
#   outputs:my_output_attribute(int) Number of times the compute function was executed.
def compute(db):
    compute_count = db.node.get_compute_count()
    db.outputs.my_output_attribute = compute_count


# # # DELIMITER # # #
Title = "Fibonacci"
# In this example, we produce the Fibonacci sequence 0, 1, 1, 2, 3, 5, 8, 13, 21...
# Each time this node is evaluated, the next Fibonacci number will be set as the output value.
# This illustrates how variables declared in the setup script can be used to keep persistent information.
# Add these attributes first:
#   outputs:my_output_attribute(int) The current number in the Fibonacci sequence.
#   state:last(int) The most recent Fibonacci number.
#   state:previous(int) The second-most recent Fibonacci number.


def compute(db):
    # Bootstrap the first call
    if db.state.previous == 0:
        db.state.last = 0
        db.state.previous = 1
    total = db.state.last + db.state.previous
    db.state.last = db.state.previous
    db.state.previous = total

    db.outputs.my_output_attribute = db.state.last


# # # DELIMITER # # #
Title = "Controller"
# In this example, we use omni.graph.core.Controller to create cube prims.
# Each time this node is evaluated, it will create a new cube prim on the scene.
# When the 'Reset' button is pressed or the node is deleted, the created cube prims will be deleted.

import omni.kit.commands


def setup(db):
    state = db.per_instance_state
    state.cube_count = 0


def compute(db):
    state = db.per_instance_state
    state.cube_count += 1
    og.Controller.edit(
        db.node.get_graph(), {og.Controller.Keys.CREATE_PRIMS: [(f"/World/Cube{state.cube_count}", "Cube")]}
    )


def cleanup(db):
    import omni.usd
    from pxr import Usd

    for prim in Usd.PrimRange(omni.usd.get_context().get_stage().GetPrimAtPath("/World")):
        print(f"Deleting prim {prim.GetPrimPath()}", flush=True)

    state = db.per_instance_state
    omni.kit.commands.execute("DeletePrims", paths=[f"/World/Cube{i}" for i in range(1, state.cube_count + 1)])


# # # DELIMITER # # #
Title = "Sine Deformer With Warp"
# In this example, we deform input points using a Warp kernel, varying the deformation based on a time sequence
# Add these attributes first:
#   inputs:points(pointf[3][]) Points to be deformed
#   inputs:time(float) Point in time at which the deformation is to be computed
#   outputs:points(pointf[3][]) Deformed positions of the points at the given time

import omni.warp
import warp as wp


@wp.kernel
def deform(points_in: wp.array(dtype=wp.vec3), points_out: wp.array(dtype=wp.vec3), time: float):
    tid = wp.tid()
    points_out[tid] = points_in[tid] + wp.vec3(0.0, wp.sin(time + points_in[tid][0] * 0.1) * 10.0, 0.0)


def compute(db):
    # Indicate that inputs:points and outputs:points should be stored in cuda memory.
    # outputs:points can be connected to a similarly configured input to avoid
    # that attribute being copied from the device
    db.set_dynamic_attribute_memory_location(
        on_gpu=True,
        gpu_ptr_kind=og.PtrToPtrKind.CPU,
    )
    with wp.ScopedDevice(f"cuda:{og.get_compute_cuda_device()}"):
        points_in = omni.warp.from_omni_graph(db.inputs.points, dtype=wp.vec3)
        n = db.inputs.points.shape[0]
        if not n:
            return
        out_points = wp.zeros_like(points_in)
        # launch kernel
        wp.launch(kernel=deform, dim=n, inputs=[points_in, out_points, db.inputs.time])

        # allocate output array
        db.outputs.points_size = n
        # copy points
        points_out = omni.warp.from_omni_graph(db.outputs.points, dtype=wp.vec3)
        wp.copy(points_out, out_points)


# # # DELIMITER # # #
Title = "Value Changed Callbacks"
# In this example, we register a value changed callback function for inputs:my_input_attribute.
# The callback is called when the value of inputs:my_input_attribute is changed from the property panel.
# Add these attributes first:
#   inputs:my_input_attribute(int) The attribute triggering the callback.


def on_my_input_attribute_changed(attr):
    print(f"inputs:my_input_attribute = {attr.get_attribute_data().get()}", flush=True)


def setup(db):
    print("Setting up the value changed callback", flush=True)
    attr = db.node.get_attribute("inputs:my_input_attribute")
    attr.register_value_changed_callback(on_my_input_attribute_changed)


def cleanup(db):
    print("Removing the value changed callback", flush=True)
    attr = db.node.get_attribute("inputs:my_input_attribute")
    attr.register_value_changed_callback(None)


def compute(db):
    pass


# # # DELIMITER # # #
Title = "Compute Timer"
# The setup() and cleanup() functions can be used to manage resources required by the node that correspond to the
# lifetime of the node. The management of state attributes is something that would commonly be done at these times
# since such attributes persist for the life of the node and remember their values throughout.
#
# Here is an example of how you might add state information and use the `setup()` and `cleanup()` functions to create an
# accumulated execution time for a node.
#
# For this example you will not create an Action Graph. Instead, create a `Generic Graph` and add a script node to it.
# Copy and paste the snippet below into your script node's script field. No attributes are required for this example.
#
# Since the node is in a push graph it will evaluate continuously. You can periodically hit the ``Reset`` button and it
# will report the total amount of time it spent in the `compute()` function since the last time you reset it.
#
# You can see how this simple mechanism could generalize to include per-compute averages, min and max times, etc.
# If you wish to accumulate all of the data you could even add a file descriptor to your internal state information,
# opening the file in the `setup()` function, printing to it in the `compute()` function, and closing it in the
#  `cleanup()` function to create a journal of every computation time. -->


import time


def setup(db: og.Database):
    """Initialize the timing information to start at 0.0"""
    db.per_instance_state.elapsed = 0.0
    print("Initialize the elapsed time", flush=True)


def cleanup(db: og.Database):
    """Report the final timing information"""
    print(f"Total time spent in the script compute = {db.per_instance_state.elapsed}", flush=True)


def compute(db: og.Database):
    """Accumulate the timing information for each compute.
    For the `compute()` a simple delay will serve to illustrate how the time accumulates as the node executes. As with any
    other node, the database has a member called `per_instance_state` in which you can store any data that belongs to the node
    which will persist for the lifetime of that node.
    """

    start_time = time.perf_counter()
    time.sleep(0.1)  # Normally this would be your actual computation
    end_time = time.perf_counter()

    db.per_instance_state.elapsed = db.per_instance_state.elapsed + (end_time - start_time)

    return True
