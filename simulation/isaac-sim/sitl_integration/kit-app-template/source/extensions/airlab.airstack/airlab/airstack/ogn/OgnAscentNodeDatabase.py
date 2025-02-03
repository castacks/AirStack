"""Support for simplified access to data on nodes of type airlab.airstack.AscentNode

 __   ___ .  .  ___  __       ___  ___  __      __   __   __   ___
/ _` |__  |\ | |__  |__)  /\   |  |__  |  \    /  ` /  \ |  \ |__
\__| |___ | \| |___ |  \ /--\  |  |___ |__/    \__, \__/ |__/ |___

 __   __     .  .  __  ___     .  .  __   __     ___
|  \ /  \    |\ | /  \  |      |\/| /  \ |  \ | |__  \ /
|__/ \__/    | \| \__/  |      |  | \__/ |__/ | |     |

This script node allows you to execute arbitrary Python code inside an OmniGraph.
The compute function is defined by you
at runtime and can be unique in every instance of a script node
that you create. Inside that function you can access the
database for the node, which is used for
getting and setting all attribute values that you define.

"""

import carb
import sys
import traceback

import omni.graph.core as og
import omni.graph.core._omni_graph_core as _og
import omni.graph.tools.ogn as ogn

import omni.ext
import omni.ui as ui
import omni.kit.commands

from omni.isaac.core.utils.stage import get_current_stage
from omni.isaac.core.utils.prims import define_prim, get_prim_at_path

import sys
import dronekit
import os
import numpy as np
from scipy.spatial.transform import Rotation
import carb
import omni
import omni.graph.core as og
import omni.replicator.core as rep
import omni.timeline
import usdrt.Sdf
from omni.isaac.core.prims import GeometryPrim, RigidPrim
from omni.isaac.core.utils import extensions, stage
from omni.isaac.core.world import World
from pxr import Gf, Usd, UsdGeom
from .ascent_sitl_launch_tool import AscentSitlLaunchTool
import time
import threading
import struct
import socket

import asyncio

drone_sim_dict = {}
initialized_timeline_callback = False


def timeline_callback(event):
    global drone_sim_dict
    print("ascent node timeline callback", event, event.type, dir(event))

    if event.type == int(omni.timeline.TimelineEventType.PLAY):
        pass
    elif event.type == int(omni.timeline.TimelineEventType.PAUSE):
        pass
    elif event.type == int(omni.timeline.TimelineEventType.STOP):
        AscentSitlLaunchTool.kill_all_ascent_sessions()

        # reset orientation and position of the drone
        for k, v in drone_sim_dict.items():
            prim = v["prim"]
            if prim.GetAttribute("xformOp:orient").Get() is not None:
                prim.GetAttribute("xformOp:orient").Set(Gf.Quatf(1, 0, 0, 0))
            prim.GetAttribute("xformOp:translate").Set((0, 0, 0))

        drone_sim_dict = {}


def incremental_rotate(prim):
    rot = Gf.Quatf(0.9702957272529602, 0, 0.2419208437204361, 0)  # 7 degrees
    o = prim.GetAttribute("xformOp:orient")
    if o.Get() != None:
        o.Set(o.Get() * rot)
    else:
        o = prim.GetAttribute("xformOp:rotateXYZ")
        if o.Get() != None:
            o.Set(o.Get() + Gf.Vec3d(0.0, 27, 0.0))


class OgnAscentNodeDatabase(og.Database):
    """Helper class providing simplified access to data on nodes of type airlab.airstack.AscentNode

    Class Members:
        node: Node being evaluated

    Attribute Value Properties:
        Inputs:
            inputs.execIn
            inputs.script
            inputs.scriptPath
            inputs.usePath
        Outputs:
            outputs.execOut
        State:
            state.omni_initialized
    """

    # Imprint the generator and target ABI versions in the file for JIT generation
    GENERATOR_VERSION = (1, 76, 0)
    TARGET_VERSION = (2, 170, 0)

    # This is an internal object that provides per-class storage of a per-node data dictionary
    PER_NODE_DATA = {}

    # This is an internal object that describes unchanging attributes in a generic way
    # The values in this list are in no particular order, as a per-attribute tuple
    #     Name, Type, ExtendedTypeIndex, UiName, Description, Metadata,
    #     Is_Required, DefaultValue, Is_Deprecated, DeprecationMsg
    # You should not need to access any of this data directly, use the defined database interfaces
    INTERFACE = og.Database._get_interface(
        [
            (
                "inputs:execIn",
                "execution",
                0,
                None,
                "Signal to the graph that this node is ready to be executed.",
                {},
                True,
                None,
                False,
                "",
            ),
            # (
            #     "inputs:deltaSimulationTime",
            #     "double",
            #     0,
            #     "Simulation Delta Time",
            #     "Description.",
            #     {},
            #     True,
            #     0.0,
            #     False,
            #     "",
            # ),
            # (
            #     "inputs:deltaSystemTime",
            #     "double",
            #     0,
            #     "System Delta Time",
            #     "Description.",
            #     {},
            #     True,
            #     0.0,
            #     False,
            #     "",
            # ),
            (
                "inputs:dronePrim",
                "target",
                0,
                "Drone Prim",
                "Description.",
                {},
                True,
                None,
                False,
                "",
            ),
            (
                "outputs:execOut",
                "execution",
                0,
                None,
                "Signal to the graph that execution can continue downstream.",
                {},
                True,
                None,
                False,
                "",
            ),
            (
                "inputs:domain_id",
                "uchar",
                0,
                "Domain ID",
                "Description.",
                {},
                True,
                0,
                False,
                "",
            ),
            (
                "inputs:nodeNamespace",
                "string",
                0,
                "Node Namespace",
                "Description.",
                {},
                True,
                None,
                False,
                "",
            ),
            (
                "state:omni_initialized",
                "bool",
                0,
                None,
                "State attribute used to control when the script should be reloaded.\nThis should be set to false to trigger a reload of the script.",
                {},
                True,
                None,
                False,
                "",
            ),
        ]
    )

    @classmethod
    def _populate_role_data(cls):
        """Populate a role structure with the non-default roles on this node type"""
        role_data = super()._populate_role_data()
        role_data.inputs.execIn = og.AttributeRole.EXECUTION
        # role_data.inputs.script = og.AttributeRole.TEXT
        role_data.outputs.execOut = og.AttributeRole.EXECUTION
        return role_data

    class ValuesForInputs(og.DynamicAttributeAccess):
        LOCAL_PROPERTY_NAMES = {
            "execIn",
            #"deltaSimulationTime",
            #"deltaSystemTime",
            "dronePrim",
            "domain_id",
            "nodeNamespace",
            "_setting_locked",
            "_batchedReadAttributes",
            "_batchedReadValues",
        }
        """Helper class that creates natural hierarchical access to input attributes"""

        def __init__(
            self,
            node: og.Node,
            attributes,
            dynamic_attributes: og.DynamicAttributeInterface,
        ):
            """Initialize simplified access for the attribute data"""
            print("init 1")
            context = node.get_graph().get_default_graph_context()
            print("init 2")
            super().__init__(context, node, attributes, dynamic_attributes)
            print("init 3")
            self._batchedReadAttributes = [
                self._attributes.execIn,
                #self._attributes.deltaSimulationTime,
                #self._attributes.deltaSystemTime,
                self._attributes.dronePrim,
                self._attributes.domain_id,
                self._attributes.nodeNamespace,
            ]
            print("init 4")
            self._batchedReadValues = [None, None, None, False]
            print("init 5")

        @property
        def execIn(self):
            print("execIn 1")
            return self._batchedReadValues[0]

        @execIn.setter
        def execIn(self, value):
            print("execIn 2")
            self._batchedReadValues[0] = value
        '''
        @property
        def deltaSimulationTime(self):
            return self._batchedReadValues[1]

        @deltaSimulationTime.setter
        def deltaSimulationTime(self, value):
            self._batchedReadValues[1] = value

        @property
        def deltaSystemTime(self):
            return self._batchedReadValues[2]

        @deltaSystemTime.setter
        def deltaSystemTime(self, value):
            self._batchedReadValues[2] = value
        '''
        @property
        def dronePrim(self):
            return self._batchedReadValues[1]

        @dronePrim.setter
        def dronePrim(self, value):
            self._batchedReadValues[1] = value

        @property
        def domain_id(self):
            return self._batchedReadValues[2]

        @domain_id.setter
        def domain_id(self, value):
            self._batchedReadValues[2] = value

        @property
        def nodeNamespace(self):
            return self._batchedReadValues[3]

        @nodeNamespace.setter
        def nodeNamespace(self, value):
            self._batchedReadValues[3] = value

        def __getattr__(self, item: str):
            if item in self.LOCAL_PROPERTY_NAMES:
                return object.__getattribute__(self, item)
            else:
                return super().__getattr__(item)

        def __setattr__(self, item: str, new_value):
            if item in self.LOCAL_PROPERTY_NAMES:
                object.__setattr__(self, item, new_value)
            else:
                super().__setattr__(item, new_value)

        def _prefetch(self):
            readAttributes = self._batchedReadAttributes
            newValues = _og._prefetch_input_attributes_data(readAttributes)
            if len(readAttributes) == len(newValues):
                self._batchedReadValues = newValues

    class ValuesForOutputs(og.DynamicAttributeAccess):
        LOCAL_PROPERTY_NAMES = {"execOut", "_batchedWriteValues"}
        """Helper class that creates natural hierarchical access to output attributes"""

        def __init__(
            self,
            node: og.Node,
            attributes,
            dynamic_attributes: og.DynamicAttributeInterface,
        ):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)
            self._batchedWriteValues = {}

        @property
        def execOut(self):
            value = self._batchedWriteValues.get(self._attributes.execOut)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.execOut)
                return data_view.get()

        @execOut.setter
        def execOut(self, value):
            self._batchedWriteValues[self._attributes.execOut] = value

        def __getattr__(self, item: str):
            if item in self.LOCAL_PROPERTY_NAMES:
                return object.__getattribute__(self, item)
            else:
                return super().__getattr__(item)

        def __setattr__(self, item: str, new_value):
            if item in self.LOCAL_PROPERTY_NAMES:
                object.__setattr__(self, item, new_value)
            else:
                super().__setattr__(item, new_value)

        def _commit(self):
            _og._commit_output_attributes_data(self._batchedWriteValues)
            self._batchedWriteValues = {}

    class ValuesForState(og.DynamicAttributeAccess):
        """Helper class that creates natural hierarchical access to state attributes"""

        def __init__(
            self,
            node: og.Node,
            attributes,
            dynamic_attributes: og.DynamicAttributeInterface,
        ):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)

        @property
        def omni_initialized(self):
            data_view = og.AttributeValueHelper(self._attributes.omni_initialized)
            return data_view.get()

        @omni_initialized.setter
        def omni_initialized(self, value):
            data_view = og.AttributeValueHelper(self._attributes.omni_initialized)
            data_view.set(value)

    def __init__(self, node):
        super().__init__(node)
        dynamic_attributes = self.dynamic_attribute_data(
            node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_INPUT
        )
        self.inputs = OgnAscentNodeDatabase.ValuesForInputs(
            node, self.attributes.inputs, dynamic_attributes
        )
        dynamic_attributes = self.dynamic_attribute_data(
            node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_OUTPUT
        )
        self.outputs = OgnAscentNodeDatabase.ValuesForOutputs(
            node, self.attributes.outputs, dynamic_attributes
        )
        dynamic_attributes = self.dynamic_attribute_data(
            node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_STATE
        )
        self.state = OgnAscentNodeDatabase.ValuesForState(
            node, self.attributes.state, dynamic_attributes
        )

        # init timeline callback
        global initialized_timeline_callback
        if not initialized_timeline_callback:
            initialized_timeline_callback = True
            self.timeline = omni.timeline.get_timeline_interface()
            self.play_listener = self.timeline.get_timeline_event_stream().create_subscription_to_pop_by_type(
                int(omni.timeline.TimelineEventType.PLAY), timeline_callback
            )
            self.pause_listener = self.timeline.get_timeline_event_stream().create_subscription_to_pop_by_type(
                int(omni.timeline.TimelineEventType.PAUSE), timeline_callback
            )
            self.stop_listener = self.timeline.get_timeline_event_stream().create_subscription_to_pop_by_type(
                int(omni.timeline.TimelineEventType.STOP), timeline_callback
            )

    class abi:
        """Class defining the ABI interface for the node type"""

        @staticmethod
        def get_node_type():
            get_node_type_function = getattr(
                OgnAscentNodeDatabase.NODE_TYPE_CLASS, "get_node_type", None
            )
            if callable(get_node_type_function):  # pragma: no cover
                return get_node_type_function()
            return "airlab.airstack.AscentNode"

        @staticmethod
        def compute(context, node):
            #print('compute 1')
            def database_valid():
                return True

            try:
                #print('1')
                per_node_data = OgnAscentNodeDatabase.PER_NODE_DATA[node.node_id()]
                #print('2')
                db = per_node_data.get("_db")
                #print('3')
                if db is None:
                    #print('4')
                    db = OgnAscentNodeDatabase(node)
                    #print('5')
                    per_node_data["_db"] = db
                    #print('6')
                if not database_valid():
                    #print('7')
                    per_node_data["_db"] = None
                    #print('8')
                    return False
            except Exception as e:
                #print('9', e)
                #traceback.print_exc()
                db = OgnAscentNodeDatabase(node)
            # print('input test', dir(db.inputs))
            # print('delta sim time', db.inputs.deltaSimulationTime)
            # print('prim', db.inputs.dronePrim, type(db.inputs.dronePrim))

            try:
                if db.inputs.dronePrim:
                    global drone_sim_dict
                    node_id = node.node_id()

                    if node_id not in drone_sim_dict.keys():
                        world = World()
                        print("INPUT", db.inputs.domain_id, db.inputs.nodeNamespace)
                        drone_sim_dict[node_id] = {
                            "initialized": False,
                            "prim": world.stage.GetPrimAtPath(
                                str(db.inputs.dronePrim[0])
                            ),
                            "prop": world.stage.GetPrimAtPath(
                                str(db.inputs.dronePrim[0])
                                + "/base_link/meshes/mesh_17"
                            ),
                            "sitl_tool": AscentSitlLaunchTool(
                                "/sitl_integration/drag_and_drop/",
                                int(db.inputs.domain_id),
                                db.inputs.domain_id,
                                db.inputs.nodeNamespace,
                            ),
                            "dronekit_connection": None,
                        }
                    initialized = drone_sim_dict[node_id]["initialized"]
                    prim = drone_sim_dict[node_id]["prim"]
                    prop = drone_sim_dict[node_id]["prop"]
                    sitl_tool = drone_sim_dict[node_id]["sitl_tool"]
                    dronekit_connection = drone_sim_dict[node_id]["dronekit_connection"]

                    if dronekit_connection == None:
                        if not initialized:
                            drone_sim_dict[node_id]["initialized"] = True
                            sitl_tool.launch()

                            def f():
                                drone_sim_dict[node_id]["dronekit_connection"] = (
                                    dronekit.connect(
                                        sitl_tool.get_dronekit_address(),
                                        wait_ready=True,
                                        timeout=999999,
                                        rate=120,
                                    )
                                )

                            threading.Thread(target=f).start()
                    else:
                        r = dronekit_connection._roll
                        p = dronekit_connection._pitch
                        y = dronekit_connection._yaw

                        rot = Rotation.from_euler("xyz", [r, p, -y], degrees=False)
                        # quaternion: xyzw
                        q = rot.as_quat()

                        o = Gf.Quatf(q[3], q[0], q[1], q[2])

                        north, east, down = (
                            dronekit_connection.location.local_frame.north,
                            dronekit_connection.location.local_frame.east,
                            dronekit_connection.location.local_frame.down,
                        )
                        # print('ned', n, e, d, prim)
                        if down is not None:
                            if prim.GetAttribute("xformOp:orient").Get() == None:
                                prim.GetAttribute("xformOp:rotateXYZ").Set(
                                    Gf.Vec3f(r, p, y)
                                )
                            else:
                                prim.GetAttribute("xformOp:orient").Set(o)

                            forward, left, up = north, -east, -down
                            p = (forward, left, up)  # FLU
                            prim.GetAttribute("xformOp:translate").Set(p)
                            if prop:
                                incremental_rotate(prop)
                        else:
                            # print("Drone location from dronekit is None")
                            pass
            except Exception as e:
                print(f"error: {e}")

            # Set outputs:execOut if not hidden
            if (
                db.node.get_attribute("outputs:execOut").get_metadata(
                    ogn.MetadataKeys.HIDDEN
                )
                != "1"
            ):
                db.outputs.execOut = og.ExecutionAttributeState.ENABLED

            try:
                compute_function = getattr(
                    OgnAscentNodeDatabase.NODE_TYPE_CLASS, "compute", None
                )
                if (
                    callable(compute_function)
                    and compute_function.__code__.co_argcount > 1
                ):  # pragma: no cover
                    return compute_function(context, node)

                db.inputs._prefetch()
                db.inputs._setting_locked = True
                with og.in_compute():
                    return OgnAscentNodeDatabase.NODE_TYPE_CLASS.compute(db)
            except Exception as error:  # pragma: no cover
                stack_trace = "".join(traceback.format_tb(sys.exc_info()[2].tb_next))
                db.log_error(
                    f"Assertion raised in compute - {error}\n{stack_trace}",
                    add_context=False,
                )
            finally:
                db.inputs._setting_locked = False
                db.outputs._commit()
            return False

        @staticmethod
        def initialize(context, node):
            OgnAscentNodeDatabase._initialize_per_node_data(node)
            initialize_function = getattr(
                OgnAscentNodeDatabase.NODE_TYPE_CLASS, "initialize", None
            )
            if callable(initialize_function):  # pragma: no cover
                initialize_function(context, node)

            per_node_data = OgnAscentNodeDatabase.PER_NODE_DATA[node.node_id()]

            def on_connection_or_disconnection(*args):
                per_node_data["_db"] = None

            node.register_on_connected_callback(on_connection_or_disconnection)
            node.register_on_disconnected_callback(on_connection_or_disconnection)

        @staticmethod
        def release(node):
            release_function = getattr(
                OgnAscentNodeDatabase.NODE_TYPE_CLASS, "release", None
            )
            if callable(release_function):  # pragma: no cover
                release_function(node)
            OgnAscentNodeDatabase._release_per_node_data(node)

        @staticmethod
        def init_instance(node, graph_instance_id):
            init_instance_function = getattr(
                OgnAscentNodeDatabase.NODE_TYPE_CLASS, "init_instance", None
            )
            if callable(init_instance_function):  # pragma: no cover
                init_instance_function(node, graph_instance_id)

        @staticmethod
        def release_instance(node, graph_instance_id):
            release_instance_function = getattr(
                OgnAscentNodeDatabase.NODE_TYPE_CLASS, "release_instance", None
            )
            if callable(release_instance_function):  # pragma: no cover
                release_instance_function(node, graph_instance_id)
            OgnAscentNodeDatabase._release_per_node_instance_data(
                node, graph_instance_id
            )

        @staticmethod
        def update_node_version(context, node, old_version, new_version):
            update_node_version_function = getattr(
                OgnAscentNodeDatabase.NODE_TYPE_CLASS, "update_node_version", None
            )
            if callable(update_node_version_function):  # pragma: no cover
                return update_node_version_function(
                    context, node, old_version, new_version
                )
            return False

        @staticmethod
        def initialize_type(node_type):
            initialize_type_function = getattr(
                OgnAscentNodeDatabase.NODE_TYPE_CLASS, "initialize_type", None
            )
            needs_initializing = True
            if callable(initialize_type_function):  # pragma: no cover
                needs_initializing = initialize_type_function(node_type)
            if needs_initializing:
                node_type.set_metadata(ogn.MetadataKeys.EXTENSION, "airlab.airstack")
                node_type.set_metadata(ogn.MetadataKeys.UI_NAME, "Ascent Node")
                node_type.set_metadata(ogn.MetadataKeys.CATEGORIES, "airstack")
                node_type.set_metadata(
                    ogn.MetadataKeys.DESCRIPTION,
                    "Runs the Ascent Spirit SITL, mavproxy and mavros.",
                )
                node_type.set_metadata(ogn.MetadataKeys.LANGUAGE, "Python")
                icon_path = carb.tokens.get_tokens_interface().resolve(
                    "${airlab.airstack}"
                )
                icon_path = icon_path + "/" + "ogn/icons/airlab.airstack.AscentNode.svg"
                node_type.set_metadata(ogn.MetadataKeys.ICON_PATH, icon_path)
                __hints = node_type.get_scheduling_hints()
                if __hints is not None:
                    __hints.set_data_access(
                        og.eAccessLocation.E_USD, og.eAccessType.E_WRITE
                    )
                OgnAscentNodeDatabase.INTERFACE.add_to_node_type(node_type)
                node_type.set_has_state(True)

        @staticmethod
        def on_connection_type_resolve(node):
            on_connection_type_resolve_function = getattr(
                OgnAscentNodeDatabase.NODE_TYPE_CLASS,
                "on_connection_type_resolve",
                None,
            )
            if callable(on_connection_type_resolve_function):  # pragma: no cover
                on_connection_type_resolve_function(node)

    NODE_TYPE_CLASS = None

    @staticmethod
    def register(node_type_class):
        print("node type class", node_type_class, type(node_type_class))
        OgnAscentNodeDatabase.NODE_TYPE_CLASS = node_type_class
        og.register_node_type(OgnAscentNodeDatabase.abi, 2)

    @staticmethod
    def deregister():
        og.deregister_node_type("airlab.airstack.AscentNode")
