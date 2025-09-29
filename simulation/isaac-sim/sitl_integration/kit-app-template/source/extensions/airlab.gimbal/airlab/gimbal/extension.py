import omni.ext
import subprocess

import omni.ui as ui
import omni.kit.window.filepicker as filepicker
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from pxr import Usd
from omni.isaac.core import World
# from omni.ext import get_extension_path
import omni.usd
from pxr import UsdGeom, Gf, UsdPhysics
import omni.graph.core as og
import time

class MyExtension(omni.ext.IExt):
    def on_startup(self):
        """Called when the extension is loaded."""
        print("[GimbalExtension] Startup called")  # Debug log

        # self.world = World.get_instance()
        self.ui = GimbalUI()
        print("[GimbalExtension] UI initialized")  # Debug log

    def on_shutdown(self):
        """Called when the extension is unloaded."""
        # self.world.cleanup()
        self.ui.window.destroy()
        
class GimbalUI:
    def __init__(self):
        self.window = ui.Window("Gimbal Extension UI", width=400, height=300)
        self.usd_file_path = ""
        self.robot_prim_path = ""
        self.robot_name = 1
        self.default_usd_path = "omniverse://airlab-nucleus.andrew.cmu.edu/Library/Assets/Gimbal/gimbal_test.usd"
        self.default_robot_path = "/World/TEMPLATE_spirit_uav_robot/map_FLU/spirit_uav"

        self.build_ui()

    def build_ui(self):
        with self.window.frame:
            with ui.VStack(spacing=10):
                # USD File Input
                ui.Label("USD File Path", height=20)
                with ui.HStack(height=30):
                    self.usd_input_field = ui.StringField(
                        model=ui.SimpleStringModel(self.default_usd_path), height=20
                    )
                    ui.Button("Browse", clicked_fn=self.open_file_picker)
    
                # Robot Prim Path Input
                ui.Label("Robot Prim Path (copy from Stage)", height=20)
                self.robot_prim_input_field = ui.StringField(
                    model=ui.SimpleStringModel(self.default_robot_path), height=20
                )

                # Robot Name Input
                ui.Label("Robot Index", height=20)
                self.robot_name_input_field = ui.StringField(
                    model=ui.SimpleStringModel("2"), height=20
                )
                # Apply Button
                ui.Button("Apply and Add Gimbal", height=40, clicked_fn=self.on_apply)

    def open_file_picker(self):
        """Opens a file picker to select the USD file."""
        file_picker = filepicker.FilePickerDialog(
            title="Select Gimbal USD File",
            apply_button_label="Select",
            file_extension_filter="*.usd",
            apply_clicked_fn=self.on_file_selected
        )
        file_picker.show()

    def on_file_selected(self, file_path):
        """Callback for file selection."""
        self.usd_input_field.model.set_value(file_path)

    def on_apply(self):
        """Apply button callback to add the gimbal and configure the ActionGraph."""
        self.usd_file_path = self.usd_input_field.model.get_value_as_string()
        self.robot_prim_path = self.robot_prim_input_field.model.get_value_as_string()
        self.robot_name = self.robot_name_input_field.model.get_value_as_string()

        # Validate input
        if not self.usd_file_path or not self.robot_prim_path or not self.robot_name:
            print("Please fill in all fields.")
            return

        # Add the gimbal to the stage
        self.add_gimbal_to_stage()

    def find_existing_op(self, xform_ops, op_type):
        for op in xform_ops:
            if op.GetOpName() == op_type:
                return op
        return None

    def add_gimbal_to_stage(self):
        """Adds the gimbal to the robot and configures the OmniGraph."""
        stage = omni.usd.get_context().get_stage()

        # Add the USD file reference
        gimbal_prim_path = f"{self.robot_prim_path}/gimbal"
        add_reference_to_stage(self.usd_file_path, gimbal_prim_path)


        # Apply transformations (scale and translation)
        gimbal_prim = stage.GetPrimAtPath(gimbal_prim_path)
        if gimbal_prim.IsValid():
            gimbal_xform = UsdGeom.Xformable(gimbal_prim)
            xform_ops = gimbal_xform.GetOrderedXformOps()

            # Check if a scale operation already exists
            scale_op = self.find_existing_op(xform_ops, "xformOp:scale")
            if not scale_op:
                scale_op = gimbal_xform.AddScaleOp()
            scale_value = Gf.Vec3d(0.01, 0.01, 0.01)
            scale_op.Set(scale_value)

            # Check if a translate operation already exists
            translate_op = self.find_existing_op(xform_ops, "xformOp:translate")
            if not translate_op:
                translate_op = gimbal_xform.AddTranslateOp()
            translation_value = Gf.Vec3d(0.02, 0.015, 0.1)
            translate_op.Set(translation_value)

            print(f"Gimbal added at {gimbal_prim_path} with scale {scale_value} and translation {translation_value}.")

        # Add a fixed joint between the gimbal and the robot
        self.add_fixed_joint(stage, self.robot_prim_path, gimbal_prim_path)

        # """Enables the ActionGraph within the gimbal and sets inputs."""
        action_graph_path = f"{gimbal_prim_path}/ActionGraph"
        action_graph_prim = stage.GetPrimAtPath(action_graph_path)

        if action_graph_prim.IsValid():
            # Access the graph
            graph = og.Controller.graph(action_graph_path)
            graph_handle = og.get_graph_by_path(action_graph_path)

            # Set the input value
            node_path = "/World/ActionGraph/MyNode"  # Replace with the path to your node
            input_name = "myInput"  # Replace with the name of the input
            value = 10  # Replace with the value you want to set

            # Define the path to ros2_context node
            ros2_context_path = action_graph_path+"/ros2_context"
            self.list_node_attributes(ros2_context_path)

            self.set_or_create_node_attribute(ros2_context_path, "inputs:domain_id", int(self.robot_name))     
            self.set_or_create_node_attribute(action_graph_path+"/ros2_subscriber", "inputs:topicName", "robot_"+self.robot_name+"/gimbal/not_used")
            self.set_or_create_node_attribute(action_graph_path+"/ros2_subscribe_joint_state", "inputs:topicName", "robot_"+self.robot_name+"/gimbal/joint_command")
            self.set_or_create_node_attribute(action_graph_path+"/ros2_publish_joint_state", "inputs:topicName", "robot_"+self.robot_name+"/gimbal/joint_states")
            self.set_or_create_node_attribute(action_graph_path+"/ros2_camera_helper", "inputs:topicName", "robot_"+self.robot_name+"/gimbal/rgb")

            # og.Controller.attribute(action_graph_path+"/ros2_context.inputs:domain_id").set(int(self.robot_name))
            # og.Controller.attribute(action_graph_path+"/ros2_subscriber.inputs:topicName").set("robot_"+self.robot_name+"/gimbal_yaw_control")
            # og.Controller.attribute(action_graph_path+"/ros2_subscribe_joint_state.inputs:topicName").set("robot_"+self.robot_name+"/joint_command")
            # og.Controller.attribute(action_graph_path+"/ros2_publish_joint_state.inputs:topicName").set("robot_"+self.robot_name+"/joint_states")


    def add_fixed_joint(self, stage, robot_prim_path, gimbal_prim_path):
        """Adds a fixed joint between the robot and the gimbal."""
        joint_path = f"{gimbal_prim_path}/FixedJoint"
        joint_prim = stage.DefinePrim(joint_path, "PhysicsFixedJoint")

        # Define the fixed joint's relationship to the robot and gimbal components
        joint = UsdPhysics.FixedJoint(joint_prim)
        if not joint:
            print(f"Failed to create fixed joint at {joint_path}")
            return

        # Set joint relationships
        joint.CreateBody0Rel().SetTargets([f"{robot_prim_path}/base_link/meshes/Cone"])
        joint.CreateBody1Rel().SetTargets([f"{gimbal_prim_path}/yaw"])

        print(f"Fixed joint created between {robot_prim_path} and {gimbal_prim_path}.")

    def list_node_attributes(self, node_path):
        """Lists all attributes of a given node in OmniGraph."""
        stage = omni.usd.get_context().get_stage()
        
        if not stage:
            print("Error: USD stage not found.")
            return
        
        node_prim = stage.GetPrimAtPath(node_path)

        if not node_prim.IsValid():
            print(f"Error: Node not found at {node_path}")
            return

        print(f"Attributes in node '{node_path}':")

        for attr in node_prim.GetAttributes():
            print(f"- {attr.GetName()}")       

    def set_or_create_node_attribute(self, node_path, attribute_name, value):
        """Sets an attribute value for a given node in OmniGraph, creating it if necessary."""
        stage = omni.usd.get_context().get_stage()
        node_prim = stage.GetPrimAtPath(node_path)

        if not node_prim.IsValid():
            print(f"Error: Node not found at {node_path}")
            return

        attr = node_prim.GetAttribute(attribute_name)

        if not attr:
            print(f"Attribute {attribute_name} not found, creating it...")
            attr = node_prim.CreateAttribute(attribute_name, Sdf.ValueTypeNames.Int)
        
        if attr:
            attr.Set(value)
            print(f"Set {attribute_name} to {value} on node {node_path}.")
        else:
            print(f"Failed to create or set attribute {attribute_name}.")
