# SPDX-FileCopyrightText: Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: LicenseRef-NvidiaProprietary
#
# NVIDIA CORPORATION, its affiliates and licensors retain all intellectual
# property and proprietary rights in and to this material, related
# documentation and any modifications thereto. Any use, reproduction,
# disclosure or distribution of this material and related documentation
# without an express license agreement from NVIDIA CORPORATION or
# its affiliates is strictly prohibited.

import omni.ext
import omni.ui as ui
import subprocess

# common omnigraphs test
from pathlib import Path
import omni.graph.core as og
import omni.ui as ui
import omni.usd
from omni.isaac.core.utils.stage import get_next_free_path
from omni.isaac.ui.callbacks import on_docs_link_clicked, on_open_IDE_clicked
from omni.isaac.ui.style import get_style
from omni.isaac.ui.ui_utils import dropdown_builder
from omni.isaac.ui.widgets import ParamWidget, SelectPrimWidget
from omni.kit.notification_manager import NotificationStatus, post_notification
from omni.kit.window.extensions import SimpleCheckBox
from pxr import OmniGraphSchema, Sdf
from omni.isaac.ui.menu import make_menu_item_description
from omni.kit.menu.utils import MenuItemDescription, add_menu_items, remove_menu_items

def make_camera_graph():
    timeline = omni.timeline.get_timeline_interface()
    timeline.stop()

    keys = og.Controller.Keys
    (graph, nodes, _, _) = og.Controller.edit(
        {"graph_path": "/airstack_camera", "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("IsaacRunOneSimulationFrame", "omni.isaac.core_nodes.OgnIsaacRunOneSimulationFrame"),
                ("ScriptNode", "omni.graph.scriptnode.ScriptNode"),
                ("IsaacCreateRenderProduct", "omni.isaac.core_nodes.IsaacCreateRenderProduct"),
                ("ROS2CameraHelper", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "IsaacRunOneSimulationFrame.inputs:execIn"),
                ("IsaacRunOneSimulationFrame.outputs:step", "ScriptNode.inputs:execIn"),
                ("ScriptNode.outputs:execOut", "IsaacCreateRenderProduct.inputs:execIn"),
                ("IsaacCreateRenderProduct.outputs:execOut", "ROS2CameraHelper.inputs:execIn"),
                ("IsaacCreateRenderProduct.outputs:renderProductPath", "ROS2CameraHelper.inputs:renderProductPath"),
                ("ScriptNode.outputs:frameId", "ROS2CameraHelper.inputs:frameId"),
                ("ScriptNode.outputs:nodeNamespace", "ROS2CameraHelper.inputs:nodeNamespace"),
                ("ScriptNode.outputs:topicName", "ROS2CameraHelper.inputs:topicName"),
                ("ScriptNode.outputs:cameraPrim", "IsaacCreateRenderProduct.inputs:cameraPrim"),
            ],
            keys.CREATE_ATTRIBUTES: [
                ("ScriptNode.inputs:namespaceDepthFromRoot", "uint"),
                ("ScriptNode.inputs:overrideNamespace", "string"),
                ("ScriptNode.outputs:cameraPrim", "target"),
                ("ScriptNode.outputs:cameraPrimRight", "target"),
                ("ScriptNode.outputs:frameId", "string"),
                ("ScriptNode.outputs:frameIdRight", "string"),
                ("ScriptNode.outputs:nodeNamespace", "string"),
                ("ScriptNode.outputs:topicName", "string"),
                ("ScriptNode.outputs:topicNameRight", "string"),
                ("ScriptNode.outputs:topicNameDepth", "string"),
                ("ScriptNode.outputs:topicNameDepthRight", "string"),
                ("ScriptNode.outputs:isStereo", "bool"),
            ],
            keys.SET_VALUES: [
                ("ScriptNode.inputs:namespaceDepthFromRoot", 1),
                ("ScriptNode.inputs:isStereo", False),
                ("ScriptNode.inputs:script", open("/extras/omnigraph_sensor_parameters.py").read()),
            ],
        },
    )

def make_stereo_camera_graph():
    timeline = omni.timeline.get_timeline_interface()
    timeline.stop()

    keys = og.Controller.Keys
    (graph, nodes, _, _) = og.Controller.edit(
        {"graph_path": "/airstack_stereo_camera", "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("IsaacRunOneSimulationFrame", "omni.isaac.core_nodes.OgnIsaacRunOneSimulationFrame"),
                ("ScriptNode", "omni.graph.scriptnode.ScriptNode"),
                ("IsaacCreateRenderProductLeft", "omni.isaac.core_nodes.IsaacCreateRenderProduct"),
                ("IsaacCreateRenderProductRight", "omni.isaac.core_nodes.IsaacCreateRenderProduct"),
                ("ROS2CameraHelperLeftImage", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
                ("ROS2CameraHelperRightImage", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
                ("ROS2CameraHelperLeftDepth", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
                ("ROS2CameraHelperRightDepth", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
                ("ROS2CameraInfoHelper", "omni.isaac.ros2_bridge.ROS2CameraInfoHelper"),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "IsaacRunOneSimulationFrame.inputs:execIn"),
                ("IsaacRunOneSimulationFrame.outputs:step", "ScriptNode.inputs:execIn"),
                
                ("ScriptNode.outputs:execOut", "IsaacCreateRenderProductRight.inputs:execIn"),
                ("ScriptNode.outputs:execOut", "IsaacCreateRenderProductLeft.inputs:execIn"),
                
                ("IsaacCreateRenderProductLeft.outputs:execOut", "ROS2CameraHelperLeftImage.inputs:execIn"),
                ("IsaacCreateRenderProductLeft.outputs:execOut", "ROS2CameraHelperLeftDepth.inputs:execIn"),
                ("IsaacCreateRenderProductLeft.outputs:execOut", "ROS2CameraInfoHelper.inputs:execIn"),
                ("IsaacCreateRenderProductLeft.outputs:renderProductPath", "ROS2CameraHelperLeftImage.inputs:renderProductPath"),
                ("IsaacCreateRenderProductLeft.outputs:renderProductPath", "ROS2CameraHelperLeftDepth.inputs:renderProductPath"),
                ("IsaacCreateRenderProductLeft.outputs:renderProductPath", "ROS2CameraInfoHelper.inputs:renderProductPath"),
                
                ("IsaacCreateRenderProductRight.outputs:execOut", "ROS2CameraHelperRightImage.inputs:execIn"),
                ("IsaacCreateRenderProductRight.outputs:execOut", "ROS2CameraHelperRightDepth.inputs:execIn"),
                ("IsaacCreateRenderProductRight.outputs:renderProductPath", "ROS2CameraHelperRightImage.inputs:renderProductPath"),
                ("IsaacCreateRenderProductRight.outputs:renderProductPath", "ROS2CameraHelperRightDepth.inputs:renderProductPath"),
                ("IsaacCreateRenderProductRight.outputs:renderProductPath", "ROS2CameraInfoHelper.inputs:renderProductPathRight"),
                
                ("ScriptNode.outputs:frameId", "ROS2CameraHelperLeftImage.inputs:frameId"),
                ("ScriptNode.outputs:frameIdRight", "ROS2CameraHelperLeftDepth.inputs:frameId"),
                ("ScriptNode.outputs:frameId", "ROS2CameraHelperRightImage.inputs:frameId"),
                ("ScriptNode.outputs:frameIdRight", "ROS2CameraHelperRightDepth.inputs:frameId"),
                ("ScriptNode.outputs:frameId", "ROS2CameraInfoHelper.inputs:frameId"),
                ("ScriptNode.outputs:frameIdRight", "ROS2CameraInfoHelper.inputs:frameIdRight"),
                ("ScriptNode.outputs:infoTopicName", "ROS2CameraInfoHelper.inputs:topicName"),
                ("ScriptNode.outputs:infoTopicNameRight", "ROS2CameraInfoHelper.inputs:topicNameRight"),
                ("ScriptNode.outputs:nodeNamespace", "ROS2CameraHelperLeftImage.inputs:nodeNamespace"),
                ("ScriptNode.outputs:nodeNamespace", "ROS2CameraHelperRightImage.inputs:nodeNamespace"),
                ("ScriptNode.outputs:nodeNamespace", "ROS2CameraHelperLeftDepth.inputs:nodeNamespace"),
                ("ScriptNode.outputs:nodeNamespace", "ROS2CameraHelperRightDepth.inputs:nodeNamespace"),
                ("ScriptNode.outputs:topicName", "ROS2CameraHelperLeftImage.inputs:topicName"),
                ("ScriptNode.outputs:topicNameRight", "ROS2CameraHelperRightImage.inputs:topicName"),
                ("ScriptNode.outputs:topicNameDepth", "ROS2CameraHelperLeftDepth.inputs:topicName"),
                ("ScriptNode.outputs:topicNameDepthRight", "ROS2CameraHelperRightDepth.inputs:topicName"),
                ("ScriptNode.outputs:cameraPrim", "IsaacCreateRenderProductLeft.inputs:cameraPrim"),
                ("ScriptNode.outputs:cameraPrimRight", "IsaacCreateRenderProductRight.inputs:cameraPrim"),
            ],
            keys.CREATE_ATTRIBUTES: [
                ("ScriptNode.inputs:namespaceDepthFromRoot", "uint"),
                ("ScriptNode.inputs:overrideNamespace", "string"),
                ("ScriptNode.inputs:isStereo", "bool"),
                ("ScriptNode.outputs:cameraPrim", "target"),
                ("ScriptNode.outputs:cameraPrimRight", "target"),
                ("ScriptNode.outputs:frameId", "string"),
                ("ScriptNode.outputs:frameIdRight", "string"),
                ("ScriptNode.outputs:nodeNamespace", "string"),
                ("ScriptNode.outputs:topicName", "string"),
                ("ScriptNode.outputs:topicNameRight", "string"),
                ("ScriptNode.outputs:topicNameDepth", "string"),
                ("ScriptNode.outputs:topicNameDepthRight", "string"),
                ("ScriptNode.outputs:infoTopicName", "string"),
                ("ScriptNode.outputs:infoTopicNameRight", "string"),
            ],
            keys.SET_VALUES: [
                ("ScriptNode.inputs:namespaceDepthFromRoot", 1),
                ("ScriptNode.inputs:isStereo", True),
                #("ScriptNode.inputs:script", open("/extras/omnigraph_sensor_parameters.py").read()),
                ("ScriptNode.inputs:usePath", True),
                ("ScriptNode.inputs:scriptPath", "/extras/omnigraph_sensor_parameters.py"),
                ("ROS2CameraHelperLeftDepth.inputs:type", "depth"),
                ("ROS2CameraHelperRightDepth.inputs:type", "depth"),
            ],
        },
    )
    
# Functions and vars are available to other extensions as usual in python: `airlab.tmux_manager.some_public_function(x)`
def some_public_function(x: int):
    print(f"[airlab.tmux_manager] some_public_function was called with {x}")
    return x ** x


def get_tmux_sessions():
    text = subprocess.getoutput('tmux ls')

    if ':' not in text:
        return []
    return list(map(lambda x:x.split(':')[0], text.split('\n')))

# Any class derived from `omni.ext.IExt` in the top level module (defined in `python.modules` of `extension.toml`) will
# be instantiated when the extension gets enabled, and `on_startup(ext_id)` will be called.
# Later when the extension gets disabled on_shutdown() is called.
class MyExtension(omni.ext.IExt):
    # ext_id is the current extension id. It can be used with the extension manager to query additional information,
    # like where this extension is located on the filesystem.
    def on_startup(self, ext_id):
        print("[airlab.tmux_manager] Extension startup")
        self.window_handle = None
        ros_og_menu = [
            make_menu_item_description(ext_id, "AirStack Camera", onclick_fun=make_camera_graph),
            make_menu_item_description(ext_id, "AirStack Stereo Camera", onclick_fun=make_stereo_camera_graph),
        ]

        self._menu_items = [
            MenuItemDescription(
                name="Common Omnigraphs",
                sub_menu=ros_og_menu,
            )
        ]

        add_menu_items(self._menu_items, "Airstack")

        self._count = 0

        self.window = ui.Window("TMUX Manager", width=300, height=300)
        self.window.deferred_dock_in("Property", ui.DockPolicy.CURRENT_WINDOW_IS_ACTIVE)

        #'''
        with self.window.frame:
            self.scroll = ui.ScrollingFrame(horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
                                                vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON)
            with self.scroll:
                with ui.VStack():
                    with ui.HStack(height=50):
                        ui.Label('TMUX Sessions:')
                        ui.Button('Refresh', clicked_fn=self.refresh_tmux_sessions)

                    ui.Spacer(height=5)
                    self.sessions_stack = ui.VStack()
        #'''

        self.sessions_dict = {}
        
        '''
        with self.window.frame:
            with ui.ScrollingFrame(horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
                                   vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON):
                with ui.VStack():
                    label = ui.Label("")
                    
                    def on_click():
                        self._count += 1
                        label.text = f"count: {self._count}"
                        
                    def on_reset():
                        self._count = 0
                        label.text = "empty"
                            
                    on_reset()

                    with ui.HStack():
                        ui.Button("Add", clicked_fn=on_click)
                        ui.Button("Reset", clicked_fn=on_reset)
        #'''

    def refresh_tmux_sessions(self):
        sessions = get_tmux_sessions()

        # even when the hstack is destroyed it still takes up space so it is made not visible
        # this probably means that the stack is still around and maybe if enough of them are created it will slow things down
        # maybe a better way is to reuse them and if there are less than before make them not visible until more are needed
        # for now it doesn't seem like it is impacting performance
        for k in self.sessions_dict.keys():
            self.sessions_dict[k]['label'].destroy()
            self.sessions_dict[k]['attach_button'].destroy()
            self.sessions_dict[k]['kill_button'].destroy()
            self.sessions_dict[k]['hstack'].destroy()
            self.sessions_dict[k]['hstack'].visible = False
        self.sessions_dict = {}
        
        for s in sessions:
            if s in self.sessions_dict.keys():
                continue
            self.sessions_dict[s] = {}
            
            with self.sessions_stack:
                self.sessions_dict[s]['hstack'] = ui.HStack(height=50)
                with self.sessions_dict[s]['hstack']:
                    self.sessions_dict[s]['label'] = ui.Label(s)

                    def get_attach(session_name):
                        def attach():
                            #print('xterm -e "tmux a -t ' + session_name + '"')
                            subprocess.Popen('xterm -e "tmux a -t \\"' + session_name + '\\""', shell=True)
                        return attach

                    def get_kill(session_name):
                        def kill():
                            #print('xterm -e "tmux kill-session -t ' + session_name + '"')
                            subprocess.Popen('xterm -e "tmux kill-session -t \\"' + session_name + '\\""', shell=True)
                        return kill
                    
                    self.sessions_dict[s]['attach_button'] = ui.Button('Attach', clicked_fn=get_attach(s))
                    self.sessions_dict[s]['kill_button'] = ui.Button('Kill', clicked_fn=get_kill(s))
        
        keys_to_remove = []
        for k in self.sessions_dict.keys():
            if k not in sessions:
                self.sessions_dict[k]['hstack'].destroy()
                keys_to_remove.append(k)
        for k in keys_to_remove:
            del self.sessions_dict[k]
        
    def on_shutdown(self):
        print("[airlab.tmux_manager] Extension shutdown")
