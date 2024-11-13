"""Support required by the Carbonite extension loader"""

import asyncio
from contextlib import suppress
from typing import List
import threading
import socket
import struct
import time
import subprocess

import carb
import carb.dictionary
import omni
import omni.ext
import omni.ui as ui
import omni.graph.core as og
from omni.kit.app import get_app
from omni.isaac.core.world import World

ASCENTNODE_OPT_IN_SETTING = "/app/airlab.airstack/opt_in"
ASCENTNODE_ENABLE_OPT_IN_SETTING = "/app/airlab.airstack/enable_opt_in"
OMNIGRAPH_STAGEUPDATE_ORDER = 100  # We want our attach() to run after OG so that nodes have been instantiated


# ==============================================================================================================
def set_all_graphs_enabled(enable: bool):
    """Set the enabled state of all OmniGraphs"""
    graphs = og.get_all_graphs()
    if graphs and not isinstance(graphs, list):
        graphs = [graphs]
    for graph in graphs:
        graph.set_disabled(not enable)


# ==============================================================================================================
def is_check_enabled():
    """Returns True if ascentnode opt-in is enabled"""
    settings = carb.settings.get_settings()
    if not settings.is_accessible_as(carb.dictionary.ItemType.BOOL, ASCENTNODE_ENABLE_OPT_IN_SETTING):
        # The enable-setting is not present, we enable the check
        return True

    if not settings.get(ASCENTNODE_ENABLE_OPT_IN_SETTING):
        # The enable-setting is present and False, disable the check
        return False
    # the enable-setting is present and True, enable the check
    return True


# ==============================================================================================================
def on_opt_in_change(item: carb.dictionary.Item, change_type: carb.settings.ChangeEventType):
    """Update the local cache of the setting value"""
    if change_type != carb.settings.ChangeEventType.CHANGED:
        return
    settings = carb.settings.get_settings()
    should_run = bool(settings.get(ASCENTNODE_OPT_IN_SETTING))
    if should_run:
        set_all_graphs_enabled(True)


# ==============================================================================================================
def verify_ascentnode_load(script_nodes: List[og.Node]):
    """
    Get verification from the user that they want to run ascentnodes.
    This opt-in applies to the current session only.

    Args:
        script_nodes: The list of script nodes on the stage that have
                      been disabled.
    """
    from omni.kit.window.popup_dialog import MessageDialog

    def on_cancel(dialog: MessageDialog):
        settings = carb.settings.get_settings()
        settings.set(ASCENTNODE_OPT_IN_SETTING, False)
        dialog.hide()

    def on_ok(dialog: MessageDialog):
        settings = carb.settings.get_settings()
        settings.set(ASCENTNODE_OPT_IN_SETTING, True)
        dialog.hide()

    message = """
This stage contains ascentnodes.

There is currently no limitation on what code can be executed by this node. This means that graphs that contain these nodes should only be used when the author of the graph is trusted.

Do you want to enable the ascentnode functionality for this session?
"""

    dialog = MessageDialog(
        title="Warning",
        width=400,
        message=message,
        cancel_handler=on_cancel,
        ok_handler=on_ok,
        ok_label="Yes",
        cancel_label="No",
    )

    async def show_async():
        # wait a few frames to allow the app ui to finish loading
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()
        dialog.show()

    asyncio.ensure_future(show_async())


# ==============================================================================================================
def check_for_ascentnodes():
    """
    Check for presence of airlab.airstack instances and confirm user wants to enable them.
    """
    # If the check is not enabled then we are good
    if not is_check_enabled():
        return

    # Check is enabled - see if they already opted-in
    settings = carb.settings.get_settings()
    ascentnode_opt_in = settings.get(ASCENTNODE_OPT_IN_SETTING)
    if ascentnode_opt_in:
        # The check is enabled, and they opted-in
        return

    # The check is enabled but they opted out, or haven't been prompted yet

    try:
        import omni.kit.window.popup_dialog  # noqa
    except ImportError:  # pragma: no cover
        # Don't prompt in headless mode
        return
    script_nodes = []
    graphs = og.get_all_graphs()
    if graphs and not isinstance(graphs, list):
        graphs = [graphs]
    for graph in graphs:
        for node in graph.get_nodes():
            node_type = node.get_node_type()
            if node_type.get_node_type() == "airlab.airstack.AscentNode":
                # Found one
                script_nodes.append(node)
    if not script_nodes:
        # No script nodes means we can leave them enabled
        return

    # Disable them until we get the opt-in via the async dialog
    set_all_graphs_enabled(False)
    verify_ascentnode_load(script_nodes)


def on_attach(ext_id: int, _):
    """Called when USD stage is attached"""
    check_for_ascentnodes()


# ==============================================================================================================
class TimeSyncServer:
    def __init__(self, host='127.0.0.1', port=65432):
        self.mutex = threading.Lock()
        self.client_count = 1
        self.current_sim_time = 0.
        
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.bind((host, port))
            s.listen()
            print(f'Server listening on {host}:{port}')
            def f():
                while True:
                    conn, addr = s.accept()
                    client_thread = threading.Thread(target=self.handle_client, args=(conn, addr))
                    client_thread.start()
            threading.Thread(target=f).start()
        except:
            client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            client.connect((host, port))
            message = struct.pack('cQ', b'n', 0)
            client.sendall(message)
            drone_count = struct.unpack('i', client.recv(4))[0]
        
    def get_sim_time(self):
        return self.current_sim_time

    def handle_client(self, conn, addr):
        print(f'Connected by {addr}')

        initial_sitl_time = -1.
        initial_sim_time = -1.

        with conn:
            while True:
                data = conn.recv(16)
                if not data:
                    break

                #print('data', len(data), data)
                message_type, t = struct.unpack('cQ', data)
                #print('message type', message_type, t)
                if message_type == b't':
                    s = self.get_sim_time()

                    if initial_sitl_time < 0:
                        initial_sitl_time = t
                        initial_sim_time = s

                    sitl_time = t - initial_sitl_time
                    sim_time = (s - initial_sim_time)*1000000
                    time_to_sleep = int(sitl_time - sim_time)
                    #print('sitl, sim, sleep', sitl_time, sim_time, time_to_sleep, self.get_sim_time(), self.current_sim_time, initial_sim_time)

                    conn.sendall(struct.pack('i', time_to_sleep))
                elif message_type == b'n':
                    print('message type', message_type, t)
                    with self.mutex:
                        self.client_count += 1
                    conn.sendall(struct.pack('i', self.client_count))

    def set_current_sim_time(self, current_sim_time):
        self.current_sim_time = current_sim_time
    

def get_tmux_sessions():
    text = subprocess.getoutput("tmux ls")

    if ":" not in text:
        return []
    return list(map(lambda x: x.split(":")[0], text.split("\n")))
        
# ==============================================================================================================
class _PublicExtension(omni.ext.IExt):
    """Object that tracks the lifetime of the Python part of the extension loading"""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.__stage_subscription = None
        self.__opt_in_setting_sub = None

        with suppress(ImportError):
            manager = get_app().get_extension_manager()
            # This is a bit of a hack to make the template directory visible to the OmniGraph UI extension
            # if it happens to already be enabled. The "hack" part is that this logic really should be in
            # omni.graph.ui, but it would be much more complicated there, requiring management of extensions
            # that both do and do not have dependencies on omni.graph.ui.
            if manager.is_extension_enabled("omni.graph.ui"):
                import omni.graph.ui as ogui  # noqa: PLW0621

                ogui.ComputeNodeWidget.get_instance().add_template_path(__file__)

    def on_startup(self):
        stage_update = omni.stageupdate.get_stage_update_interface()
        self.__stage_subscription = stage_update.create_stage_update_node("OmniGraphAttach", on_attach_fn=on_attach)
        assert self.__stage_subscription
        nodes = stage_update.get_stage_update_nodes()
        stage_update.set_stage_update_node_order(len(nodes) - 1, OMNIGRAPH_STAGEUPDATE_ORDER + 1)
        self.__opt_in_setting_sub = omni.kit.app.SettingChangeSubscription(ASCENTNODE_OPT_IN_SETTING, on_opt_in_change)
        assert self.__opt_in_setting_sub

        # init variables
        self.current_sim_time = 0.
        self.time_sync_server = TimeSyncServer()

        # init timeline callback
        self.timeline = omni.timeline.get_timeline_interface()
        self.play_listener = self.timeline.get_timeline_event_stream().create_subscription_to_pop_by_type(
            int(omni.timeline.TimelineEventType.PLAY), self.timeline_callback)
        self.pause_listener = self.timeline.get_timeline_event_stream().create_subscription_to_pop_by_type(
            int(omni.timeline.TimelineEventType.PAUSE), self.timeline_callback)
        self.stop_listener = self.timeline.get_timeline_event_stream().create_subscription_to_pop_by_type(
            int(omni.timeline.TimelineEventType.STOP), self.timeline_callback)

        # init physics callback
        self.init_physics()
        
        
        self.window = ui.Window("TMUX Manager", width=300, height=300)
        self.window.deferred_dock_in("Property", ui.DockPolicy.DO_NOTHING)

        #'''
        with self.window.frame:
            self.scroll = ui.ScrollingFrame(
                horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
                vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
            )
            with self.scroll:
                with ui.VStack():
                    with ui.HStack(height=50):
                        ui.Label("TMUX Sessions:")
                        ui.Button("Refresh", clicked_fn=self.refresh_tmux_sessions)

                    ui.Spacer(height=5)
                    self.sessions_stack = ui.VStack()
        #'''
        self.sessions_dict = {}

        self.refresh_tmux_sessions()

    def refresh_tmux_sessions(self):
        sessions = get_tmux_sessions()

        # even when the hstack is destroyed it still takes up space so it is made not visible
        # this probably means that the stack is still around and maybe if enough of them are created it will slow things down
        # maybe a better way is to reuse them and if there are less than before make them not visible until more are needed
        # for now it doesn't seem like it is impacting performance
        for k in self.sessions_dict.keys():
            self.sessions_dict[k]["label"].destroy()
            self.sessions_dict[k]["attach_button"].destroy()
            self.sessions_dict[k]["kill_button"].destroy()
            self.sessions_dict[k]["hstack"].destroy()
            self.sessions_dict[k]["hstack"].visible = False
        self.sessions_dict = {}

        for s in sessions:
            if s in self.sessions_dict.keys():
                continue
            self.sessions_dict[s] = {}

            with self.sessions_stack:
                self.sessions_dict[s]["hstack"] = ui.HStack(height=50)
                with self.sessions_dict[s]["hstack"]:
                    self.sessions_dict[s]["label"] = ui.Label(s)

                    def get_attach(session_name):
                        def attach():
                            # print('xterm -e "tmux a -t ' + session_name + '"')
                            subprocess.Popen(
                                'xterm -bg black -fg white -e "tmux a -t \\"'
                                + session_name
                                + '\\""',
                                shell=True,
                            )

                        return attach

                    def get_kill(session_name):
                        def kill():
                            # print('xterm -e "tmux kill-session -t ' + session_name + '"')
                            subprocess.Popen(
                                'xterm -e "tmux kill-session -t \\"'
                                + session_name
                                + '\\""',
                                shell=True,
                            )

                        return kill

                    self.sessions_dict[s]["attach_button"] = ui.Button(
                        "Attach", clicked_fn=get_attach(s)
                    )
                    self.sessions_dict[s]["kill_button"] = ui.Button(
                        "Kill", clicked_fn=get_kill(s)
                    )

        keys_to_remove = []
        for k in self.sessions_dict.keys():
            if k not in sessions:
                self.sessions_dict[k]["hstack"].destroy()
                keys_to_remove.append(k)
        for k in keys_to_remove:
            del self.sessions_dict[k]

    def init_physics(self, was_playing=False):
        self.world = World()
        if self.world._physics_context != None:
            print('physics already initialized')
            return
        async def init_physics():
            if self.world._physics_context == None:
                await self.world.initialize_simulation_context_async()
                if was_playing:
                    self.world.play()
            self.world.add_physics_callback('airstack_physics_callback', self.physics_callback)
            print('added physics callback', self.world._physics_context)
        asyncio.ensure_future(init_physics())

    def timeline_callback(self, event):
        print('timeline callback', event, event.type, dir(event))

        if event.type == int(omni.timeline.TimelineEventType.PLAY):
            self.init_physics(True)
        elif event.type == int(omni.timeline.TimelineEventType.PAUSE):
            self.init_physics()
        elif event.type == int(omni.timeline.TimelineEventType.STOP):
            self.current_sim_time = 0.
            
        self.refresh_tmux_sessions()

    def physics_callback(self, sim_time_delta):
        self.current_sim_time += sim_time_delta
        #print('setting time', self.current_sim_time)
        self.time_sync_server.set_current_sim_time(self.current_sim_time)
        
    def on_shutdown(self):
        print('SHUTDOWN')
        self.refresh_tmux_sessions()
        self.__stage_subscription = None
        self.__opt_in_setting_sub = None
        self.world.remove_physics_callback('airstack_physics_callback')
