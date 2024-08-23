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
