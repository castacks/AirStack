# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Dorian Scholz
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from python_qt_binding.QtWidgets import QVBoxLayout, QWidget
from rqt_gui_py.plugin import Plugin
from qt_gui_py_common.simple_settings_dialog import SimpleSettingsDialog
from rqt_behavior_tree.py_console_widget import PyConsoleWidget
import threading

from python_qt_binding.QtCore import Slot, Qt, qVersion, qWarning, Signal
import python_qt_binding.QtWidgets as qt
import python_qt_binding.QtCore as core
import python_qt_binding.QtGui as gui

import rqt_behavior_tree
print(dir(rqt_behavior_tree))

from rqt_behavior_tree.xdot.xdot_qt import DotWidget

from std_msgs.msg import String

try:
    from rqt_behavior_tree.spyder_console_widget import SpyderConsoleWidget
    _has_spyderlib = True
except ImportError:
    _has_spyderlib = False


class PyConsole(Plugin):
    """
    Plugin providing an interactive Python console
    """

    def __init__(self, context):
        super(PyConsole, self).__init__(context)
        self.setObjectName('PyConsole')

        self.tree = None

        self.initialized_buttons = False
        self.prev_graphviz = ''

        #self.behavior_tree_graphviz_sub = rospy.Subscriber('behavior_tree_graphviz', String, self.behavior_tree_graphviz_callback)
        #self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)
        self.functions_mutex = threading.Lock()
        self.functions = {}
        self.last_graphviz_string = ''
        
        self.widget = QWidget()
        self.vbox = qt.QVBoxLayout()
        self.widget.setLayout(self.vbox)
        context.add_widget(self.widget)
        #self.widget.setStyleSheet('QWidget{margin-left:-1px;}')

        self.top_widget = qt.QWidget()
        self.top_layout = qt.QVBoxLayout()
        self.top_widget.setLayout(self.top_layout)
        
        self.graph_widget = qt.QWidget()
        self.graph_layout = qt.QVBoxLayout()
        self.graph_widget.setLayout(self.graph_layout)
        self.image_label = qt.QLabel('asdfadsf')
        #self.graph_layout.addWidget(self.image_label)
        
        self.xdot_widget = DotWidget()
        self.graph_layout.addWidget(self.xdot_widget)
        
        self.top_layout.addWidget(self.graph_widget)
        self.graph_widget.setStyleSheet("background-color: rgb(255, 255, 255);")
        
        self.config_widget = qt.QWidget()
        self.config_widget.setStyleSheet('QWidget{margin-left:-1px;}')
        self.config_layout = qt.QHBoxLayout()
        self.config_widget.setLayout(self.config_layout)
        self.config_widget.setFixedHeight(50)
        
        self.config_button = qt.QPushButton('Open Config...')
        #self.config_button.clicked.connect(self.select_config_file)
        self.config_layout.addWidget(self.config_button)

        self.tree_label = qt.QLabel('tree filename: ')
        self.config_layout.addWidget(self.tree_label)

        self.debug_checkbox = qt.QCheckBox('Debug Mode')
        self.config_layout.addWidget(self.debug_checkbox)
        #self.debug_checkbox.stateChanged.connect(self.debug_mode_changed)

        #self.config_widget.setStyleSheet("background-color: rgb(255, 0, 0);")
        #self.top_layout.addWidget(self.config_widget)

        #self.vbox.addWidget(self.top_widget)

        self.button_container_widget = qt.QWidget()
        self.button_container_layout = qt.QVBoxLayout()
        self.button_container_widget.setLayout(self.button_container_layout)
        #self.vbox.addWidget(self.button_container_widget)
        
        self.button_widget = qt.QWidget()
        self.button_layout = qt.QHBoxLayout()
        self.button_widget.setLayout(self.button_layout)
        #self.button_widget.setStyleSheet("background-color: rgb(0, 0, 255);")

        
        self.condition_widget = qt.QWidget()
        self.condition_layout = qt.QVBoxLayout()
        self.condition_widget.setLayout(self.condition_layout)
        self.button_layout.addWidget(self.condition_widget)
        
        self.condition_label = qt.QLabel()
        self.condition_label.setText('Conditions')
        self.condition_label.setAlignment(Qt.AlignCenter)
        self.condition_label.setFont(gui.QFont("SansSerif", 18, gui.QFont.Bold))
        self.condition_layout.addWidget(self.condition_label)
        
        self.action_widget = qt.QWidget()
        self.action_layout = qt.QVBoxLayout()
        self.action_widget.setLayout(self.action_layout)
        self.button_layout.addWidget(self.action_widget)
        
        self.action_label = qt.QLabel()
        self.action_label.setText('Actions')
        self.action_label.setAlignment(Qt.AlignCenter)
        self.action_label.setFont(gui.QFont("SansSerif", 18, gui.QFont.Bold))
        self.action_layout.addWidget(self.action_label)
        
        self.button_scroll_area = qt.QScrollArea()
        self.button_scroll_area.setWidget(self.button_widget)
        #self.button_scroll_area.setFixedHeight(200)
        #self.button_container_widget.setFixedHeight(200)
        self.button_container_layout.addWidget(self.button_scroll_area)
        self.button_widget.setMinimumWidth(self.button_scroll_area.sizeHint().width())
        self.button_scroll_area.setWidgetResizable(True)
        
        self.horizontal_splitter = qt.QSplitter(core.Qt.Vertical)
        self.horizontal_splitter.addWidget(self.top_widget)
        self.horizontal_splitter.addWidget(self.button_container_widget)
        self.vbox.addWidget(self.horizontal_splitter)

        self.button_container_widget.hide()
        
        self.behavior_tree_graphviz_sub = context.node.create_subscription(String,
                                                                           'behavior_tree_graphviz',
                                                                           self.behavior_tree_graphviz_callback,
                                                                           10)
        '''
        self.setObjectName('PyConsole')

        self._context = context
        self._use_spyderlib = _has_spyderlib
        self._console_widget = None
        self._widget = QWidget()
        self._widget.setLayout(QVBoxLayout())
        self._widget.layout().setContentsMargins(0, 0, 0, 0)
        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        self._context.add_widget(self._widget)
        '''
    
    def behavior_tree_graphviz_callback(self, msg):
        if msg.data != self.prev_graphviz:
            self.xdot_widget.set_dotcode(msg.data)
        self.prev_graphviz = msg.data
        
    def _switch_console_widget(self):
        return
        self._widget.layout().removeWidget(self._console_widget)
        self.shutdown_console_widget()

        '''
        if _has_spyderlib and self._use_spyderlib:
            self._console_widget = SpyderConsoleWidget(self._context)
            self._widget.setWindowTitle('SpyderConsole')
        else:
            self._console_widget = PyConsoleWidget(self._context)
            self._widget.setWindowTitle('PyConsole')
        if self._context.serial_number() > 1:
            self._widget.setWindowTitle(
                self._widget.windowTitle() + (' (%d)' % self._context.serial_number()))
        '''
        self._widget.layout().addWidget(self._console_widget)

    def save_settings(self, plugin_settings, instance_settings):
        return
        instance_settings.set_value('use_spyderlib', self._use_spyderlib)

    def restore_settings(self, plugin_settings, instance_settings):
        return
        self._use_spyderlib = _has_spyderlib and (
            instance_settings.value('use_spyderlib', True) in [True, 'true'])
        self._switch_console_widget()

    def trigger_configuration(self):
        options = [
            {'title': 'SpyderConsole',
             'description':
                'Advanced Python console with tab-completion (needs spyderlib to be installed).',
             'enabled': _has_spyderlib},
            {'title': 'PyConsole',
             'description': 'Simple Python console.'},
        ]
        dialog = SimpleSettingsDialog(title='PyConsole Options')
        dialog.add_exclusive_option_group(
            title='Console Type', options=options, selected_index=int(not self._use_spyderlib))
        console_type = dialog.get_settings()[0]
        new_use_spyderlib = {0: True, 1: False}.get(
            console_type['selected_index'], self._use_spyderlib)
        if self._use_spyderlib != new_use_spyderlib:
            self._use_spyderlib = new_use_spyderlib
            self._switch_console_widget()

    def shutdown_console_widget(self):
        if self._console_widget is not None and hasattr(self._console_widget, 'shutdown'):
            self._console_widget.shutdown()

    def shutdown_plugin(self):
        self.shutdown_console_widget()
