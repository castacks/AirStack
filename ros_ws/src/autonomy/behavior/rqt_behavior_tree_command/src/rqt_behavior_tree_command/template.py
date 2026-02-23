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
from rqt_py_console.py_console_widget import PyConsoleWidget

import python_qt_binding.QtWidgets as qt
import python_qt_binding.QtWidgets as QtWidgets
import python_qt_binding.QtGui as QtGui
import python_qt_binding.QtCore as QtCore

from ament_index_python.packages import get_package_share_directory
import yaml
import os

from behavior_tree_msgs.msg import Status, BehaviorTreeCommand, BehaviorTreeCommands

class BehaviorTreeCommandPlugin(Plugin):
    """
    Plugin providing an interactive Python console
    """

    def __init__(self, context):
        super(BehaviorTreeCommandPlugin, self).__init__(context)
        self.setObjectName('BehaviorTreeCommandPlugin')

        self.config_filename = ''
        self.button_groups = {}
        
        self.context = context
        self.command_pub = self.context.node.create_publisher(BehaviorTreeCommands, 'behavior_tree_commands', 10)

        # main layout
        self.widget = QWidget()
        self.vbox = qt.QVBoxLayout()
        self.widget.setLayout(self.vbox)
        context.add_widget(self.widget)

        # config widget
        self.config_widget = qt.QWidget()
        self.config_widget.setStyleSheet('QWidget{margin-left:-1px;}')
        self.config_layout = qt.QHBoxLayout()
        self.config_widget.setLayout(self.config_layout)
        self.config_widget.setFixedHeight(50)

        self.config_button = qt.QPushButton('Open Config...')
        self.config_button.clicked.connect(self.select_config_file)
        self.config_layout.addWidget(self.config_button)

        self.config_label = qt.QLabel('config filename: ')
        self.config_layout.addWidget(self.config_label)
        self.vbox.addWidget(self.config_widget)

        # button widget
        self.button_widget = qt.QWidget()
        self.button_layout = qt.QVBoxLayout()
        self.button_widget.setLayout(self.button_layout)
        self.vbox.addWidget(self.button_widget)

    def select_config_file(self):
        starting_path = get_package_share_directory('rqt_behavior_tree_command') + '/config/'
        filename = qt.QFileDialog.getOpenFileName(self.widget, 'Open Config File', starting_path, "Config Files (*.yaml)")[0]
        print(filename)
        self.set_config(filename)

    def set_config(self, filename):
        if filename != '':
            self.config_filename = filename
            if self.config_filename != None:
                self.config_label.setText('config filename: ' + os.path.basename(self.config_filename))
                self.init_buttons(filename)

    def init_buttons(self, filename):
        y = yaml.load(open(filename, 'r').read(), Loader=yaml.Loader)
        print(y)

        def get_click_function(group, button):
            def click_function():
                commands = BehaviorTreeCommands()
                for i in range(len(self.button_groups[group]['buttons'])):
                    b = self.button_groups[group]['buttons'][i]
                    command = BehaviorTreeCommand()
                    command.condition_name = self.button_groups[group]['condition_names'][i]

                    if b != button and b.isChecked():
                        b.toggle()
                        command.status = Status.FAILURE
                    elif b == button and not b.isChecked():
                        command.status = Status.FAILURE
                    elif b == button and b.isChecked():
                        command.status = Status.SUCCESS
                    commands.commands.append(command)
                self.command_pub.publish(commands)
            return click_function

        for group in y['groups']:
            group_name = list(group.keys())[0]
            if group_name not in self.button_groups.keys():
                self.button_groups[group_name] = {'buttons' : [], 'condition_names': []}

            group_widget = qt.QWidget()
            group_layout = qt.QVBoxLayout()
            group_widget.setLayout(group_layout)
            self.button_layout.addWidget(group_widget)

            group_layout.addWidget(qt.QLabel(group_name))

            button_widget = qt.QWidget()
            button_layout = qt.QHBoxLayout()
            button_widget.setLayout(button_layout)
            group_layout.addWidget(button_widget)

            for buttons in group[group_name]:
                button_name = list(buttons.keys())[0]
                condition_name = buttons[button_name]['condition_name']

                button = qt.QPushButton(button_name)
                button.clicked.connect(get_click_function(group_name, button))
                button.setCheckable(True)
                button_layout.addWidget(button)

                #print(condition_name, bt.get_condition_topic_name(condition_name))
                self.button_groups[group_name]['buttons'].append(button)
                self.button_groups[group_name]['condition_names'].append(condition_name)

    def save_settings(self, plugin_settings, instance_settings):
        instance_settings.set_value('config_filename', self.config_filename)

    def restore_settings(self, plugin_settings, instance_settings):
        self.set_config(instance_settings.value('config_filename'))

    def trigger_configuration(self):
        options = [
            {'title': 'Option 1',
             'description': 'Description of option 1.',
             'enabled': True},
            {'title': 'Option 2',
             'description': 'Description of option 2.'},
        ]
        dialog = SimpleSettingsDialog(title='Options')
        dialog.add_exclusive_option_group(title='List of options:', options=options, selected_index=0)
        selected_index = dialog.get_settings()[0]
        if selected_index != None:
            selected_index = selected_index['selected_index']
            print('selected_index ', selected_index)

    def shutdown_console_widget(self):
        pass

    def shutdown_plugin(self):
        self.shutdown_console_widget()
