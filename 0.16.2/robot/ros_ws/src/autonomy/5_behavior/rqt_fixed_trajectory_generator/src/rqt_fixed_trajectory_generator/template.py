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

import os
import time
import numpy as np
import yaml
import collections
import pickle
from ament_index_python.packages import get_package_share_directory

from std_msgs.msg import Bool
from airstack_msgs.msg import FixedTrajectory
from diagnostic_msgs.msg import KeyValue

try:
    from rqt_py_console.spyder_console_widget import SpyderConsoleWidget
    _has_spyderlib = True
except ImportError:
    _has_spyderlib = False


class FixedTrajectoryGenerator(Plugin):
    """
    Plugin providing an interactive Python console
    """

    def __init__(self, context):
        super(FixedTrajectoryGenerator, self).__init__(context)
        self.setObjectName('FixedTrajectoryGenerator')

        self.context = context
        # to access ros2 node use self.context.node

        self.config_filename = ''

        self.button_dct = {}
        self.attribute_settings = collections.OrderedDict()

        self.timer = self.context.node.create_timer(1./10., self.timer_callback)

        self.fixed_trajectory_pub = self.context.node.create_publisher(FixedTrajectory, 'fixed_trajectory_command', 1)
        self.global_plan_fixed_trajectory_pub = self.context.node.create_publisher(FixedTrajectory, 'global_plan_fixed_trajectory', 1)

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

        # trajectory widget
        self.trajectory_widget = qt.QWidget()
        self.trajectory_layout = qt.QVBoxLayout()
        self.trajectory_widget.setLayout(self.trajectory_layout)
        self.vbox.addWidget(self.trajectory_widget)
        
        self.tab_widget = qt.QTabWidget()
        self.trajectory_layout.addWidget(self.tab_widget)

        # button widget
        self.button_widget = qt.QWidget()
        self.button_layout = qt.QHBoxLayout()
        self.button_widget.setLayout(self.button_layout)
        self.vbox.addWidget(self.button_widget)
        
        self.publish_button = qt.QPushButton('Publish')
        self.publish_button.clicked.connect(self.publish_trajectory)
        self.button_layout.addWidget(self.publish_button)
        
        self.trajectory_type_label = qt.QLabel('Type: ')
        self.button_layout.addWidget(self.trajectory_type_label)

        self.trajectory_type_combo_box = qt.QComboBox()
        self.trajectory_type_combo_box.addItem('Fixed Trajectory')
        self.trajectory_type_combo_box.addItem('Global Plan')
        self.button_layout.addWidget(self.trajectory_type_combo_box)

    def publish_trajectory(self):
        trajectory_type = self.trajectory_type_combo_box.currentText()
        trajectory_name = self.tab_widget.tabText(self.tab_widget.currentIndex())
        msg = FixedTrajectory()
        msg.type = trajectory_name
        for attribute, value in iter(self.attribute_settings[trajectory_name].items()):
            key_value = KeyValue()
            key_value.key = attribute
            key_value.value = value
            msg.attributes.append(key_value)
        if trajectory_type == 'Fixed Trajectory':
            self.fixed_trajectory_pub.publish(msg)
        elif trajectory_type == 'Global Plan':
            self.global_plan_fixed_trajectory_pub.publish(msg)

    def select_config_file(self):
        starting_path = get_package_share_directory('rqt_fixed_trajectory_generator') + '/config/'
        print(starting_path)
        filename = qt.QFileDialog.getOpenFileName(self.widget, 'Open Config File', starting_path, "Config Files (*.yaml)")[0]
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

        def get_attribute_changed_function(trajectory_name, attribute_name):
            def attribute_changed(text):
                if trajectory_name not in self.attribute_settings:
                    self.attribute_settings[trajectory_name] = {}
                self.attribute_settings[trajectory_name][attribute_name] = text
            return attribute_changed

        def get_publish_function(trajectory_name):
            def publish_function():
                msg = FixedTrajectory()
                msg.type = trajectory_name
                for attribute, value in iter(self.attribute_settings[trajectory_name].items()):
                    key_value = KeyValue()
                    key_value.key = attribute
                    key_value.value = value
                    msg.attributes.append(key_value)
                self.fixed_trajectory_pub.publish(msg)
            return publish_function
                

        for trajectory in y['trajectories']:
            trajectory_name = list(trajectory.keys())[0]
            attributes = trajectory[trajectory_name]['attributes']
            
            trajectory_tab = qt.QWidget()
            trajectory_layout = qt.QVBoxLayout()
            trajectory_tab.setLayout(trajectory_layout)

            for attribute in attributes:
                attribute_widget = qt.QWidget()
                attribute_layout = qt.QHBoxLayout()
                attribute_widget.setLayout(attribute_layout)
                
                attribute_label = qt.QLabel()
                attribute_label.setText(attribute)
                attribute_layout.addWidget(attribute_label)

                attribute_default = '0'
                if attribute == 'frame_id':
                    attribute_default = 'world'
                if trajectory_name in self.attribute_settings.keys():
                    if attribute in self.attribute_settings[trajectory_name].keys():
                        attribute_default = self.attribute_settings[trajectory_name][attribute]
                
                attribute_edit = qt.QLineEdit()
                attribute_edit.textChanged.connect(get_attribute_changed_function(trajectory_name,
                                                                                  attribute))
                attribute_edit.setText(attribute_default)
                
                attribute_layout.addWidget(attribute_edit)
                
                trajectory_layout.addWidget(attribute_widget)

            #publish_button = qt.QPushButton('Publish')
            #publish_button.clicked.connect(get_publish_function(trajectory_name))
            #trajectory_layout.addWidget(publish_button)
                
            self.tab_widget.addTab(trajectory_tab, trajectory_name)
            
            
        
    def timer_callback(self):
        bool_msg = Bool()
        for key in self.button_dct.keys():
            bool_msg.data = self.button_dct[key]['data']
            self.button_dct[key]['publisher'].publish(bool_msg)

    def save_settings(self, plugin_settings, instance_settings):
        instance_settings.set_value('config_filename', self.config_filename)
        instance_settings.set_value('attribute_settings', pickle.dumps(self.attribute_settings))

    def restore_settings(self, plugin_settings, instance_settings):
        attribute_settings = instance_settings.value('attribute_settings')
        if attribute_settings != None:
            self.attribute_settings = pickle.loads(attribute_settings)
        self.set_config(instance_settings.value('config_filename'))

        '''
    def save_settings(self, plugin_settings, instance_settings):
        pass
        #instance_settings.set_value('some_variable', self.some_variable)

    def restore_settings(self, plugin_settings, instance_settings):
        pass
        #self.some_variable = instance_settings.value('some_variable', default_value)
        '''
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
