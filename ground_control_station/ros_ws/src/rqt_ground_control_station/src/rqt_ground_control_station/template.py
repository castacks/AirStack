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
import python_qt_binding.QtGui as gui
import python_qt_binding.QtCore as core

from ament_index_python.packages import get_package_share_directory
import yaml
import os
import collections
import copy
import inspect
import time
import pickle

from behavior_tree_msgs.msg import Status, BehaviorTreeCommand, BehaviorTreeCommands
from airstack_msgs.msg import FixedTrajectory
from diagnostic_msgs.msg import KeyValue

from .drag_and_drop import DragWidget, DragItem
from .trajectory_dialog import TrajectoryDialog

logger = None

class GroundControlStation(Plugin):

    def __init__(self, context):
        super(GroundControlStation, self).__init__(context)
        self.setObjectName('GroundControlStation')

        self.settings = {'command_config_filename': None, 'trajectory_config_filename': None,
                         'robots': [], 'groups': {'commands': []}, 'publishers': {}}

        #self.config_filename = ''
        self.button_groups = {}
        
        self.context = context
        self.node = self.context.node
        global logger
        logger = self.node.get_logger()
        
        # main layout
        self.widget = QWidget()
        self.vbox = qt.QVBoxLayout()
        self.widget.setLayout(self.vbox)
        context.add_widget(self.widget)

        # config widget
        self.config_widget = qt.QWidget()
        #self.config_widget.setStyleSheet('QWidget{margin-left:-1px;}')
        self.config_layout = qt.QHBoxLayout()
        self.config_widget.setLayout(self.config_layout)
        self.config_widget.setFixedHeight(50)

        self.robot_selection_label = qt.QLabel('Robot to command:')
        self.config_layout.addWidget(self.robot_selection_label)
        
        self.robot_combo_box = qt.QComboBox()
        self.config_layout.addWidget(self.robot_combo_box)
        
        self.vbox.addWidget(self.config_widget)
        
        spacer = qt.QSpacerItem(20, 40, qt.QSizePolicy.Minimum, qt.QSizePolicy.Expanding)
        self.vbox.addItem(spacer)

        # button widget
        self.button_widget = qt.QWidget()
        self.button_layout = qt.QVBoxLayout()
        self.button_widget.setLayout(self.button_layout)
        self.vbox.addWidget(self.button_widget)
        
        spacer = qt.QSpacerItem(20, 40, qt.QSizePolicy.Minimum, qt.QSizePolicy.Expanding)
        self.vbox.addItem(spacer)
        
        # timeline widget
        self.timeline_widget = qt.QWidget()
        self.timeline_layout = qt.QHBoxLayout()
        self.timeline_widget.setLayout(self.timeline_layout)

        self.timeline_button_widget = qt.QWidget()
        self.timeline_button_layout = qt.QVBoxLayout(self.timeline_button_widget)
        self.timeline_layout.addWidget(self.timeline_button_widget)
        
        self.default_button_style = 'QPushButton{font-size: 40px; font-weight: bold}'
        self.green_button_style = 'QPushButton{font-size: 40px; font-weight: bold; background-color: green}'
        self.yellow_button_style = 'QPushButton{font-size: 40px; font-weight: bold; background-color: yellow}'
        self.red_button_style = 'QPushButton{font-size: 40px; font-weight: bold; background-color: red}'
        self.timeline_play_button = AspectRatioButton('▶')
        self.timeline_play_button.setStyleSheet(self.default_button_style)
        self.timeline_play_button.clicked.connect(self.play)
        self.timeline_button_layout.addWidget(self.timeline_play_button)
        
        self.timeline_pause_button = AspectRatioButton('❚❚')
        self.timeline_pause_button.setStyleSheet(self.default_button_style)
        self.timeline_pause_button.clicked.connect(self.pause)
        self.timeline_button_layout.addWidget(self.timeline_pause_button)
        
        self.timeline_stop_button = AspectRatioButton('◼')
        self.timeline_stop_button.setStyleSheet(self.red_button_style)
        self.timeline_stop_button.clicked.connect(self.stop)
        self.timeline_button_layout.addWidget(self.timeline_stop_button)

        self.timeline_drag_container_widget = qt.QWidget()
        self.timeline_drag_container_layout = qt.QHBoxLayout(self.timeline_drag_container_widget)
        
        self.timeline_drag_widget = DragWidget()
        self.timeline_drag_container_layout.addWidget(self.timeline_drag_widget)
        self.timeline_drag_container_layout.addStretch(1)
        #self.timeline_layout.addWidget(self.timeline_drag_widget)
        self.timeline_scroll_area = qt.QScrollArea()
        self.timeline_scroll_area.setWidgetResizable(True)
        self.timeline_scroll_area.setFixedHeight(300)
        self.timeline_scroll_area.setWidget(self.timeline_drag_container_widget)
        #self.timeline_scroll_area.setWidget(self.timeline_drag_widget)
        self.timeline_layout.addWidget(self.timeline_scroll_area)

        self.right_widget = qt.QWidget()
        self.right_layout = qt.QVBoxLayout(self.right_widget)
        
        self.timeline_add_button = AspectRatioButton('+')
        self.timeline_add_button.setStyleSheet(self.default_button_style)
        self.timeline_add_button.clicked.connect(self.add_timeline_item)
        self.right_layout.addWidget(self.timeline_add_button)
        self.right_layout.addStretch(1)

        self.timeline_save_button = qt.QPushButton('Save Mission')
        self.timeline_save_button.clicked.connect(self.save_timeline)
        self.right_layout.addWidget(self.timeline_save_button)
        
        self.timeline_load_button = qt.QPushButton('Load Mission')
        self.timeline_load_button.clicked.connect(self.load_timeline)
        self.right_layout.addWidget(self.timeline_load_button)
        
        self.timeline_clear_button = qt.QPushButton('Clear Mission')
        self.timeline_clear_button.clicked.connect(self.clear_timeline)
        self.right_layout.addWidget(self.timeline_clear_button)

        self.timeline_layout.addWidget(self.right_widget)
        
        self.vbox.addWidget(self.timeline_widget)

        self.timer = core.QTimer(self)
        self.timer.timeout.connect(self.play)

    def save_timeline(self):
        timeline_widgets = self.get_timeline_widgets()
        save_data = []
        for t in timeline_widgets:
            s = t.get_save_data()
            save_data.append(s)#t.get_save_data())
        filename = qt.QFileDialog.getSaveFileName(self.widget, 'Save Mission', '', 'Mission Files (*.mission)')[0]
        if filename == '':
            return
        if not filename.endswith('.mission'):
            filename += '.mission'
        with open(filename, 'wb') as handle:
            pickle.dump(save_data, handle, protocol=pickle.HIGHEST_PROTOCOL)

    def load_timeline(self):
        filename = qt.QFileDialog.getOpenFileName(self.widget, 'Load Mission', '', 'Mission Files (*.mission)')[0]
        if not os.path.isfile(filename):
            return
        self.clear_timeline()
        with open(filename, 'rb') as handle:
            save_data = pickle.load(handle)
        for s in save_data:
            item = DragItem(TimelineEventWidget(self.settings, s))
            item.setMinimumSize(260, 260)
            item.setMaximumSize(260, 260)
            self.timeline_drag_widget.add_item(item)

    def clear_timeline(self):
        timeline_widgets = self.get_timeline_widgets()
        for t in timeline_widgets:
            t.deleteLater()

    def get_timeline_widgets(self):
        timeline_widgets = []
        for i in range(self.timeline_drag_widget.blayout.count()):
            try:
                timeline_widgets.append(self.timeline_drag_widget.blayout.itemAt(i).widget().widget)
            except:
                pass
            
        return timeline_widgets

    def update_timeline_widgets(self):
        timeline_widgets = self.get_timeline_widgets()
        for tw in timeline_widgets:
            tw.set_done(tw.event.is_done())
    
    def play(self):
        timeline_widgets = self.get_timeline_widgets()
        if len(timeline_widgets) == 0:
            return
        self.timeline_play_button.setStyleSheet(self.green_button_style)
        self.timeline_pause_button.setStyleSheet(self.default_button_style)
        self.timeline_stop_button.setStyleSheet(self.default_button_style)
        for tw in timeline_widgets:
            if not tw.event.is_done():
                tw.event.play()
                break
        self.timer.start(100)
        self.update_timeline_widgets()
        self.timeline_drag_widget.setEnabled(False)
        self.timeline_add_button.setEnabled(False)

    def pause(self):
        self.timeline_play_button.setStyleSheet(self.default_button_style)
        self.timeline_pause_button.setStyleSheet(self.yellow_button_style)
        self.timeline_stop_button.setStyleSheet(self.default_button_style)
        self.timer.stop()
        timeline_widgets = self.get_timeline_widgets()
        for tw in timeline_widgets:
            tw.event.pause()
        self.update_timeline_widgets()

    def stop(self):
        self.timeline_play_button.setStyleSheet(self.default_button_style)
        self.timeline_pause_button.setStyleSheet(self.default_button_style)
        self.timeline_stop_button.setStyleSheet(self.red_button_style)
        self.timer.stop()
        timeline_widgets = self.get_timeline_widgets()
        for tw in timeline_widgets:
            tw.event.stop()
        self.update_timeline_widgets()
        self.timeline_drag_widget.setEnabled(True)
        self.timeline_add_button.setEnabled(True)

    def add_timeline_item(self):
        item = DragItem(TimelineEventWidget(self.settings))
        item.setMinimumSize(260, 260)
        item.setMaximumSize(260, 260)
        self.timeline_drag_widget.add_item(item)

    def select_config_file(self):
        starting_path = get_package_share_directory('rqt_ground_control_station') + '/config/'
        filename = qt.QFileDialog.getOpenFileName(self.widget, 'Open Config File', starting_path, "Config Files (*.yaml)")[0]
        print(filename)
        self.set_command_config(filename)

    def set_command_config(self, filename):
        if filename != '':
            self.settings['command_config_filename'] = filename
            if self.settings['command_config_filename'] != None:
                self.init_buttons(filename)

    def set_trajectory_config(self, filename):
        if filename != '':
            self.settings['trajectory_config_filename'] = filename

    def init_buttons(self, filename):
        y = yaml.load(open(filename, 'r').read(), Loader=yaml.Loader)
        #self.node.get_logger().info(str(y))
        
        for i in reversed(range(self.button_layout.count())): 
            self.button_layout.itemAt(i).widget().setParent(None)
        self.button_groups = {}

        def get_click_function(group, button):
            def click_function():
                commands = BehaviorTreeCommands()
                #self.node.get_logger().info(str(self.button_groups))
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
                #self.command_pub.publish(commands)
                self.settings['publishers'][self.robot_combo_box.currentText()]['command_pub'].publish(commands)
                #self.node.get_logger().info(str(self.settings['publishers'][self.robot_combo_box.currentText()]['command_pub'].topic_name))
            return click_function

        self.robot_combo_box.clear()
        for robot in y['robots']:
            self.robot_combo_box.addItem(robot)
            #self.robot_combo_box.model().item(0).setBackground(gui.QColor('red'))
            self.settings['robots'].append(robot)

        # init publishers
        for robot in self.settings['robots']:
            self.settings['publishers'][robot] = {'command_pub': 
                                                  self.node.create_publisher(BehaviorTreeCommands,
                                                                             '/' + robot + '/behavior/behavior_tree_commands', 1),
                                                  'trajectory_pub': 
                                                  self.node.create_publisher(FixedTrajectory,
                                                                             '/' + robot + '/fixed_trajectory_generator/fixed_trajectory_command', 1)}

        for group in y['groups']:
            group_name = list(group.keys())[0]
            if group_name not in self.button_groups.keys():
                self.button_groups[group_name] = {'buttons' : [], 'condition_names': []}
                self.settings['groups'][group_name] = {'condition_titles' : [], 'condition_names': []}

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
                
                self.settings['groups'][group_name]['condition_titles'].append(button_name)
                self.settings['groups'][group_name]['condition_names'].append(condition_name)

    def save_settings(self, plugin_settings, instance_settings):
        instance_settings.set_value('command_config_filename', self.settings['command_config_filename'])
        instance_settings.set_value('trajectory_config_filename', self.settings['trajectory_config_filename'])

    def restore_settings(self, plugin_settings, instance_settings):
        self.set_command_config(instance_settings.value('command_config_filename'))
        self.set_trajectory_config(instance_settings.value('trajectory_config_filename'))

    def trigger_configuration(self):
        sd = SettingsDialog(self.settings)
        if sd.exec():
            self.set_command_config(sd.command_config_filename)
            self.set_trajectory_config(sd.trajectory_config_filename)
        else:
            pass

    def shutdown_console_widget(self):
        pass

    def shutdown_plugin(self):
        self.shutdown_console_widget()

class SettingsDialog(qt.QDialog):
    def __init__(self, settings):
        super().__init__(None)
        self.setWindowTitle('Settings')
        layout = qt.QVBoxLayout()

        self.command_config_filename = settings['command_config_filename']
        self.trajectory_config_filename = settings['trajectory_config_filename']

        command_widget = qt.QWidget()
        command_layout = qt.QHBoxLayout(command_widget)
        
        command_config_button = qt.QPushButton('Open Config...')
        command_config_button.clicked.connect(self.select_command_config_file)
        command_layout.addWidget(command_config_button)

        self.command_config_label = qt.QLabel('Command Config File:' + os.path.basename(str(self.command_config_filename)))
        command_layout.addWidget(self.command_config_label)

        trajectory_widget = qt.QWidget()
        trajectory_layout = qt.QHBoxLayout(trajectory_widget)
        
        trajectory_config_button = qt.QPushButton('Open Config...')
        trajectory_layout.addWidget(trajectory_config_button)
        trajectory_config_button.clicked.connect(self.select_trajectory_config_file)

        self.trajectory_config_label = qt.QLabel('Trajectory Config File:' + os.path.basename(str(self.trajectory_config_filename)))
        trajectory_layout.addWidget(self.trajectory_config_label)

        self.buttons = qt.QDialogButtonBox(qt.QDialogButtonBox.Ok | qt.QDialogButtonBox.Cancel)
        self.buttons.accepted.connect(self.accept)
        self.buttons.rejected.connect(self.reject)
        
        layout.addWidget(command_widget)
        layout.addWidget(trajectory_widget)
        layout.addWidget(self.buttons)
        self.setLayout(layout)

    def select_command_config_file(self):
        starting_path = get_package_share_directory('rqt_ground_control_station') + '/config/'
        self.command_config_filename = \
            qt.QFileDialog.getOpenFileName(self, 'Open Config File', starting_path, 'Config Files (*.yaml)')[0]
        self.command_config_label.setText('Command Config File: ' + os.path.basename(self.command_config_filename))

    def select_trajectory_config_file(self):
        starting_path = get_package_share_directory('rqt_ground_control_station') + '/config/'
        self.trajectory_config_filename = \
            qt.QFileDialog.getOpenFileName(self, 'Open Config File', starting_path, 'Config Files (*.yaml)')[0]
        self.trajectory_config_label.setText('Trajectory Config File: ' + os.path.basename(self.trajectory_config_filename))
        
class TimelineEventWidget(QWidget):
    
    def __init__(self, global_settings, save_data=None):
        super().__init__()
        self.global_settings = global_settings
        self.event = None
        
        self.layout = qt.QVBoxLayout()
        self.setLayout(self.layout)

        self.top_widget = qt.QWidget()
        #self.top_widget.setObjectName('top')
        #self.top_widget.setStyleSheet('QWidget#top {background-color: red; border: 1px solid black;}')
        self.top_widget.setMaximumHeight(50)
        self.top_layout = qt.QHBoxLayout(self.top_widget)
        #self.top_layout.setSpacing(0)
        #self.top_layout.setContentsMargins(0, 0, 0, 0)
        
        self.task_combo_box = qt.QComboBox()
        self.task_combo_box.addItem('Wait', WaitEvent)
        self.task_combo_box.addItem('Command', CommandEvent)
        self.task_combo_box.addItem('Trajectory', TrajectoryEvent)
        self.top_layout.addWidget(self.task_combo_box)
        #self.layout.addWidget(self.task_combo_box)

        self.delete_button = qt.QPushButton('X')
        self.delete_button.clicked.connect(self.deleteLater)
        self.delete_button.setMaximumWidth(20)
        self.top_layout.addWidget(self.delete_button)
        self.layout.addWidget(self.top_widget)
        #self.layout.addStretch(1)

        self.content_widget = qt.QWidget()
        self.content_widget.setObjectName('content')
        self.content_layout = qt.QVBoxLayout()
        self.content_widget.setLayout(self.content_layout)
        self.layout.addWidget(self.content_widget)

        if save_data == None:
            self.task_combo_box.currentIndexChanged.connect(self.task_combo_box_change)
            self.task_combo_box_change(0)
        else:
            self.task_combo_box.setCurrentText(save_data['type'])
            self.clear_content()
            self.event = self.task_combo_box.itemData(self.task_combo_box.currentIndex())(self.global_settings,
                                                                                          save_data['local_settings'])
            self.event.init_widgets(self.content_layout)

    def get_save_data(self):
        dct = {'type': self.task_combo_box.currentText(), 'local_settings': {}}
        if self.event != None:
            dct['local_settings'] = self.event.local_settings
        return dct

    def set_done(self, b):
        if b:
            self.content_widget.setStyleSheet('QWidget#content {background-color: green; border: 1px solid black;}')
        else:
            self.content_widget.setStyleSheet('QWidget#content {background-color: lightcyan;}')
    
    def task_combo_box_change(self, s):
        self.clear_content()
        self.event = self.task_combo_box.itemData(s)(self.global_settings)
        self.event.init_widgets(self.content_layout)
    
    def clear_content(self):
        for i in reversed(range(self.content_layout.count())):
            self.content_layout.itemAt(i).widget().setParent(None)


class TimelineEvent:
    def __init__(self, global_settings, local_settings=None):
        self.global_settings = global_settings
        self.local_settings = local_settings
        if self.local_settings == None:
            self.local_settings = {}
        
        self.done = False

    def play(self):
        self.done = True

    def pause(self):
        pass

    def stop(self):
        self.done = False

    def init_widgets(self, parent_layout):
        pass

    def is_done(self):
        return self.done

class RobotEvent(TimelineEvent):
    def __init__(self, global_settings, local_settings=None):
        super(RobotEvent, self).__init__(global_settings, local_settings)
        if 'robot' not in self.local_settings.keys():
            self.local_settings['robot'] = self.global_settings['robots'][0]

    def init_widgets(self, parent_layout):
        super().init_widgets(parent_layout)

        widget = qt.QWidget()
        layout = qt.QHBoxLayout(widget)
        
        self.robot_label = qt.QLabel('Robot:')
        layout.addWidget(self.robot_label)
        
        self.robots_combo_box = qt.QComboBox()
        self.robots_combo_box.addItems(self.global_settings['robots'])
        self.robots_combo_box.currentIndexChanged.connect(self.robots_combo_box_change)
        self.robots_combo_box.setCurrentText(self.local_settings['robot'])
        layout.addWidget(self.robots_combo_box)

        parent_layout.addWidget(widget)

    def robots_combo_box_change(self, index):
        self.local_settings['robot'] = self.robots_combo_box.itemText(index)

class PublisherEvent(TimelineEvent):
    def __init__(self, global_settings, local_settings=None):
        super(PublisherEvent, self).__init__(global_settings, local_settings)

    def init_widgets(self, parent_layout):
        super().init_widgets(parent_layout)

    def get_publisher(self):
        return None

    def get_message(self):
        return None

    def play(self):
        self.done = True
        pub = self.get_publisher()
        msg = self.get_message()
        if pub == None or msg == None:
            return
        pub.publish(msg)

class CommandEvent(RobotEvent, PublisherEvent):
    def __init__(self, global_settings, local_settings=None):
        super(CommandEvent, self).__init__(global_settings, local_settings)
        if 'command_title' not in self.local_settings.keys():
            self.local_settings['command_title'] = self.global_settings['groups']['commands']['condition_titles'][0]
        if 'command_name' not in self.local_settings.keys():
            self.local_settings['command_name'] = self.global_settings['groups']['commands']['condition_names'][0]

    def get_publisher(self):
        if self.local_settings['robot'] in self.global_settings['publishers']:
            return self.global_settings['publishers'][self.local_settings['robot']]['command_pub']
        return None

    def get_message(self):
        commands = BehaviorTreeCommands()
        selected = self.local_settings['command_title']#self.commands_combo_box.currentText()
        for i in range(self.commands_combo_box.count()):
            command = BehaviorTreeCommand()
            command.condition_name = self.commands_combo_box.itemData(i)
            if selected == self.commands_combo_box.itemText(i):
                command.status = Status.SUCCESS
            else:
                command.status = Status.FAILURE
            commands.commands.append(command)
        return commands

    def init_widgets(self, parent_layout):
        super().init_widgets(parent_layout)
        
        self.commands_combo_box = qt.QComboBox()
        for i in range(len(self.global_settings['groups']['commands']['condition_titles'])):
            title = self.global_settings['groups']['commands']['condition_titles'][i]
            name = self.global_settings['groups']['commands']['condition_names'][i]
            self.commands_combo_box.addItem(title, name)
        self.commands_combo_box.currentIndexChanged.connect(self.commands_combo_box_change)
        self.commands_combo_box.setCurrentText(self.local_settings['command_title'])
        #self.commands_combo_box_change(self.global_settings['groups']['commands']['condition_titles'].index(self.local_settings['command_title']))
        parent_layout.addWidget(self.commands_combo_box)

    def commands_combo_box_change(self, index):
        self.local_settings['command_title'] = self.commands_combo_box.itemText(index)
        self.local_settings['command_name'] = self.commands_combo_box.itemData(index)

class TrajectoryEvent(RobotEvent, PublisherEvent):
    def __init__(self, global_settings, local_settings=None):
        super(TrajectoryEvent, self).__init__(global_settings, local_settings)
        if 'trajectory_attributes' not in self.local_settings.keys():
            self.local_settings['trajectory_attributes'] = collections.OrderedDict()

    def get_publisher(self):
        if self.local_settings['robot'] in self.global_settings['publishers']:
            return self.global_settings['publishers'][self.local_settings['robot']]['trajectory_pub']
        return None

    def get_message(self):
        trajectory_name = self.label.text().split(':')[-1].strip()
        if trajectory_name == 'None':
            return None
        msg = FixedTrajectory()
        msg.type = trajectory_name
        for attribute, value in iter(self.local_settings['trajectory_attributes'][trajectory_name].items()):
            key_value = KeyValue()
            key_value.key = attribute
            key_value.value = value
            msg.attributes.append(key_value)
        return msg

    def init_widgets(self, parent_layout):
        super().init_widgets(parent_layout)
        
        widget = qt.QWidget()
        layout = qt.QGridLayout(widget)

        traj_name = 'None'
        if 'trajectory_name' in self.local_settings['trajectory_attributes'].keys():
            traj_name = self.local_settings['trajectory_attributes']['trajectory_name']
        self.label = qt.QLabel('Trajectory: ' + traj_name)
        layout.addWidget(self.label, 1, 0)
        
        button = qt.QPushButton('Configure')
        def click(s):
            td = TrajectoryDialog(self.global_settings['trajectory_config_filename'],
                                  copy.deepcopy(self.local_settings['trajectory_attributes']))
            ret = td.exec()
            if ret:
                self.local_settings['trajectory_attributes'] = copy.deepcopy(td.attribute_settings)
                self.label.setText('Trajectory: ' + td.attribute_settings['trajectory_name'])
        button.clicked.connect(click)
        layout.addWidget(button, 2, 0)

        parent_layout.addWidget(widget)

class WaitEvent(TimelineEvent):
    def __init__(self, global_settings, local_settings=None):
        super(WaitEvent, self).__init__(global_settings, local_settings)
        if 'wait_time' not in self.local_settings.keys():
            self.local_settings['wait_time'] = 5.
        self.start_time = None
        self.paused_elapsed = 0.
        self.elapsed = 0.

    def init_widgets(self, parent_layout):
        super().init_widgets(parent_layout)

        widget = qt.QWidget()
        layout = qt.QHBoxLayout(widget)
        
        time_label = qt.QLabel('Time:')
        layout.addWidget(time_label)
        
        self.line_edit = qt.QLineEdit()
        self.line_edit.textChanged.connect(self.text_changed)
        self.line_edit.setText(str(self.local_settings['wait_time']))
        layout.addWidget(self.line_edit)
        
        s_label = qt.QLabel('s')
        layout.addWidget(s_label)

        self.elapsed_label = qt.QLabel('Elapsed: %0.1f / %0.1f s' % (0., self.local_settings['wait_time']))

        parent_layout.addWidget(widget)
        parent_layout.addWidget(self.elapsed_label)

    def text_changed(self, text):
        try:
            self.local_settings['wait_time'] = float(text)
            self.elapsed_label.setText('Elapsed: %0.1f / %0.1f s' % (0., self.local_settings['wait_time']))
        except:
            pass

    def play(self):
        if self.start_time == None or self.paused_elapsed != 0.:
            self.start_time = time.time() - self.paused_elapsed
            self.paused_elapsed = 0.
        self.elapsed = time.time() - self.start_time
        self.elapsed_label.setText('Elapsed: %0.1f / %0.1f s' % (self.elapsed, self.local_settings['wait_time']))

    def pause(self):
        if self.start_time != None:
            self.paused_elapsed = time.time() - self.start_time

    def stop(self):
        self.start_time = None
        self.paused_elapsed = 0.
        self.elapsed = 0.
        self.elapsed_label.setText('Elapsed: %0.1f / %0.1f s' % (self.elapsed, self.local_settings['wait_time']))

    def is_done(self):
        return (self.start_time != None) and (self.elapsed >= self.local_settings['wait_time'])
            

        
class AspectRatioButton(qt.QPushButton):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.aspect_ratio = 1.0

    def resizeEvent(self, event):
        size = event.size()
        if size.height() == 0 or size.width() / size.height() > self.aspect_ratio:
            size.setWidth(int(size.height() * self.aspect_ratio))
        else:
            size.setHeight(int(size.width() / self.aspect_ratio))
        self.resize(size)
