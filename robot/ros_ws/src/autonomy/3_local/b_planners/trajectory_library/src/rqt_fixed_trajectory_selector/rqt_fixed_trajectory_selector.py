import os
import time
import rospy
import rospkg
from std_msgs.msg import String, Bool
from behavior_tree_msgs.msg import Status
import numpy as np
import yaml
import collections

from qt_gui.plugin import Plugin
import python_qt_binding.QtWidgets as qt
import python_qt_binding.QtCore as core
import python_qt_binding.QtGui as gui

from python_qt_binding import QT_BINDING, QT_BINDING_VERSION

from python_qt_binding.QtCore import Slot, Qt, qVersion, qWarning, Signal
from python_qt_binding.QtGui import QColor
from python_qt_binding.QtWidgets import QWidget, QVBoxLayout, QSizePolicy

from airstack_msgs.msg import FixedTrajectory
from diagnostic_msgs.msg import KeyValue

class FixedTrajectorySelectorPlugin(Plugin):
    def __init__(self, context):
        super(FixedTrajectorySelectorPlugin, self).__init__(context)
        self.setObjectName('FixedTrajectorySelectorPlugin')

        self.config_filename = ''

        self.button_dct = {}
        self.attribute_settings = collections.OrderedDict()

        self.timer = rospy.Timer(rospy.Duration(1./10.), self.timer_callback)

        self.fixed_trajectory_pub = rospy.Publisher('fixed_trajectory_command', FixedTrajectory, queue_size=1)
        self.global_plan_fixed_trajectory_pub = rospy.Publisher('global_plan_fixed_trajectory', FixedTrajectory, queue_size=1)

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
        for attribute, value in self.attribute_settings[trajectory_name].iteritems():
            key_value = KeyValue()
            key_value.key = attribute
            key_value.value = value
            msg.attributes.append(key_value)
        if trajectory_type == 'Fixed Trajectory':
            self.fixed_trajectory_pub.publish(msg)
        elif trajectory_type == 'Global Plan':
            self.global_plan_fixed_trajectory_pub.publish(msg)
        

    def select_config_file(self):
        starting_path = os.path.join(rospkg.RosPack().get_path('trajectory_library'), 'config')
        filename = qt.QFileDialog.getOpenFileName(self.widget, 'Open Config File', starting_path, "Config Files (*.yaml)")[0]
        self.set_config(filename)

    def set_config(self, filename):
        if filename != '':
            self.config_filename = filename
            if self.config_filename != None:
                self.config_label.setText('config filename: ' + self.config_filename)
                self.init_buttons(filename)

    def init_buttons(self, filename):
        y = yaml.load(open(filename, 'r').read())
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
                for attribute, value in self.attribute_settings[trajectory_name].iteritems():
                    key_value = KeyValue()
                    key_value.key = attribute
                    key_value.value = value
                    msg.attributes.append(key_value)
                self.fixed_trajectory_pub.publish(msg)
            return publish_function
                

        for trajectory in y['trajectories']:
            trajectory_name = trajectory.keys()[0]
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
            
            
        
    def timer_callback(self, msg):
        bool_msg = Bool()
        for key in self.button_dct.keys():
            bool_msg.data = self.button_dct[key]['data']
            self.button_dct[key]['publisher'].publish(bool_msg)
        
    def shutdown_plugin(self):
        pass

    def save_settings(self, plugin_settings, instance_settings):
        instance_settings.set_value('config_filename', self.config_filename)
        instance_settings.set_value('attribute_settings', self.attribute_settings)

    def restore_settings(self, plugin_settings, instance_settings):
        attribute_settings = instance_settings.value('attribute_settings')
        if attribute_settings != None:
            self.attribute_settings = attribute_settings
        self.set_config(instance_settings.value('config_filename'))

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog

