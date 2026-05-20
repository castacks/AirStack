from python_qt_binding.QtWidgets import QVBoxLayout, QWidget
from rqt_gui_py.plugin import Plugin
from qt_gui_py_common.simple_settings_dialog import SimpleSettingsDialog
from rqt_py_console.py_console_widget import PyConsoleWidget

import python_qt_binding.QtWidgets as qt
import python_qt_binding.QtWidgets as QtWidgets
import python_qt_binding.QtGui as gui
import python_qt_binding.QtCore as QtCore

from ament_index_python.packages import get_package_share_directory
import yaml
import os
import collections


class TrajectoryDialog(qt.QDialog):
    '''
    def __init__(self, parent=None):
        super().__init__(parent)

        self.setWindowTitle("HELLO!")

        QBtn = qt.QDialogButtonBox.Ok | qt.QDialogButtonBox.Cancel

        self.buttonBox = qt.QDialogButtonBox(QBtn)
        self.buttonBox.accepted.connect(self.accept)
        self.buttonBox.rejected.connect(self.reject)

        layout = qt.QVBoxLayout()
        message = qt.QLabel("Something happened, is that OK?")
        layout.addWidget(message)
        layout.addWidget(self.buttonBox)
        self.setLayout(layout)
    '''
    def __init__(self, trajectory_config_filename, default_attribute_settings):
        super().__init__(None)

        self.config_filename = ''

        self.button_dct = {}
        self.attribute_settings = default_attribute_settings

        # main layout
        self.vbox = qt.QVBoxLayout()
        self.setLayout(self.vbox)

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

        # ok/cancel buttons
        self.buttons = qt.QDialogButtonBox(qt.QDialogButtonBox.Ok | qt.QDialogButtonBox.Cancel)
        self.buttons.accepted.connect(self.accept)
        self.buttons.rejected.connect(self.reject)
        self.vbox.addWidget(self.buttons)

        self.set_config(trajectory_config_filename)
        self.tab_widget.currentChanged.connect(self.on_tab_changed)

    def on_tab_changed(self, index):
        self.attribute_settings['tab_index'] = index
        self.attribute_settings['trajectory_name'] = self.tab_widget.tabText(index)

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
                self.init_buttons(filename)
                if 'trajectory_name' not in self.attribute_settings:
                    self.on_tab_changed(0)

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
                    attribute_default = 'base_link'
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
        if 'tab_index' in self.attribute_settings:
            print(self.attribute_settings['tab_index'])
            self.tab_widget.setCurrentIndex(self.attribute_settings['tab_index'])
