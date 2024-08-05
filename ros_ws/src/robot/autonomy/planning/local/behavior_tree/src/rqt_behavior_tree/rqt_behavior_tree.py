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

from behavior_tree import behavior_tree as bt
from behavior_tree import behavior_tree_graphviz as gv

#import graphviz
#import cv2
from threading import Lock
from xdot.xdot_qt import DotWidget

class BehaviorTreePlugin(Plugin):
    def __init__(self, context):
        super(BehaviorTreePlugin, self).__init__(context)
        self.setObjectName('BehaviorTreePlugin')

        self.tree = None

        self.initialized_buttons = False
        self.prev_graphviz = ''
        
        self.behavior_tree_graphviz_sub = rospy.Subscriber('behavior_tree_graphviz', String, self.behavior_tree_graphviz_callback)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)
        self.functions_mutex = Lock()
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
        self.config_button.clicked.connect(self.select_config_file)
        self.config_layout.addWidget(self.config_button)

        self.tree_label = qt.QLabel('tree filename: ')
        self.config_layout.addWidget(self.tree_label)

        self.debug_checkbox = qt.QCheckBox('Debug Mode')
        self.config_layout.addWidget(self.debug_checkbox)
        self.debug_checkbox.stateChanged.connect(self.debug_mode_changed)

        #self.config_widget.setStyleSheet("background-color: rgb(255, 0, 0);")
        self.top_layout.addWidget(self.config_widget)

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

    def debug_mode_changed(self, state):
        if state == Qt.Checked:
            self.button_container_widget.show()
        else:
            self.button_container_widget.hide()
        
    def select_config_file(self):
        starting_path = os.path.join(rospkg.RosPack().get_path('behavior_tree'), 'config')
        filename = qt.QFileDialog.getOpenFileName(self.widget, 'Open Config File', starting_path, "Config Files (*.tree)")[0]
        self.set_config(filename)

    def set_config(self, filename):
        if filename != '':
            self.config_filename = filename
            if self.config_filename != None:
                self.tree_label.setText('tree filename: ' + self.config_filename)
                self.tree = bt.BehaviorTree(self.config_filename)
            self.init_buttons()

    def timer_callback(self, event):
        funcs = []
        
        self.functions_mutex.acquire()
        for node_label, function in self.functions.iteritems():
            funcs.append(function)
        self.functions_mutex.release()

        for func in funcs:
            func()
    
    def init_buttons(self):
        added_node_labels = []
        
        if self.tree != None:
            for node in self.tree.nodes:
                if node.label in added_node_labels:
                    continue
                added_node_labels.append(node.label)
                
                def get_publish_function(widget, button, other_buttons, node, message_type, message_data):
                    pub = rospy.Publisher(node.get_subscriber_name(), message_type, queue_size=1)
                    def add_publish_function():
                        def publish_function():
                            if message_data != 'NO MESSAGE':
                                msg = message_type()
                                if isinstance(msg, Bool):
                                    msg.data = message_data
                                elif isinstance(msg, Status):
                                    msg.status = message_data
                                pub.publish(msg)
                                #print(node.label + ' publishing to ' + node.get_subscriber_name() + ' ' + str(message_data))
                        self.functions_mutex.acquire()
                        self.functions[node.label] = publish_function
                        self.functions_mutex.release()
                        if message_data == Status.SUCCESS or message_data == True:
                            widget.setStyleSheet("background-color: rgb(0, 255, 0);")
                        elif message_data == Status.RUNNING:
                            widget.setStyleSheet("background-color: rgb(0, 0, 255);")
                        elif message_data == Status.FAILURE or message_data == False:
                            widget.setStyleSheet("background-color: rgb(255, 0, 0);")
                        elif message_data == 'NO MESSAGE':
                            widget.setStyleSheet("background-color: rgb(255, 255, 255);")
                        button.setStyleSheet("background-color: rgb(0, 255, 0);")
                        for b in other_buttons:
                            b.setStyleSheet("background-color: rgb(255, 0, 0);")
                    return add_publish_function
                
                if isinstance(node, bt.Action):
                    action_widget = qt.QWidget()
                    action_layout = qt.QHBoxLayout()
                    action_widget.setLayout(action_layout)
                    self.action_layout.addWidget(action_widget)

                    
                    
                    label = qt.QLabel()
                    label.setText(node.label)
                    action_layout.addWidget(label)
                    success_button = qt.QPushButton('SUCCESS')
                    success_button.setStyleSheet("background-color: rgb(255, 0, 0);")
                    action_layout.addWidget(success_button)
                    running_button = qt.QPushButton('RUNNING')
                    running_button.setStyleSheet("background-color: rgb(255, 0, 0);")
                    action_layout.addWidget(running_button)
                    failure_button = qt.QPushButton('FAILURE')
                    failure_button.setStyleSheet("background-color: rgb(255, 0, 0);")
                    action_layout.addWidget(failure_button)
                    no_message_button = qt.QPushButton('NO MESSAGE')
                    no_message_button.setStyleSheet("background-color: rgb(0, 255, 0);")
                    action_layout.addWidget(no_message_button)
                    
                    success_button.clicked.connect(get_publish_function(action_widget, success_button, [running_button, failure_button, no_message_button], node, Status, Status.SUCCESS))
                    running_button.clicked.connect(get_publish_function(action_widget, running_button, [success_button, failure_button, no_message_button], node, Status, Status.RUNNING))
                    failure_button.clicked.connect(get_publish_function(action_widget, failure_button, [success_button, running_button, no_message_button], node, Status, Status.FAILURE))
                    no_message_button.clicked.connect(get_publish_function(action_widget, no_message_button, [success_button, running_button, failure_button], node, Status, 'NO MESSAGE'))
                elif isinstance(node, bt.Condition):
                    condition_widget = qt.QWidget()
                    condition_layout = qt.QHBoxLayout()
                    condition_widget.setLayout(condition_layout)
                    self.condition_layout.addWidget(condition_widget)
                    
                    label = qt.QLabel()
                    label.setText(node.label)
                    condition_layout.addWidget(label)
                    success_button = qt.QPushButton('SUCCESS')
                    success_button.setStyleSheet("background-color: rgb(255, 0, 0);")
                    condition_layout.addWidget(success_button)
                    failure_button = qt.QPushButton('FAILURE')
                    failure_button.setStyleSheet("background-color: rgb(255, 0, 0);")
                    condition_layout.addWidget(failure_button)
                    no_message_button = qt.QPushButton('NO MESSAGE')
                    no_message_button.setStyleSheet("background-color: rgb(0, 255, 0);")
                    condition_layout.addWidget(no_message_button)
                    
                    success_button.clicked.connect(get_publish_function(condition_widget, success_button, [failure_button, no_message_button], node, Bool, True))
                    failure_button.clicked.connect(get_publish_function(condition_widget, failure_button, [success_button, no_message_button], node, Bool, False))
                    no_message_button.clicked.connect(get_publish_function(condition_widget, no_message_button, [success_button, failure_button], node, Bool, 'NO MESSAGE'))
            

    def behavior_tree_graphviz_callback(self, msg):
        if msg.data != self.prev_graphviz:
            self.xdot_widget.set_dotcode(msg.data)
        self.prev_graphviz = msg.data
        '''
        if msg.data == self.last_graphviz_string:
            return
        self.last_graphviz_string = msg.data
        gv.get_graphviz_image(msg.data, 'rqt_temp')
        self.pixmap = gui.QPixmap('rqt_temp.png')
        self.image_label.setPixmap(self.pixmap)
        '''
        
    
    def shutdown_plugin(self):
        pass

    def save_settings(self, plugin_settings, instance_settings):
        instance_settings.set_value('config_filename', self.config_filename)

    def restore_settings(self, plugin_settings, instance_settings):
        self.set_config(instance_settings.value('config_filename'))

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog

