# (Yunwoo) RQT Keyboard Capture Widget for Keyboard Controller integration
# This widget captures keyboard input when focused and publishes to the keyboard_input topic

import os
from python_qt_binding.QtWidgets import QWidget, QVBoxLayout, QLabel, QPushButton
from python_qt_binding.QtCore import Qt
from python_qt_binding.QtGui import QKeyEvent

from rqt_gui_py.plugin import Plugin
from std_msgs.msg import String, Bool


class KeyboardCaptureWidget(Plugin):
    """
    RQT Plugin that captures keyboard input and publishes to ROS topic.
    Click 'Enable Capture' and then use WASD/QE/ZC keys to control the drone.
    """

    def __init__(self, context):
        super(KeyboardCaptureWidget, self).__init__(context)
        self.setObjectName('KeyboardCaptureWidget')

        self.context = context
        self.is_capturing = False

        # Get robot name from environment variable
        robot_name = os.environ.get('ROBOT_NAME', 'robot')
        
        # Publishers - publish to robot-namespaced topics
        self.keyboard_pub = self.context.node.create_publisher(String, f'/{robot_name}/keyboard_input', 10)
        
        # Main widget
        self.widget = QWidget()
        self.widget.setFocusPolicy(Qt.StrongFocus)
        layout = QVBoxLayout()
        self.widget.setLayout(layout)

        # Title
        title = QLabel('Keyboard Controller')
        title.setStyleSheet('font-weight: bold; font-size: 14px;')
        layout.addWidget(title)

        # Instructions
        instructions = QLabel(
            'Controls:\n'
            '  W/S - Forward/Backward\n'
            '  A/D - Left/Right\n'
            '  Z/C - Down/Up\n'
            '  Q/E - Yaw Left/Right\n'
            '  O/P - Decrease/Increase linear step\n'
            '  K/L - Decrease/Increase yaw step'
        )
        instructions.setStyleSheet('font-family: monospace;')
        layout.addWidget(instructions)

        # Status label
        self.status_label = QLabel('Status: Click widget and press keys')
        self.status_label.setStyleSheet('color: gray;')
        layout.addWidget(self.status_label)

        # Last key label
        self.key_label = QLabel('Last key: -')
        layout.addWidget(self.key_label)

        # Enable focus on click
        self.widget.mousePressEvent = self.on_mouse_press
        self.widget.keyPressEvent = self.on_key_press
        self.widget.focusInEvent = self.on_focus_in
        self.widget.focusOutEvent = self.on_focus_out

        context.add_widget(self.widget)

    def on_mouse_press(self, event):
        self.widget.setFocus()

    def on_focus_in(self, event):
        self.status_label.setText('Status: ACTIVE - Capturing keyboard')
        self.status_label.setStyleSheet('color: green; font-weight: bold;')

    def on_focus_out(self, event):
        self.status_label.setText('Status: Click to activate')
        self.status_label.setStyleSheet('color: gray;')

    def on_key_press(self, event: QKeyEvent):
        key = event.key()
        
        # Map Qt keys to characters
        key_map = {
            Qt.Key_W: 'w',
            Qt.Key_S: 's',
            Qt.Key_A: 'a',
            Qt.Key_D: 'd',
            Qt.Key_Z: 'z',
            Qt.Key_C: 'c',
            Qt.Key_Q: 'q',
            Qt.Key_E: 'e',
            Qt.Key_O: 'o',
            Qt.Key_P: 'p',
            Qt.Key_K: 'k',
            Qt.Key_L: 'l',
        }

        if key in key_map:
            char = key_map[key]
            self.key_label.setText(f'Last key: {char.upper()}')
            
            # Publish to ROS
            msg = String()
            msg.data = char
            self.keyboard_pub.publish(msg)

    def shutdown_plugin(self):
        pass

    def save_settings(self, plugin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        pass
