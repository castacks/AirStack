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

import subprocess

try:
    from rqt_py_console.spyder_console_widget import SpyderConsoleWidget
    _has_spyderlib = True
except ImportError:
    _has_spyderlib = False


class InfoConfigDialog(qt.QDialog):
    def __init__(self, settings):
        super().__init__()
        
        self.setWindowTitle('Configuration')
        layout = qt.QVBoxLayout()

        self.name_label = qt.QLabel('Name:')
        layout.addWidget(self.name_label)
        self.name_entry = qt.QLineEdit()
        self.name_entry.setText(settings['name'])
        layout.addWidget(self.name_entry)
        
        self.username_label = qt.QLabel('Username:')
        layout.addWidget(self.username_label)
        self.username_entry = qt.QLineEdit()
        self.username_entry.setText(settings['username'])
        layout.addWidget(self.username_entry)
        
        self.hostname_label = qt.QLabel('Hostname:')
        layout.addWidget(self.hostname_label)
        self.hostname_entry = qt.QLineEdit()
        self.hostname_entry.setText(settings['hostname'])
        layout.addWidget(self.hostname_entry)
        
        self.namespace_label = qt.QLabel('Namespace:')
        layout.addWidget(self.namespace_label)
        self.namespace_entry = qt.QLineEdit()
        self.namespace_entry.setText(settings['namespace'])
        layout.addWidget(self.namespace_entry)
        
        self.path_label = qt.QLabel('Docker Compose Path:')
        layout.addWidget(self.path_label)
        self.path_entry = qt.QLineEdit()
        self.path_entry.setText(settings['path'])
        layout.addWidget(self.path_entry)
        
        self.services_label = qt.QLabel('Services to Hide (comma separated):')
        layout.addWidget(self.services_label)
        self.services_entry = qt.QLineEdit()
        self.services_entry.setText(', '.join(settings['excluded_services']))
        layout.addWidget(self.services_entry)
        
        self.submit_button = qt.QPushButton('Submit')
        self.submit_button.clicked.connect(self.submit)
        layout.addWidget(self.submit_button)
        
        self.setLayout(layout)
        
        self.result = None
    
    def submit(self):
        name = self.name_entry.text()
        hostname = self.hostname_entry.text()
        username = self.username_entry.text()
        namespace = self.namespace_entry.text()
        path = self.path_entry.text()
        services = list(map(lambda x:x.strip(), self.services_entry.text().split(',')))
        services.remove('')
        self.result = {'name': name, 'username': username, 'hostname': hostname, 'namespace': namespace, 'path': path, 'excluded_services': services}
        self.accept()

class CommandThread(QtCore.QThread):
    output_signal = QtCore.pyqtSignal(str)
    
    def __init__(self, command):
        super().__init__()
        self.command = command
        self.running = True

    def run(self):
        process = subprocess.Popen(self.command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

        while self.running:
            line = process.stdout.readline()
            if not line:
                break
            self.output_signal.emit(line.strip())

        process.terminate()

    def stop(self):
        self.running = False
        self.terminate()


class InfoWidget(qt.QWidget):
    def __init__(self, settings={'name': 'Name', 'username': 'airlab', 'hostname': 'localhost', 'namespace': 'none', 'path': '~/airstack', 'excluded_services': []}):
        super(qt.QWidget, self).__init__()
        self.layout = qt.QVBoxLayout(self)

        self.settings = settings

        self.ping_thread = None

        # info panel
        self.info_widget = qt.QWidget()
        self.info_layout = qt.QHBoxLayout(self.info_widget)
        self.layout.addWidget(self.info_widget)

        self.name_label = qt.QLabel(self.settings['name'])
        self.info_layout.addWidget(self.name_label)

        self.hostname_label = qt.QLabel('Hostname: ' + self.settings['hostname'])
        self.info_layout.addWidget(self.hostname_label)
        
        self.ping_label = qt.QLabel('Ping:')
        self.info_layout.addWidget(self.ping_label)
        
        self.recording_label = qt.QLabel('Recording:')
        self.info_layout.addWidget(self.recording_label)

        # command panel
        self.command_widget = qt.QWidget()
        self.command_layout = qt.QHBoxLayout(self.command_widget)
        self.layout.addWidget(self.command_widget)

        self.refresh_button = qt.QPushButton(text='Refresh')
        self.refresh_button.clicked.connect(self.refresh_docker)
        self.command_layout.addWidget(self.refresh_button)

        self.restart_button = qt.QPushButton(text='Restart')
        self.command_layout.addWidget(self.restart_button)

        self.up_button = qt.QPushButton(text='Up')
        self.command_layout.addWidget(self.up_button)

        self.down_button = qt.QPushButton(text='Down')
        self.command_layout.addWidget(self.down_button)
        
        self.record_button = qt.QPushButton(text='Record')
        self.command_layout.addWidget(self.record_button)

        self.config_button = qt.QPushButton(text='Config')
        self.config_button.clicked.connect(self.config_clicked)
        self.command_layout.addStretch(1)
        self.command_layout.addWidget(self.config_button)

        self.delete_button = qt.QPushButton(text='X')
        self.delete_button.clicked.connect(self.delete_clicked)
        self.command_layout.addWidget(self.delete_button)

        # docker panel
        self.docker_widget = qt.QWidget()
        self.docker_layout = qt.QHBoxLayout(self.docker_widget)
        self.layout.addWidget(self.docker_widget)

        self.update_info()

    def config_clicked(self):
        dialog = InfoConfigDialog(self.settings)
        if dialog.exec():
            self.settings = dialog.result
            self.update_info()

    def delete_clicked(self):
        self.delete_function()

    def update_info(self):
        self.name_label.setText(self.settings['name'])
        self.hostname_label.setText('Hostname: ' + self.settings['hostname'])

        if self.ping_thread != None:
            self.ping_thread.stop()
            del self.ping_thread
            self.ping_thread = None
        self.ping_thread = CommandThread(['ping', self.settings['hostname']])
        self.ping_thread.start()
        self.ping_thread.output_signal.connect(self.handle_ping)

    def refresh_docker(self):
        command = ''
        if 'localhost' not in self.settings['hostname']:
            command = 'ssh -t -o StrictHostKeyChecking=no', self.settings['username'] + '@' + self.settings['hostname']
        command += 'cd ' + self.settings['path'] + '; docker compose config --services'
        
        result = subprocess.run(command, capture_output=True, text=True, check=True, shell=True)

    def handle_ping(self, text):
        if 'time=' in text:
            ping = text.split('time=')[1]
        else:
            ping = 'can\'t ping'
        
        self.ping_label.setText('Ping: ' + ping)

    def get_dct(self):
        return self.settings
        
    
class AirstackControlPanel(Plugin):
    def __init__(self, context):
        super(AirstackControlPanel, self).__init__(context)
        self.setObjectName('AirstackControlPanel')

        self.context = context
        # to access ros2 node use self.context.node

        self.info_widgets = []

        self.widget = qt.QWidget()
        self.layout = qt.QVBoxLayout(self.widget)

        self.info_widget = qt.QWidget()
        self.info_layout = qt.QVBoxLayout(self.info_widget)
        self.layout.addWidget(self.info_widget)

        self.add_button = qt.QPushButton('+')
        self.add_button.clicked.connect(lambda x:self.add_info_widget())
        self.layout.addWidget(self.add_button)
        self.layout.addStretch(1)
        
        self.context.add_widget(self.widget)

    def add_info_widget(self, settings=None):
        if settings == None:
            info_widget = InfoWidget()
        else:
            info_widget = InfoWidget(settings)
        def get_delete_function(i):
            def delete_function():
                self.info_layout.removeWidget(self.info_widgets[i])
                self.info_widgets[i] = None
            return delete_function
        info_widget.delete_function = get_delete_function(len(self.info_widgets))
        self.info_layout.addWidget(info_widget)
        self.info_widgets.append(info_widget)

    def remove_info_widget(self, index):
        del self.info_widgets[index]
        self.info_widgets[index] = None

    def save_settings(self, plugin_settings, instance_settings):
        info_dcts = [w.get_dct() for w in self.info_widgets if w != None]
        instance_settings.set_value('info_dcts', info_dcts)

    def restore_settings(self, plugin_settings, instance_settings):
        info_dcts = instance_settings.value('info_dcts', {})
        for settings in info_dcts:
            self.add_info_widget(settings)

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
