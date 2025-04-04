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
import threading
import time
import copy

from std_msgs.msg import Bool

try:
    from rqt_py_console.spyder_console_widget import SpyderConsoleWidget
    _has_spyderlib = True
except ImportError:
    _has_spyderlib = False


logger = None

def xor_encrypt_decrypt(data, key=983476):
    return ''.join(chr(ord(c) ^ key) for c in data)
    
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

        self.password_label = qt.QLabel('Password:')
        layout.addWidget(self.password_label)
        self.password_entry = qt.QLineEdit()
        self.password_entry.setEchoMode(qt.QLineEdit.Password)
        self.password_entry.setText(settings['password'])
        layout.addWidget(self.password_entry)
        
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

        self.enable_display_checkbox = qt.QCheckBox('Enable Display')
        self.enable_display_checkbox.setChecked(settings['enable_display'])
        layout.addWidget(self.enable_display_checkbox)
        
        self.submit_button = qt.QPushButton('Submit')
        self.submit_button.clicked.connect(self.submit)
        layout.addWidget(self.submit_button)
        
        self.setLayout(layout)
        
        self.result = None
    
    def submit(self):
        name = self.name_entry.text()
        hostname = self.hostname_entry.text()
        username = self.username_entry.text()
        password = self.password_entry.text()
        namespace = self.namespace_entry.text()
        path = self.path_entry.text()
        services = list(map(lambda x:x.strip(), self.services_entry.text().split(',')))
        try:
            services.remove('')
        except:
            pass
        enable_display = self.enable_display_checkbox.isChecked()
        self.result = {'name': name, 'username': username, 'password': password, 'hostname': hostname, 'namespace': namespace, 'path': path, 'excluded_services': services, 'enable_display': enable_display}
        self.accept()

class CommandThread(QtCore.QThread):
    output_signal = QtCore.pyqtSignal(str)
    
    def __init__(self, command, callback=None, wait_until_finished=False, timeout=None):
        super().__init__()
        self.command = command
        if callback != None:
            self.output_signal.connect(callback)
        self.wait_until_finished = wait_until_finished
        self.timeout = timeout
        self.running = True
        self.start()

    def run(self):
        process = subprocess.Popen(self.command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, shell=True)
        
        if not self.wait_until_finished:
            while self.running:
                line = process.stdout.readline()
                if not line:
                    break
                self.output_signal.emit(line.strip())
        if self.wait_until_finished:
            try:
                process.wait(timeout=self.timeout)
                self.output_signal.emit(process.stdout.read())
            except subprocess.TimeoutExpired:
                process.kill()

        process.terminate()

    def stop(self):
        self.running = False
        self.terminate()

class InfoWidget(qt.QWidget):
    def __init__(self, node, settings={'name': 'Name', 'username': 'airlab', 'password': 'passme24', 'hostname': 'localhost', 'namespace': 'none', 'path': '~/airstack', 'excluded_services': [], 'enable_display': False}):
        super(qt.QWidget, self).__init__()
        self.layout = qt.QVBoxLayout(self)
        #self.setObjectName('info_widget')
        #self.setStyleSheet('#info_widget {border: 2px solid black;}')

        self.node = node
        self.recording_sub = None
        self.recording_pub = None
        
        self.setAttribute(QtCore.Qt.WA_StyledBackground, True)
        self.setObjectName('info_widget')
        self.stylesheet = 'border: 2px solid lightgrey;  border-radius: 10px;'
        self.setStyleSheet('QWidget#info_widget { ' + self.stylesheet + ' }')
 
        self.settings = settings

        self.ping_thread = None
        self.connected = False
        self.services = {}

        # info panel
        self.info_widget = qt.QWidget()
        self.info_layout = qt.QHBoxLayout(self.info_widget)
        self.layout.addWidget(self.info_widget)

        self.name_label = qt.QLabel(self.settings['name'])
        self.name_label.setStyleSheet('font-weight: bold;')
        self.info_layout.addWidget(self.name_label)

        self.hostname_label = qt.QLabel('Hostname: ' + self.settings['hostname'])
        self.info_layout.addWidget(self.hostname_label)
        
        self.ping_label = qt.QLabel('Ping:')
        self.info_layout.addWidget(self.ping_label)
        
        self.recording_label = qt.QLabel('Recording:')
        self.info_layout.addWidget(self.recording_label)

        line = qt.QFrame()
        line.setFrameShape(qt.QFrame.HLine)
        line.setFrameShadow(qt.QFrame.Plain)
        line.setStyleSheet('QFrame {background-color: #cccccc; max-height: 1px; border: none; }')
        self.layout.addWidget(line)

        # command panel
        self.command_widget = qt.QWidget()
        self.command_layout = qt.QHBoxLayout(self.command_widget)
        self.layout.addWidget(self.command_widget)

        self.refresh_button = qt.QPushButton(text='Refresh')
        self.refresh_button.clicked.connect(self.refresh_docker)
        self.command_layout.addWidget(self.refresh_button)

        self.restart_button = qt.QPushButton(text='Restart')
        self.restart_button.clicked.connect(lambda :self.docker_command('restart'))
        self.command_layout.addWidget(self.restart_button)

        self.up_button = qt.QPushButton(text='Up')
        self.up_button.clicked.connect(lambda :self.docker_command('up'))
        self.command_layout.addWidget(self.up_button)

        self.down_button = qt.QPushButton(text='Down')
        self.down_button.clicked.connect(lambda :self.docker_command('down'))
        self.command_layout.addWidget(self.down_button)
        
        self.record_button = qt.QPushButton(text='Record')
        self.record_button.setCheckable(True)
        self.record_button.clicked.connect(self.record)
        self.command_layout.addWidget(self.record_button)
        
        self.ssh_button = qt.QPushButton(text='ssh')
        self.ssh_button.clicked.connect(self.ssh)
        self.command_layout.addWidget(self.ssh_button)

        self.config_button = qt.QPushButton(text='Config')
        self.config_button.clicked.connect(self.config_clicked)
        self.command_layout.addStretch(1)
        self.command_layout.addWidget(self.config_button)

        self.delete_button = qt.QPushButton(text='X')
        self.delete_button.clicked.connect(self.delete_clicked)
        self.command_layout.addWidget(self.delete_button)
        
        line = qt.QFrame()
        line.setFrameShape(qt.QFrame.HLine)
        line.setFrameShadow(qt.QFrame.Plain)
        line.setStyleSheet('QFrame {background-color: #cccccc; max-height: 1px; border: none; }')
        self.layout.addWidget(line)

        # docker panel
        self.docker_widget = qt.QWidget()
        self.docker_layout = qt.QHBoxLayout(self.docker_widget)
        self.layout.addWidget(self.docker_widget)

        self.update_info()
        #self.refresh_docker()

    def config_clicked(self):
        dialog = InfoConfigDialog(self.settings)
        if dialog.exec():
            self.settings = dialog.result
            self.update_info()

    def delete_clicked(self):
        self.delete_function()

    def record(self):
        if self.recording_pub != None:
            msg = Bool()
            msg.data = self.record_button.isChecked()
            self.recording_pub.publish(msg)
            

    def update_info(self):
        self.name_label.setText(self.settings['name'])
        self.hostname_label.setText('Hostname: ' + self.settings['hostname'])

        if self.ping_thread != None:
            self.ping_thread.stop()
            del self.ping_thread
            self.ping_thread = None
        #self.ping_thread = CommandThread('ping ' + self.settings['hostname'])
        self.ping_thread = CommandThread('HOST="' + self.settings['hostname'] + '"; while true; do OUTPUT=$(ping -c 1 -w 3 $HOST 2>&1); if echo "$OUTPUT" | grep -q "time="; then PING_TIME=$(echo "$OUTPUT" | grep -oP "time=\K[\d.]+"); echo "$PING_TIME ms"; else echo "failed"; fi; sleep 1; done', self.handle_ping)

        if self.recording_sub != None:
            self.node.destroy_subscription(self.recording_sub)
        self.recording_sub = self.node.create_subscription(Bool, self.settings['namespace'] + '/bag_record/bag_recording_status',
                                                           self.recording_callback, 1)
        self.recording_pub = self.node.create_publisher(Bool, self.settings['namespace'] + '/bag_record/set_recording_status', 1)

    def recording_callback(self, msg):
        if msg.data:
            self.recording_label.setText('Recording: YES')
        else:
            self.recording_label.setText('Recording: NO')

    def ssh_t(self, command):
        ssh = 'ssh -t -o StrictHostKeyChecking=no'
        if self.settings['enable_display']:
            ssh += ' -X'

        display = ''
        if self.settings['enable_display']:
            display = 'export DISPLAY=:1; '
            
        return 'sshpass -p "' + self.settings['password'] + '" ' + ssh + ' ' + \
            self.settings['username'] + '@' + self.settings['hostname'] + ' "' + display + command + '"'

    def refresh_docker(self):
        command = self.ssh_t('cd ' + self.settings['path'] + ';  docker compose --profile \'*\' config --services && echo SPLIT && docker compose ps --format {{.Service}}')
        
        #result = subprocess.run(command, capture_output=True, text=True, check=True, shell=True)

        self.refresh_thread = CommandThread(command, self.handle_refresh_docker, True, 5)

    def handle_refresh_docker(self, stdout):
        if 'SPLIT' not in stdout:
            return
        
        services = sorted(stdout.split('SPLIT')[0].split('\n'))
        running = stdout.split('SPLIT')[1].split('\n')
        
        for i in reversed(range(self.docker_layout.count())): 
            self.docker_layout.itemAt(i).widget().setParent(None)
        
        for service in services:
            if service != '' and service not in self.settings['excluded_services']:
                button = qt.QPushButton(text=service)
                if service in running:
                    button.setStyleSheet('background-color: lightgreen;')
                button.setCheckable(True)
                if service in self.services:
                    button.setChecked(self.services[service])
                else:
                    self.services[service] = False
                def get_click_function(s, b):
                    def click_function():
                        self.services[s] = b.isChecked()
                    return click_function
                button.clicked.connect(get_click_function(service, button))

                def get_open_menu_function(s, b):
                    def menu_triggered(action):
                        if action.text() == 'bash':
                            self.docker_exec(s, 'bash')
                        elif action.text() == 'tmux':
                            self.docker_exec(s, 'tmux a')
                    
                    def open_menu_function():
                        menu = qt.QMenu(self)
                        action1 = menu.addAction('bash')
                        action2 = menu.addAction('tmux')
                        menu.triggered.connect(menu_triggered)
                        menu.popup(b.mapToGlobal(b.rect().topLeft()))
                    return open_menu_function
                
                button.setContextMenuPolicy(3)
                button.customContextMenuRequested.connect(get_open_menu_function(service, button))
                
                self.docker_layout.addWidget(button)

    def docker_command(self, command):
        services_str = ' '.join(self.get_selected_services())

        logger.info('docker command: ' + command + ' ' + services_str)
                
        if len(services_str) > 0:
            command = 'dbus-launch gnome-terminal --wait -- bash -c \'' + \
                self.ssh_t('cd ' + self.settings['path'] + ';  docker compose ' + command + ' ' + services_str + \
                           (' -d' if command == 'up' else '')) + '\''
            logger.info('docker_command: ' + command)
            self.docker_command_thread = CommandThread(command, lambda :self.refresh_docker(), True)

    def docker_exec(self, service, command):
        proc = f'''
        mapfile -t names <<< $(sshpass -p "{self.settings['password']}" \
                               ssh -t -o StrictHostKeyChecking=no {self.settings['username']}@{self.settings['hostname']} \
                                 "cd {self.settings['path']}; docker ps -f name={service} --format '{{{{.Names}}}}'");
        for item in ${{names[@]}}; do
            item=$(echo $item| tr -d '\\r')
            command="dbus-launch gnome-terminal -- bash -c 'sshpass -p \\"{self.settings['password']}\\" \
                                                ssh -t -o StrictHostKeyChecking=no \
                                                  {self.settings['username']}@{self.settings['hostname']} \
                                                    \\"cd {self.settings['path']};  docker exec -it $item {command};\\"';"
            eval "$command"
        done
        '''
        
        logger.info(proc)
        subprocess.Popen(proc, shell=True, executable='/usr/bin/bash')

    def ssh(self):
        proc = f'''dbus-launch gnome-terminal -- bash -c 'sshpass -p "{self.settings['password']}" \
                                                          ssh -o StrictHostKeyChecking=no \
                                                          {self.settings['username']}@{self.settings['hostname']}' '''
        logger.info(proc)
        subprocess.Popen(proc, shell=True, executable='/usr/bin/bash')

    def handle_ping(self, text):
        if text == 'failed':
            self.setStyleSheet('QWidget#info_widget { ' + self.stylesheet + 'background-color: rgb(255, 144, 144) }')
            self.connected = False
        else:
            self.setStyleSheet('QWidget#info_widget { ' + self.stylesheet + 'background-color: rgb(239, 239, 239) }')
            if not self.connected:
                self.refresh_docker()
            self.connected = True
            
        self.ping_label.setText('Ping: ' + text)

    def get_dct(self):
        return self.settings
    
    def get_selected_services(self):
        selected_services = []
        for service, selected in self.services.items():
            if selected:
                selected_services.append(service)
        return selected_services
        
    
class AirstackControlPanel(Plugin):
    def __init__(self, context):
        super(AirstackControlPanel, self).__init__(context)
        self.setObjectName('AirstackControlPanel')

        self.context = context
        # to access ros2 node use self.context.node

        global logger
        logger = self.context.node.get_logger()

        self.info_widgets = []

        self.widget = qt.QWidget()
        self.layout = qt.QVBoxLayout(self.widget)

        self.info_widget = qt.QWidget()
        self.info_layout = qt.QVBoxLayout(self.info_widget)
        
        self.info_scroll_area = qt.QScrollArea()
        self.info_scroll_area.setWidget(self.info_widget)
        self.info_scroll_area.setWidgetResizable(True)
        self.info_scroll_area.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.info_scroll_area.setMinimumHeight(800)
        self.layout.addWidget(self.info_scroll_area)

        self.add_button = qt.QPushButton('+')
        self.add_button.clicked.connect(lambda x:self.add_info_widget())
        self.layout.addWidget(self.add_button)
        self.layout.addStretch(1)
        
        self.context.add_widget(self.widget)

    def add_info_widget(self, settings=None):
        if settings == None:
            info_widget = InfoWidget(self.context.node)
        else:
            info_widget = InfoWidget(self.context.node, settings)
        
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
        info_dcts = [copy.deepcopy(w.get_dct()) for w in self.info_widgets if w != None]
        for settings in info_dcts:
            settings['password'] = xor_encrypt_decrypt(settings['password'])
        instance_settings.set_value('info_dcts', info_dcts)

    def restore_settings(self, plugin_settings, instance_settings):
        info_dcts = instance_settings.value('info_dcts', {})
        for settings in info_dcts:
            settings['password'] = xor_encrypt_decrypt(settings['password'])
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
