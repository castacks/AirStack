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

try:
    from rqt_py_console.spyder_console_widget import SpyderConsoleWidget
    _has_spyderlib = True
except ImportError:
    _has_spyderlib = False


class Template(Plugin):
    """
    Plugin providing an interactive Python console
    """

    def __init__(self, context):
        super(Template, self).__init__(context)
        self.setObjectName('Template')

        self.context = context
        # to access ros2 node use self.context.node

        self.widget = QWidget()
        self.button = QtWidgets.QPushButton(text='Button')
        self.layout = QtWidgets.QVBoxLayout(self.widget)
        self.layout.addWidget(self.button)
        self.context.add_widget(self.widget)

    def save_settings(self, plugin_settings, instance_settings):
        pass
        #instance_settings.set_value('some_variable', self.some_variable)

    def restore_settings(self, plugin_settings, instance_settings):
        pass
        #self.some_variable = instance_settings.value('some_variable', default_value)

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
