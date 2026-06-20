#!/usr/bin/python3

# Copyright 2024 Laboratoire des signaux et systèmes
#
# This program is free software: you can redistribute it and/or 
# modify it under the terms of the GNU General Public License as 
# published by the Free Software Foundation, either version 3 of 
# the License, or (at your option) any later version.
# 
# This program is distributed in the hope that it will be useful, 
# but WITHOUT ANY WARRANTY; without even the implied warranty of 
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
# See the GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License 
# along with this program. If not, see <https://www.gnu.org/licenses/>. 
#
# Author: Aarsh Thakker <aarsh.thakker@centralesupelec.fr>

PACKAGE='natnet_ros2'
from ament_index_python.packages import get_package_share_directory
PKG_PATH = get_package_share_directory(PACKAGE)

import sys
import rclpy
from rclpy.node import Node
from ros2param.api import load_parameter_file
from natnet_ros2_py.node_module import HelperNode

import subprocess
import os
import re
import yaml

from PyQt5 import QtWidgets, uic
import sys

from PyQt5 import QtGui, QtWidgets
from PyQt5.QtCore import QObject, QThread, pyqtSignal

def kill_subp():
  procs = str(subprocess.check_output(["ps","-a"])).split('\\n')
  #print(procs)
  for i in range(len(procs)-2):
    if "bash <defunct>" in procs[i]:
      pid = None
      if "ros2" in procs[i+1]:
        pid = procs[i+1].split(' ')[1]
      if "ros2" in procs[i+2]:
        pid = procs[i+1].split(' ')[1]
      if pid!=None:
        try:
          subprocess.check_call(["kill","-9",str(pid)])
        except:
          pass
  try:
    subprocess.check_call(["pkill","_ros2_daemon"])
  except:
    pass

class WorkerThread(QThread):
  finished_signal = pyqtSignal(int)
  def __init__(self, cmd) -> None:
    super().__init__()
      
    self.cmd = cmd

  def run(self):
    proc = subprocess.run(self.cmd,shell=True,executable='/bin/bash')
    self.finished_signal.emit(proc.returncode)

class PyQt5Widget(QtWidgets.QMainWindow):
  def __init__(self,node:Node):
    super(PyQt5Widget, self).__init__()
    self.node = node
    ui_file = os.path.join(PKG_PATH, 'ui', 'helper.ui')
    print('')
    print(ui_file)
    print('')
    uic.loadUi(ui_file, self)
    self.setWindowIcon(QtGui.QIcon(os.path.join(PKG_PATH,'ui','logo.png')))

    self.ros_dist = os.environ['ROS_DISTRO']
    self.pwd = os.environ['PWD']
    self.msg_types = ["PoseStamped","PointStamped"]
    self.im_msg_type = self.msg_types[0]
    if os.path.exists(os.path.join(PKG_PATH, 'config','conf_autogen.yaml')):
      self.config_file = os.path.join(PKG_PATH, 'config','conf_autogen.yaml')
    self.name = 'natnet_ros2'
    self.pub_params = {"pub_individual_marker":False,
                      "pub_rigid_body": False,
                      "pub_rigid_body_marker": False,
                      "pub_pointcloud": False,
                      "pub_rigid_body_wmc": False,
                      "individual_marker_msg_type": "PoseStamped",
                      }
    self.log_params = {"log_internals": False,
                      "log_frames": False,
                      "log_latencies": False,
                      }
    self.conn_params = {"serverIP": None,
                        "clientIP": None,
                        "serverType": None,
                        "multicastAddress": None,
                        "serverCommandPort": None,
                        "serverDataPort": None,
                        "globalFrame": None,
                        }
    self.natnet_params = {"{name}".format(name=self.name):None}
    self.error_pass = True
    self.num_of_markers=0
    self.x_position=None
    self.y_position=None
    self.z_position=None
    self.id = 0
    
    self.outputBox.append("""<div style="color: #ff8c00">[NOTE]</div>""")
    self.outputBox.append('- This prompt does not show all the errors. For full error, check the terminal where this node has been excecuted')
    self.outputBox.append('- It is required to launch the node from your main workspace folder where this package has been built')
    self.outputBox.append('- Stopping node is done by killing PID because of issue with lifecycle node shutdown process')
    self.Log('info',' If your WIFI or LAN IP is not detected in the list of the network, You can use the remote IP option and add the ip address manually.')
    self.Log('info',' rosmaster will be selected initially based on the environment variables. You can change it anytime. Provided rosmaster will be used to run the main natnet node.')
    #self.Log('info',' ')
    IP_data = subprocess.check_output(['lshw','-c','network']).decode('utf-8')
    IP_data=re.sub(r"[^a-zA-Z0-9. ]", "", IP_data)
    IP_data=IP_data.split('network')
    IP_data.pop(0)
    self.IP_LIST = []

    for data in IP_data:
      try:
        _ip = [ t for t in data.split() if t.startswith('ip') ]
        self.outputBox.append("""<div style="color: #656565">[INFO]</div>""" +'Found the connection on the IP ending with ' + str(_ip[0].split('.')[-1]) )
        self.IP_LIST.append(_ip[0].replace('ip',''))
      except:
        pass

    del IP_data
    if len(self.IP_LIST)>0:
      self.client_ip_spin.setEnabled(True)
      self.client_ip_spin.setMinimum(1)
      self.textClientIP.setText(str(self.IP_LIST[self.client_ip_spin.value()-1]))
      self.textServerIP.setText('.'.join(str(self.IP_LIST[self.client_ip_spin.value()-1]).split('.')[:-1],)+'.')
      self.conn_params["clientIP"] = str(self.IP_LIST[self.client_ip_spin.value()-1])
    self.client_ip_spin.setMaximum(len(self.IP_LIST))

    self.pub_rbm.setEnabled(False)
    self.domain_id_spin.valueChanged.connect(self.select_network)
    self.client_ip_spin.valueChanged.connect(self.spin_clientIP)
    self.msg_type_spin.valueChanged.connect(self.spin_msg_type)
    self.multicast_radio.clicked.connect(self.clicked_multicast)
    self.unicast_radio.clicked.connect(self.clicked_unicast)
    self.start_node.clicked.connect(self.start)
    self.stop_node.clicked.connect(self.stop)
    self.log_frames.clicked.connect(self.log_frames_setting)
    self.log_internal.clicked.connect(self.log_internal_setting)
    self.log_latencies.clicked.connect(self.log_latencies_setting)
    self.pub_im.clicked.connect(self.pub_im_setting)
    self.pub_rb.clicked.connect(self.pub_rb_setting)
    self.pub_rbm.clicked.connect(self.pub_rbm_setting)
    self.pub_rbwmc.clicked.connect(self.pub_rbwmc_setting)
    self.pub_pc.clicked.connect(self.pub_pc_setting)
    self.push_refresh.clicked.connect(self.call_MarkerPoses_srv)
    self.push_ok.clicked.connect(self.yaml_dump)

  def Log(self,type:str="",msg=''):
    if type=="info":
      self.outputBox.append("""<div style="color: #656565">[INFO]</div>""" +msg)
    if type=="error":
      self.outputBox.append("""<div style="color:red">[ERROR]</div>""" +msg)
    if type=="warn":
      self.outputBox.append("""<div style="color: #ffae42">[WARN]</div>""" +msg)
    if type=="block":
      self.outputBox.append("""<div style="color: #656565">--------------------</div>""")
    if type=="":
      self.outputBox.append(msg)

#-----------------------------------------------------------------------------------------
# START/STOP NATNET BUTTON

  def start(self):
    kill_subp()
    self.Log('block')
    self.check_all_params()
    #print(f'{self.conn_params=},{self.natnet_params=},{self.pub_params=},{self.log_params=}')
    command='''
    source /opt/ros/{dist}/setup.bash;
    source {pwd}/install/setup.bash;
    export ROS_DOMAIN_ID={id};
    ros2 launch natnet_ros2 natnet_ros2.launch.py node_name:={name} serverIP:={sip} \
    clientIP:={cip} serverType:={st} multicastAddress:={mca} \
    serverCommandPort:={scp} serverDataPort:={sdp} global_frame:={gf} \
    remove_latency:={rl} pub_rigid_body:={prb} pub_rigid_body_marker:={prbm} \
    pub_individual_marker:={pim} pub_pointcloud:={ppc} log_internals:={li} \
    log_frames:={lf} log_latencies:={ll} conf_file:={cf} activate:={activate} \
    immt:={immt}
    '''.format(dist=self.ros_dist,id=self.id, pwd=self.pwd, name=self.name, sip=self.conn_params["serverIP"],
              cip=self.conn_params["clientIP"],st=self.conn_params["serverType"],mca=self.conn_params["multicastAddress"],
              scp=self.conn_params["serverCommandPort"],sdp=self.conn_params["serverDataPort"],gf=self.conn_params["globalFrame"],
              rl=False,prb=self.pub_params["pub_rigid_body"],prbm=self.pub_params["pub_rigid_body_marker"],
              pim=self.pub_params["pub_individual_marker"],ppc=self.pub_params["pub_pointcloud"],
              li=self.log_params["log_internals"],lf=self.log_params["log_frames"],ll=self.log_params["log_latencies"],
              cf='conf_autogen.yaml',activate=True,immt=self.pub_params["individual_marker_msg_type"])
    self.start_node_thread = WorkerThread(command)
    self.start_node_thread.start()

  def stop(self):
    
    command='''
    source /opt/ros/{dist}/setup.bash;
    export ROS_DOMAIN_ID={id};
    ros2 lifecycle set /{name} shutdown
    '''.format(dist=self.ros_dist,id=self.id,name=self.name)
    #stop_node_thread = WorkerThread(command)
    #stop_node_thread.start()
    #pid = int(subprocess.check_output(["pidof","-s","natnet_ros2"]))
    #subprocess.check_call(["kill","-9",str(pid)])
    try:
      self.start_node_thread.terminate()
      subprocess.run(command,shell=True,check=False,executable='/bin/bash',bufsize=0, timeout=1)
    except:
      pass

    self.Log('info',' Killed natnet node' )
    self.Log('block')

#----------------------------------------------------------------------------------------
# PUBLISHING RELATED STUFF

  def pub_im_setting(self):
    self.Log('block')
    
    if self.pub_im.isChecked():
      self.set_conn_params('marker_poses_server')
    if self.error_pass:
      if self.pub_im.isChecked():
        self.pub_params["pub_individual_marker"] = True
        self.Log('block')
        msg = '''
        It will only take upto 40 unlabled markers from the available list.
        If you do not see the marker in the list, make sure things other than markers are masked and markers are clearly visible.
        '''
        self.Log('info',msg)
        self.Log('info','Go to Single marker naming tab and press the refresh button to complete the configuration of for initial position of the markers (wait for a second or two after pressing the refresh).')
        self.Log('info',' If you do not wish to name some marker, you can leave it empty. Do not repeat names of the markers.')
      else:
        self.pub_params["pub_individual_marker"] = False
    else:
      self.Log('error','Can not get the data of markers from the natnet server. One or more parameters from conenction settings are missing')

  def pub_rb_setting(self):
    if self.pub_rb.isChecked():
      self.pub_params["pub_rigid_body"] = True
      self.Log('info','Enabled publishing rigidbody')
      self.pub_rbm.setEnabled(True)
    else:
      self.pub_params["pub_rigid_body"] = False
      self.pub_rbm.setEnabled(False)

  def pub_rbm_setting(self):
    if self.pub_rbm.isChecked():
      self.pub_params["pub_rigid_body_marker"] = True
      self.Log('info','Enabled publishing rigidbody markers')
    else:
      self.pub_params["pub_rigid_body_marker"] = False

  def pub_pc_setting(self):
    if self.pub_pc.isChecked():
      self.pub_params["pub_pointcloud"] = True
      self.Log('info','Enabled publishing pointcloud')
    else:
      self.pub_params["pub_pointcloud"] = False
  
  def pub_rbwmc_setting(self):
    self.Log('info','This functionality is not supported yet.')
    
  def pub_immt_setting(self):
    self.pub_params["individual_marker_msg_type"] = self.im_msg_type

#----------------------------------------------------------------------------------------
# NETWORK SELECTION THINGS

  def select_network(self):
    self.id = int(self.domain_id_spin.value())
    self.Log('warn','Do not cahnge Domain ID until you know what you are trying.')
    self.Log('info','Using ROS domain ID: '+str(self.id))

#----------------------------------------------------------------------------------------
# NATNET CONNECTION RELATED

  def get_server_ip(self):
    self.conn_params["serverIP"] = self.textServerIP.text()
    self.Log('info','setting servet ip '+str(self.conn_params["serverIP"]))
    #self.Log('info','setting server ip '+str(self.conn_params["serverIP"]).split('.')[0]+'***'+'***'+str(self.conn_params["serverIP"]).split('.')[-1])

  def get_client_ip(self):
    self.conn_params["clientIP"] = self.textClientIP.text()
    self.Log('info','setting client ip '+str(self.conn_params["clientIP"]))
    #self.Log('info','setting client ip '+str(self.conn_params["clientIP"]).split('.')[0]+'***'+'***'+str(self.conn_params["clientIP"]).split('.')[-1])

  def get_server_type(self):
    if self.multicast_radio.isChecked():
      self.conn_params["serverType"] = 'multicast'
      self.Log('info','setting broadcat to '+str(self.conn_params["serverType"]))
      self.get_multicast_addr()
    if self.unicast_radio.isChecked():
      self.conn_params["serverType"] = 'unicast'
      self.Log('info','setting broadcat to '+str(self.conn_params["serverType"]))

  def clicked_unicast(self):
    self.conn_params["serverType"] = 'unicast'
    self.textMulticastAddr.setEnabled(False)
    self.Log('info','setting broadcat to '+str(self.conn_params["serverType"]))

  def clicked_multicast(self):
    self.conn_params["serverType"] = 'multicast'
    self.textMulticastAddr.setEnabled(True)
    self.Log('info','setting broadcat to '+str(self.conn_params["serverType"]))

  def get_multicast_addr(self):
    self.conn_params["multicastAddress"] = self.textMulticastAddr.text()
    self.Log('info','setting multicast address '+str(self.conn_params["multicastAddress"]))

  def get_command_port(self):
    self.conn_params["serverCommandPort"] = int(self.textCommandPort.text())
    self.Log('info','setting command port '+str(self.conn_params["serverCommandPort"]))

  def get_data_port(self):
    self.conn_params["serverDataPort"] = int(self.textDataPort.text())
    self.Log('info','setting data port '+str(self.conn_params["serverDataPort"]))

  def get_world_frame(self):
    self.conn_params["globalFrame"] = self.textFrameName.text()
    if self.conn_params["globalFrame"]=='':
      self.conn_params["globalFrame"]='world'
      self.Log('warn','setting world frame name to world as no input provided')
    else:
      self.Log('info','setting world frame name '+str(self.conn_params["globalFrame"]))

  def spin_clientIP(self):
    self.textClientIP.setText(str(self.IP_LIST[self.client_ip_spin.value()-1]))
    self.conn_params["clientIP"] = str(self.IP_LIST[self.client_ip_spin.value()-1])
    self.textServerIP.setText('.'.join(str(self.IP_LIST[self.client_ip_spin.value()-1]).split('.')[:-1],)+'.')
    self.Log('info','setting client ip '+str(self.conn_params["clientIP"]))

  def spin_msg_type(self):
    self.textMsgType.setText(self.msg_types[self.msg_type_spin.value()-1])
    self.im_msg_type = self.msg_types[self.msg_type_spin.value()-1]

  def get_node_name(self):
    self.name = self.textNodeName.text()
    if self.name=='':
      self.name='natnet_ros2'
      self.Log('warn','setting natnet_ros2 name to world as no input provided')
    else:
      self.Log('info','setting name '+self.name)
#----------------------------------------------------------------------------------------
# PARAM CHECK RELATED

  def chek_conn_params(self):
    self.get_server_ip()
    if self.conn_params["serverIP"]==None or self.conn_params["serverIP"]=='':
      self.Log('error','No server ip provided. Can not connect.')
      self.error_pass=False
    self.get_client_ip()
    if self.conn_params["clientIP"]==None or self.conn_params["clientIP"]=='':
      self.Log('error','No client ip provided. Can not connect.')
      self.error_pass=False
    self.get_server_type()
    self.get_command_port()
    if self.conn_params["serverCommandPort"]==None or self.conn_params["serverCommandPort"]=='':
      self.Log('error','No command port provided. Can not connect.')
      self.error_pass=False
    self.get_data_port()
    if self.conn_params["serverDataPort"]==None or self.conn_params["serverDataPort"]=='':
      self.Log('error','No data port provided. Can not connect.')
      self.error_pass=False
    self.get_world_frame()
    self.get_node_name()

  def set_conn_params(self,node_name:str):
    self.chek_conn_params()
    if self.error_pass:
      self.node.call_set_parameters(node_name,self.conn_params)

  def check_all_params(self):
    self.chek_conn_params()
    self.pub_im_setting()
    self.pub_rb_setting()
    self.pub_rbm_setting()
    self.pub_pc_setting()
    self.pub_immt_setting()
    self.pub_params["pub_rigid_body_wmc"] = False
    self.log_frames_setting()
    self.log_internal_setting()
    self.log_latencies_setting()

#----------------------------------------------------------------------------------------
# MARKER POSE SERVER RELATED

  def set_lcds(self,num_of_markers,x_position,y_position,z_position):
    if (len(x_position) or len(y_position) or len(z_position))!=num_of_markers:
      self.Log('error',' Length of positions and number of detected markers are not matching, Press refresh to retry.')
    else:
      for i in range(min(40,num_of_markers)):
        eval('self.markerBox_'+str(i+1)+'.setEnabled(True)')
        eval('self.X_'+str(i+1)+'.display(x_position[i])')
        eval('self.Y_'+str(i+1)+'.display(y_position[i])')
        eval('self.Z_'+str(i+1)+'.display(z_position[i])')
      self.num_of_markers = num_of_markers
      self.x_position = x_position
      self.y_position = y_position
      self.z_position = z_position

  def yaml_dump(self):
    self.im_msg_type = self.msg_types[self.msg_type_spin.value()-1]
    empty=0
    object_names={'object_names':[]}
    if self.num_of_markers!=0:
      for i in range(min(40,self.num_of_markers)):
        if eval('self.name_'+str(i+1)+'.text()') == '': empty+=1
        else:
          object_names['object_names'].append(eval('self.name_'+str(i+1)+'.text()'))
          object_names[object_names['object_names'][i-empty]]={'marker_config':0,
                                                    'pose':
                                                    {'position':[self.x_position[i],self.y_position[i],self.z_position[i]],
                                                    'orientation':[0,0,0]}}
      self.natnet_params = object_names
      try:
        os.remove(os.path.join(self.config_file))
      except OSError:
        pass

      with open(self.config_file,'w') as f:
        yaml.dump({self.name: {'ros__parameters':self.natnet_params}},f,indent=2,default_flow_style=False)
        f.close()
    else:
      self.Log('error','Number of markers are not recieved. Something went wrong.')

  def call_MarkerPoses_srv(self):
    self.set_conn_params('marker_poses_server')
    if self.error_pass:
      try:
        res = self.node.request_markerposes()
        self.set_lcds(res.num_of_markers,res.x_position,res.y_position,res.z_position)
      except RuntimeError as e:
        self.Log('error','Service call failed: '+str(e))

#----------------------------------------------------------------------------------------
# LOGGING RELATED

  def log_frames_setting(self):
    if self.log_frames.isChecked():
      self.log_params["log_frames"] = True
      self.Log('info','Enabled logging frames in terminal')
    else:
      self.log_params["log_frames"] = False

  def log_internal_setting(self):
    if self.log_internal.isChecked():
      self.log_params["log_internals"] = True
      self.Log('info','Enabled logging internal in terminal')
    else:
      self.log_params["log_internals"] = False

  def log_latencies_setting(self):
    if self.log_latencies.isChecked():
      self.log_params["log_latencies"] = True
      self.Log('info','Enabled logging latencies in terminal')
    else:
      self.log_params["log_latencies"] = False


def main(args=None):
    rclpy.init(args=args)

    node = HelperNode()

    app = QtWidgets.QApplication(sys.argv)
    window = PyQt5Widget(node)
    window.show()

    try:
      sys.exit(app.exec_())
    finally:
      kill_subp()
      # Clean up ROS when the application is closed
      node.destroy_node()
      rclpy.shutdown()

if __name__ == '__main__':
    main()
