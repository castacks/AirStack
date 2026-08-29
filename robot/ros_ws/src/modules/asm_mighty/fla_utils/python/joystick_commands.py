#!/usr/bin/env python 

import rospy
from fla_msgs.msg import JoyDef
from sensor_msgs.msg import Joy
from fla_msgs.msg import FlightCommand

class JoystickCommander:
    def __init__(self):
        self.pub = rospy.Publisher('/flight/command', FlightCommand, queue_size=10)
        rospy.Subscriber('/joy', Joy, self.joyCB)
        self.joyinfo = JoyDef()

    def joyCB(self, data):
      cmd = FlightCommand()
      cmd.command = 255
      if data.buttons[self.joyinfo.A]:
          cmd.command = cmd.CMD_TAKEOFF
      elif data.buttons[self.joyinfo.B]:
          cmd.command = cmd.CMD_KILL
      elif data.buttons[self.joyinfo.X]:
          cmd.command = cmd.CMD_LAND
      elif data.buttons[self.joyinfo.Y]:
          cmd.command = cmd.CMD_HOVER
      elif data.buttons[self.joyinfo.START]:
          cmd.command = cmd.CMD_INIT
      elif data.buttons[self.joyinfo.CENTER]:
          cmd.command = cmd.CMD_GO
      if cmd.command is not 255:
          self.pub.publish(cmd)

if __name__ == '__main__':
    try:
        rospy.init_node('joystick_commands')
        JoystickCommander()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
