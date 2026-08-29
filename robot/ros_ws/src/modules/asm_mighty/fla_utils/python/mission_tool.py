#!/usr/bin/python
import cv2
import matplotlib.pyplot as plt
import numpy as np
import math
from PIL import Image
import matplotlib.patches as patches
import matplotlib.image as mpimg
import matplotlib.lines as lines
import progress_bar_util as pbu
import os
import time
import sys
from yaml import load, dump

class MissionTool:

    def __init__(self, yaml_file):

        stream = file(yaml_file, 'r')
        self.config = load(stream)
        self.mpp = float(self.config["meters_per_pixel"])
        self.click_start = bool(self.config["click_start"])
        self.start = None
        if not self.click_start:
          self.start = [float(self.config["start_x"]), float(self.config["start_y"])]
        self.subgoals_x = []
        self.subgoals_y = []
        self.fig = plt.figure("Waypoint selection")
        self.ax = self.fig.add_subplot(1,1,1)
        self.fig.canvas.set_window_title('Choose waypoints here')
        self.image = np.flipud(np.array(Image.open(self.config["overhead_image"])));
        self.image_height = self.image.shape[0]
        self.image_width = self.image.shape[1]
        self.fig.canvas.mpl_connect('button_press_event', self._on_click) 


    def updatedrawing(self):
        plt.figure("Waypoint selection")
        if self.start:
          self.ax.plot([self.start[0]] + self.subgoals_x, [self.start[1]] + self.subgoals_y, color='b', markerfacecolor='r', zorder=1, marker='o', markersize=13)        
          self.ax.annotate('s', (self.start[0], self.start[1]), ha='center', va='center')
        for i, x in enumerate(self.subgoals_x):
            self.ax.annotate(i + 1, (x, self.subgoals_y[i]), ha='center', va='center')
        self.fig.canvas.draw()
       
 
    def _on_click(self, event):
        ix, iy = event.xdata, event.ydata
        ix = int(ix)
        iy = int(iy)
        if not self.start:
          self.start = [float(ix), float(iy)]
        else:
          print "Adding subgoal: %i, %i"  % (ix, iy)
          self.subgoals_x.append(ix)
          self.subgoals_y.append(iy)
        self.updatedrawing()

        
        #compute mission frame based on first subgoal
        if len(self.subgoals_x) == 1:
          dx = self.subgoals_x[0] - self.start[0]
          dy = self.subgoals_y[0] - self.start[1]
          self.theta_rad = math.atan2(dy, dx) - math.pi/2
          print "Rotation Angle: %f" % (math.degrees(self.theta_rad))
          c = math.cos(self.theta_rad)
          s = math.sin(self.theta_rad)
          start_x_world, start_y_world = self.pixels_to_world(self.start[0], self.start[1])
          self.world_to_mission = np.matrix([[c, s, -start_y_world * s - start_x_world * c], \
                                      [-s, c, start_x_world * s - start_y_world * c], \
                                      [0, 0, 1]])

    def pixels_to_world(self, x_pixels, y_pixels):
        return (y_pixels - self.image_height / 2) * self.mpp, -(x_pixels - self.image_width / 2) * self.mpp

    def pixels_to_mission(self, x_pixels, y_pixels):
      x_world, y_world = self.pixels_to_world(x_pixels, y_pixels)
      mission_coords = np.matmul(self.world_to_mission, np.array([x_world, y_world, 1]))      
      mission_x = float(mission_coords[0, 0])
      mission_y = float(mission_coords[0, 1])
      return mission_x, mission_y 

    def buildyaml(self):
        output_dict = {}
        output_dict["objects"] = [] 
        start_x, start_y = self.pixels_to_mission(self.start[0], self.start[1])
        start_x_world, start_y_world = self.pixels_to_world(self.start[0], self.start[1])
        output_dict["objects"].append({"type"  : "vehicle", \
                                       "frame_id"     : "body" \
                                       })

        output_dict["objects"].append({"type"  : "overhead", \
                                       "frame_id" : "overhead", \
                                       "x"     : float(self.world_to_mission[0, 2]), \
                                       "y"     : float(self.world_to_mission[1, 2]), \
                                       "theta"     : -float(np.degrees(self.theta_rad)) \
                                       })

        output_dict["objects"].append({"type"  : "start", \
                                       "x"     : start_x, \
                                       "y"     : start_y, \
                                       "explored" : "false", \
                                       "near"     : "false", \
                                       "radius": self.config["subgoal_radius"], \
                                       })

        #assume search region is centered at the start location
        output_dict["objects"].append({"type"  : "goal", \
                                       "x"     : start_x, \
                                       "y"     : start_y, \
                                       "explored" : "false", \
                                       "near"     : "false", \
                                       "radius": self.config["goal_radius"], \
                                       })

        for i, x in enumerate(self.subgoals_x):
            world_x, world_y = self.pixels_to_mission(x, self.subgoals_y[i])
            output_dict["objects"].append({"type" : "subgoal", \
                               "x"        : world_x, \
                               "y"        : world_y, \
                               "order"    : i, \
                               "explored" : "false", \
                               "near"     : "false", \
                               "radius": self.config["subgoal_radius"] \
                               })

        return dump(output_dict)


    def run(self):
        self.ax.imshow(self.image, zorder=0)
        plt.gca().invert_yaxis()
        self.updatedrawing()
        plt.show()
        output_file = open("database.yaml", "w")
        output_file.write(self.buildyaml())
        output_file.close()
        print "done"

if __name__ == "__main__":
    
    if len(sys.argv) != 2:
        print "Usage: mission_tool.py <path_to_yaml>"
        exit()

    mission_tool = MissionTool(sys.argv[1])
    mission_tool.run()
