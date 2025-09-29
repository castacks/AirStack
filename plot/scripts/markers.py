#!/usr/bin/python3
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
from visualization_msgs.msg import Marker

def draw_markers(ax, markers):
    ellipse_args = []
    
    for m in markers:
        marker = m[0]
        if marker.type == Marker.CYLINDER:
            args = (marker.pose.position.x, marker.pose.position.y), marker.scale.x, marker.scale.y
            if args in ellipse_args:
                continue
            ellipse_args.append(args)
            ellipse = Ellipse(*args, color='r')
            ax.add_patch(ellipse)
            

def plot(ax, markers):
    print('script plot')

    draw_markers(ax, markers.markers)
    
    #circle = plt.Circle((0, 0), 0.2, color='r')
    #ax.add_patch(circle)
    
    return {'x': np.array([0., 10.]), 'y': np.array([0., 20.])}
