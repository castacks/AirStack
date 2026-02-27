#!/usr/bin/python
import numpy as np
import matplotlib.pyplot as plt
import yaml
import sys

def generate_curve(traj_config):
    lin_vel = float(traj_config['linear_velocity'])
    ang_vel = np.pi/180.*float(traj_config['angular_velocity'])

    dt = 0.1
    if 'dt' in traj_config.keys():
        dt = traj_config['dt']

    time = 0.
    if 'distance' in traj_config.keys():
        time = float(traj_config['distance'])/lin_vel
    if 'time' in traj_config.keys():
        time = traj_config['time']

    xs = [0.]
    ys = [0.]
    yaws = [0.]

    for t in np.arange(0., time, dt):
        xs.append(xs[-1] + np.cos(yaws[-1])*lin_vel*dt)
        ys.append(ys[-1] + np.sin(yaws[-1])*lin_vel*dt)
        yaws.append(yaws[-1] + ang_vel*dt)

    if 'yaw' in traj_config.keys():
        if traj_config['yaw'] != 'heading':
            yaw = np.pi/180.*float(traj_config['yaw'])
            for i in range(len(yaws)):
                yaws[i] = yaw

    plt.plot(xs, ys, '.-')
    plt.axis('equal')
    plt.show()


def generate_arc(traj_config):
    radius = float(traj_config['radius'])
    angle = np.pi/180.*float(traj_config['angle'])
    angle_increment = np.pi/180.*float(traj_config['angle_increment'])
    
    xs = radius*np.cos(np.arange(0., angle, angle_increment) - np.pi/2.)
    ys = radius*np.sin(np.arange(0., angle, angle_increment) - np.pi/2.)
    print(xs)
    
    plt.plot(xs, ys, '.-')
    plt.axis('equal')
    plt.show()

def generate_acceleration(traj_config):
    pass

def generate_trajectory(traj_config):
    if traj_config['type'] == 'curve':
        return generate_curve(traj_config)
    elif traj_config['type'] == 'arc':
        return generate_arc(traj_config)
    elif traj_config['type'] == 'acceleration':
        return generate_acceleration(traj_config)

if __name__ == '__main__':
    input_filename = sys.argv[1]
    output_filename = sys.argv[2]


    config = yaml.load(open(input_filename, 'r').read())

    for traj_config in config['trajectories']:
        traj_config = traj_config['trajectory']
        traj = generate_trajectory(traj_config)
        
