#!/usr/bin/python
# http://docs.opencv.org/3.0-beta/doc/py_tutorials/py_gui/py_mouse_handling/py_mouse_handling.html
import cv2
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image
import matplotlib.patches as patches
import matplotlib.image as mpimg
import matplotlib.lines as lines
import progress_bar_util as pbu
import argparse
import os

class VelocityMapMaker:

    def __init__(self, image_location="/home/katherine/Desktop/test1.png", underlay_image_path=None, output=None, min_speed = 6.0,
                 load_from_txt = None, use_potential=False):
        print "Min speed: ", min_speed
        self.min_speed = float(min_speed)
        self.clicked_point = None
        self.load_from_txt = load_from_txt
        self.accepted_waypoint = False
        self.accepting_waypoints = False
        self.candidate_waypoint_marker = None
        self.candidate_waypoint = None
        self.candidate_line_marker = None
        self.velocity_map = None
        self.map_output_location = output
        self.underlay_image = None
        self.underlay_image_path = underlay_image_path
        self.use_potential = use_potential
        self.waypoints = []
        self.speeds = []
        self.coords = []
        self.all_plot_objs = []

        # Optional image underlay
        if underlay_image_path is not None:
            self.underlay_fig = plt.figure("Underlay")
            self.underlay_ax = self.underlay_fig.add_subplot(1,1,1)
            self.underlay_image = np.array(Image.open(underlay_image_path))
            # self.underlay_image = np.fliplr(self.underlay_image)
            # plt.imshow(np.flipud(np.fliplr(self.underlay_image)))
            plt.imshow(self.underlay_image)

        self.results_fig = plt.figure("Results")
        self.results_ax = self.results_fig.add_subplot(1,1,1)

        self.fig = plt.figure("Waypoint selection")
        self.ax = self.fig.add_subplot(1,1,1)
        self.fig.canvas.set_window_title('Choose waypoints here')

        # Masked image
        self.image = np.array(Image.open(image_location).convert("L"))
        print np.shape(self.image)
        # plt.imshow(self.image, cmap='gray')
        # self.image = np.flipud(np.fliplr(self.image))
        plt.imshow(self.image, cmap='gray')
        plt.show(block=False)   
        self.fig.canvas.mpl_connect('button_press_event', self._on_click) 

    '''
        On user click, register location of click and draw candidate
        waypoint as an ellipse. Draw connecting lines if enough points. 
    '''        
    def _on_click(self, event):
        plt.figure("Waypoint selection")
        ix, iy = event.xdata, event.ydata
        ix = int(ix)
        iy = int(iy)
        # print ix, iy
        if self.accepting_waypoints:
            # TODO(katliu): Image coords are flipped...?
            if self.image[iy, ix] == 255: 
                self.candidate_waypoint = [iy, ix]
                if self.candidate_waypoint_marker != None:
                    # remove the previous candidate
                    self.candidate_waypoint_marker.remove()
                if self.candidate_line_marker != None:
                    self.candidate_line_marker.remove()
                self.candidate_waypoint_marker = patches.Ellipse((ix,iy), 10, 10)
                self.ax.add_patch(self.candidate_waypoint_marker)

                if len(self.waypoints)>0:
                    line = [self.waypoints[-1],self.candidate_waypoint]
                    (y, x) = zip(*line)
                    self.candidate_line_marker = self.ax.add_line(lines.Line2D(x, y, linewidth=1, color='red'))
                plt.draw()
        else:
            print "Enter [w] to enter the next waypoint"

    def _distance(self, a, b):
        return (a[0]-b[0])**2 + (a[1]-b[1])**2

    def potential_calculate_speeds(self, image, waypoint_list, speeds):
        # Downsamplings
        image2 = image[::2, ::2]
        image4 = image[::4, ::4]
        image8 = image[::8, ::8]
        image16 = image[::16, ::16]

        w_l2 = []
        w_l4 = []
        w_l8 = []
        w_l16 = []
        for j in range(0, len(speeds)):
            # Set value
            w_l2.append([waypoint_list[j][0]/2, waypoint_list[j][1]/2])
            w_l4.append([waypoint_list[j][0]/4, waypoint_list[j][1]/4])
            w_l8.append([waypoint_list[j][0]/8, waypoint_list[j][1]/8])
            w_l16.append([waypoint_list[j][0]/16, waypoint_list[j][1]/16])

        pot16 = self.calc(image16, w_l16, speeds, np.zeros(np.shape(image16)), 10000)
        pot8 = self.calc(image8, w_l8, speeds, cv2.resize(pot16, (image8.shape[1], image8.shape[0])))
        pot4 = self.calc(image4, w_l4, speeds, cv2.resize(pot8, (image4.shape[1], image4.shape[0])))
        pot2 = self.calc(image2, w_l2, speeds, cv2.resize(pot4, (image2.shape[1], image2.shape[0])))
        potential = self.calc(image, waypoint_list, speeds, cv2.resize(pot2, (image.shape[1], image.shape[0])))

        # Finally, set the min speed
        potential = potential + (image==0) * self.min_speed
        potential = self.calc_final(image, potential)

        # Clamp the max and min speeds
        potential[potential>20] = 20
        potential[potential<=0] = 0.1

        plt.imshow(potential)
        self.velocity_map = potential
        plt.colorbar()
        plt.show(block=False)
        plt.savefig('map.png')


    def calc(self, image, waypoint_list, speeds, i_cond=None, num=500):
        # Create the input image
        image_dim = np.shape(image)
        if i_cond is None:
            potential = np.zeros(image_dim)
        else: 
            potential = i_cond

        # Calculate the counter matrix
        image = (image>0).astype('float')
        counter = image[0:-3, 1:-2] + image[2:-1, 1:-2] + image[1:-2, 0:-3] + image[1:-2, 2:-1]
        mul_mat = np.zeros(image_dim)
        mul_mat[1:-2, 1:-2] = 1.0/counter
        mul_mat[1:-2, 1:-2][counter==0] = 0
        mul_mat[image==0] = 0

        # Iterate
        for it in range(num):
            # print(it)
            # Set the boundary conditions
            # First apply the waypoints
            for j in range(0, len(speeds)):
                # Set value
                potential[waypoint_list[j][0],waypoint_list[j][1]] = speeds[j]
                for x in range(-2,2):
                    for y in range(-2,2):
                        potential[waypoint_list[j][0]+x,waypoint_list[j][1]+y] = speeds[j]

            # Now apply the iteration
            net = potential[0:-3, 1:-2] + potential[2:-1, 1:-2] + potential[1:-2, 0:-3] + potential[1:-2, 2:-1]
            net = np.multiply(net, mul_mat[1:-2, 1:-2])
            potential[1:-2, 1:-2]  = net

        # return the potential
        return potential

    def calc_final(self, image, i_cond, num=500):
        # Create the input image
        image_dim = np.shape(image)

        potential = i_cond.copy()

        # Calculate the counter matrix
        image_b = (image==0).astype('float')
        counter = image_b[0:-3, 1:-2] + image_b[2:-1, 1:-2] + image_b[1:-2, 0:-3] + image_b[1:-2, 2:-1]
        mul_mat = np.zeros(image_dim)
        mul_mat[1:-2, 1:-2] = 1.0/counter
        mul_mat[1:-2, 1:-2][counter==0] = 0
        mul_mat[1:-1, 1:-2] = 1.0/4.0

        # Iterate
        for it in range(num):
            # print(it)
            # Set the boundary conditions
            # First apply the waypoints
            potential[image>0] = i_cond[image>0]

            # Now apply the iteration
            net = potential[0:-3, 1:-2] + potential[2:-1, 1:-2] + potential[1:-2, 0:-3] + potential[1:-2, 2:-1]
            net = np.multiply(net, mul_mat[1:-2, 1:-2])
            potential[1:-2, 1:-2]  = net

        potential[image>0] = i_cond[image>0]
        # return the potential
        return potential
        
    '''
        for each point in free space
        get closest valid wp 
            consider both segments attached to the closest wp
            choose the segement with shortest projection
        calculate the distance to the two segments
        interpolate the cost for the index that is larger
        save to member variable
    '''
    def calculate_speeds(self, image, waypoint_list, speeds):
        # for each point in free space
        # get closest valid wp 
        #   consider both segments attached to the closest wp
          # choose the segement with shortest projection
        # calculate the distance to the two segments
        # interpolate the cost for the index that is larger
        plt.figure("Results")
        print "WAYPOINT LIST: ", waypoint_list
        print "SPEEDS: ", speeds
        image_dim = np.shape(self.image)
        velocity_map = np.ones(image_dim)*self.min_speed
        for i in range(0,image_dim[0]-1):
            pbu.print_progress_bar(i, image_dim[0]-1)
            for j in range(0,image_dim[1]-1):
                # print "value of ", i, j, ": ", image[i,j]
                if image[i,j] == 255:
                    neighbor_wp = None
                    # get the closest waypoint to the query point
                    closest_wp = self.get_closest_wp([i,j], waypoint_list)
                    if closest_wp is not None:
                        # if there is only one waypoint then the neighbor waypoint is the same as closest
                        if np.shape(waypoint_list)[0] == 1:
                            # print "only one wp"
                            neighbor_wp = 0
                        # if the closest waypoint is the first one, the next one must be the neighbor
                        elif closest_wp == 0:
                            neighbor_wp = 1
                        # if the closest waypoint is the last in the list, the neighbor must be the previous
                        elif closest_wp == np.shape(waypoint_list)[0]-1:
                            neighbor_wp = closest_wp - 1
                        # otherwise, we have to check both segments to see which has the shortest projection
                        # results returns a tuple of (projection distance, projection point)
                        else:
                            results_a = self.get_projection([i,j], waypoint_list[closest_wp], waypoint_list[closest_wp-1])
                            results_b = self.get_projection([i,j], waypoint_list[closest_wp+1], waypoint_list[closest_wp])
                            if results_a[0]>results_b[0]:
                                neighbor_wp = closest_wp+1
                            else:
                                neighbor_wp = closest_wp-1
                        # Some ugly swtich code to make sure that the wayupoints are given in the right order to the projection method
                        if closest_wp < neighbor_wp:
                            # print "closest, neighbor: ", closest_wp, neighbor_wp
                            results = self.get_projection([i,j], waypoint_list[closest_wp], waypoint_list[neighbor_wp])
                            velocity_map[i,j] = self.interpolate_speed(results[1], waypoint_list[closest_wp], 
                                waypoint_list[neighbor_wp], self.speeds[closest_wp], self.speeds[neighbor_wp])
                        else:
                            results = self.get_projection([i,j], waypoint_list[neighbor_wp], waypoint_list[closest_wp])
                            velocity_map[i,j] = self.interpolate_speed(results[1], waypoint_list[neighbor_wp], 
                                waypoint_list[closest_wp], self.speeds[neighbor_wp], self.speeds[closest_wp])
        # show the velocity map
        plt.imshow(velocity_map)
        self.velocity_map = velocity_map
        plt.colorbar()
        plt.show(block=False)
        plt.savefig('map.png')


    '''
        Returns the index of the closest waypoint that doesn't
        intersect the occupied portions of the prior map
        if there are no valid waypoints, return None
    '''
    def get_closest_wp(self, point, waypoint_list):
        # Sort the waypoint list first to check in order of nearness
        sorted_waypoint_list = sorted(waypoint_list, key=lambda e: self._distance(e, point))
        sorted_idx_list = sorted(range(len(sorted_waypoint_list)), 
            key=lambda x: self._distance(waypoint_list[x], point))
        intersects = False
        for idx, wp in enumerate(sorted_waypoint_list):
            if point == wp:
                return sorted_idx_list[idx]
            else:
                delta = np.array(wp)-np.array(point)
                # get the minimum number of steps we should take
                min_steps = max(abs(wp[1]-point[1]), abs(wp[0]-point[0]))
                # step along the line
                for t in np.linspace(0, 1, min_steps*2):
                    query = point + t*delta
                    query = query.astype(int)
                    # TODO(katliu): Debug, remove
                    # if point == [3,3]:
                    #     print query
                    
                    if self.image[query[0], query[1]] == 0:
                        intersects = True
                        break
                if not intersects:
                    return sorted_idx_list[idx]
                else:
                    # reset the variable
                    intersects = False
        return None


    '''
        Get the distance from point to 
        line segment between wp_ind_a and wp_ind_b
        return a tuple of (distance, intersection)
    '''
    def get_projection(self, point, waypoint_a, waypoint_b):

        wp_a = np.array(waypoint_a, dtype=np.float32)
        wp_b = np.array(waypoint_b, dtype=np.float32)

        # ab = wp_a - wp_b
        ab = wp_b - wp_a
        l2 = np.dot(ab, ab)

        if l2 == 0.0:
            return wp_a

        # TODO: Flipped from Jake and John's version
        line_t = np.dot((point - wp_a), ab)/l2
        # line_t = np.dot((wp_a - point), ab)/l2
        seg_t = np.max([0.0, np.min([1.0, line_t])])
        projection = wp_a + seg_t*ab
        distance = np.linalg.norm(point - projection)
        projection = projection.astype(int)
        # TODO(katliu): Debug, remove
        # print "line_t: ", line_t, " seg_t: ", seg_t, " distance: ", distance, " projection: ", projection
        return (distance, projection)

    '''
        Get the speed for a point that falls at point 
        between a and b
    '''
    def interpolate_speed(self, point, waypoint_a, waypoint_b, speed_a, speed_b):

        wp_a = np.array(waypoint_a)
        wp_b = np.array(waypoint_b)    

        dist = np.linalg.norm(wp_a - wp_b)
        if dist == 0.0:
            return float(speed_b)
        point_dist = np.linalg.norm(wp_a - point)
        alpha = (point_dist/dist)
        # TODO(katliu): Something about the algebra here is a bit weird (flipepd from Jake + John's)
        return alpha*speed_b + (1.0-alpha)*speed_a
        # return alpha*speed_a + (1.0-alpha)*speed_b

    '''
        Save waypoints to file so that user can load them later
    '''
    def save_waypoints_to_file(self):
        concatenated_numbers = np.concatenate((np.array(self.waypoints), np.transpose(np.array([self.speeds]))), axis=1)
        file_location = os.path.splitext(self.map_output_location)[0]
        np.savetxt(file_location+".txt", concatenated_numbers)

    '''
        Load waypoints from text file into member variables
    '''
    def load_waypoints_from_file(self):
        file_location = os.path.splitext(self.map_output_location)[0]
        data = np.loadtxt(file_location+".txt")
        self.waypoints = data[:,0:2]
        self.waypoints = self.waypoints.tolist()
        self.speeds = data[:,2]
        self.speeds = self.speeds.tolist()

    '''
        Render waypoints on the axis provided
    '''
    def render_on_map(self):
        plt.figure("Waypoint selection")
        for idx, wp in enumerate(self.waypoints):
            waypoint_marker = patches.Ellipse((wp[1], wp[0]), 10, 10)
            self.ax.add_patch(waypoint_marker)
            self.ax.annotate(str(self.speeds[idx]), xy=(wp[1], wp[0]), color='green')
            if idx>0 and idx < len(self.waypoints):
                line = [self.waypoints[idx-1],wp]
                (y, x) = zip(*line)
                line_marker = self.ax.add_line(lines.Line2D(x, y, linewidth=1, color='red'))
            plt.draw()

    def main_loop(self):
        if self.load_from_txt is not None:
            self.load_waypoints_from_file()
            self.render_on_map()
        keep_going = True
        # keep looping until the 'q' key is pressed
        prompt = """Enter 
        [w] to enter the next waypoint, 
        [p] to print the existing waypoints, 
        [f] to write the existing waypoints to file,
        [a] to analyze the selection,
        [r] to render the waypoints on the underlay,
        [s] to save to file, 
        [q] to quit\n"""
        while keep_going:
            # display the image and wait for a keypress
            key = str(raw_input(prompt))
            # if the 'w' key is pressed, set down a waypoint
            if key == "w":
                print "==== ACCEPTING WAYPOINTS"
                self.accepting_waypoints = True
                self.accepted_waypoint = False
                while self.accepting_waypoints:
                    key = str(raw_input('Enter [e] to exit, [y] to accept waypoint selection\n'))
                    if key == "e":
                        print ("Exit waypoint selection")
                        self.accepting_waypoints = False
                    elif key == "y":
                        if len(self.waypoints) ==0 or (self.waypoints[-1]!=self.candidate_waypoint):
                            # Accept the waypoint, and commit to the markers on the plot
                            print ("Accepting waypoint selection")
                            self.waypoints.append(self.candidate_waypoint)
                            self.all_plot_objs.append(self.candidate_waypoint_marker)
                            self.all_plot_objs.append(self.candidate_line_marker)
                            self.candidate_waypoint_marker = None
                            self.candidate_line_marker = None
                            self.accepting_waypoints = False
                            print "Enter the speed for this waypoint:"
                            # make sure a valid number is input 
                            while True:
                                try:
                                   speed = float(raw_input())
                                except ValueError:
                                   print '\x1b[1;33;41m' + 'ERROR: please enter an valid float.' + '\x1b[0m' 
                                   continue
                                else:
                                    if speed > 0.0 and speed <= 20.0:
                                        break
                                    else:
                                        print '\x1b[1;33;41m' + 'ERROR: please enter a number between 0.0 and 20.0' + '\x1b[0m' 
                            self.speeds.append(speed)
                            self.ax.annotate(str(speed), xy=(self.waypoints[-1][1], self.waypoints[-1][0]), color='green')
                            plt.draw()
                            plt.show(block=False)
                        else:
                            print '\x1b[1;33;41m' + 'ERROR: Redundant waypoint entered. Please choose a different valid waypoint.' + '\x1b[0m'

            elif key == "a":
                print "==== ANALYZING"
                if self.use_potential:
                    self.potential_calculate_speeds(self.image, self.waypoints, self.speeds)
                else:
                    self.calculate_speeds(self.image, self.waypoints, self.speeds)
                self.save_waypoints_to_file()
                plt.show(block=False)

            elif key == "p":
                print "Waypoints: ", self.waypoints
                print "Speeds: ", self.speeds

            elif key == "f":
                self.save_waypoints_to_file()

            # elif key == "o":
            #     self.load_waypoints_from_file()
            #     self.render_on_map(self.ax)

            elif key == "s":
                print "==== SAVING TO FILE"
                if self.velocity_map is not None:
                    #Rescale to 0-255 and convert to uint8
                    scaled_copy = self.velocity_map.copy()
                    scaled_copy = scaled_copy*(100/20)  # map from [0,20] to [0,100]
                    scaled_copy = np.uint8(scaled_copy)
                    # plt.figure("scaled_map")
                    # plt.imshow(scaled_copy, cmap='gray')
                    im = Image.fromarray(scaled_copy)
                    im.save(self.map_output_location)
                else:
                    print '\x1b[1;33;41m' + 'ERROR: Velocity map has not been generated yet. Try option [a] first.' + '\x1b[0m'

            elif key == "r":
                print "==== RENDERING ON UNDERLAY IMAGE"
                if self.underlay_image_path is not None:
                    # TODO(katliu): Save to file
                    plt.figure("Underlay")
                    for idx, wp in enumerate(self.waypoints):
                        waypoint_marker = patches.Ellipse((wp[1], wp[0]), 10, 10)
                        self.underlay_ax.add_patch(waypoint_marker)
                        self.underlay_ax.annotate(str(self.speeds[idx]), xy=(wp[1], wp[0]), color='green')
                        if idx>0 and idx < len(self.waypoints):
                            line = [self.waypoints[idx-1],wp]
                            (y, x) = zip(*line)
                            line_marker = self.underlay_ax.add_line(lines.Line2D(x, y, linewidth=1, color='red'))
                        plt.draw()
                else:
                    print '\x1b[1;33;41m' + 'ERROR: Underlay image was not provided.' + '\x1b[0m'
                      
           # if the 'q' key is pressed, break from the loop
            elif key == "q":
                break

if __name__ == "__main__":
    # TODO(katliu): Would be nice to let user click on the waypoint to move it slightly

    # Parse the args
    parser = argparse.ArgumentParser(
        description="Create a velocity map png.")
    parser.add_argument('mask', help='Path to the black and white masked image.', action='store', default='/home/simbox/fla_configuration/mission/building_77_simulation_overhead.png')
    parser.add_argument('output', help='Path to output location', action='store')
    parser.add_argument('--overhead', help='Path to input image.', action='store', default=None)
    parser.add_argument('--min_speed', help='Minimum speed.', action='store', default=6.0)
    parser.add_argument('--load_from_txt', help='File location with saved waypoints.', action='store')
    parser.add_argument('--potential', help='Use the "potential" method for analyzing.', dest='use_potential', action='store_true')
    parser.add_argument('--no-potential', help='Use the "potential" method for analyzing.', dest='use_potential', action='store_false')
    parser.set_defaults(potential=False)
    args = parser.parse_args()

    velocity_map_maker = VelocityMapMaker(args.mask, args.overhead, args.output, args.min_speed, args.load_from_txt, args.use_potential)
    velocity_map_maker.main_loop()
