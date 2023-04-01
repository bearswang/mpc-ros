from utils.dubins_path import dubins_path
import matplotlib.pyplot as plt
import numpy as np
from math import atan2

class curve_generator:
    def __init__(self, point_list=[], select_mode='default', curve_style='line', point_style='state', min_radius=3, theta_trans=True):

        # select_mode: mode of point selection
            # 'default', use the input point_list
            # 'mouse', use mouse to select points (vectors)
        
        # curve_style: 
            # line: connect the points with line
            # reeds: connect the points with reeds shepp path
            # dubins: connect the points with dubins path
            # cubic: connect the points with cubic spline
        # point style: 
            # waypoint: x,y 2 * 1 matrix
            # state: x, y, theta, 3*1 matrix

        # point: x, y, theta

        self.point_list = point_list
        self.select_mode = select_mode
        self.curve_style = curve_style
        self.point_style = point_style
        self.min_radius = min_radius
        self.theta_trans = theta_trans
        # self.index = 

    def generate_curve(self, step_size=1, x_limit = [0, 10], y_limit=[0, 10] ):
        
        if self.select_mode == 'default':
            pass

        elif self.select_mode == 'mouse':
            # generate reference path by mouse 
            fig, ax = plt.subplots()
            ax.set_xlim(x_limit)
            ax.set_ylim(y_limit)
            print('Using mouth to select the way point')

            def onclick(event):
                print('%s click: button=%d, x=%d, y=%d, xdata=%f, ydata=%f' %
                    ('double' if event.dblclick else 'single', event.button,
                    event.x, event.y, event.xdata, event.ydata))

                ax.scatter(event.xdata, event.ydata, c='k')
                plt.pause(0.001)
                waypoint = np.array([ [event.xdata], [event.ydata] ])
                self.point_list.append(waypoint)

            cid = fig.canvas.mpl_connect('button_press_event', onclick)
            plt.show()
        
        if self.curve_style == 'dubins':
            dubins = dubins_path(min_radius=self.min_radius, theta_trans=self.theta_trans)
            path_point_list = []
            for i in range(len(self.point_list) - 1):
                start_point = self.point_list[i]
                goal_point = self.point_list[i+1]
                path_point_list = path_point_list + dubins.shortest_path(start_point, goal_point, step_size)

                if i != len(self.point_list) - 2:
                    del path_point_list[-1]

            return path_point_list

        if self.curve_style == 'reeds':
            pass

            
      
    def mouse_select(self, x_limit=[0, 10], y_limit=[0, 10]): 

        fig, ax = plt.subplots()

        ax.set_xlim(x_limit)
        ax.set_ylim(y_limit)

        point_list = []
        click_ind = 0

        def onclick(event):
            
            print('%s click: button=%d, x=%d, y=%d, xdata=%f, ydata=%f' %
                ('double' if event.dblclick else 'single', event.button,
                event.x, event.y, event.xdata, event.ydata))
            
            if self.point_style == 'waypoint':
                
                ax.scatter(event.xdata, event.ydata, c='k')
                waypoint = np.array([ [event.xdata], [event.ydata] ])
                point_list.append(waypoint)
                plt.pause(0.001)
            
            elif self.point_style == 'vector':
                pass
                
        cid = fig.canvas.mpl_connect('button_press_event', onclick)
        plt.show()
        
    