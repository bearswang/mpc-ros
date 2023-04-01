#! /usr/bin/env python3

from ast import Pass
import rospy
from geometry_msgs.msg import Twist, PoseStamped, Quaternion
from carla_msgs.msg import CarlaEgoVehicleControl
import numpy as np
from collections import namedtuple
from utils.curve_generator import curve_generator
from nav_msgs.msg import Odometry, Path
from math import atan2, sin, cos, pi
from std_msgs.msg import Float64
from gazebo_msgs.msg import ModelStates
from visualization_msgs.msg import MarkerArray, Marker
import time
import carla
import random
import numpy as np
import rospy
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path 
from collections import namedtuple
from math import atan2
from mpc.mpc_path_tracking import mpc_path_tracking
import yaml
import os

car = namedtuple('car', 'G g cone_type wheelbase abs_speed abs_steer abs_acce abs_acce_steer')
obstacle = namedtuple('obstacle', 'A b cone_type ')
scaling = 10
BASE_DIR = os.path.abspath(os.path.join( os.path.dirname( __file__ ), '..' ))
with open(f"{BASE_DIR}/src/config.yaml", 'r') as f:
    try:
        cfg = yaml.safe_load(f, Loader=yaml.FullLoader)
    except:
        cfg = yaml.safe_load(f)

odom_topic= cfg["odom_topic"]
ctl_topic= cfg["ctl_topic"]

class mpc_core:
    def __init__(self):
        
        # ros parameter
        receding = rospy.get_param('receding', 10)
        max_speed = rospy.get_param('max_speed', 8)
        ref_speed = rospy.get_param('ref_speed', 4)
        max_acce = rospy.get_param('max_acce', 1)
        max_acce_steer = rospy.get_param('max_acce_steer', 0.02)
        sample_time = rospy.get_param('sample_time', 0.2)
        max_steer = rospy.get_param('max_steer', 0.5)
        wheelbase = rospy.get_param('wheelbase', 2.87)
        self.shape = rospy.get_param('shape', [4.69, 1.85, 2.87, 1.75])   # [length, width, wheelbase, wheelbase_w] limo
     
        cs = rospy.get_param('cs', [1, 1, 1])
        cs = np.diag(cs)
        cu = rospy.get_param('cu', 1)
        cst = rospy.get_param('cst', [1, 1, 1])
        cst = np.diag(cst)
        cut = rospy.get_param('cut', 1)

        # goals
        start_position = rospy.get_param('start_position', [120.0, 170.0, 0]) 
        goal1_position = rospy.get_param('goal_position', [190.0, 170.0, 0])
        goal2_position = rospy.get_param('goal_position', [254, 170.0, 0])
        goal3_position = rospy.get_param('goal_position', [254, 123.0, -pi/2]) 
        goal4_position = rospy.get_param('finish_position', [240, 120.0, 170/90*pi/2]) 
        goal5_position = rospy.get_param('finish_position', [225, 120.0, 130/90*pi/2])
        finish_position = rospy.get_param('finish_position', [200, 150.0, pi/2]) 
        
        self.name = 'Town04'
        self.vehicle_list = []
        self.vel_linear = []
        self.vel_steer = []
        self.time_record = []
        self.marker_id = 0
        self.marker_array = MarkerArray()
        self.marker_array_car = MarkerArray()

        # mpc
        self.mpc_track = mpc_path_tracking(receding=receding, max_speed=max_speed, ref_speed=ref_speed, max_acce=max_acce, max_acce_steer=max_acce_steer, sample_time=sample_time, max_steer=max_steer, wheelbase=wheelbase, cs=np.diag([1, 1, 1]), cu=cu, cst=cst, cut=cut)
        rospy.init_node('mpc_node')

        start_point = np.c_[start_position]
        goal1_point = np.c_[goal1_position]
        goal2_point = np.c_[goal2_position]
        goal3_point = np.c_[goal3_position]
        goal4_point = np.c_[goal4_position]
        goal5_point = np.c_[goal5_position]
        finish_point = np.c_[finish_position]

        point_list = [start_point, goal1_point, goal2_point, goal3_point, goal4_point, goal5_point, finish_point]

        cg = curve_generator(point_list=point_list, curve_style='dubins', min_radius=0.5)

        self.ref_path_list = cg.generate_curve(step_size=0.1)
        self.vel = Twist()
        self.output = CarlaEgoVehicleControl()
        self.robot_state = start_point.copy()
        self.x = 0
        self.y = 0
        self.z = 0
        self.angle = 0
        
        # rviz show marker of car
        self.pub_marker_car = rospy.Publisher('car_marker', MarkerArray, queue_size=10)
        # ros topic
        rospy.Subscriber(odom_topic, Odometry, self.robot_state_callback)
        self.pub_vel = rospy.Publisher(ctl_topic, CarlaEgoVehicleControl, queue_size=10)
        self.pub_path = rospy.Publisher('dubin_path', Path, queue_size=10)
        self.pub_opt_path = rospy.Publisher('opt_path', Path, queue_size=10)
        self.path = self.generate_path(self.ref_path_list)

    def cal_vel(self, freq=10, **kwargs):

        rate = rospy.Rate(freq)
        while not rospy.is_shutdown():
            opt_vel, info, flag, _ = self.mpc_track.controller(self.robot_state, self.ref_path_list, iter_num=5)

            print('******')
            print('states:\n', self.robot_state)
            
            if flag == True:
                self.vel.linear.x = 0
                self.vel.angular.z = 0
                self.output.throttle = 0
                self.output.steer = 0
            else:
                self.vel.linear.x = round(opt_vel[0, 0], 2)
                self.vel.angular.z = round(opt_vel[1, 0], 2)
                self.output.throttle = round(opt_vel[0, 0], 2) / scaling
                self.output.steer = - round(opt_vel[1, 0], 2)

            print('actions:')
            print('linear', round(opt_vel[0, 0], 2), 'steer', round(opt_vel[1, 0], 2))
            
            self.pub_vel.publish(self.output)
            self.pub_path.publish(self.path)
            self.pub_marker_car.publish(self.marker_array_car)

            rate.sleep()

    def robot_state_callback(self, data):
        
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
        z = data.pose.pose.position.z

        quat = data.pose.pose.orientation

        raw = mpc_core.quat_to_yaw(quat)

        raw_degree = mpc_core.quat_to_yaw(quat)*180/pi

        self.x = x
        self.y = y
        self.z = z
        self.angle = raw_degree

        offset = self.shape[2] / 2
        
        self.robot_state[0] = x - offset * cos(raw)
        self.robot_state[1] = y - offset * sin(raw)
        self.robot_state[2] = raw
    

    @staticmethod
    def generate_path(ref_path_list):
        path = Path()

        path.header.seq = 0
        path.header.stamp = rospy.get_rostime()
        path.header.frame_id = 'map'

        for i in range(len(ref_path_list)):
            ps = PoseStamped()
            ps.pose.position.x = ref_path_list[i][0, 0]
            ps.pose.position.y = ref_path_list[i][1, 0]
            ps.pose.orientation.w = 1

            path.poses.append(ps)

        return path

    @staticmethod
    def quat_to_yaw(quater):
         
        w = quater.w
        x = quater.x
        y = quater.y
        z = quater.z

        raw = atan2(2* ( w*z + x *y), 1 - 2 * (pow(z, 2) + pow(y, 2)))

        return raw

    @staticmethod
    def yaw_to_quat(yaw):
         
        w = cos(yaw/2)
        x = 0
        y = 0
        z = sin(yaw/2)

        quat = Quaternion(w=w, x=x, y=y, z=z)

        return quat
