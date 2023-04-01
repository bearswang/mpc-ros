"""
code author: Han Ruihua
reference: paper 'Classification of the Dubins set, 2001', github https://github.com/AndrewWalker/Dubins-Curves/blob/master/src/dubins.c
Note: there are some typos in the paper
"""

import numpy as np
from math import pi, sin, cos, atan2, sqrt, acos, inf

class dubins_path:
    def __init__(self, min_radius=1, theta_trans=True):
        self.min_r = min_radius
        self.theta_trans = theta_trans  # trans the theta to -pi--pi

    def preprocess(self, start_point, goal_point):
        
        assert start_point.shape == (3, 1) and goal_point.shape == (3, 1)

        dis, radian = self.relative(start_point[0:2], goal_point[0:2])

        d = dis / self.min_r

        start_theta = start_point[2, 0]
        goal_theta = goal_point[2, 0]

        alpha = (start_theta - radian) % (2*pi)
        beta = (goal_theta - radian) % (2*pi)

        return alpha, beta, d

    def shortest_path(self, start_point=np.zeros((3, 1)), goal_point=np.ones((3, 1)), step_size=0.1):

        alpha, beta, d = self.preprocess(start_point, goal_point)

        admissible_path = [self.dubins_LSL, self.dubins_RSR, self.dubins_RSL, self.dubins_LSR, self.dubins_RLR, self.dubins_LRL]

        min_length = inf
        
        for ap in admissible_path:
            t, p, q, mode = ap(alpha, beta, d)
            if t == None:
                continue
            total_length = abs(t) + abs(p) + abs(q)
            if total_length < min_length:
                min_length = total_length
                sh_t = t
                sh_p = p
                sh_q = q
                sh_mode = mode
        
        path_point_list = self.dubines_path_generate(start_point, sh_t, sh_p, sh_q, sh_mode, step_size)

        return path_point_list

    def dubines_path_generate(self, start_point, t, p, q, mode, step_size):

        length = [t, p, q]

        path_point_list = [start_point]
        init_point = start_point

        for i in range(3):
            
            if length[i] != 0:
                path_list, end_point = self.element_sample(length=length[i], start_point=init_point, steer_mode=mode[i], step_size=step_size)
                path_point_list = path_point_list + path_list
                init_point = end_point

        return path_point_list

    def element_sample(self, length, start_point, steer_mode, step_size):

        cur_x = start_point[0, 0]
        cur_y = start_point[1, 0]
        cur_theta = start_point[2, 0]

        path_list = []

        endpoint = np.zeros((3, 1))

        if steer_mode == 'L':
            steer = 1
        elif steer_mode == 'R':
            steer = -1
        if steer_mode == 'S':
            steer = 0

        curvature = steer * 1/self.min_r
        real_length = length * self.min_r
    
        rot_theta = real_length * curvature

        # calculate end point
        if curvature == 0:
            end_x = cur_x + cos(cur_theta) * real_length
            end_y = cur_y + sin(cur_theta) * real_length
        else:
            center_theta = cur_theta + steer*pi/2
            center_x = cur_x + cos(center_theta) * self.min_r
            center_y = cur_y + sin(center_theta) * self.min_r

            end_cir_theta = cur_theta + rot_theta - steer*pi/2
            end_x = center_x + self.min_r * cos(end_cir_theta)
            end_y = center_y + self.min_r * sin(end_cir_theta)

        end_theta = cur_theta + rot_theta

        if self.theta_trans:
            end_theta = self.wraptopi(end_theta)

        endpoint = np.array([[end_x], [end_y], [end_theta]])
        
        cur_length = 0
        
        # loop to generate the points
        while cur_length <= real_length - step_size:
            
            next_theta = cur_theta + curvature * step_size
            cur_length = cur_length + step_size

            if curvature == 0:
                next_x = cur_x + cos(next_theta) * cur_length
                next_y = cur_y + sin(next_theta) * cur_length
            else:
                temp_theta = next_theta - steer*pi/2
                next_x = center_x + cos(temp_theta) * self.min_r
                next_y = center_y + sin(temp_theta) * self.min_r

            if self.theta_trans:
                next_theta = self.wraptopi(next_theta)

            next_point = np.array([[next_x], [next_y], [next_theta]])

            path_list.append(next_point)

            cur_theta = next_theta

        path_list.append(endpoint)

        return path_list, endpoint

    def dubins_LSL(self, alpha, beta, d):

        mode = ['L', 'S', 'L']

        temp0 = atan2(cos(beta)- cos(alpha), d+sin(alpha)-sin(beta))
        t = (-alpha + temp0) % (2*pi)

        temp1 = 2 + d**2 - 2*cos(alpha-beta) + 2 * d * (sin(alpha) - sin(beta))

        if temp1 < 0:
            return None, None, None, mode

        p = sqrt(temp1)

        q = (beta - temp0) % (2*pi)

        return t, p, q, mode

    def dubins_RSR(self, alpha, beta, d):

        mode = ['R', 'S', 'R']

        temp0 = atan2(cos(alpha)- cos(beta), d-sin(alpha)+sin(beta))
        t = (alpha - temp0) % (2*pi)

        temp1 = 2 + d**2 - 2*cos(alpha-beta) + 2*d*(sin(beta)-sin(alpha))
        if temp1 < 0:
            return None, None, None, mode

        p = sqrt(temp1)

        q = (-beta % (2*pi) + temp0) % (2*pi)

        return t, p, q, mode

    def dubins_LSR(self, alpha, beta, d):

        mode=['L', 'S', 'R']

        temp0 = -2 + d**2 + 2*cos(alpha-beta) + 2*d*(sin(alpha) + sin(beta) )
        if temp0 < 0:
            return None, None, None, mode

        p = sqrt(temp0)

        temp1 = atan2( (-cos(alpha)-cos(beta)), (d+sin(alpha)+sin(beta)) )
        t = (-alpha+temp1 - atan2(-2, p)) % (2*pi)

        q = (-beta % (2*pi)+temp1-atan2(-2, p)) % (2*pi)

        return t, p, q, mode
        
    def dubins_RSL(self, alpha, beta, d):
        
        mode = ['R', 'S', 'L']

        temp0 = d**2-2+2*cos(alpha-beta)-2*d*(sin(alpha)+sin(beta))
        if temp0 < 0:
            return None, None, None, mode

        p = sqrt(temp0)

        temp1 = atan2( (cos(alpha)+cos(beta)), (d-sin(alpha)-sin(beta) ) )
        t = (alpha - temp1 + atan2(2, p)) % (2*pi)

        q = (beta % (2*pi) - temp1 + atan2(2, p)) % (2*pi)

        return t, p, q, mode

    def dubins_RLR(self, alpha, beta, d):

        mode = ['R', 'L', 'R']

        temp0 = (6-d**2+2*cos(alpha-beta)+2*d*(sin(alpha) - sin(beta))) / 8
        
        if abs(temp0) > 1:
            return None, None, None, mode

        p = acos(temp0)
        
        temp1 = atan2( (cos(alpha) - cos(beta)), (d-sin(alpha)+sin(beta)) )

        t = (alpha - temp1 + p/2) % (2*pi)

        q = (alpha - beta - t + p) % (2*pi)

        return t, p, q, mode

    # TYPO IN PAPER 
    def dubins_LRL(self, alpha, beta, d):
        
        mode=['L', 'R', 'L']

        temp0 = (6-d**2+2*cos(alpha-beta)+2*d*(sin(beta) - sin(alpha))) / 8 
        
        if abs(temp0) > 1:
            return None, None, None, mode

        p = acos(temp0)

        temp1 = atan2( (cos(beta)) - cos(alpha), (d+sin(alpha)-sin(beta)) )
        t = (- alpha + temp1 + p/2) % (2*pi)

        q = ( beta % (2*pi) - alpha - t  + p) % (2*pi)

        return t, p, q, mode

    def relative(self, state1, state2):
        
        dif = state2 - state1

        dis = np.linalg.norm(dif)
        radian = atan2(dif[1, 0], dif[0, 0])
        
        return dis, radian


    def wraptopi(self, radian):
        # -pi to pi

        if radian > pi:
            radian2 = radian - 2 * pi
        elif radian < -pi:
            radian2 = radian + 2 * pi
        else:
            radian2 = radian

        # diff = radian1 - radian2
        # print(diff)
        return radian2 

    