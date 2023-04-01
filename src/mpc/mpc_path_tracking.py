import math
import cvxpy
import numpy as np
from math import inf, sin, cos, tan, sqrt, atan2, pi


# reference: A Survey of Motion Planning and Control Techniques for Self-Driving Urban Vehicles, PythonRobotics

class mpc_path_tracking:
    def __init__(self, receding=5, cs=np.diag([1, 1, 1]), cu=1, cst=np.diag([1, 1, 1]), cut=1, max_speed=6, max_steer=pi/4, ref_speed=4, max_acce=6, max_acce_steer=1, sample_time=0.1, wheelbase=2, iter_threshold=0.1):

        self.receding = receding # receding horizon length
        
        self.cs = cs  # coefficient, weight of s-s_ref 3*3
        self.cu = cu  # coefficient, weight of u-u_ref 1
        self.cst = cst # coefficient, weight of sT-s_refT 3*3
        self.cut = cut # coefficient, weight of uT-u_refT

        self.cur_ind = 0  # reference path point index
        self.max_speed = max_speed
        self.max_steer = max_steer
        self.dt = sample_time
        self.L = wheelbase
        self.ref_speed = ref_speed
        self.max_acce = max_acce
        self.max_acce_steer = max_acce_steer
        
        self.iter_threshold = iter_threshold

        self.cur_vel_array = np.zeros((2, receding))  # speed, steer
        self.gear = 0 # 0 forward   1 backward
        self.speed_flag = 1 # 1 forward -1 backward

    def controller(self, car_state, ref_path, iter_num=5, debug=False, **kwargs):

        # car_state: x, y, theta, 3*1
        # vel_list: a list of vel under preceding horizon
        # ref_path: waypoint list, waypoint: x, y, theta
        
        assert car_state.shape == (3, 1)

        flag = False

        # if self.cur_ind == 0:
        min_dis, self.cur_ind = self.closest_point(car_state, ref_path, self.cur_ind, **kwargs)
        
        if self.cur_ind == len(ref_path) - 1:
            self.ref_speed = 0
            flag = True
            print('arrive at the goal')

        u_opt_array, state_pre, ref_traj = self.iterative_solver(car_state, ref_path, self.cur_vel_array, iter_num=iter_num, **kwargs)

        self.cur_vel_array[:] = u_opt_array[:]

        if u_opt_array[0, 0] > 0:
            self.gear = 0
            self.speed_flag = 1

        if u_opt_array[0, 0] < 0:
            self.gear = 1
            self.speed_flag = -1

        return u_opt_array[:, 0:1], state_pre, flag, ref_traj

    def linear_test(self, car_state, vel):

        next_state = self.motion_predict_model(car_state, vel, self.L, self.dt)
        A, B, C = self.linear_ackermann_model(car_state, vel)
        next_linear_state = A @ car_state + B @ vel + C

        diff = next_linear_state - next_state

        return next_state, next_linear_state
        
    def closest_point(self, car_state, ref_path, start_ind, threshold=0.1, ind_range=10):
        
        min_dis = inf
        min_ind = start_ind
    
        for i, waypoint in enumerate(ref_path[start_ind:start_ind+ind_range]):

            dis = self.distance(car_state[0:2], waypoint[0:2])

            if dis < min_dis:
                min_dis = dis
                min_ind = start_ind + i

                if dis < threshold:
                    break

        return min_dis, min_ind
    
    def iterative_solver(self, car_state, ref_path, cur_vel_array, iter_num = 5):

        for i in range(iter_num):
            
            s_array_bar = self.state_predict(car_state, cur_vel_array)
            ref_traj = self.match_traj(cur_vel_array, ref_path)  # find the refer trajectory form the refer path
            s_array_bar, ref_traj = self.match_theta(s_array_bar, ref_traj) # expend the theta to match the orientation

            u_opt_array = self.convx_optimizer(car_state, ref_traj, cur_vel_array, s_array_bar)

            if u_opt_array is None:
                print('No solution')
                break

            dis = np.linalg.norm(u_opt_array - cur_vel_array)

            if dis < self.iter_threshold:
                break
            
            cur_vel_array = u_opt_array

        return u_opt_array, s_array_bar, ref_traj


    def convx_optimizer(self, car_state, ref_traj, cur_vel_array, s_array_bar):
        
        indep_s = cvxpy.Variable((3, self.receding+1))
        indep_u = cvxpy.Variable((2, self.receding))

        cost = 0
        constraints = [] 

        for t in range(self.receding):
            cost += cvxpy.quad_form(indep_s[:, t+1] - ref_traj[:, t+1], self.cs)   
            cost += self.cu * ( indep_u[0, t] - self.ref_speed * self.speed_flag) ** 2

            if t == self.receding - 1:
                cost += cvxpy.quad_form(indep_s[:, t+1] - ref_traj[:, t+1], self.cst) 
                cost += self.cut * ( indep_u[0, t] - self.ref_speed * self.speed_flag) ** 2

            A, B, C = self.linear_ackermann_model(s_array_bar[:, t:t+1], cur_vel_array[:, t:t+1])
            constraints += [ indep_s[:, t+1:t+2] == A @ indep_s[:,t:t+1] + B @ indep_u[:, t:t+1] + C]

            if t < self.receding - 1:
                constraints += [ cvxpy.abs( indep_u[0, t+1] - indep_u[0, t] ) <= self.max_acce * self.dt ]
                constraints += [ cvxpy.abs( indep_u[1, t+1] - indep_u[1, t] ) <= self.max_acce_steer * self.dt ]

        constraints += [ cvxpy.abs(indep_u[0, :]) <= self.max_speed]
        constraints += [ cvxpy.abs(indep_u[1, :]) <= self.max_steer]
        constraints += [ indep_s[:, 0] == car_state[:, 0] ]   # init state

        prob = cvxpy.Problem(cvxpy.Minimize(cost), constraints)
        prob.solve(solver=cvxpy.OSQP)
         
        if prob.status == cvxpy.OPTIMAL:
            return indep_u.value
        else:
            print('can not solve')
            return None
        
    def state_predict(self, car_state, cur_vel_array):
        
        # car_state: x, y, theta, 3*1
        # vel_list:   speed, steer, 2*1

        s_array = np.zeros((3, self.receding+1))
        s_array[:, 0] = car_state[:, 0]

        start_state = car_state

        for i in range(self.receding):
            next_s = self.motion_predict_model(start_state, cur_vel_array[:, i:i+1], self.L, self.dt)
            s_array[:, i+1] = np.squeeze(next_s) 
            start_state = next_s

        return s_array  
    
    def match_traj(self, cur_vel_array, ref_path):
        
        # cur_vel_array: t -- t+receding
        # ref_traj_array: t+1 -- t+1+receding

        ref_traj_array = np.zeros((3, self.receding+1)) 
        ref_traj_array[:, 0] = ref_path[self.cur_ind][:, 0]

        cur_ind = self.cur_ind
        traj_point = ref_path[self.cur_ind][0:2]

        for i in range(self.receding):
            # vel = cur_vel_array[0, i]
            vel = self.ref_speed
            move_len = vel * self.dt

            traj_point, cur_ind = self.inter_point(traj_point, ref_path, cur_ind, move_len)
            ref_traj_array[:, i+1] = traj_point[:]

        return ref_traj_array

    def match_theta(self, s_bar, ref_traj):
        
        for i in range(s_bar.shape[1]):
            diff = (ref_traj[2, i] - s_bar[2 ,i]) 
            ref_traj[2, i] = s_bar[2 ,i] + self.wraptopi(diff)

        return s_bar, ref_traj

    def inter_point(self, traj_point, ref_path, cur_ind, length):

        start_point = ref_path[cur_ind]
        circle = np.squeeze(traj_point[0:2])

        while True:

            if cur_ind+1 > len(ref_path) - 1:
                
                end_point = ref_path[-1][:, 0]
                end_point[2] = self.wraptopi(end_point[2] + self.gear * pi)

                return end_point, cur_ind

            cur_point = ref_path[cur_ind]
            next_point = ref_path[cur_ind + 1]

            segment = [np.squeeze(cur_point[0:2]) , np.squeeze(next_point[0:2])]
            int_point = self.range_cir_seg(circle, length, segment)

            if int_point is None:
                cur_ind = cur_ind + 1
            else:
                
                diff = self.wraptopi(next_point[2, 0] - cur_point[2, 0])
                theta = self.wraptopi(cur_point[2, 0] + diff / 2 + self.gear * pi)
                traj_point = np.append(int_point, theta)

                return traj_point, cur_ind


    def range_cir_seg(self, circle, r, segment):
        
        assert circle.shape == (2,) and segment[0].shape == (2,) and segment[1].shape == (2,)

        sp = segment[0]
        ep = segment[1]

        d = ep - sp

        if np.linalg.norm(d) == 0:
            return None

        # if d.all() == 0:
        #     return None

        f = sp - circle

        a = d @ d
        b = 2* f@d
        c = f@f - r ** 2

        discriminant = b**2 - 4 * a * c

        if discriminant < 0:
            return None
        else:
            
            t1 = (-b - sqrt(discriminant)) / (2*a)
            t2 = (-b + sqrt(discriminant)) / (2*a)

            # 3x HIT cases:
            #          -o->             --|-->  |            |  --|->
            # Impale(t1 hit,t2 hit), Poke(t1 hit,t2>1), ExitWound(t1<0, t2 hit), 

            # 3x MISS cases:
            #       ->  o                     o ->              | -> |
            # FallShort (t1>1,t2>1), Past (t1<0,t2<0), CompletelyInside(t1<0, t2>1)

            
            # if t2 >= 0 and t2 <=1:
            #     int_point = sp + t2 * d
            #     return int_point
            # if t1 >=0 and t1<=1:
            #     int_point = sp + t1 * d
            #     return int_point
    
            # else:
            #     if t1 >=0 and t1<=1:
            #         int_point = sp + t1 * d
            #         return int_point
                
            if t2 >= 0 and t2 <=1:
                int_point = sp + t2 * d
                return int_point

            #  ExitWound(t1<0, t2 hit), 
            return None

    def motion_predict_model(self, car_state, vel, wheel_base, sample_time):
        
        assert car_state.shape == (3, 1) and vel.shape == (2, 1) 

        phi = car_state[2, 0]

        v = vel[0, 0]
        psi = vel[1, 0]

        ds = np.array([ [v*cos(phi)], [v*sin(phi)], [ v*tan(psi) / wheel_base ] ])  
    
        next_state = car_state + ds * sample_time

        return next_state
    
    def linear_ackermann_model(self, state_bar, vel_bar):
        
        phi = state_bar[2, 0]
        v = vel_bar[0, 0]
        psi = vel_bar[1, 0]

        A = np.array([ [1, 0, -v * self.dt * sin(phi)], [0, 1, v * self.dt * cos(phi)], [0, 0, 1] ])

        B = np.array([ [cos(phi)*self.dt, 0], [sin(phi)*self.dt, 0], 
                       [ tan(psi)*self.dt / self.L, v*self.dt/(self.L * (cos(psi))**2 ) ] ])

        C = np.array([ [ phi*v*sin(phi)*self.dt ], [ -phi*v*cos(phi)*self.dt ], 
                      [ -psi * v*self.dt / ( self.L * (cos(psi))**2) ]])

        return A, B, C

    def distance(self, point1, point2):
        return np.linalg.norm(point2[0:2]-point1[0:2])

    def wraptopi(self, radian):
        while radian > pi:
            radian = radian - 2 * pi
        while radian < -pi:
            radian = radian + 2 * pi

        return radian
 

    
