from model import *
from control import LQRYankOmegaController, YankOmegaController
from utils.env_builder import Environment
import numpy as np
import rclpy

class YOState:
    
    def __init__(self, r,p,y,T,vx,vy,vz,px,py,pz):
        self.r = r
        self.p = p
        self.y = y
        self.T = T
        self.vx = vx
        self.vy = vy
        self.vz = vz
        self.px = px
        self.py = py
        self.pz = pz

    
    def set_from_idx(self, idx, value):
        if idx == 0:
            self.r = value
        elif idx == 1:
            self.p = value
        elif idx == 2:
            self.y = value
        elif idx == 3:
            self.T = value
        elif idx == 4:
            self.vx = value
        elif idx == 5:
            self.vy = value
        elif idx == 6:
            self.vz = value
        elif idx == 7:
            self.px = value
        elif idx == 8:
            self.py = value
        elif idx == 9:
            self.pz = value
        else:
            raise ValueError("Index out of bounds")

    def pos(self):
        return np.array([self.px, self.py, self.pz])
    
    def rpy(self):
        return np.array([self.r, self.p, self.y])
    
    def vel(self):
        return np.array([self.vx, self.vy, self.vz])
    
    def set_pos(self, pos):
        self.px = pos[0]
        self.py = pos[1]
        self.pz = pos[2]
    
    def set_rpy(self, rpy):
        self.r = rpy[0]
        self.p = rpy[1]
        self.y = rpy[2]
    
    def set_vel(self, vel):
        self.vx = vel[0]
        self.vy = vel[1]
        self.vz = vel[2]
    
    def __setitem__(self, index, value):
        self.set_from_idx(index, value)
    
    def __getitem__(self, index):
        return self.get_state_vec()[index]

    def copy(self):
        return YOState(self.r, self.p, self.y, self.T, self.vx, self.vy, self.vz, self.px, self.py, self.pz)
    
    def get_state_vec(self):
        return np.array([self.r,
                         self.p,
                         self.y,
                         self.T,
                         self.vx,
                         self.vy,
                         self.vz,
                         self.px,
                         self.py,
                         self.pz])


class YO_Controller:

    def __init__(self,params, N2cfThrust_conv_factor=1.0): 
        [G,M,MAX_THRUST,CTRL_TIMESTEP,
        max_yank_steps,max_pitch_roll_rate_error,max_yaw_rate_error,
        max_vel_error,max_pos_error,max_yaw_error,max_pitch_roll_error] = params       

        env = Environment(G=G, M=M, MAX_THRUST=MAX_THRUST, CTRL_TIMESTEP=CTRL_TIMESTEP)
        self.env = env
        self.yo_model = LinearizedYankOmegaModel(env)
        self.robot_idx = 0
        yo_ctrl = YankOmegaController(env)

        Q,R = self.update_lqr_params(params)
        self.lqr = LQRYankOmegaController(self.env, self.yo_model, yo_ctrl, Q=Q, R=R)
        self.N2cfThrust_conv_factor = N2cfThrust_conv_factor
        print(Q)
        print(R)

    def update_setpoint(self, pos, vel=[0,0,0], yaw=0):
        acc = [0,0,0] #these are ignored for YO controller
        omega = [0,0,0]
        self.lqr.set_desired_trajectory(None, pos, vel, acc, yaw, omega)


    def update_lqr_params(self, params, apply=False):
        [G,M,MAX_THRUST,CTRL_TIMESTEP,
            max_yank_steps,max_pitch_roll_rate_error,max_yaw_rate_error,
            max_vel_error,max_pos_error,max_yaw_error,max_pitch_roll_error] = params  
                # Brysons rule, essentially set Rii to be 1/(u^2_i) where u_i is the max input for the ith value)
        max_yank = (MAX_THRUST / CTRL_TIMESTEP ) / max_yank_steps # guess? don't have an intuition for yank yet

        rflat = [1 / (max_yank ** 2), 1 / (max_pitch_roll_rate_error ** 2), 1 / (max_pitch_roll_rate_error ** 2),
                1 / (max_yaw_rate_error ** 2)]
        R = np.diag(rflat)

        lqr_max_thrust = MAX_THRUST - M * G #max thrust in equilibrium input

        # stack into Q in order of state x=[r, p, y, T, vx, vy, vz, px, py, pz] (T is thrust)
        qflat = [1 / (max_pitch_roll_error ** 2), 1 / (max_pitch_roll_error ** 2), 1 / (max_yaw_error ** 2),
                1/(lqr_max_thrust**2), 1 / (max_vel_error ** 2), 1 / (max_vel_error ** 2), 1 / (max_vel_error ** 2),
                1 / (max_pos_error ** 2), 1 / (max_pos_error ** 2), 1 / (max_pos_error ** 2)]
        Q = np.diag(qflat)

        if apply:
            self.lqr.Q = Q
            self.lqr.R = R
            self.lqr.compute_gain_matrix()
        else:
            return Q,R

    def get_singlecf_control(self, x: YOState, dt=None):
        _, u = self.lqr.compute(x=x.get_state_vec(), skip_low_level=True)
        cf_input = self.convert_to_cf_input(u, x, dt)
        return cf_input
    
    def convert_to_cf_input(self, u, x: YOState, dt=None):
        yank, w_x, w_y, w_z = u
        w_z = 0 #TODO investigate, ignore yaw rate for now
        if dt is None:
            dt = self.env.CTRL_TIMESTEP
        thrust = x.T + yank * dt 
        roll = x.r + w_x * dt 
        pitch = x.p + w_y * dt
        yaw_rate = -w_z
       
        cf_thrust = max(min(self.N2cfThrust_conv_factor * thrust, 60000.0), 0.0)
        roll, pitch, yaw_rate = (180.0/np.pi)*np.array([roll, pitch, yaw_rate]) #convert to degrees for crazyflie library
        pitch = max(min(pitch, 30.0), -30.0)
        roll = max(min(roll, 30.0), -30.0)

        return np.array([roll, pitch, yaw_rate, cf_thrust])

    def cf_input_to_u(self, cf_input, x, dt=None):
        if dt is None:
            dt = self.env.CTRL_TIMESTEP
        roll, pitch, yaw_rate, cf_thrust = cf_input
        roll_rad = (np.pi/180.0) * roll
        pitch_rad = (np.pi/180.0) * pitch
        yaw_rate_rad = -yaw_rate * (np.pi/180.0)
        thrust = cf_thrust / self.N2cfThrust_conv_factor
        yank = (thrust - x.T) / dt
        w_x = (roll_rad - x.r) / dt
        w_y = (pitch_rad - x.p) / dt
        w_z = yaw_rate_rad
        return np.array([yank, w_x, w_y, w_z])