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

    def __getitem__(self, index):
        return self.get_state_vec()[index]
    
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

    def __init__(self,params): 
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
        if dt is None:
            dt = self.env.CTRL_TIMESTEP
        thrust = x.T + yank * dt 
        roll = x.r + w_x * dt 
        pitch = x.p + w_y * dt
        w_z = 0 #TODO investigate, ignore yaw rate for now
        return np.array([roll, pitch, w_z, thrust])
