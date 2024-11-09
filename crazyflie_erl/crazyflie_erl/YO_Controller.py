from model import *
from control import LQRYankOmegaController, YankOmegaController
from utils.env_builder import Environment
import numpy as np
import rclpy

class YO_Controller:

    def __init__(self,params): 
        [G,M,MAX_THRUST,CTRL_TIMESTEP,
         max_yank_steps,max_pitch_roll_rate_error,max_yaw_rate_error,
         max_vel_error,max_pos_error,max_yaw_error,max_pitch_roll_error] = params       

        env = Environment(G=G, M=M, MAX_THRUST=MAX_THRUST, CTRL_TIMESTEP=CTRL_TIMESTEP)
        self.env = env
        self.yo_model = LinearizedYankOmegaModel(env)

        yo_ctrl = YankOmegaController(env)

        # Brysons rule, essentially set Rii to be 1/(u^2_i) where u_i is the max input for the ith value)
        max_yank = (env.MAX_THRUST / env.CTRL_TIMESTEP ) / max_yank_steps # guess? don't have an intuition for yank yet

        rflat = [1 / (max_yank ** 2), 1 / (max_pitch_roll_rate_error ** 2), 1 / (max_pitch_roll_rate_error ** 2),
                1 / (max_yaw_rate_error ** 2)]
        R = np.diag(rflat)

        max_thrust = env.MAX_THRUST - env.M* env.G #max thrust in equilibrium input

        # stack into Q in order of state x=[r, p, y, T, vx, vy, vz, px, py, pz] (T is thrust)
        qflat = [1 / (max_pitch_roll_error ** 2), 1 / (max_pitch_roll_error ** 2), 1 / (max_yaw_error ** 2),
                1/(max_thrust**2), 1 / (max_vel_error ** 2), 1 / (max_vel_error ** 2), 1 / (max_vel_error ** 2),
                1 / (max_pos_error ** 2), 1 / (max_pos_error ** 2), 1 / (max_pos_error ** 2)]
        Q = np.diag(qflat)


        self.lqr = LQRYankOmegaController(self.env, self.yo_model, yo_ctrl, Q=Q, R=R)
        print(Q)
        print(R)


    def get_singlecf_control():
        raise NotImplementedError



def main():
    rclpy.init()
    node = YO_Controller()
    rclpy.spin(node)



