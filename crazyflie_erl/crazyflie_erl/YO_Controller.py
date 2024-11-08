from model import *
from control import LQRYankOmegaController, YankOmegaController
from utils.env_builder import Environment
import numpy as np
import rclpy

class YO_Controller(rclpy.node.Node):

    def __init__(self):          
        G = self.get_parameter('Environment/G').get_parameter_value()
        M = self.get_parameter('Environment/M').get_parameter_value()
        MAX_THRUST = self.get_parameter('Environment/MAX_THRUST').get_parameter_value()
        CTRL_TIMESTEP = self.get_parameter('Environment/CTRL_TIMESTEP').get_parameter_value()

        env = Environment(G=G, M=M, MAX_THRUST=MAX_THRUST, CTRL_TIMESTEP=CTRL_TIMESTEP)
        self.env = env
        self.yo_model = LinearizedYankOmegaModel(env)

        yo_ctrl = YankOmegaController(env)

        # Brysons rule, essentially set Rii to be 1/(u^2_i) where u_i is the max input for the ith value)
        max_yank_steps = self.get_parameter('yo_lqr/max_yank_steps').get_parameter_value()

        max_yank = (env.MAX_THRUST / env.CTRL_TIMESTEP ) / max_yank_steps # guess? don't have an intuition for yank yet

        max_pitch_roll_rate_error = self.get_parameter('yo_lqr/max_pitch_roll_rate_error').get_parameter_value()
        max_yaw_rate_error = self.get_parameter('yo_lqr/max_yaw_rate_error').get_parameter_value()

        rflat = [1 / (max_yank ** 2), 1 / (max_pitch_roll_rate_error ** 2), 1 / (max_pitch_roll_rate_error ** 2),
                1 / (max_yaw_rate_error ** 2)]
        R = np.diag(rflat)


        max_vel_error = self.get_parameter('yo_lqr/max_vel_error').get_parameter_value()
        max_pos_error = self.get_parameter('yo_lqr/max_pos_error').get_parameter_value()
        max_yaw_error = self.get_parameter('yo_lqr/max_yaw_error').get_parameter_value()
        max_pitch_roll_error = self.get_parameter('yo_lqr/max_pitch_roll_error').get_parameter_value()
        max_thrust = env.MAX_THRUST - env.M* env.G #max thrust in equilibrium input

        # stack into Q in order of state x=[r, p, y, T, vx, vy, vz, px, py, pz] (T is thrust)
        qflat = [1 / (max_pitch_roll_error ** 2), 1 / (max_pitch_roll_error ** 2), 1 / (max_yaw_error ** 2),
                1/(max_thrust**2), 1 / (max_vel_error ** 2), 1 / (max_vel_error ** 2), 1 / (max_vel_error ** 2),
                1 / (max_pos_error ** 2), 1 / (max_pos_error ** 2), 1 / (max_pos_error ** 2)]
        Q = np.diag(qflat)


        self.lqr = LQRYankOmegaController(self.env, self.yo_model, yo_ctrl, Q=Q, R=R)
        print(Q)
        print(R)



def main():
    rclpy.init()
    node = YO_Controller()
    rclpy.spin(node)



