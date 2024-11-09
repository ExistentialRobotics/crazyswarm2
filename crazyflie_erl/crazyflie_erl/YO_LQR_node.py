import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist, PoseStamped
from motion_capture_tracking_interfaces.msg import NamedPoseArray
from YO_Controller import YOState, YO_Controller
import time
import numpy as np
from transforms3d.euler import quat2euler

#################################################################
##        Control node that commands all the crazyflies        ##
#################################################################


class Crazyswarm2ERLCommander(Node):

    def __init__(self):
        super.__init__('CS2ERLCommander_node')

        self.controller, self.dt = self.define_controller()
        self.cf_locked = True

        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
        self.subscriber = self.create_subscription(NamedPoseArray, '/cf231/pose',self.callback,qos_profile)
        self.publisher = self.create_publisher(Twist, '/cf231/cmd_vel_legacy', 10)

        self.last_thrust = 0
        self.last_pos = np.zeros(3)


    def define_controller(self):
        G = self.get_parameter('Environment/G').get_parameter_value()
        M = self.get_parameter('Environment/M').get_parameter_value()
        MAX_THRUST = self.get_parameter('Environment/MAX_THRUST').get_parameter_value()
        CTRL_TIMESTEP = self.get_parameter('Environment/CTRL_TIMESTEP').get_parameter_value()
        max_yank_steps = self.get_parameter('yo_lqr/max_yank_steps').get_parameter_value()
        max_pitch_roll_rate_error = self.get_parameter('yo_lqr/max_pitch_roll_rate_error').get_parameter_value()
        max_yaw_rate_error = self.get_parameter('yo_lqr/max_yaw_rate_error').get_parameter_value()
        max_vel_error = self.get_parameter('yo_lqr/max_vel_error').get_parameter_value()
        max_pos_error = self.get_parameter('yo_lqr/max_pos_error').get_parameter_value()
        max_yaw_error = self.get_parameter('yo_lqr/max_yaw_error').get_parameter_value()
        max_pitch_roll_error = self.get_parameter('yo_lqr/max_pitch_roll_error').get_parameter_value()
        c_params = [G,M,MAX_THRUST,CTRL_TIMESTEP,
                    max_yank_steps,max_pitch_roll_rate_error,max_yaw_rate_error,
                    max_vel_error,max_pos_error,max_yaw_error,max_pitch_roll_error]
        controller = YO_Controller(c_params)
       
        return controller, CTRL_TIMESTEP


    def callback(self,msg:PoseStamped):
        
        control_msg = Twist()

        if self.cf_locked:
            self.cf_locked = False
            self.last_pos = curr_pos
            for i in range(5):     # needed to switch to our controller 
                self.publisher.publish(control_msg)

        ori = quat2euler([msg.pose.orientation.x,
                          msg.pose.orientation.y,
                          msg.pose.orientation.z,
                          msg.pose.orientation.w])
        
        curr_pos = np.array([msg.pose.position.x,
                             msg.pose.position.y,
                             msg.pose.position.z])
        
        curr_vel = (curr_pos - self.last_pos)/self.dt
        
        state = YOState(*ori,self.last_thrust,*curr_vel,*curr_pos)
        command = self.controller.get_singlecf_control(state)

        control_msg.linear.y, control_msg.linear.x, control_msg.angular.z = command[:-1]
        control_msg.linear.z = int(command[-1])

        self.publisher.publish(control_msg)
        self.get_logger().info(f'Published {control_msg} at time {time.time()}')

        self.last_pos = curr_pos
        self.last_thrust = control_msg.linear.z
        


def main(args=None):
    rclpy.init(args=args)
    CS2ERLCommander = Crazyswarm2ERLCommander()
    rclpy.spin(CS2ERLCommander)
    rclpy.shutdown()

if __name__ == '__main__':
    main()