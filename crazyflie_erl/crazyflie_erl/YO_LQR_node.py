import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist, PoseStamped
from motion_capture_tracking_interfaces.msg import NamedPoseArray
from YO_Controller import YOState, YO_Controller
import time
import numpy as np
import yaml
from transforms3d.euler import quat2euler
from trajectories import CircleTrajectory, LineTrajectory, CompoundTrajectory
from joystick import XboxHandler, BUTTON_MAP, AXES_MAP, XboxJoyMessage
from joystick import JoySetpointControl

#################################################################
##        Control node that commands all the crazyflies        ##
#################################################################


class Crazyswarm2ERLCommander(Node):

    def __init__(self):
        super().__init__('CS2ERLCommander_node')

        self.define_controller_ros()
        self.cf_locked = True
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
        self.pose_subscriber = self.create_subscription(PoseStamped, '/cf231/pose', self.callback, qos_profile)
        self.setpoint_subscriber = self.create_subscription(Twist, '/cf231/setpoint', self.setpoint_callback, qos_profile)
        self.cmd_publisher = self.create_publisher(Twist, '/cf231/cmd_vel_legacy', 10)
        
        self.joy_handler = XboxHandler(self)
        self.joy_handler.register_callback(self.land, buttons=(BUTTON_MAP["Y"],))
        self.joy_handler.register_callback(self.reload_params, buttons=(BUTTON_MAP["Squares"],))
        self.joy_handler.register_callback(self.track_circle_traj, buttons=(BUTTON_MAP["X"],))
        self.get_logger().info("Created Xbox Handler")
        

        self.joy_setpoint_ctrl = JoySetpointControl(handler=self.joy_handler, callback=self.joy_setpoint_callback)
        self.get_logger().info(str(self.joy_handler))

        self.last_thrust = 0
        self.last_pos = np.zeros(3)
        self.setpoint = None
        radius = 0.5
        vel = 0.25
        self.circle_traj = CircleTrajectory(r=radius, v=vel)
        self.from_hover_traj = LineTrajectory(start=np.array([0,0,1]), end=np.array([radius,0,1]), speed=vel)
        self.traj = CompoundTrajectory([self.from_hover_traj, self.circle_traj])
        self.traj_time_start = None


    def define_controller_ros(self):
        self.declare_parameters(
            namespace='',
            parameters=[
                ('M', .1),
                ('G', 9.8),
                ('MAX_THRUST', 1.0),
                ('CTRL_TIMESTEP', .01),
                ('max_yank_steps', 2000),
                ('max_yaw_rate_error', .1),
                ('max_pitch_roll_rate_error', .1),
                ('max_vel_error', .1),
                ('max_pos_error', .1),
                ('max_yaw_error', .1),
                ('max_pitch_roll_error', .1),
                ('CF_HOVER_THRUST', 42000)
            ])
        M = self.get_parameter('M').get_parameter_value().double_value
        print(type(M))
        G = self.get_parameter('G').get_parameter_value().double_value 
        MAX_THRUST = self.get_parameter('MAX_THRUST').get_parameter_value().double_value
        CTRL_TIMESTEP = self.get_parameter('CTRL_TIMESTEP').get_parameter_value().double_value
        max_yank_steps = self.get_parameter('max_yank_steps').get_parameter_value().integer_value
        max_pitch_roll_rate_error = self.get_parameter('max_pitch_roll_rate_error').get_parameter_value().double_value
        max_yaw_rate_error = self.get_parameter('max_yaw_rate_error').get_parameter_value().double_value
        max_vel_error = self.get_parameter('max_vel_error').get_parameter_value().double_value
        max_pos_error = self.get_parameter('max_pos_error').get_parameter_value().double_value
        max_yaw_error = self.get_parameter('max_yaw_error').get_parameter_value().double_value
        max_pitch_roll_error = self.get_parameter('max_pitch_roll_error').get_parameter_value().double_value
        c_params = [G,M,MAX_THRUST,CTRL_TIMESTEP,
                    max_yank_steps,max_pitch_roll_rate_error,max_yaw_rate_error,
                    max_vel_error,max_pos_error,max_yaw_error,max_pitch_roll_error]

        self.controller = YO_Controller(c_params)

        #assume linear releationship between thrust in Newtons and cf thrust units
        # hover_thrust_int = mg
        # 42000/.027*9,81
        CF_HOVER_THRUST = self.get_parameter('CF_HOVER_THRUST').get_parameter_value().integer_value
        self.N2cfThrust_conv_factor = CF_HOVER_THRUST/(M*G)
        self.dt = CTRL_TIMESTEP


    def land(self, btn):
        if btn:
            control_msg = Twist()

            self.get_logger().info("Landing") 
            #blocking call to send zero motor rpm
            while(True):
                control_msg.linear.z = 0.0
                self.cmd_publisher.publish(control_msg)
    

    def track_circle_traj(self, btn):
        # self.get_logger().info("Tracking circle trajectory")
        #go from hover position 
        if btn:
            if self.traj_time_start is None:
                self.traj_time_start = time.time()
                return
            t = time.time() - self.traj_time_start
            pos, vel, acc, yaw, omega_yaw = self.traj(t)
            self.joy_setpoint_callback(pos, vel)
            if t > self.traj.get_total_time():
                self.traj_time_start = None
                self.joy_setpoint_callback(np.array([0,0,1]), np.zeros((3,)))
                return
        
        
                                       
    def reload_params(self, btn):
        self.get_logger().info("Reloading params")
        self.define_controller_yaml()
        self.get_logger().info("Reloaded params")

    def define_controller_yaml(self):
        #load the yaml file
        with open('/home/erl/multi-robot/ros2_ws/src/crazyswarm2/crazyflie_erl/config/yo_lqr_params.yaml') as file:
            params = yaml.load(file, Loader=yaml.FullLoader)
            params = params[list(params.keys())[0]]['ros__parameters']
            M = self.get_parameter('M').get_parameter_value().double_value
            G = self.get_parameter('G').get_parameter_value().double_value 
            MAX_THRUST = self.get_parameter('MAX_THRUST').get_parameter_value().double_value
            CTRL_TIMESTEP = self.get_parameter('CTRL_TIMESTEP').get_parameter_value().double_value
            max_yank_steps = params.get('max_yank_steps')
            max_pitch_roll_rate_error = params.get('max_pitch_roll_rate_error')
            max_yaw_rate_error = params.get('max_yaw_rate_error')
            max_vel_error = params.get('max_vel_error')
            max_pos_error = params.get('max_pos_error')
            max_yaw_error = params.get('max_yaw_error')
            max_pitch_roll_error = params.get('max_pitch_roll_error')
            c_params = [G,M,MAX_THRUST,CTRL_TIMESTEP,
                        max_yank_steps,max_pitch_roll_rate_error,max_yaw_rate_error,
                        max_vel_error,max_pos_error,max_yaw_error,max_pitch_roll_error]

            self.controller.update_lqr_params(c_params, apply=True)


    def setpoint_callback(self, msg:Twist):
        pos = [msg.linear.x, msg.linear.y, msg.linear.z]
        vel =  [msg.angular.x, msg.angular.y, msg.angular.z]
        yaw = 0 

        self.setpoint = (pos, vel, yaw)
        self.controller.update_setpoint(pos=pos, vel=vel, yaw=yaw)

    def joy_setpoint_callback(self, pos, vel):
        self.setpoint = (pos, vel, 0)
        # self.get_logger().info(f"Setpoint: {self.setpoint}")
        self.controller.update_setpoint(pos=pos, vel=vel, yaw=0)
    
    def callback(self,msg:PoseStamped):
        
        control_msg = Twist()

        if self.setpoint is None:
            self.get_logger().info("Waiting for setpoint...")
            # self.cmd_publisher.publish(control_msg)
            return 
        
        if self.cf_locked:
            self.cf_locked = False
            self.last_pos = np.array([msg.pose.position.x,msg.pose.position.y,msg.pose.position.z])
            for i in range(5):     # needed to switch to our controller 
                self.cmd_publisher.publish(control_msg)
            self.last_time = time.time()
            return

        # deltaR = quat2Mat(curQ).T @ quat2Mat(lastQ)
        # delta_euler = mat2Euler(deltaR)
        # ori = delta_euler + last_euler
        
        quat = [msg.pose.orientation.w,
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z]
        
        ori = quat2euler(quat)
        
        # self.get_logger().debug(f"Ori: {ori} quat: {quat}")

        curr_pos = np.array([msg.pose.position.x,
                             msg.pose.position.y,
                             msg.pose.position.z])
        
        curr_vel = (curr_pos - self.last_pos) / self.dt
        
        state = YOState(*ori,self.last_thrust,*curr_vel,*curr_pos)
        dt = time.time() - self.last_time
        command = self.controller.get_singlecf_control(state, dt)
        cf_thrust = max(min(self.N2cfThrust_conv_factor * command[-1], 60000.0), 0.0)
        roll, pitch, yaw_rate = (180.0/np.pi)*command[:-1] #convert to degrees for crazyflie library
        pitch = max(min(pitch, 30.0), -30.0)
        roll = max(min(roll, 30.0), -30.0)
        control_msg.linear.y, control_msg.linear.x, control_msg.angular.z = (roll, pitch, -yaw_rate)
        control_msg.linear.z = cf_thrust
 
        self.cmd_publisher.publish(control_msg)

        deg_ori = (180.0/np.pi)*np.array(ori)
        # self.get_logger().info(f'Published {control_msg} at time {time.time()}')
        # self.get_logger().info(f'cmd (pitch, roll): ({pitch:.2f} {roll:.2f}, {yaw_rate:.2f}) \t euler: ({deg_ori[0]:.2f} {deg_ori[1]:.2f} {deg_ori[2]:.2f})')
        self.get_logger().info(f'pos: {curr_pos} vel: {curr_vel} thrust: {cf_thrust}')
        self.last_pos = curr_pos
        self.last_thrust = float(cf_thrust) / self.N2cfThrust_conv_factor
        self.last_time = time.time()

def main(args=None):
    rclpy.init(args=args)
    CS2ERLCommander = Crazyswarm2ERLCommander()
    rclpy.spin(CS2ERLCommander)
    rclpy.shutdown()