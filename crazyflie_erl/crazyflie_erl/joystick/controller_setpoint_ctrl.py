from joystick.xbox_buttons import XboxJoyMessage, XboxHandler, BUTTON_MAP, AXES_MAP
import numpy as np
import time

class JoySetpointControl:

    def __init__(self, node=None, handler=None, joy_topic='/joy', default_setpoint=np.array([0.0,0.0,1.0]), callback=None) -> None:
        #for now right stick will be x,y velocity, right stick y will be z velocity
        
        if handler is None and node is None:
            raise ValueError("Either node or handler must be provided")
        
        if handler is None:
            handler = XboxHandler(node, topic_name=joy_topic)
        
        self.node = handler.node
        self.setpoint = default_setpoint.copy()
        self.default_setpoint = default_setpoint.copy()
        # if any of the right stick is moved, or the left vertical is moved while A is pressed, update the setpoint
        handler.register_gated_axis_callback(self.setpoint_callback, (AXES_MAP["RStickX"], AXES_MAP["RStickY"], AXES_MAP["LStickY"]), (BUTTON_MAP["LB"],))
        handler.register_callback(self.reset_setpoint, buttons=(BUTTON_MAP["RB"],))
        handler.register_callback(self.toggle_goal_vel, buttons=(BUTTON_MAP["A"],))
        self.SCALE_FACTOR = 0.5
        self.last_time = None
        self.use_goal_vel = False
        self.goal_vel = np.array([0.0,0.0,0.0])
        self.callback = callback
    

    def reset_setpoint(self, button):
        if button:
            self.node.get_logger().info("Resetting setpoint to default: {}".format(self.default_setpoint))
            self.setpoint = self.default_setpoint.copy()
            self.goal_vel = np.array([0,0,0])
            if self.callback is not None:
                self.callback(self.setpoint, self.goal_vel)


    def toggle_goal_vel(self, button):
        if button: # toggle on rising edge
            self.use_goal_vel = not self.use_goal_vel
        print(f"Use goal vel: {self.use_goal_vel}")

    def get_current_goal(self):
        return self.setpoint, self.goal_vel
    
    def setpoint_callback(self, rx_axis, ry_axis, ly_axis):
        # map the joystick values to the setpoint
        vx = ry_axis
        vy = rx_axis
        vz = ly_axis

        time_now = time.time()
        # get the time
        if self.last_time is None:
            self.last_time = time.time()
            return
        
        dt = time_now - self.last_time
        
        #move the setpoint by the joystick values by some scaled dt
        self.setpoint += self.SCALE_FACTOR * np.array([vx,vy,vz]) * dt

        if self.use_goal_vel:
            self.goal_vel = self.SCALE_FACTOR * np.array([vx,vy,vz])
        else:
            self.goal_vel = np.array([0,0,0])

        self.last_time = time_now

        if self.callback is not None:
            self.callback(self.setpoint, self.goal_vel)