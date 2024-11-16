import sensor_msgs
from sensor_msgs.msg import Joy

BUTTON_MAP = {
        "A": 0,
        "B": 1,
        "X": 2,
        "Y": 3,
        "LB": 4, #left bumper
        "RB": 5, #right bumper
        "Squares": 6, #squares
        "Hamburger": 7, # maybe change names
        "LStick": 9,
        "RStick": 10
    }

INV_BUTTON_MAP = {v: k for k, v in BUTTON_MAP.items()}
#the axes are all x left is positive, y up is positive, the triggers are 1 at rest, -1 at full press
AXES_MAP = {
        "LStickX": 0,
        "LStickY": 1,
        "RStickX": 3,
        "RStickY": 4,
        "left_trigger": 2,
        "right_trigger": 5
    }

#TO IMPLEMENT LATER
DPAD_BTN_MAP = {
    "up": 7,
    "down": -7,
    "left": 6,
    "right": -6
}

INV_AXES_MAP = {v: k for k, v in AXES_MAP.items()}
#simple wrapper around JoystickMessage to make it easier to access the buttons and axes
class XboxJoyMessage:
  
    def __init__(self, joy_msg: Joy):
        self.joy_msg = joy_msg
        self.header = joy_msg.header
        self.axes = joy_msg.axes
        self.buttons = joy_msg.buttons
        
        # Buttons
        self.a = self.buttons[BUTTON_MAP["A"]]
        self.b = self.buttons[BUTTON_MAP["B"]]
        self.x = self.buttons[BUTTON_MAP["X"]]
        self.y = self.buttons[BUTTON_MAP["Y"]]
        self.lb = self.buttons[BUTTON_MAP["LB"]]
        self.rb = self.buttons[BUTTON_MAP["RB"]]
        self.squares = self.buttons[BUTTON_MAP["Squares"]]
        self.hamburger = self.buttons[BUTTON_MAP["Hamburger"]]
        self.lstick = self.buttons[BUTTON_MAP["LStick"]]
        self.rstick = self.buttons[BUTTON_MAP["RStick"]]
        self.lstick_x = self.axes[AXES_MAP["LStickX"]]
        self.lstick_y = self.axes[AXES_MAP["LStickY"]]
        self.rstick_x = self.axes[AXES_MAP["RStickX"]]
        self.rstick_y = self.axes[AXES_MAP["RStickY"]]
        self.left_trigger = self.axes[AXES_MAP["left_trigger"]]
        self.right_trigger = self.axes[AXES_MAP["right_trigger"]]

    def __repr__(self):
        return "XboxJoyMessage(header=%s, axes=%s, buttons=%s)" % (self.header, self.axes, self.buttons)


class XboxHandler:
    def __init__(self, node, topic_name="/joy"):
        self.joy_sub = node.create_subscription(Joy, topic_name, self.joy_callback, 10)
        self.joy_msg = None
        self.topic_name = topic_name
        self.axis_callbacks = {}
        self.button_callbacks= {}
        self.cond_callbascks = {}
        self.last_joy_msg = None
        self.node = node
    
    def register_callback(self, callback, axes=None, buttons=None):
        '''
        NOTE these callbacks are blocking! If you need to do something that takes a non-trivial amount of time, do it outside of the callback.
        you can pass in a single value or a list, if a list is given, the callback will pass in all values of the list, and will be called if any of the values change
        '''

        if buttons is not None:
            self.button_callbacks[buttons] = callback
        elif axes is not None:
            self.axis_callbacks[axes] = callback
        else:
            raise ValueError("expected a tuple of axes or buttons")

    def register_gated_axis_callback(self, callback, axes: tuple, button_conds: tuple):
        '''
        Register a callback that is only called when the button_cond is true
        '''
        #simple for now, maybe consider a more general solution later
        self.cond_callbascks[axes] = (callback, button_conds)


    def joy_callback(self, msg: Joy):
        #call the appropriate callbacks for each new value

        self.joy_msg = XboxJoyMessage(msg)
        if self.last_joy_msg is None:
            self.last_joy_msg = self.joy_msg
            return
        for axes, callback in self.axis_callbacks.items():
            # changed = False
            # for axis in axes:
            #     if self.joy_msg.axes[axis] != self.last_joy_msg.axes[axis]:
            #         changed = True
            #         break
            
            #constantly call axis callbacks, even if the value hasn't changed
            callback(*(self.joy_msg.axes[axis] for axis in axes))
        
        for buttons, callback in self.button_callbacks.items():
            changed = False
            for button in buttons:
                if self.joy_msg.buttons[button] != self.last_joy_msg.buttons[button]:
                    changed = True
                    break
            if changed:
                callback(*(bool(self.joy_msg.buttons[button]) for button in buttons))
        
        for axes, (callback, button_conds) in self.cond_callbascks.items():
            # changed = False
            valid = True
            
            # for axis in axes:
            #     if self.joy_msg.axes[axis] != self.last_joy_msg.axes[axis]:
            #         changed = True
            #         break
            
            for button_cond in button_conds:
                if not bool(self.joy_msg.buttons[button_cond]):
                    valid = False
                    break
            
            if valid:
                callback(*(self.joy_msg.axes[axis] for axis in axes))

        self.last_joy_msg = self.joy_msg
        
    def __repr__(self) -> str:
        # return a list of all the registered callbacks, and the name of the function they are tied to
        repr_str = f"XboxHandler(joy_sub={self.topic_name})\nAxes callbacks: \n"
        for i, (axes, callback) in enumerate(self.axis_callbacks.items()):
            repr_str += f"\t{i}: Axes ({list(INV_AXES_MAP[axis] for axis in axes)}) -> {callback.__name__}\n"

        repr_str += "Button callbacks: \n"
        for i, (buttons, callback) in enumerate(self.button_callbacks.items()):
            repr_str += f"\t{i}: Buttons ({list(INV_BUTTON_MAP[button] for button in buttons)}) -> {callback.__name__}\n"
        
        repr_str += "Conditional callbacks: \n"
        for i, (axes, (callback, button_conds)) in enumerate(self.cond_callbascks.items()):
            repr_str += f"\t{i}: Axes ({list(INV_AXES_MAP[axis] for axis in axes)}) -> {callback.__name__} if {' and '.join(map(str, list(INV_BUTTON_MAP[button_cond] for button_cond in button_conds))) } button(s)\n"
        
        return  repr_str

