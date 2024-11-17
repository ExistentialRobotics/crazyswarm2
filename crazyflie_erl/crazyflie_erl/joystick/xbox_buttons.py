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
INV_AXES_MAP = {v: k for k, v in AXES_MAP.items()}

DPAD_BTN_MAP = {
    "up": (7, 1),
    "down": (7, -1),
    "left": (6, 1),
    "right": (6, -1)
}

INV_DPAD_BTN_MAP = {v: k for k, v in DPAD_BTN_MAP.items()}

#simple wrapper around JoystickMessage to make it easier to access the buttons and axes
class XboxJoyMessage:
  
    def __init__(self, joy_msg: Joy):
        self.joy_msg = joy_msg
        self.header = joy_msg.header
        self.axes = joy_msg.axes
        self.buttons = joy_msg.buttons
        
        # Buttons
        self.a = bool(self.buttons[BUTTON_MAP["A"]])
        self.b = bool(self.buttons[BUTTON_MAP["B"]])
        self.x = bool(self.buttons[BUTTON_MAP["X"]])
        self.y = bool(self.buttons[BUTTON_MAP["Y"]])
        self.lb = bool(self.buttons[BUTTON_MAP["LB"]])
        self.rb = bool(self.buttons[BUTTON_MAP["RB"]])
        self.squares = bool(self.buttons[BUTTON_MAP["Squares"]])
        self.hamburger = bool(self.buttons[BUTTON_MAP["Hamburger"]])
        self.lstick = bool(self.buttons[BUTTON_MAP["LStick"]])
        self.rstick = bool(self.buttons[BUTTON_MAP["RStick"]])

        self.lstick_x = self.axes[AXES_MAP["LStickX"]]
        self.lstick_y = self.axes[AXES_MAP["LStickY"]]
        self.rstick_x = self.axes[AXES_MAP["RStickX"]]
        self.rstick_y = self.axes[AXES_MAP["RStickY"]]
        self.left_trigger = self.axes[AXES_MAP["left_trigger"]]
        self.right_trigger = self.axes[AXES_MAP["right_trigger"]]

        # Since Dpad is on the axes, but behaves like a button, we read it as a button, the dict value is a tuple of the axis and the value that the axis should be at the pressed state.
        self.dpad_up = self.axes[DPAD_BTN_MAP["up"][0]] == DPAD_BTN_MAP["up"][1]
        self.dpad_down = self.axes[DPAD_BTN_MAP["down"][0]] == DPAD_BTN_MAP["down"][1]
        self.dpad_left = self.axes[DPAD_BTN_MAP["left"][0]] == DPAD_BTN_MAP["left"][1]
        self.dpad_right = self.axes[DPAD_BTN_MAP["right"][0]] == DPAD_BTN_MAP["right"][1]

    def __repr__(self):
        return "XboxJoyMessage(header=%s, axes=%s, buttons=%s)" % (self.header, self.axes, self.buttons)


class XboxHandler:
    def __init__(self, node, topic_name="/joy"):
        self.joy_sub = node.create_subscription(Joy, topic_name, self.joy_callback, 10)
        self.joy_msg = None
        self.topic_name = topic_name
        self.axis_callbacks = {}
        self.button_callbacks= {}
        self.dpad_callbacks = {}
        self.cond_callbacks = {}
        self.last_joy_msg = None
        self.node = node
    
    def register_callback(self, callback, axes=None, buttons=None, dpads=None, continuous=False):
        '''
        NOTE these callbacks are blocking! If you need to do something that takes a non-trivial amount of time, do it outside of the callback.
        you can pass in a single value or a list, if a list is given, the callback will pass in all values of the list, and will be called if any of the values change
        '''

        if buttons is not None:
            self.button_callbacks[buttons] = (callback, continuous)
        elif axes is not None:
            self.axis_callbacks[axes] = callback
        elif dpads is not None:
            self.dpad_callbacks[dpads] = (callback, continuous)
        else:
            raise ValueError("expected a tuple of axes or buttons")

    def register_gated_axis_callback(self, callback, axes: tuple, button_conds: tuple):
        '''
        Register a callback that is only called when the button_cond is true 
        '''
        #simple for now, maybe consider a more general solution later
        self.cond_callbacks[axes] = (callback, button_conds)

    def register_full_dpad_callback(self, callback):
        '''
        Register a callback called when any of the dpad buttons are pressed, btn order (up, down, left, right)
        '''
        dpads = tuple(DPAD_BTN_MAP.values())
        self.register_callback(callback=callback, dpads=dpads)



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
        
        for buttons, (callback, continuous) in self.button_callbacks.items():
            changed = continuous # only check if  button has changed if continuous is False
            if not continuous:
                for button in buttons:
                    if self.joy_msg.buttons[button] != self.last_joy_msg.buttons[button]:
                        changed = True
                        break
            if changed:
                callback(*(self.joy_msg.buttons[button] for button in buttons))
        
        # no list for dpad because only one can be active at a time
        for dpad, (callback, continuous) in self.dpad_callbacks.items():
            changed = continuous # only check if  button has changed if continuous is False
            if not continuous:
                for dpad_btn in dpad:
                    if (self.joy_msg.axes[dpad_btn[0]] == dpad_btn[1]) != (self.last_joy_msg.axes[dpad_btn[0]] == dpad_btn[1]):

                        changed = True
                        break
            if changed:
                callback(*((self.joy_msg.axes[dpad_btn[0]] == dpad_btn[1]) for dpad_btn in dpad))
        
        for axes, (callback, button_conds) in self.cond_callbacks.items():
            # changed = False
            valid = True
            
            # for axis in axes:
            #     if self.joy_msg.axes[axis] != self.last_joy_msg.axes[axis]:
            #         changed = True
            #         break
            
            for button_cond in button_conds:
                if not self.joy_msg.buttons[button_cond]:
                    valid = False
                    break
            
            if valid:
                callback(*(self.joy_msg.axes[axis] for axis in axes))

        self.last_joy_msg = self.joy_msg
        
    def __repr__(self) -> str:
        # return a list of all the registered callbacks, and the name of the function they are tied to
        
        repr_str = f"XboxHandler(joy_sub={self.topic_name})\n"
        if len(self.axis_callbacks) != 0:
            repr_str += "Axes callbacks: \n"
            for i, (axes, callback) in enumerate(self.axis_callbacks.items()):
                repr_str += f"\t{i}: Axes ({list(INV_AXES_MAP[axis] for axis in axes)}) -> {callback.__name__}\n"

        if len(self.button_callbacks) != 0:
            repr_str += "Button callbacks: \n"
            for i, (buttons, (callback, continuous)) in enumerate(self.button_callbacks.items()):
                repr_str += f"\t{i}: " + ("Continuous" if continuous else "") + f"Button(s) ({list(INV_BUTTON_MAP[button] for button in buttons)}) )-> {callback.__name__}\n"
        
        if len(self.dpad_callbacks) != 0:
            repr_str += "Dpad callbacks: \n"
            for i, (dpad, (callback, continuous)) in enumerate(self.dpad_callbacks.items()):
                repr_str += f"\t{i}: " + ("Continuous" if continuous else "") + f" Dpad ({list(INV_DPAD_BTN_MAP[dpad_btn] for dpad_btn in dpad)}) -> {callback.__name__}\n"

        if len(self.cond_callbacks) != 0:
            repr_str += "Conditional callbacks: \n"
            for i, (axes, (callback, button_conds)) in enumerate(self.cond_callbacks.items()):
                repr_str += f"\t{i}: Axes ({list(INV_AXES_MAP[axis] for axis in axes)}) -> {callback.__name__} if {' and '.join(map(str, list(INV_BUTTON_MAP[button_cond] for button_cond in button_conds))) } button(s)\n"
            
        return  repr_str

