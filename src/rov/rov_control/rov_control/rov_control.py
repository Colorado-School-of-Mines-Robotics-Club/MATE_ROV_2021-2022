from typing import Dict
import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from sensor_msgs.msg import Joy
from rov_interfaces.msg import ManipulatorSetpoints, ThrusterSetpoints
from transitions import Machine
from std_msgs.msg import String

from enum import Enum


class Buttons:
    def __init__(self):
        self.trigger = False
        self.thumb = False
        self.button_3 = False
        self.button_4 = False
        self.button_5 = False
        self.button_6 = False
        self.button_7 = False
        self.button_8 = False
        self.button_9 = False
        self.button_10 = False
        self.button_11 = False
        self.button_12 = False
        self.hat_up = False
        self.hat_down = False
        self.hat_left = False
        self.hat_right = False


class ParsedJoy(Buttons):
    last_buttons: Buttons = Buttons()

    def __init__(self, msg: Joy, mapping: Dict[str, str]):
        super().__init__()
        self.trigger = bool(msg.buttons[mapping["trigger"]])
        self.thumb = bool(msg.buttons[mapping["thumb"]])
        self.button_3 = bool(msg.buttons[mapping["button_3"]])
        self.button_4 = bool(msg.buttons[mapping["button_4"]])
        self.button_5 = bool(msg.buttons[mapping["button_5"]])
        self.button_6 = bool(msg.buttons[mapping["button_6"]])
        self.button_7 = bool(msg.buttons[mapping["button_7"]])
        self.button_8 = bool(msg.buttons[mapping["button_8"]])
        self.button_9 = bool(msg.buttons[mapping["button_9"]])
        self.button_10 = bool(msg.buttons[mapping["button_10"]])
        self.button_11 = bool(msg.buttons[mapping["button_11"]])
        self.button_12 = bool(msg.buttons[mapping["button_12"]])
        # Parse the button pushes for the dpad
        self.hat_up = msg.axes[mapping["hat_y"]] > 0
        self.hat_down = msg.axes[mapping["hat_y"]] < 0
        self.hat_left = msg.axes[mapping["hat_x"]] < 0
        self.hat_right = msg.axes[mapping["hat_x"]] > 0
        self.toggledButtons = self.get_toggled()
        self.roll = msg.axes[mapping["roll"]]
        self.pitch = msg.axes[mapping["pitch"]]
        self.yaw = msg.axes[mapping["yaw"]]
        self.throttle = msg.axes[mapping["throttle"]]
        self.hat_x = msg.axes[mapping["hat_x"]]
        self.hat_y = msg.axes[mapping["hat_y"]]

    def get_toggled(self):
        toggled_buttons = Buttons()
        for button in dir(toggled_buttons):
            # Here, we raise our middle fingers to the Python gods in our
            # religious pursuit of avoiding code duplication, and pray that
            # their retribution is mild
            if callable(getattr(toggled_buttons, button)):
                continue
            if getattr(self, button) and not getattr(self.last_buttons, button):
                setattr(
                    toggled_buttons,
                    button,
                    # Set this button to "Toggled" if
                    # A) it's pushed now and
                    # B) it wasn't pushed before
                    (getattr(self, button) and not getattr(self.last_buttons, button)),
                )
        self.last_buttons = self
        return toggled_buttons


class States(Enum):
    shutdown = 0
    paused = 1
    teleop_drive = 2
    teleop_manip = 3


class ROV_Control(Node):
    def __init__(self):
        super().__init__("rov_control")
        self.thruster_setpoint_publisher = self.create_publisher(ThrusterSetpoints, "thruster_setpoints", 10)
        self.machine = Machine(self, states=States, initial=States.teleop_drive)
        self.machine.add_transition("estop_normal", "*", States.paused)
        self.machine.add_transition("estop_fatal", "*", States.shutdown)
        self.machine.add_transition("teleop_mode_switch", States.teleop_manip, States.teleop_drive)
        self.machine.add_transition("teleop_mode_switch", States.teleop_drive, States.teleop_manip)
        self.mapping = self.dictionaryafy(self.get_parameters_by_prefix("mapping"))
        self.last_manip = ManipulatorSetpoints(
            clamp=self.get_parameter("clamp_initial").value,
            level=self.get_parameter("level_initial").value,
            wrist=self.get_parameter("wrist_initial").value,
            elbow=self.get_parameter("elbow_initial").value,
        )
        self.last_joy_time = 0
        self.joy_subscription = self.create_subscription(Joy, "joy", self.joystick_callback, 10)
        self.manipulator_setpoint_pub = self.create_publisher(ManipulatorSetpoints, "manipulator_setpoints", 2)
        self.thruster_setpoint_pub = self.create_publisher(ThrusterSetpoints, "thruster_setpoints", 2)
        self.state_pub = self.create_publisher(String)
        # self.create_timer(5, lambda: self.state_pub.publish())
        self.create_subscription(Bool, "estop", self.estop_callback, 0)

    def dictionaryafy(self, in_dict):
        tree = {}
        for key, value in in_dict.items():
            t = tree
            parts = key.split(".")
            for part in parts[:-1]:
                t = t.setdefault(part, {})
            t[parts[-1]] = value.value
        return tree

    def joystick_callback(self, msg: Joy):
        # self.joystick = ParsedJoy(msg)
        joystick = ParsedJoy(msg, self.mapping)

        current_sec = self.get_clock().now().nanoseconds / 1000000.0
        if self.last_joy_time == 0:
            delta_time = 0
        else:
            delta_time = min(current_sec - self.last_joy_time, 0.2)
        self.last_joy_time = current_sec

        # if mode switch button has been pressed, toggle the mode if you are in teleop
        if self.state in [States.teleop_manip, States.teleop_drive]:
            if joystick.button_8:
                self.teleop_mode_switch()

        # update teleop modes
        if self.state == States.teleop_manip:
            self.do_manip_setpoint_update(joystick, delta_time)
        elif self.state == States.teleop_drive:
            self.do_thrust_setpoint_update(joystick)

    def do_manip_setpoint_update(self, joystick: ParsedJoy, delta_time: float):
        manip_setpoints = self.last_manip
        chicken_speed = self.get_parameter("chicken_speed").value
        manip_setpoints.elbow += joystick.pitch * chicken_speed * delta_time
        # chicken
        if joystick.thumb:
            # Do IK here :)
            manip_setpoints.level = 180 - manip_setpoints.elbow
        else:
            manip_setpoints.level += joystick.hat_y * chicken_speed * delta_time

        if joystick.toggledButtons.button_3:
            manip_setpoints.clamp = not manip_setpoints.clamp

        manip_setpoints.wrist += joystick.roll * self.get_parameter("wrist_speed").value * delta_time

        self.manipulator_setpoint_pub.publish(manip_setpoints)
        self.last_manip = manip_setpoints

    def do_thrust_setpoint_update(self, joystick: ParsedJoy):
        thrust_setpoints = ThrusterSetpoints()
        
        thrust_setpoints.
        pass

    def estop_callback(self, msg):
        if msg.data:
            self.estop_fatal()
        else:
            self.estop_normal()

    def on_enter_paused(self):
        # Stop motors and all that
        self.get_logger().warn("ROV_Control has entered the paused state")
        pass

    def on_enter_shutdown(self):
        # The sky is falling and we're all going to die
        self.get_logger().fatal("ROV_Control has entered emergency shutdown")
        pass


def main():
    rclpy.init(args=None)
    rclpy.spin(ROV_Control())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
