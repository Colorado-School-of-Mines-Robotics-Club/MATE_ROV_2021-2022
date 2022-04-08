import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from sensor_msgs.msg import Joy
from rov_interfaces.msg import ManipulatorSetpoints, ThrusterSetpoints
from transitions import Machine
from std_msgs.msg import String

from enum import Enum

class Joystick_Map_Axes(Enum):
    # AXES
    PITCH = 0
    YAW = 1
    ROLL = 2
    THROTTLE = 3

class Joystick_Map_Buttons(Enum):  
    # BUTTONS
    INTERACT = 0
    MODE_SWITCH = 1

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
        self.joy_subscription = self.create_subscription(Joy, "joy", self.joystick_callback, 10)
        self.joystick = Joy()
        self.manipulator_setpoint_publisher = self.create_publisher(ManipulatorSetpoints, "manipulator_setpoints", 10)
        self.state_pub = self.create_publisher(String)
        self.create_timer(5, lambda: self.state_pub.publish())
        self.create_subscription(Bool, "estop", self.estop_callback, 0)
        

    def joystick_callback(self, msg:Joy):
        self.joystick=msg

        # if mode switch button has been pressed, toggle the mode if you are in teleop
        if(self.state in [States.teleop_manip, States.teleop_drive]):
            buttons = self.joystick.buttons
            if(buttons[Joystick_Map_Buttons.MODE_SWITCH]):
                self.teleop_mode_switch()

        # update teleop modes
        if(self.state == States.teleop_manip):
            self.do_manip_setpoint_update()
        elif(self.state == States.teleop_drive):
            self.do_thrust_setpoint_update()
    
    def do_manip_setpoint_update(self):
        axes = self.joystick.axes
        manip_setpoints = ManipulatorSetpoints()
        
    def do_thrust_setpoint_update(self):
        axes = self.joystick.axes
        thrust_setpoints = ThrusterSetpoints()

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

if __name__ == '__main__':
    main()