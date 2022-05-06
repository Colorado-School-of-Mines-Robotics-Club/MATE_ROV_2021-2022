import rclpy
from rclpy.node import Node
from rclpy.qos import QoSReliabilityPolicy
from std_msgs.msg import Bool

class Heartbeat(Node):
    def __init__(self):
        super().__init__("heartbeat")
        self.heartbeat_publisher = self.create_publisher(Bool, "heartbeat", QoSReliabilityPolicy)

        self.timer = self.create_timer(0.05, self.heartbeat)
    
    def heartbeat(self):
        self.heartbeat_publisher.publish(Bool(data=True))

def main():
    rclpy.init(args=None)
    rclpy.spin(Heartbeat())
    rclpy.shutdown()

if __name__ == '__main__':
    main()