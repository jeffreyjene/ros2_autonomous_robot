import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped

class CmdVelAdapter(Node):
    def __init__(self):
        super().__init__('cmd_vel_adapter')

        self.sub = self.create_subscription(
            TwistStamped,
            '/cmd_vel_out',
            self.cb,
            10
        )

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def cb(self, msg: TwistStamped):
        self.pub.publish(msg.twist)

def main():
    rclpy.init()
    node = CmdVelAdapter()
    rclpy.spin(node)
    rclpy.shutdown()