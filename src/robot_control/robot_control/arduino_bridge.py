#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

import serial
import time
import math


class ArduinoBridge(Node):

    def __init__(self):
        super().__init__('arduino_bridge')
        self.get_logger().info("Arduino Bridge started!")
        # ------------------------
        # Parameters
        # ------------------------
        self.declare_parameter(
            'port',
            '/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0'
        )
        self.declare_parameter('baud', 115200)
        self.declare_parameter('wheel_base', 0.22)
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'base_link')

        self.port = self.get_parameter('port').value
        self.baud = self.get_parameter('baud').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.frame_id = self.get_parameter('frame_id').value
        self.child_frame_id = self.get_parameter('child_frame_id').value

        # ------------------------
        # ROS interfaces
        # ------------------------
        self.cmd_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_callback,
            10
        )

        self.odom_pub = self.create_publisher(
            Odometry,
            '/odom',
            10
        )

        self.tf_broadcaster = TransformBroadcaster(self)

        # ------------------------
        # Serial
        # ------------------------
        self.serial = None
        self.connect_serial()

        # ------------------------
        # Robot state
        # ------------------------
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.last_time = self.get_clock().now()

        # ------------------------
        # Timers
        # ------------------------
        self.serial_timer = self.create_timer(
            0.01,  # 100 Hz
            self.read_serial
        )

        self.get_logger().info('Arduino bridge started')

    # ======================================================
    # Serial handling
    # ======================================================

    def connect_serial(self):
        if self.serial and self.serial.is_open:
            return

        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baud,
                timeout=0.1,
                write_timeout=0.1
            )
            time.sleep(2.0)  # allow Arduino reset
            self.get_logger().info(f'Connected to Arduino on {self.port}')
        except Exception:
            self.serial = None
            # Do NOT log as error – this is expected during re-enumeration
            self.get_logger().debug(f'Waiting for Arduino on {self.port}')

    def read_serial(self):
        try:
            if not self.serial or not self.serial.is_open:
                return

            raw = self.serial.readline()

            if not raw:
                return

            try:
                line = raw.decode('utf-8', errors='ignore').strip()
            except Exception:
                return

            if not line:
                return

            self.handle_serial_line(line)

        except serial.SerialException as e:
            self.get_logger().warn(f'Serial error: {e}')
            time.sleep(0.1)

        except Exception as e:
            self.get_logger().error(f'Unexpected serial error: {e}')

    def handle_serial_line(self, line: str):
        if line.startswith('$ODOM'):
            self.handle_odom(line)
        elif line.startswith('$DEBUG'):
            self.get_logger().info(line)

    # ======================================================
    # Odometry
    # ======================================================

    def handle_odom(self, line: str):
        """
        Expected format:
        $ODOM,x,y,yaw,linear,angular
        """
        try:
            parts = line.split(',')
            if len(parts) < 6:
                return

            x = float(parts[1])
            y = float(parts[2])
            yaw = float(parts[3])
            linear = float(parts[4])
            angular = float(parts[5])

            now = self.get_clock().now()

            # Publish odometry
            odom = Odometry()
            odom.header.stamp = now.to_msg()
            odom.header.frame_id = self.frame_id
            odom.child_frame_id = self.child_frame_id

            odom.pose.pose.position.x = x
            odom.pose.pose.position.y = y
            odom.pose.pose.position.z = 0.0

            qz = math.sin(yaw * 0.5)
            qw = math.cos(yaw * 0.5)

            odom.pose.pose.orientation.z = qz
            odom.pose.pose.orientation.w = qw

            odom.twist.twist.linear.x = linear
            odom.twist.twist.angular.z = angular

            self.odom_pub.publish(odom)

            # Publish TF
            t = TransformStamped()
            t.header.stamp = now.to_msg()
            t.header.frame_id = self.frame_id
            t.child_frame_id = self.child_frame_id

            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.translation.z = 0.0
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw

            self.tf_broadcaster.sendTransform(t)

        except Exception as e:
            self.get_logger().warn(f'Failed to parse odom: {e}')

    # ======================================================
    # Command handling
    # ======================================================

    def cmd_callback(self, msg: Twist):
        if not self.serial or not self.serial.is_open:
            return

        try:
            linear = msg.linear.x
            angular = msg.angular.z

            cmd = f'$CMD,{linear:.3f},{angular:.3f}\n'
            self.serial.write(cmd.encode('utf-8'))

        except Exception as e:
            self.get_logger().warn(f'Failed to send CMD: {e}')


# ======================================================
# Main
# ======================================================

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.serial and node.serial.is_open:
            node.serial.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

