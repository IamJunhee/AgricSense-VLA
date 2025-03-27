#!/usr/bin/env python3
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy

class ReliableRelay(Node):
    def __init__(self):
        super().__init__('reliable_pointcloud_relay')

        self.sub = self.create_subscription(
            PointCloud2,
            '/zed/points',
            self.callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )

        self.pub = self.create_publisher(
            PointCloud2,
            '/zed/points_reliable',
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )

    def callback(self, msg):
        self.get_logger().info('PointCloud2 message received')
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ReliableRelay()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
