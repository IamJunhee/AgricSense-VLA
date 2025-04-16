#!/usr/bin/env python3

import rclpy as rp
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class CommandToVelocity(Node):

    def __init__(self):
        super().__init__('command_to_velocity_node')
        
        # 📨 문자열 명령 구독자
        self.sub_cmd = self.create_subscription(
            String,
            '/command',
            self.callback_cmd,
            10
        )

        # 📤 Twist 퍼블리셔
        self.pub_twist = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # 📝 Twist 메시지 객체 미리 생성
        self.twist_msg = Twist()

    def callback_cmd(self, msg):
        if msg.data.strip() == "돌아":
            self.twist_msg.linear.x = 0.0
            self.twist_msg.angular.z = 1.0  # 제자리 회전
            self.pub_twist.publish(self.twist_msg)
            self.get_logger().info("↪️ 회전 명령 수행 중 (angular.z = 1.0)")
        else:
            self.get_logger().info(f"❌ 알 수 없는 명령: {msg.data}")


def main(args=None):
    rp.init(args=args)
    node = CommandToVelocity()
    rp.spin(node)
    node.destroy_node()
    rp.shutdown()


if __name__ == '__main__':
    main()

