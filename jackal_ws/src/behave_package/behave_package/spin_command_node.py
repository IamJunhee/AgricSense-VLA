import rclpy
from rclpy.node import Node
from behave_package_msg.msg import CommandAndSpin  # ✅ 커스텀 메시지
from geometry_msgs.msg import Twist

class SpinCommandNode(Node):
    def __init__(self):
        super().__init__('spin_command_node')

        # 🔽 커스텀 메시지 구독
        self.subscription = self.create_subscription(
            CommandAndSpin,
            'spin_command',  # 이 토픽에 CommandAndSpin 메시지 발행하면 됨
            self.command_callback,
            10
        )

        # 🔼 Twist 퍼블리셔
        self.cmd_vel_pub = self.create_publisher(Twist, '/jackal_velocity_controller/cmd_vel_unstamped', 10)

    def command_callback(self, msg: CommandAndSpin):
        command = msg.command.strip().lower()
        self.get_logger().info(f"📥 Received command: '{command}' with angular_velocity: {msg.angular_velocity}, angle_degree: {msg.angle_degree}")

        if command in ['spin', '돌아']:
            twist_msg = Twist()
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = msg.angular_velocity  # ✅ 받은 속도로 회전
            self.cmd_vel_pub.publish(twist_msg)
            self.get_logger().info("🔁 Publishing twist to /cmd_vel")
        else:
            self.get_logger().info("⚠️ Unknown command, ignoring.")

def main(args=None):
    rclpy.init(args=args)
    node = SpinCommandNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


