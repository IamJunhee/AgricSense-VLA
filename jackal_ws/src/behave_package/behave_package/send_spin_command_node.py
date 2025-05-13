import rclpy
from rclpy.node import Node
from behave_package_msg.msg import CommandAndSpin

class SpinCommandSender(Node):
    def __init__(self):
        super().__init__('spin_command_sender')

        self.publisher_ = self.create_publisher(
            CommandAndSpin,
            '/spin_command',
            10
        )

        timer_period = 1.0  # 🔁 1초마다 반복 발행
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = CommandAndSpin()
        msg.command = 'spin'               # 예: '돌아'
        msg.angular_velocity = 1.0         # rad/s
        msg.angle_degree = 180.0           # 사용 안 하더라도 포함 가능

        self.publisher_.publish(msg)
        self.get_logger().info(f"🚀 Sent command: '{msg.command}' with {msg.angular_velocity} rad/s for {msg.angle_degree} degrees")

def main(args=None):
    rclpy.init(args=args)
    node = SpinCommandSender()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


