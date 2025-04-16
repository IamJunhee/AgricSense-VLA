import rclpy
from rclpy.node import Node
from behave_package_msg.msg import MoveToGoal
from nav_msgs.msg import Odometry
import random

class MoveCommandSender(Node):
    def __init__(self):
        super().__init__('move_command_sender')

        self.publisher = self.create_publisher(MoveToGoal, '/move_goal', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 10)

        self.current_pose = None
        self.timer = self.create_timer(10.0, self.timer_callback)  

    def odom_callback(self, msg: Odometry):
        self.current_pose = msg.pose.pose

    def timer_callback(self):
        if self.current_pose is None:
            self.get_logger().info("⏳ 현재 위치 수신 대기 중...")
            return

        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y

        # 반경 2m 이내 무작위 위치
        dx = random.uniform(-1.0, 1.0)
        dy = random.uniform(-1.0, 1.0)

        goal_x = current_x + dx
        goal_y = current_y + dy
        goal_z = 0.0  # 평면 이동만 고려

        msg = MoveToGoal()
        msg.command = 'move'
        msg.goal_x = float(goal_x)
        msg.goal_y = float(goal_y)
        msg.goal_z = float(goal_z)

        self.publisher.publish(msg)
        self.get_logger().info(f"🚀 목표 좌표 전송: ({msg.goal_x:.2f}, {msg.goal_y:.2f})")

def main(args=None):
    rclpy.init(args=args)
    node = MoveCommandSender()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

