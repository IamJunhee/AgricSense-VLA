import rclpy
from rclpy.node import Node
from behave_package_msg.msg import MoveToGoal
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class MoveToGoalNode(Node):
    def __init__(self):
        super().__init__('move_to_goal_node')

        # 퍼블리셔: 속도 명령
        self.cmd_pub = self.create_publisher(Twist, '/jackal_velocity_controller/cmd_vel_unstamped', 10)

        # 커스텀 명령 구독
        self.subscription = self.create_subscription(
            MoveToGoal,
            '/move_goal',
            self.command_callback,
            10
        )

        # 현재 위치 구독
        self.odom_sub = self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 10)

        self.current_pose = None
        self.goal = None  # (x, y, z)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def odom_callback(self, msg: Odometry):
        self.current_pose = msg.pose.pose

    def command_callback(self, msg: MoveToGoal):
        if msg.command.lower() in ['move', '움직여']:
            self.get_logger().info("📥 이동 명령 수신")
            self.goal = (msg.goal_x, msg.goal_y, msg.goal_z)
        else:
            self.get_logger().info("⚠️ move 명령이 아님")

    def timer_callback(self):
        if self.goal is None or self.current_pose is None:
            return

        goal_x, goal_y, goal_z = self.goal
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y

        dx = goal_x - current_x
        dy = goal_y - current_y
        distance = math.sqrt(dx**2 + dy**2)

        if distance < 0.1:
            self.get_logger().info("✅ 목표 도달")
            self.goal = None
            self.cmd_pub.publish(Twist())  # 정지
            return

        angle = math.atan2(dy, dx)  # 목표 각도
        twist = Twist()
        twist.linear.x = 0.5  # 정방향 속도 (단순히 0.5로 고정)
        twist.angular.z = 0.0  # 추후: 회전 구현

        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = MoveToGoalNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

