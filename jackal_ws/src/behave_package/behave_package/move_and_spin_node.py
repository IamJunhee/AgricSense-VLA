import rclpy
from rclpy.node import Node
from behave_package_msg.msg import MoveToGoal
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class MoveToGoalNode(Node):
    def __init__(self):
        super().__init__('move_to_goal_node')

        self.cmd_pub = self.create_publisher(Twist, '/jackal_velocity_controller/cmd_vel_unstamped', 10)

        self.subscription = self.create_subscription(
            MoveToGoal,
            '/move_goal',
            self.command_callback,
            10
        )

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

        # --- 현재 방향(yaw) 계산 ---
        yaw = self.get_yaw_from_quaternion(self.current_pose.orientation)

        # --- 목표 방향 계산 ---
        target_angle = math.atan2(dy, dx)
        angle_diff = self.normalize_angle(target_angle - yaw)

        twist = Twist()

        if abs(angle_diff) > 0.1:
            # 아직 방향이 맞지 않음 → 회전 먼저
            twist.linear.x = 0.0
            twist.angular.z = 0.5 if angle_diff > 0 else -0.5
        else:
            # 방향 정렬 완료 → 직진
            twist.linear.x = 0.5
            twist.angular.z = 0.0

        self.cmd_pub.publish(twist)

    def get_yaw_from_quaternion(self, q):
        # 수동으로 오일러 변환 (yaw만)
        x, y, z, w = q.x, q.y, q.z, q.w
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        # -pi ~ pi로 각도 정규화
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = MoveToGoalNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

