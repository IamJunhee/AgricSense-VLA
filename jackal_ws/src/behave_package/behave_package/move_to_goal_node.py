import rclpy
from rclpy.node import Node
from behave_package_msg.msg import MoveToGoal
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped

class MoveToGoalNode(Node):
    def __init__(self):
        super().__init__('move_to_goal_node')

        self.navigator = BasicNavigator()
        self.get_logger().info("⏳ Nav2 활성화 대기 중...")
        self.navigator.waitUntilNav2Active()

        self.subscription = self.create_subscription(
            MoveToGoal,
            '/move_goal',
            self.move_callback,
            10
        )

    def move_callback(self, msg: MoveToGoal):
        if msg.command.lower() not in ['move', '움직여']:
            self.get_logger().warn(f"⚠️ move 명령 아님: {msg.command}")
            return

        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = msg.goal_x
        goal.pose.position.y = msg.goal_y
        goal.pose.position.z = msg.goal_z
        goal.pose.orientation.w = 1.0  # 정방향 (임시)

        self.get_logger().info(f"🚀 goToPose 호출: ({msg.goal_x:.2f}, {msg.goal_y:.2f})")
        self.navigator.goToPose(goal)

        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback:
                self.get_logger().info(f"🛤 남은 거리: {feedback.distance_remaining:.2f} m")

        result = self.navigator.getResult()
        if result:
            self.get_logger().info("✅ 목표 도달 완료")
        else:
            self.get_logger().warn("❌ 목표 도달 실패 또는 취소됨")

def main(args=None):
    rclpy.init(args=args)
    node = MoveToGoalNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

