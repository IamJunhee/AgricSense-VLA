import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from behave_interfaces.action import MoveToGoal
from nav2_simple_commander.robot_navigator import BasicNavigator
import math
import asyncio
import time 
class MoveToGoalActionServer(Node):
    def __init__(self):
        super().__init__('move_to_goal_server')

        self.navigator = BasicNavigator()
        self.current_pose = None
        self.start_distance = None

        # Subscribe to odometry
        self.create_subscription(Odometry, '/odometry/global', self.odom_callback, 10)

        # Action Server
        self._action_server = ActionServer(
            self,
            MoveToGoal,
            'move_to_goal',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            # cancel_callback=self.cancel_callback
        )

        self.get_logger().info("✅ MoveToGoalActionServer 노드 시작됨")

    def odom_callback(self, msg: Odometry):
        self.current_pose = msg.pose.pose

    def calculate_distance(self, gx, gy):
        if self.current_pose is None:
            return float('inf')
        dx = gx - self.current_pose.position.x
        dy = gy - self.current_pose.position.y
        return math.sqrt(dx**2 + dy**2)

    def goal_callback(self, goal_request):  
        if goal_request.command.lower() not in ['move', '움직여']:
            self.get_logger().warn("⚠️ 잘못된 명령: " + goal_request.command)
            return GoalResponse.REJECT
        self.get_logger().info(f"📥 Goal 수신됨: ({goal_request.goal_x}, {goal_request.goal_y})")
        return GoalResponse.ACCEPT

    # def cancel_callback(self, goal_handle):
    #     self.get_logger().info("🛑 Goal 취소 요청 수신됨")
    #     return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        goal = goal_handle.request
        self.get_logger().info("🚀 이동 시작")

        while self.current_pose is None:
            self.get_logger().info("📡 위치 수신 대기 중...")
            time.sleep(0.5)

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = goal.goal_x
        goal_pose.pose.position.y = goal.goal_y
        goal_pose.pose.orientation.w = 1.0

        self.navigator.goToPose(goal_pose)
        self.start_distance = self.calculate_distance(goal.goal_x, goal.goal_y)

        stuck_timer = 0.0
        prev_progress = None

        while not self.navigator.isTaskComplete():
            distance = self.calculate_distance(goal.goal_x, goal.goal_y)
            progress = max(0.0, min(100.0 * (1 - distance / self.start_distance), 100.0))

            # ✅ 1. 목표 근처일 경우 도착 성공으로 간주
            # if distance <= 0.3:  # ← 허용 오차 (m), 필요시 조정
            #     self.get_logger().info("🎉 목표 근처 도달로 판단되어 성공 처리합니다")
            #     break  # 루프 탈출하여 성공 처리로 넘어감
            if distance <= 0.3:
                self.get_logger().info("🎉 목표 근처 도달로 판단되어 성공 처리합니다")
                result = MoveToGoal.Result()
                result.success = True
                result.message = "🎯 목표 근처 도달 완료 (거리 기준)"
                goal_handle.succeed()
                return result


            # ✅ 2. stuck 감지 (목표 근처 아닐 때만 판단)
            if prev_progress is not None and abs(progress - prev_progress) < 5.0:
                stuck_timer += 5.0
                if stuck_timer >= 10.0:
                    self.get_logger().warn("🛑 로봇이 stuck 상태로 판단되어 중단합니다")
                    self.navigator.cancelTask()
                    result = MoveToGoal.Result()
                    result.success = False
                    result.message = "❌ 목표 지점에 도달하지 못했습니다 (stuck)"
                    goal_handle.abort()
                    return result
            else:
                stuck_timer = 0.0

            prev_progress = progress

            # 상태 및 피드백 전송
            status = "경로 탐색 중" if distance > self.start_distance * 0.8 else \
                    "목표 접근 중" if distance > 0.2 else "정지"

            feedback = MoveToGoal.Feedback()
            feedback.distance_remaining = distance
            feedback.status = status
            feedback.progress_percent = progress
            goal_handle.publish_feedback(feedback)

            time.sleep(5.0)

        # # 성공/실패 처리
        # result = MoveToGoal.Result()
        # if self.navigator.getResult():
        #     result.success = True
        #     result.message = "🎯 목표 도달 완료!"
        #     self.get_logger().info(result.message)
        # else:
        #     result.success = False
        #     result.message = "❌ 실패 또는 취소됨"
        #     self.get_logger().warn(result.message)
        #
        # goal_handle.succeed()
        # return result


def main(args=None):
    rclpy.init(args=args)
    node = MoveToGoalActionServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

