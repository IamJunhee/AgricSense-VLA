import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from behave_interfaces.action import MoveToGoal

class MoveToGoalClient(Node):
    def __init__(self):
        super().__init__('move_to_goal_client')
        self._client = ActionClient(self, MoveToGoal, 'move_to_goal')

    def send_goal(self, command, x, y):
        self.get_logger().info('🟡 서버 연결 대기 중...')
        self._client.wait_for_server()

        goal_msg = MoveToGoal.Goal()
        goal_msg.command = command
        goal_msg.goal_x = x
        goal_msg.goal_y = y

        self.get_logger().info(f'📤 목표 전송: ({x}, {y}), command="{command}"')

        self._send_goal_future = self._client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('❌ 목표 거절됨')
            return

        self.get_logger().info('✅ 목표 수락됨')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'📡 진행률: {feedback.progress_percent:.1f}% | 상태: {feedback.status} | 남은 거리: {feedback.distance_remaining:.2f} m')

    def result_callback(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info(f'🎉 성공: {result.message}')
        else:
            self.get_logger().warn(f'⚠️ 실패: {result.message}')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    client = MoveToGoalClient()

    # 예시 목표: (x=1.0, y=2.0), command="move"
    client.send_goal("move", 0.0, 7.0)

    rclpy.spin(client)

if __name__ == '__main__':
    main()

