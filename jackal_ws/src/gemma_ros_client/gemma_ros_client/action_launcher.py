from nav2_simple_commander.robot_navigator import BasicNavigator
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from math import cos, sin, pi

class ActionLauncher(BasicNavigator):
    def __init__(self):
        super().__init__('ActionLauncher')

        self.__action_table = {
            "move": self._move,
            "spin": self._spin
        }

        self.create_subscription(Odometry, '/odometry/global', self.odom_callback, 10)
        self.current_pose = None

    def odom_callback(self, msg: Odometry):
        self.current_pose = msg.pose.pose

    async def launch_action(self, action_dict: dict):
        action = action_dict["action"]
        await self.__action_table[action["name"]](**action["parameters"])

        # TODO: 결과 딕셔너리
        return
    
    async def _move(self, x, y, angle):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.w = cos(angle/2)
        goal.pose.orientation.z = sin(angle/2)

        self.goToPose(goal)
        start_distance = None
        prev_progress = None
        stuck_timer = 0.0
        is_success = False

        while not self.isTaskComplete():
            feedback = self.getFeedback()
            if feedback:
                distance = feedback.distance_remaining
                if start_distance is None:
                    start_distance = distance

                progress = max(0.0, min(100.0 * (1 - distance / start_distance), 100.0))
                self.get_logger().info(f"🛤 남은 거리: {distance:.2f} m ({progress:.2f} %)")

                if distance <= 0.3:
                    self.get_logger().info("목표 근처 도달: 성공 처리")
                    self.cancelTask()
                    is_success = True
                    break
                    

                if prev_progress is not None and abs(progress - prev_progress) < 5.0:
                    stuck_timer += 5.0
                    if stuck_timer >= 10.0:
                        self.get_logger().warn("Stuck으로 판단: 중단 처리")
                        self.cancelTask()
                        break

                else: # Stuck 벗어남
                    stuck_timer = 0.0

                prev_progress = progress

        if not is_success:
            # TODO: 실패 후?
            return
        
        # TODO: 도착 후?
        return
                    
                


    async def _spin(self, angle):
        self.spin(angle * 180.0 / pi)
        
        while not self.isTaskComplete():
            pass

