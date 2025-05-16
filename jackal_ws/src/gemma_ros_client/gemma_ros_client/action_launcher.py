from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
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
        goal.pose.orientation.w = cos(angle/360*pi)
        goal.pose.orientation.z = sin(angle/360*pi)

        self.goToPose(goal)

        while not self.isTaskComplete():
            pass

        if self.getResult() == TaskResult.SUCCEEDED:
            # TODO: 도착 후?
            return

        else:
            # TODO: 실패 후?
            return
                    
    async def _spin(self, angle):
        self.spin(angle / 180.0 * pi)
        
        while not self.isTaskComplete():
            pass

        if self.getResult() == TaskResult.SUCCEEDED:
            # TODO: 도착 후?
            return

        else:
            # TODO: 실패 후?
            return
