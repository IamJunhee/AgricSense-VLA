from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from math import cos, sin, pi
from .pose_util import pose_to_xy_angle

class ActionLauncher(BasicNavigator):
    def __init__(self):
        super().__init__('ActionLauncher')

        self.__action_table = {
            "move": self._move,
            "spin": self._spin,
            "forward": self._forward,
        }

        self.create_subscription(Odometry, '/odometry/filtered/global', self.odom_callback, 10)
        self.current_pose = None

    def odom_callback(self, msg: Odometry):
        self.current_pose = msg.pose.pose

    async def launch_action(self, action_dict: dict):
        action = action_dict["action"]
        is_success = await self.__action_table[action["name"]](**action["parameters"])

        action_dict["result"] = "Success" if is_success else "Failed"
        
        return action_dict
    
    async def _move(self, x, y, angle):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = float(x)
        goal.pose.position.y = float(y)
        goal.pose.orientation.w = cos(float(angle)/360*pi)
        goal.pose.orientation.z = sin(float(angle)/360*pi)

        self.goToPose(goal)

        while not self.isTaskComplete():
            pass

        if self.getResult() == TaskResult.SUCCEEDED:
            return True

        else:
            return False
                    
    async def _spin(self, angle):
        self.spin(float(angle) / 180.0 * pi)
        
        while not self.isTaskComplete():
            pass

        if self.getResult() == TaskResult.SUCCEEDED:
            return True

        else:
            return False
        
    async def _spin_using_pose(self, angle):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position = self.current_pose.position

        _, _, curr_angle = pose_to_xy_angle(self.current_pose)

        goal.pose.orientation.w = cos(float(curr_angle + angle)/360*pi)
        goal.pose.orientation.z = sin(float(curr_angle + angle)/360*pi)

        self.goToPose(goal)

        while not self.isTaskComplete():
            pass

        if self.getResult() == TaskResult.SUCCEEDED:
            return True

        else:
            return False
        
    async def _forward(self, distance):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        

        curr_x, curr_y, curr_angle = pose_to_xy_angle(self.current_pose)

        goal.pose.position.x = curr_x + float(distance) * cos(curr_angle / 180 * pi)
        goal.pose.position.y = curr_y + float(distance) * sin(curr_angle / 180 * pi)
        goal.pose.orientation.w = cos(curr_angle/360*pi)
        goal.pose.orientation.z = sin(curr_angle/360*pi)

        self.goToPose(goal)

        while not self.isTaskComplete():
            pass

        if self.getResult() == TaskResult.SUCCEEDED:
            return True

        else:
            return False
