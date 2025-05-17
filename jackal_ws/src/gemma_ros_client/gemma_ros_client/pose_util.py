import math
from geometry_msgs.msg import Pose

def pose_to_xy_angle(pose: Pose):
    qw = pose.orientation.w
    qx = pose.orientation.x
    qy = pose.orientation.y
    qz = pose.orientation.z

    standard_yaw_rad = math.atan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz))

    angle = math.degrees(standard_yaw_rad)

    return pose.position.x, pose.position.y, angle