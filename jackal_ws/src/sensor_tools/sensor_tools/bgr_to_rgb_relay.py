import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import struct
import copy

class BGRtoRGBRelay(Node):
    def __init__(self):
        super().__init__('bgr_to_rgb_relay')

        self.sub = self.create_subscription(
            PointCloud2,
            '/zed/points',
            self.callback,
            10
        )

        self.pub = self.create_publisher(
            PointCloud2,
            '/zed/points_rgb',
            10
        )

    def callback(self, msg):
        new_msg = copy.deepcopy(msg)
        rgb_offset = None

        # rgb 필드 offset 찾기
        for field in msg.fields:
            if field.name == 'rgb':
                rgb_offset = field.offset
                break

        if rgb_offset is None:
            self.get_logger().warn("No rgb field found in PointCloud2")
            return

        # bytearray로 접근
        data = bytearray(msg.data)

        for i in range(msg.width * msg.height):
            point_index = i * msg.point_step
            rgb_index = point_index + rgb_offset

            # float32로 저장된 RGB -> int로 변환
            rgb_float = struct.unpack_from('f', data, rgb_index)[0]
            rgb_int = struct.unpack('I', struct.pack('f', rgb_float))[0]

            # 기존 BGR 순서를 RGB로 재배치 (BGR → RGB)
            b = (rgb_int >> 16) & 0xFF
            g = (rgb_int >> 8) & 0xFF
            r = rgb_int & 0xFF

            # RGB로 재조합
            corrected_rgb_int = (r << 16) | (g << 8) | b
            corrected_rgb_float = struct.unpack('f', struct.pack('I', corrected_rgb_int))[0]

            struct.pack_into('f', data, rgb_index, corrected_rgb_float)

        new_msg.data = bytes(data)
        self.pub.publish(new_msg)

def main(args=None):
    rclpy.init(args=args)
    node = BGRtoRGBRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
