import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import math

class DepthConverterNode(Node):
    def __init__(self):
        super().__init__('depth_converter_node')

        self.declare_parameter('input_topic', '/zed/depth/image_raw')
        self.declare_parameter('output_topic', '/zed/depth/image_16uc1_mm')

        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value

        self.bridge = CvBridge()

        self.max_uint16 = 65535

        qos_profile_sub = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        qos_profile_pub = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.subscription = self.create_subscription(
            Image,
            input_topic,
            self.listener_callback,
            qos_profile_sub
        )
        self.publisher = self.create_publisher(
            Image,
            output_topic,
            qos_profile_pub
        )

        self.get_logger().info(f"Depth converter node started.")
        self.get_logger().info(f"Subscribing to '{input_topic}' (expecting 32FC1 meters)")
        self.get_logger().info(f"Publishing to '{output_topic}' (standard 16UC1 millimeters)")

    def listener_callback(self, msg):
        if msg.encoding != '32FC1':
            self.get_logger().warn(f"Received message with encoding '{msg.encoding}', expected '32FC1'. Skipping conversion.")
            return

        try:
            cv_image_meters = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error converting image: {e}')
            return
        except Exception as e:
             self.get_logger().error(f'Error converting image: {e}')
             return

        depth_meters_clean = np.nan_to_num(cv_image_meters, nan=0.0,
                                           posinf=self.max_uint16 / 1000.0,
                                           neginf=0.0)

        depth_mm_float = depth_meters_clean * 1000.0

        depth_mm_clamped = np.clip(depth_mm_float, 0, self.max_uint16)
        depth_mm_uint16 = depth_mm_clamped.astype(np.uint16)

        try:
            out_msg = self.bridge.cv2_to_imgmsg(depth_mm_uint16, encoding='16UC1')
            out_msg.header = msg.header
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error converting image back to message: {e}')
            return
        except Exception as e:
            self.get_logger().error(f'Error converting image back to message: {e}')
            return

        self.publisher.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    depth_converter_node = DepthConverterNode()
    try:
        rclpy.spin(depth_converter_node)
    except KeyboardInterrupt:
        pass
    finally:
        depth_converter_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()