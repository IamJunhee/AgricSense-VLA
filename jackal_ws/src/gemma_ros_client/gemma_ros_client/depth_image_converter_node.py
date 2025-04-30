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

        # 파라미터 선언
        self.declare_parameter('input_topic', '/zed/depth/image_raw') # 원본 32FC1 토픽 이름
        self.declare_parameter('output_topic', '/zed/depth/image_uint24_mm') # 변환된 토픽 이름

        # 파라미터 값 가져오기
        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value

        # CvBridge 초기화
        self.bridge = CvBridge()

        # uint24 최대값 (2^24 - 1)
        self.max_uint24 = (1 << 24) - 1 # 16,777,215

        # QoS 프로파일 설정 (원본 토픽과 유사하게 설정 권장)
        qos_profile_sub = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        qos_profile_pub = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE, # 또는 RELIABLE
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # 구독자 및 발행자 생성
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
        self.get_logger().info(f"Publishing to '{output_topic}' (custom uint24 mm)")

    def listener_callback(self, msg):
        if msg.encoding != '32FC1':
            self.get_logger().warn(f"Received message with encoding '{msg.encoding}', expected '32FC1'. Skipping conversion.")
            return

        try:
            # ROS Image (32FC1) -> OpenCV Mat (float32, meters)
            # passthrough를 사용하여 원본 데이터 타입 유지
            cv_image_meters = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error converting image: {e}')
            return
        except Exception as e:
             self.get_logger().error(f'Error converting image: {e}')
             return

        # --- 변환 로직 ---
        # 1. 유효하지 않은 값 처리 (NaN -> 0mm, Inf -> 최대값 mm)
        #    Gazebo에서는 보통 inf가 최대 거리 또는 측정 불가 의미
        #    nan_to_num: nan은 0으로, posinf는 지정값(여기선 최대 mm)으로 변경
        depth_meters_clean = np.nan_to_num(cv_image_meters, nan=0.0,
                                           posinf=self.max_uint24 / 1000.0, neginf=0.0)

        # 2. 미터(float) -> 밀리미터(float) 변환
        depth_mm_float = depth_meters_clean * 1000.0

        # 3. uint24 범위로 클램핑 [0, 16777215] 및 정수 변환
        #    np.clip으로 범위를 제한하고 uint32로 변환 (24비트 이상 정수 타입 사용)
        depth_mm_clamped = np.clip(depth_mm_float, 0, self.max_uint24)
        depth_mm_int = depth_mm_clamped.astype(np.uint32)

        # 4. uint32 -> 3-byte uint8 배열로 패킹 (uint24 시뮬레이션)
        #    결과 배열은 (height, width, 3) 형태가 됨
        height, width = depth_mm_int.shape
        uint24_bytes = np.zeros((height, width, 3), dtype=np.uint8)

        # 각 픽셀의 uint32 값을 3개의 uint8 바이트로 분리 (Little-Endian 방식)
        # byte 0: Least Significant Byte (LSB)
        # byte 1: Middle Byte
        # byte 2: Most Significant Byte (MSB)
        uint24_bytes[..., 0] = (depth_mm_int & 0x0000FF)
        uint24_bytes[..., 1] = (depth_mm_int >> 8) & 0x0000FF
        uint24_bytes[..., 2] = (depth_mm_int >> 16) & 0x0000FF

        # --- 출력 메시지 생성 ---
        out_msg = Image()
        out_msg.header = msg.header # 원본 헤더 사용 (timestamp 등 유지)
        out_msg.height = height
        out_msg.width = width

        # 인코딩: 표준이 아니므로 커스텀 문자열 사용. 수신측에서 이 의미를 알아야 함.
        # 또는 'mono8'/'rgb8' 등으로 하고 수신측에서 3바이트씩 읽도록 약속할 수도 있으나,
        # 명시적인 커스텀 인코딩이 더 나음.
        out_msg.encoding = 'custom_24UC1_mm' # 예시 커스텀 인코딩 이름

        out_msg.is_bigendian = 0 # Little-Endian (일반적)
        out_msg.step = width * 3 # 한 줄의 바이트 수 (너비 * 픽셀당 바이트 수)

        # NumPy 배열 데이터를 data 필드 (uint8[]) 로 복사
        out_msg.data = uint24_bytes.flatten().tobytes()

        # 변환된 메시지 발행
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