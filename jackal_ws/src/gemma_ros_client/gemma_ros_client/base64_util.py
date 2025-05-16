import cv2
import base64
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def image_to_base64(ros_image_msg: Image, logger, encoding_format=".jpg"):
    bridge = CvBridge()

    if not isinstance(ros_image_msg, Image) or ros_image_msg.height == 0 or ros_image_msg.width == 0:
         logger.warning(f"유효하지 않은 Image 메시지 수신 (encoding: {ros_image_msg.encoding if isinstance(ros_image_msg, Image) else 'N/A'}), Base64 변환 건너뜀.")
         return None
    try:
        cv_image = bridge.imgmsg_to_cv2(ros_image_msg, desired_encoding='passthrough')
        params = []
        
        if ros_image_msg.encoding in ['16UC1', 'mono16']:
            encoding_format = ".png"
        elif cv_image.ndim == 2: pass
        elif cv_image.shape[2] == 3: pass

        image_for_encoding = cv_image 

        if encoding_format == ".jpg" and ros_image_msg.encoding == 'rgb8':
            image_for_encoding = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)

        is_success, buffer = cv2.imencode(encoding_format, image_for_encoding, params)
        
        if is_success:
            return base64.b64encode(buffer).decode('utf-8')
        else:
            logger.warning(f"이미지 인코딩 실패 (format: {encoding_format}, encoding: {ros_image_msg.encoding}, size: {cv_image.shape})")
            return None
    except CvBridgeError as e:
        logger.error(f"CvBridge 변환 실패 (encoding: {ros_image_msg.encoding}): {e}")
        return None
    except Exception as e:
        logger.error(f"이미지 처리 중 오류 (encoding: {ros_image_msg.encoding}): {e}", exc_info=True)
        return None