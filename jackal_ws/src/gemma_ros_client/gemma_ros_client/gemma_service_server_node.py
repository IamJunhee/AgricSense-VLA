import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from gemma_ros2_interface.srv import GenerateGemma
from sensor_msgs.msg import Image

import requests
import json
import os
import base64
import cv2
from cv_bridge import CvBridge, CvBridgeError
import traceback

class GemmaServiceServerNode(Node):
    def __init__(self):
        super().__init__('gemma_service_server_node')

        self.declare_parameter('api_server_url', 'http://localhost:8000/generate')
        self.api_url = self.get_parameter('api_server_url').get_parameter_value().string_value

        self.srv = self.create_service(
            GenerateGemma,
            'generate_gemma',
            self.handle_service_request_callback)

        self.bridge = CvBridge()
        self.get_logger().info(f'Gemma 서비스 서버 준비 완료. API 서버: {self.api_url}')

    def image_to_base64(self, ros_image_msg: Image, encoding_format=".jpg"):
        if not isinstance(ros_image_msg, Image) or ros_image_msg.height == 0 or ros_image_msg.width == 0:
             self.get_logger().warning("유효하지 않은 Image 메시지 수신, Base64 변환 건너뜀.")
             return None
        try:
            cv_image = self.bridge.imgmsg_to_cv2(ros_image_msg, desired_encoding='passthrough')
            params = []
            if ros_image_msg.encoding in ['16UC1', 'mono16']:
                encoding_format = ".png"
            elif cv_image.ndim == 2: pass
            elif cv_image.shape[2] == 3: pass

            is_success, buffer = cv2.imencode(encoding_format, cv_image, params)
            if is_success:
                return base64.b64encode(buffer).decode('utf-8')
            else:
                self.get_logger().warning(f"이미지 인코딩 실패 (format: {encoding_format}, encoding: {ros_image_msg.encoding}, size: {cv_image.shape})")
                return None
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge 변환 실패 (encoding: {ros_image_msg.encoding}): {e}")
            return None
        except Exception as e:
            self.get_logger().error(f"이미지 처리 중 오류: {e}")
            return None

    def send_request_to_api(self, request_payload: dict):
        self.get_logger().debug(f"API 요청 페이로드 (이미지 제외): {{'prompt': '{request_payload.get('prompt', '')[:50]}...', 'context_keys': {list(request_payload.get('context', {}).keys())}, 'max_tokens': {request_payload.get('max_new_tokens')}}}")
        try:
            response = requests.post(self.api_url, json=request_payload, timeout=180.0)
            response.raise_for_status()
            return response.json()
        except requests.exceptions.HTTPError as e:
            self.get_logger().error(f"API 요청 실패 (HTTP Status {e.response.status_code}): {e.response.text}")
            error_detail = e.response.text
            try:
                detail_json = json.loads(error_detail)
                if isinstance(detail_json, dict) and 'detail' in detail_json:
                    error_detail = detail_json['detail']
            except json.JSONDecodeError: pass
            return {"error": f"API Error: {e.response.status_code}", "detail": error_detail}
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f"API 요청 실패 (Connection/Timeout 등): {e}")
            return {"error": f"Request Error: {e}"}
        except json.JSONDecodeError as e:
             self.get_logger().error(f"API 응답 JSON 파싱 실패: {e}")
             return {"error": f"Invalid JSON Response: {e}"}
        except Exception as e:
            self.get_logger().error(f"API 요청 중 예외 발생: {e}")
            print(traceback.format_exc())
            return {"error": str(e)}

    def process_gemma_request(self, prompt: str, rgb_image: Image = None, d_image: Image = None, context_dict: dict = None, max_tokens: int = 1000) -> tuple[bool, dict]:
        request_payload = {
            "prompt": prompt,
            "max_new_tokens": max_tokens if max_tokens > 0 else 200
        }
        if context_dict is not None:
            request_payload["context"] = context_dict

        if rgb_image:
            self.get_logger().info(f"RGB 이미지 처리 중 (Encoding: {rgb_image.encoding}, Size: {rgb_image.width}x{rgb_image.height})...")
            rgb_base64 = self.image_to_base64(rgb_image, ".jpg")
            if rgb_base64:
                request_payload["rgb_image_base64"] = rgb_base64
                self.get_logger().info("RGB 이미지 Base64 변환 완료.")
            else:
                 self.get_logger().warning("RGB 이미지 Base64 변환 실패, 요청에서 제외됨.")

        if d_image:
            self.get_logger().info(f"Depth 이미지 처리 중 (Encoding: {d_image.encoding}, Size: {d_image.width}x{d_image.height})...")
            d_base64 = self.image_to_base64(d_image, ".png")
            if d_base64:
                request_payload["d_image_base64"] = d_base64
                self.get_logger().info("Depth 이미지 Base64 변환 완료.")
            else:
                self.get_logger().warning("Depth 이미지 Base64 변환 실패, 요청에서 제외됨.")

        self.get_logger().info(f"API 서버 ({self.api_url})로 요청 전송...")
        api_response = self.send_request_to_api(request_payload)

        if isinstance(api_response, dict) and "error" in api_response:
            error_msg = api_response.get("error", "알 수 없는 API 오류")
            detail = api_response.get("detail")
            if detail: error_msg += f": {detail}"
            return False, {"error_message": error_msg}

        elif isinstance(api_response, list) and len(api_response) > 0:
            first_result = api_response[0]
            if isinstance(first_result, dict) and "generated" in first_result and "context" in first_result:
                return True, {"generated": first_result["generated"], "context": first_result["context"]}
            else:
                 return False, {"error_message": "API 응답 형식 오류: 'generated' 또는 'context' 필드 누락"}
        else:
             return False, {"error_message": f"API로부터 예상치 못한 응답 수신: {api_response}"}

    def handle_service_request_callback(self, request: GenerateGemma.Request, response: GenerateGemma.Response) -> GenerateGemma.Response:
        self.get_logger().info(f"서비스 요청 수신: prompt='{request.prompt[:30]}...'")

        context_dict = None
        if request.context_json and request.context_json != "{}":
            try:
                context_dict = json.loads(request.context_json)
            except json.JSONDecodeError:
                self.get_logger().warning(f"요청의 context JSON 파싱 실패: {request.context_json}")
                response.success = False
                response.error_message = "잘못된 context JSON 형식"
                return response

        rgb_image_msg = request.rgb_image if request.has_rgb_image else None
        d_image_msg = request.d_image if request.has_d_image else None

        success, result_data = self.process_gemma_request(
            prompt=request.prompt,
            rgb_image=rgb_image_msg,
            d_image=d_image_msg,
            context_dict=context_dict,
            max_tokens=request.max_new_tokens
        )

        response.success = success
        if success:
            self.get_logger().info("서비스 요청 처리 성공")
            response.generated_text = result_data.get("generated", "")
            try:
                response.updated_context_json = json.dumps(result_data.get("context", {}))
            except TypeError as e:
                 self.get_logger().error(f"결과 컨텍스트를 JSON으로 변환 실패: {e}")
                 response.success = False
                 response.error_message = f"컨텍스트 직렬화 오류: {e}"
        else:
            self.get_logger().error(f"서비스 요청 처리 실패: {result_data.get('error_message', '알 수 없는 오류')}")
            response.error_message = result_data.get("error_message", "알 수 없는 처리 오류")

        return response

def main(args=None):
    rclpy.init(args=args)
    node = None
    executor = None
    try:
        node = GemmaServiceServerNode()
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        if node: node.get_logger().info('KeyboardInterrupt, shutting down.')
    except Exception as e:
        logger = rclpy.logging.get_logger("main")
        if node: logger = node.get_logger()
        logger.fatal(f"노드 실행 중 치명적 오류 발생: {e}")
        print(traceback.format_exc())
    finally:
        if executor:
             executor.shutdown()
        if rclpy.ok():
             rclpy.shutdown()

if __name__ == '__main__':
    main()