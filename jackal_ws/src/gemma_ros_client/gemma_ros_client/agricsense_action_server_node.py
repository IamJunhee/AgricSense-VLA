import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import Header
from sensor_msgs.msg import Image
from agricsense_action_interfaces.action import Agricsense # 실제 액션 정의로 대체
from gemma_ros2_interface.srv import GenerateGemma # 실제 서비스 정의로 대체

from .base64_util import image_to_base64
from .json_validate import validate_custom_json
from .websocket_util import ROS2WebSocketServer

import deepl
import json
import os
import threading
import asyncio # 'await' 키워드 사용을 위해 필요
from typing import Optional, Dict, Any
from datetime import timezone, datetime

from jinja2 import Environment, FileSystemLoader

class AgricsenseActionServer(Node):
    def __init__(self):
        super().__init__('agricsense_action_server')

        self.declare_parameter('control_by_human', False)
        self.declare_parameter('loop_limit', 20)
        self.declare_parameter('use_translation', False)
        self.declare_parameter('prompt_template_path', "/root/AgricSense-VLA/jackal_ws/src/gemma_ros_client/resource/prompt_ko/")

        self.current_action: Optional[str] = None # JSON string 또는 dict가 될 수 있음
        self.active_goal_handle: Optional[rclpy.action.server.ServerGoalHandle] = None
        self.current_timestamp: Optional[str] = None

        prompt_template_path = self.get_parameter('prompt_template_path').get_parameter_value().string_value
        self.env = Environment(loader=FileSystemLoader(os.path.dirname(prompt_template_path)))
        self.prompt_template = {
            "cognition": self.env.get_template("cognition_prompt_template.txt"),
            "reasoning": self.env.get_template("reasoning_prompt_template.txt"),
            "action": self.env.get_template("action_prompt_template.txt")
        }
        with open(os.path.join(prompt_template_path, "tools.json")) as f:
            self.tools_json = json.load(f)
        
        self._action_server = ActionServer(
            self,
            Agricsense,
            'agricsense',
            execute_callback=self.execute_callback,
            cancel_callback=self._handle_cancel_request
        )

        self.rgb_sub = self.create_subscription(Image, '/zed/image_raw', self.add_rbg_to_request, 10)
        self.depth_sub = self.create_subscription(Image, '/zed/depth/image_16uc1_mm', self.add_depth_to_request, 10)
        self.gemma = self.create_client(GenerateGemma, 'generate_gemma')
        self.req = GenerateGemma.Request()

        self.websocket = ROS2WebSocketServer("localhost", 9090)
        self.websocket.register_callbacks(on_message=self.user_annotation_callback)
        self.websocket.start()

        self.on_message_event = threading.Event()
        self.get_logger().info('Agricsense Action Server is ready.')

    def _handle_cancel_request(self, goal_handle_to_cancel: rclpy.action.server.ServerGoalHandle) -> CancelResponse:
        self.get_logger().info(f"Cancel request received for goal UUID: {goal_handle_to_cancel.goal_id.uuid.tobytes().hex()}")
        if self.active_goal_handle and self.active_goal_handle.goal_id == goal_handle_to_cancel.goal_id:
            self.get_logger().info("Signaling waiting execute_callback (if any) due to cancellation.")
            self.on_message_event.set() 
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle: rclpy.action.server.ServerGoalHandle):
        self.get_logger().info(f"Executing goal UUID: {goal_handle.goal_id.uuid.tobytes().hex()}")
        self.active_goal_handle = goal_handle

        use_translation = self.get_parameter('use_translation').get_parameter_value().bool_value
        control_by_human = self.get_parameter('control_by_human').get_parameter_value().bool_value
        loop_limit = self.get_parameter('loop_limit').get_parameter_value().integer_value

        result = Agricsense.Result()
        feedback = Agricsense.Feedback()
        real_prompt = goal_handle.request.prompt
        last_gemma_response_text = ""
        is_translated_prompt = False

        if not control_by_human:
            while rclpy.ok() and not self.gemma.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('Gemma service not available, waiting (1s)...')
                if goal_handle.is_cancel_requested:
                    self.get_logger().info("Goal cancelled while waiting for Gemma service.")
                    goal_handle.canceled(); self.active_goal_handle = None; result.success = False
                    return result
            if not rclpy.ok():
                self.get_logger().error("RCLPY context shut down while waiting for Gemma service.")
                goal_handle.abort(); self.active_goal_handle = None; result.success = False
                return result
        
        self.websocket.broadcast_message("setOperationMode", {
            "mode" : "data_collection" if control_by_human else "execution_monitoring"
        })

        if use_translation:
            is_translated_prompt = True
            translated_prompt = self.translate_text_deepl(goal_handle.request.prompt, "EN-US")
            if translated_prompt is None:
                self.get_logger().info('Translate Failed using original prompt')
                is_translated_prompt = False
            else:
                real_prompt = translated_prompt
        
        previous_action_list = []

        for i in range(loop_limit):
            self.get_logger().info(f"Starting loop iteration {i+1}/{loop_limit}")
            if not rclpy.ok() or goal_handle.is_cancel_requested:
                status_msg = "RCLPY not OK" if not rclpy.ok() else "Goal cancelled"
                self.get_logger().info(f"{status_msg} at start of loop {i+1}.")
                if goal_handle.is_cancel_requested: goal_handle.canceled()
                else: goal_handle.abort()
                result.success = False; self.active_goal_handle = None
                return result

            data = {
                "cognition": {"user_prompt": real_prompt, "farm_info": "없음", "current_location": "없음", 
                              "previous_action_json": json.dumps(previous_action_list, indent=2, ensure_ascii=False)},
                "reasoning": {},
                "action": {"tools_list_json": json.dumps(self.tools_json, indent=2, ensure_ascii=False)}
            }
            rendered_prompt = {key: value.render(data[key]) for key, value in self.prompt_template.items()}
            
            try:
                self.current_timestamp = self.scene_data_to_websocket(rendered_prompt)
            except Exception as e:
                self.get_logger().error(f"[Loop {i}] Failed to send scene data: {e}")
                goal_handle.abort(); result.success = False; self.active_goal_handle = None
                return result

            try:
                if control_by_human:
                    self.get_logger().info(f"[Loop {i}] Waiting for user annotation (threading.Event)...")
                    self.on_message_event.clear()
                    self.current_action = None # 명시적 초기화

                    self.on_message_event.wait() 

                    if goal_handle.is_cancel_requested:
                        self.get_logger().info(f"[Loop {i}] Wait ended due to goal cancellation.")
                        goal_handle.canceled(); result.success = False; self.active_goal_handle = None
                        return result
                    
                    if self.current_action is not None:
                        self.get_logger().info(f"[Loop {i}] User annotation received: {str(self.current_action)[:100]}")
                    else:
                        self.get_logger().error(f"[Loop {i}] Event set, but current_action is None. Aborting.")
                        goal_handle.abort(); result.success = False; self.active_goal_handle = None
                        return result
                else: # AI control
                    agent_response= {"cognition": "", "reasoning": "", "action": ""}
                    gemma_res = None

                    gemma_res = await self.call_gemma(rendered_prompt["cognition"])
                    if goal_handle.is_cancel_requested: raise asyncio.CancelledError("Cognition Cancelled")
                    feedback.thought = f"Cognition (Loop {i}): {gemma_res.generated_text}"
                    self.get_logger().info(feedback.thought); goal_handle.publish_feedback(feedback)
                    agent_response["cognition"] = gemma_res.generated_text
                    self.agent_response_to_websocket(agent_response, self.current_timestamp)
                    self.req.has_rgb_image = False; self.req.has_d_image = False

                    gemma_res = await self.call_gemma(prompt=rendered_prompt["reasoning"], context=gemma_res.updated_context_json)
                    if goal_handle.is_cancel_requested: raise asyncio.CancelledError("Reasoning Cancelled")
                    feedback.thought = f"Reasoning (Loop {i}): {gemma_res.generated_text}"
                    self.get_logger().info(feedback.thought); goal_handle.publish_feedback(feedback)
                    agent_response["reasoning"] = gemma_res.generated_text
                    self.agent_response_to_websocket(agent_response, self.current_timestamp)
                    self.req.has_rgb_image = False; self.req.has_d_image = False
                    
                    gemma_res = await self.call_gemma(prompt=rendered_prompt["action"], context=gemma_res.updated_context_json)
                    if goal_handle.is_cancel_requested: raise asyncio.CancelledError("Action Generation Cancelled")
                    feedback.thought = f"Action (Loop {i}): {gemma_res.generated_text}"
                    self.get_logger().info(feedback.thought); goal_handle.publish_feedback(feedback)
                    agent_response["action"] = gemma_res.generated_text
                    self.agent_response_to_websocket(agent_response, self.current_timestamp)
                    
                    self.current_action = gemma_res.generated_text
                    last_gemma_response_text = gemma_res.generated_text
                    self.req = GenerateGemma.Request()

                if self.current_action is None:
                    self.get_logger().error(f"[Loop {i}] current_action is None before do_action(). Aborting.")
                    goal_handle.abort(); result.success = False; self.active_goal_handle = None
                    return result

                action_result = self.do_action(self.current_action)
                
                try:
                    action_obj = json.loads(self.current_action) if isinstance(self.current_action, str) else self.current_action
                    previous_action_list.append(action_obj)
                except json.JSONDecodeError:
                    previous_action_list.append(self.current_action) # 파싱 실패 시 문자열로 저장

                if action_result.get("is_end", False):
                    self.get_logger().info(f"[Loop {i}] 'end' action or task completion. Terminating loop.")
                    break

            except asyncio.CancelledError as ace:
                self.get_logger().warn(f"[Loop {i}] Task cancelled (e.g. during Gemma call): {ace}")
                if goal_handle.is_cancel_requested: goal_handle.canceled()
                else: goal_handle.abort()
                result.success = False; self.active_goal_handle = None
                return result
            except Exception as e:
                self.get_logger().error(f"[Loop {i}] Exception in execute_callback: {e}")
                goal_handle.abort(); result.success = False; self.active_goal_handle = None
                return result
        
        if goal_handle.status == rclpy.action.server.GoalStatus.STATUS_EXECUTING:
            if goal_handle.is_cancel_requested:
                 self.get_logger().info("Goal was cancelled just before completion.")
                 goal_handle.canceled(); result.success = False
            else:
                self.get_logger().info('All loops completed or task finished successfully.')
                goal_handle.succeed(); result.success = True
                
                final_output_text = ""
                if control_by_human:
                    final_output_text = "Task completed with human control."
                    if previous_action_list:
                        final_output_text += " Last action: " + json.dumps(previous_action_list[-1], ensure_ascii=False, indent=2)
                else:
                    final_output_text = last_gemma_response_text
                
                result.result = final_output_text
                if use_translation and is_translated_prompt:
                    translated_answer = self.translate_text_deepl(result.result, "KO")
                    if translated_answer is not None: result.result = translated_answer
        else:
             self.get_logger().info(f"Goal not in EXECUTING state (current: {goal_handle.status}). Not calling succeed().")
             result.success = False

        self.active_goal_handle = None
        return result

    def add_rbg_to_request(self, msg: Image):
        self.req.has_rgb_image = True; self.req.rgb_image = msg

    def add_depth_to_request(self, msg: Image):
        self.req.has_d_image = True; self.req.d_image = msg

    async def call_gemma(self, prompt: str, context: str = ""):
        if self.active_goal_handle and self.active_goal_handle.is_cancel_requested:
            raise asyncio.CancelledError("Gemma call cancelled by action goal state")

        self.req.prompt = prompt
        if context: self.req.context_json = context
        self.get_logger().info(f'Requesting Gemma service with prompt: {self.req.prompt[:100]}...')
        res = await self.gemma.call_async(self.req)
        self.get_logger().info('Gemma service request completed.')

        if not res.success:
            err_msg = res.error_message if hasattr(res, "error_message") else "N/A"
            self.get_logger().error(f'Gemma Response Failed. Error: {err_msg}')
            raise Exception(f"Gemma Response Failed: {err_msg}")
        return res

    def do_action(self, action_content: Any) -> Dict[str, Any]:
        action_dict = {}
        is_valid = False
        error_msg = ""

        if isinstance(action_content, str):
            is_valid, error_msg, action_dict, = validate_custom_json(action_content)
        elif isinstance(action_content, dict): # 이미 dict 형태일 경우 (예: user_annotation_callback에서 직접 dict로 설정)
            action_dict = action_content # 직접 할당
            # 필요하다면 여기서 action_dict의 유효성 검사 추가
            if "action" in action_dict and isinstance(action_dict["action"], dict) and "name" in action_dict["action"]:
                is_valid = True
            else:
                error_msg = "Provided action dictionary is missing 'action' or 'action.name' keys."
        else:
            error_msg = "action_content is not a string or dictionary."

        if not is_valid:
            log_msg = f"Invalid action content: {error_msg}. Content was: {str(action_content)[:200]}"
            self.get_logger().error(log_msg)
            raise ValueError(log_msg)

        self.get_logger().info(f"Validated action: {action_dict.get('action', {}).get('name', 'N/A')}")
        if action_dict.get("action", {}).get("name") == "end":
            return {"is_end": True}
        
        self.get_logger().info(f"Executing tool: {action_dict['action']['name']} with params {action_dict['action'].get('parameters', {})}")
        return {"is_end": False, "message": "Action executed (simulated)"}

    def scene_data_to_websocket(self, prompt_dict: Dict[str, str]) -> str:
        rgb_image_data, rgb_width, rgb_height = "", 0, 0
        d_image_data, d_width, d_height, d_encoding = "", 0, 0, ""
        current_time_iso = datetime.now(timezone.utc).isoformat(timespec='milliseconds').replace('+00:00', 'Z')
        timestamp = current_time_iso # 기본값

        if self.req.has_rgb_image and hasattr(self.req.rgb_image, 'data') and self.req.rgb_image.data:
            timestamp = datetime.fromtimestamp(
                self.req.rgb_image.header.stamp.sec + self.req.rgb_image.header.stamp.nanosec / 1e9, tz=timezone.utc
            ).isoformat(timespec='milliseconds').replace('+00:00', 'Z')
            rgb_image_data = image_to_base64(self.req.rgb_image, self.get_logger(), ".jpg")
            rgb_width, rgb_height = self.req.rgb_image.width, self.req.rgb_image.height
        else:
            self.get_logger().warn("RGB image data not available for scene_data_to_websocket.")

        if self.req.has_d_image and hasattr(self.req.d_image, 'data') and self.req.d_image.data:
            d_image_data = image_to_base64(self.req.d_image, self.get_logger(), ".png")
            d_width, d_height = self.req.d_image.width, self.req.d_image.height
            d_encoding = self.req.d_image.encoding
        else:
            self.get_logger().warn("Depth image data not available for scene_data_to_websocket.")

        scene_data = {
            "prompts": prompt_dict,
            "vision": {
                "rgbImage": {"data": rgb_image_data, "format": "jpeg", "width": rgb_width, "height": rgb_height},
                "depthImage": {"data": d_image_data, "format": "png", "width": d_width, "height": d_height, 
                               "encoding": d_encoding, "unit": "mm", "scale": "1.0"},
                "imageTimestamp": timestamp
            }
        }
        self.websocket.broadcast_message("sceneData", scene_data)
        return timestamp

    def agent_response_to_websocket(self, response_dict: Dict[str, str], timestamp: str):
        self.websocket.broadcast_message("agentResponse", {
            "responses": response_dict, "associatedImageTimestamp": timestamp
        })
        return timestamp # 일관성을 위해 timestamp 반환 (현재는 사용 안함)
    
    def user_annotation_callback(self, websocket: Any, msg: Dict[str, Any]):
        if msg.get("type") != "userAnnotation":
            return
        payload = msg.get("payload")
        if not payload or self.current_timestamp != payload.get("associatedImageTimestamp"):
            self.get_logger().info(f"Timestamp mismatch or no payload, ignoring user annotation. Expected: {self.current_timestamp}, Got: {payload.get('associatedImageTimestamp') if payload else 'N/A'}")
            return
        
        responses = payload.get("responses")
        action_from_user = responses.get("action") if responses else None
        if not action_from_user:
            self.get_logger().error("Action missing in userAnnotation payload responses.")
            return

        self.get_logger().info(f"User annotation received: {str(action_from_user)[:100]}. Setting event.")
        self.current_action = action_from_user # self.current_action을 직접 설정 (JSON 문자열 또는 dict)
        self.on_message_event.set()

    def translate_text_deepl(self, text_input: str, target_lang: str, api_key: Optional[str] = None) -> Optional[str]:
        actual_api_key = api_key or os.getenv("DEEPL_API_KEY")
        if not actual_api_key:
            self.get_logger().error("DeepL API key not provided."); return None
        try:
            translator = deepl.Translator(actual_api_key)
            result = translator.translate_text(text_input, target_lang=target_lang)
            return result.text
        except deepl.DeepLException as de:
            self.get_logger().error(f'DeepL API Error: {de}'); return None
        except Exception as e:
            self.get_logger().error(f'Translate Failed with general error: {e}'); return None

def main(args=None):
    rclpy.init(args=args)
    agricsense_action_server = AgricsenseActionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(agricsense_action_server)
    try:
        executor.spin()
    except KeyboardInterrupt:
        agricsense_action_server.get_logger().info("KeyboardInterrupt, shutting down.")
    except Exception as e:
        agricsense_action_server.get_logger().error(f"Exception during executor spin: {e}")
    finally:
        agricsense_action_server.get_logger().info("Shutting down executor.")
        executor.shutdown()
        if hasattr(agricsense_action_server, 'websocket') and hasattr(agricsense_action_server.websocket, 'stop'):
            agricsense_action_server.get_logger().info("Stopping WebSocket server...")
            agricsense_action_server.websocket.stop()
        agricsense_action_server.destroy_node()
        rclpy.shutdown()
    print("Program exited.")

if __name__ == '__main__':
    main()