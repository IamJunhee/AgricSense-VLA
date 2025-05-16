import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import Header
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from agricsense_action_interfaces.action import Agricsense
from gemma_ros2_interface.srv import GenerateGemma
from nav_msgs.msg import Odometry

from .base64_util import image_to_base64
from .json_validate import validate_custom_json
from .websocket_util import ROS2WebSocketServer
from .action_launcher import ActionLauncher

import deepl
import json
import math
import csv
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
        self.declare_parameter('loop_limit', 60)
        self.declare_parameter('use_translation', False)
        self.declare_parameter('prompt_template_path', "/root/AgricSense-VLA/jackal_ws/src/gemma_ros_client/resource/prompt_ko/")
        self.declare_parameter('csv_path', "/root/AgricSense-VLA/jackal_ws/src/gemma_ros_client/resource/farm_info/")
        self.declare_parameter('data_path', os.path.join(os.path.expanduser("~"), "AgricSense-VLA/human_dataset"))
        
        self.data_path_base = self.get_parameter('data_path').get_parameter_value().string_value
        self.csv_path_base = self.get_parameter('csv_path').get_parameter_value().string_value
        self.farm_info = read_csv(self.csv_path_base)

        self.current_action: Optional[str] = None # JSON string 또는 dict가 될 수 있음
        self.current_response: Optional[dict] = None
        self.active_goal_handle: Optional[rclpy.action.server.ServerGoalHandle] = None
        self.current_timestamp: Optional[str] = None

        prompt_template_path = self.get_parameter('prompt_template_path').get_parameter_value().string_value
        self.env = Environment(loader=FileSystemLoader(prompt_template_path))
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
        self.current_pose_sub = self.create_subscription(Odometry, '/odometry/filtered/global', self.odom_callback, 10)
        self.gemma = self.create_client(GenerateGemma, 'generate_gemma')
        self.req = GenerateGemma.Request()
        self.current_pose : Pose = None

        self.websocket = ROS2WebSocketServer("localhost", 9090)
        self.websocket.register_callbacks(on_message=self.user_annotation_callback)
        self.websocket.start()

        self.action_launcher = ActionLauncher()

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
        rgb_image_list = []

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
                "cognition": {"user_prompt": real_prompt,
                              "farm_info": self.format_farm_info(),
                              "current_location": "없음" if self.current_pose is None 
                                    else "({:0.2f}, {:0.2f}, {:0.2f})".format(*pose_to_xy_angle(self.current_pose)), 
                              "previous_action_json": json.dumps(previous_action_list, indent=2, ensure_ascii=False)},
                "reasoning": {},
                "action": {"tools_list_json": json.dumps(self.tools_json, indent=2, ensure_ascii=False)}
            }
            rendered_prompt = {key: value.render(data[key]) for key, value in self.prompt_template.items()}
            
            try:
                self.current_timestamp, rgb_64, d_64 = self.scene_data_to_websocket(rendered_prompt)
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

                action_result = await self.do_action(self.current_action)
                
                if control_by_human:
                    chat = make_chat(rgb_64, d_64, rendered_prompt, self.current_response)
                    self.save_chat(chat)

                rgb_image_list.append(rgb_64)

                if action_result.get("is_end", False):
                    self.get_logger().info(f"[Loop {i}] 'end' action or task completion. Terminating loop.")
                    # TODO: 찍어온 사진 보고하기 rgb_image_list 활용
                    break

                previous_action_list.append(action_result["result"])

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

    def odom_callback(self, msg: Odometry):
        self.current_pose = msg.pose.pose

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

    async def do_action(self, action_content: str) -> Dict[str, Any]:
        action_dict = {}
        is_valid = False
        error_msg = ""

        if isinstance(action_content, str):
            is_valid, error_msg, action_dict, = validate_custom_json(action_content)
        else:
            error_msg = "action_content is not a string."

        if not is_valid:
            log_msg = f"Invalid action content: {error_msg}. Content was: {str(action_content)[:200]}"
            self.get_logger().error(log_msg)
            raise ValueError(log_msg)

        self.get_logger().info(f"Validated action: {action_dict.get('action', {}).get('name', 'N/A')}")
        
        # TODO: 작업 완료 후 로직
        if action_dict.get("action", {}).get("name") == "end":
            return {"is_end": True}
        
        # TODO: Action 실행 로직
        self.get_logger().info(f"Executing tool: {action_dict['action']['name']} with params {action_dict['action'].get('parameters', {})}")
        
        action_result = await self.action_launcher.launch_action(action_dict)
        
        return { "is_end": False, "result": action_result }

    def scene_data_to_websocket(self, prompt_dict: Dict[str, str]):
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
        return timestamp, rgb_image_data, d_image_data

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
        self.current_response = responses
        
        action_from_user = responses.get("action") if responses else None
        if not action_from_user:
            self.get_logger().error("Action missing in userAnnotation payload responses.")
            return

        self.get_logger().info(f"User annotation received: {str(action_from_user)[:100]}. Setting event.")
        self.current_action = action_from_user # self.current_action을 직접 설정 (JSON 문자열 또는 dict)
        self.on_message_event.set()

    def format_farm_info(self):
        def calculate_distance(item):
            dx = self.current_pose.position.x - item[0]
            dy = self.current_pose.position.y - item[1]

            return math.sqrt(dx**2 + dy**2)

        sorted_list = sorted(self.farm_info, key=calculate_distance)
        formatted_list = [f"({item[0]:.2f}, {item[1]:.2f}, {item[2]:.2f}: {item[3]})" for item in sorted_list]
        
        return "\n".join(formatted_list)

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
        
    def save_chat(self, chat: dict):
        os.makedirs(self.data_path_base, exist_ok=True)

        user_identifier = os.environ.get('USER', 'unknown_user')

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        file_name = f"chat_{user_identifier}_{timestamp}.json"

        full_file_path = os.path.join(self.data_path_base, file_name)

        with open(full_file_path, 'w', encoding='utf-8') as f:
            json.dump(chat, f, ensure_ascii=False, indent=4)

        self.get_logger().info(f"Info: Chat data successfully saved to {full_file_path}")
        
def pose_to_xy_angle(pose):
    qw = pose.orientation.w
    qx = pose.orientation.x
    qy = pose.orientation.y
    qz = pose.orientation.z

    standard_yaw_rad = math.atan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz))

    angle = math.degrees(standard_yaw_rad)

    return pose.position.x, pose.position.y, angle

def read_csv(path):
    result = [ ]
    file_path = os.path.join(path, "farm_info.csv")

    with open(file_path, newline='', encoding='utf-8') as f:
        reader = csv.reader(f)
        next(reader)

        for row in reader:
            r_x_str = row[0]
            r_y_str = row[1]
            r_angle_str = row[2]
            r_extra = row[3] # 네 번째 필드가 없으면 IndexError 발생

            x = float(r_x_str)
            y = float(r_y_str)
            angle = float(r_angle_str)

            new_row = [x,y,angle, r_extra]
            result.append(new_row)

    return result

def make_chat(rgb_64, d_64, prompt_dict, response_dict) -> dict:
    messages = [
        {
            "role": "user",
            "content": [
                {"type": "image", "base64": rgb_64},
                {"type": "image", "base64": d_64},
                {"type": "text", "text": prompt_dict["cognition"]}
            ] 
        },
        {
            "role": "assistant",
            "content": [
                {"type": "text", "text": response_dict["cognition"]}
            ]
        },
        {
            "role": "user",
            "content": [
                {"type": "text", "text": prompt_dict["reasoning"]}
            ] 
        },
        {
            "role": "assistant",
            "content": [
                {"type": "text", "text": response_dict["reasoning"]}
            ]
        },
        {
            "role": "user",
            "content": [
                {"type": "text", "text": prompt_dict["action"]}
            ] 
        },
        {
            "role": "assistant",
            "content": [
                {"type": "text", "text": response_dict["action"]}
            ]
        }
    ]

    return dict(messages = messages)


def main(args=None):
    rclpy.init(args=args)
    agricsense_action_server = AgricsenseActionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(agricsense_action_server)
    executor.add_node(agricsense_action_server.action_launcher)
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