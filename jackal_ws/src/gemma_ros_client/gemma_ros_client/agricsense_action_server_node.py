import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from std_msgs.msg import Header
from sensor_msgs.msg import Image
from agricsense_action_interfaces.action import Agricsense
from gemma_ros2_interface.srv import GenerateGemma

from .base64_util import image_to_base64
from .json_validate import validate_custom_json
from .websocket_util import ROS2WebSocketServer

import deepl
import json
import os
from pprint import pformat
from typing import Optional
from datetime import timezone, datetime

from jinja2 import Environment, FileSystemLoader

class AgricsenseActionServer(Node):
# TODO: 사람이 직접 조종할 수 있도록!
    def __init__(self):
        super().__init__('agricsense_action_server')

        self.declare_parameter('control_by_human', False)
        self.declare_parameter('loop_limit', 20)
        self.declare_parameter('use_translation', False)
        self.declare_parameter('prompt_template_path', "/root/AgricSense-VLA/jackal_ws/src/gemma_ros_client/resource/prompt_ko/")

        # Prompt Template
        prompt_template_path = self.get_parameter('prompt_template_path').get_parameter_value().string_value

        self.env = Environment(
            loader=FileSystemLoader(os.path.dirname(prompt_template_path))
        )

        self.prompt_template = {
            "cognition" : self.env.get_template("cognition_prompt_template.txt"),
            "reasoning" : self.env.get_template("reasoning_prompt_template.txt"),
            "action" : self.env.get_template("action_prompt_template.txt")
        }

        # Tool list
        with open(os.path.join(prompt_template_path, "tools.json")) as f:
            self.tools_json = json.load(f)
        
        # Action Server
        self._action_server = ActionServer(
            self,
            Agricsense,
            'agricsense',
            self.execute_callback)

        # Image Topic Subscriber
        self.rgb_sub = self.create_subscription(
            Image,
            '/zed/image_raw',
            self.add_rbg_to_request,
            10)

        self.depth_sub = self.create_subscription(
            Image,
            '/zed/depth/image_16uc1_mm',
            self.add_depth_to_request,
            10)

        # Gemma Service Client
        self.gemma = self.create_client(GenerateGemma, 'generate_gemma')
        
        self.req = GenerateGemma.Request()

        self.websocket = ROS2WebSocketServer("localhost", 9090)

        self.websocket.start()

        self.get_logger().info('Action server is ready.')


    async def execute_callback(self, goal_handle):
        use_translation = self.get_parameter('use_translation').get_parameter_value().bool_value
        control_by_human = self.get_parameter('control_by_human').get_parameter_value().bool_value
        loop_limit = self.get_parameter('loop_limit').get_parameter_value().integer_value

        result = Agricsense.Result()
        feedback = Agricsense.Feedback()

        real_prompt = goal_handle.request.prompt

        if not control_by_human:
            while not self.gemma.wait_for_service(timeout_sec=10.0):
                self.get_logger().info('service not available, waiting again...')

        # 작동 모드 웹소켓에 전달
        self.websocket.broadcast_message("setOperationMode", {
                                            "mode" : "data_collection" if control_by_human else "execution_monitoring"
                                            })

        if use_translation:
            is_translated = True

            self.get_logger().info('Try translation')
            real_prompt = self.translate_text_deepl(goal_handle.request.prompt, "EN-US")

            if real_prompt is None:
                self.get_logger().info('Translate Failed using original prompt')
                
                real_prompt = goal_handle.request.prompt
                is_translated = False

        previous_action_list = []

        for i in range(loop_limit):
            # TODO : Current location 추가
            # TODO : farm_info 추가

            data = {
                "cognition" : {
                    "user_prompt" : real_prompt,
                    "farm_info" : "없음",
                    "current_location" : "없음",
                    "previous_action_json" : json.dumps(previous_action_list, indent=2, ensure_ascii=False)
                },
                "reasoning" : {},
                "action" : {
                    "tools_list_json" : json.dumps(self.tools_json, indent=2, ensure_ascii=False),
                }
            }

            rendered_prompt = {key: value.render(data[key]) for key, value in self.prompt_template.items()}

            timestamp = self.scene_data_to_websocket(rendered_prompt)

            try:
                if (control_by_human):
                    action = self.loop_by_human()

                else:
                    agent_response= dict(cognition = "", reasoning = "", action = "")

                    # Cognition
                    res = await self.call_gemma(rendered_prompt["cognition"])
                    feedback.thought = "Cognition of loop {0} : {1}".format(i, res.generated_text)
                    self.get_logger().info(feedback.thought)
                    goal_handle.publish_feedback(feedback)
                    agent_response["cognition"] = res.generated_text

                    self.agent_response_to_websocket(agent_response, timestamp)

                    # 다시 이미지를 전송할 필요는 없음
                    self.req.has_rgb_image = False
                    self.req.has_d_image = False

                    # Reasoning
                    res = await self.call_gemma(prompt = rendered_prompt["reasoning"],
                                                context = res.updated_context_json)
                    feedback.thought = "Reasoning of loop {0} : {1}".format(i, res.generated_text)
                    self.get_logger().info(feedback.thought)
                    goal_handle.publish_feedback(feedback)
                    agent_response["reasoning"] = res.generated_text

                    self.agent_response_to_websocket(agent_response, timestamp)

                    # 다시 이미지를 전송할 필요는 없음
                    self.req.has_rgb_image = False
                    self.req.has_d_image = False

                    # Action
                    res = await self.call_gemma(prompt = rendered_prompt["action"],
                                                context = res.updated_context_json)
                    feedback.thought = "Selected Action of loop {0} : {1}".format(i, res.generated_text)
                    self.get_logger().info(feedback.thought)
                    goal_handle.publish_feedback(feedback)
                    agent_response["action"] = res.generated_text

                    self.agent_response_to_websocket(agent_response, timestamp)

                    action = res.generated_text

                    self.req = GenerateGemma.Request()

                action_result = self.do_action(action)
                
                previous_action_list.append(action)

                if action_result["is_end"]:
                    # TODO: 루프 종료 시 내용 상세 구현
                    break

            except:
                goal_handle.abort()
                result.success = False
                
                return result


        self.get_logger().info('Gemma Response Success')
        goal_handle.succeed()

        self.get_logger().info('Gemma Response : {}'.format(res.generated_text))
        result = Agricsense.Result()
        generated = res.generated_text
        
        if use_translation:
            if is_translated:
                self.get_logger().info('Try translation')
                translated_answer = self.translate_text_deepl(generated, "KO")

                if translated_answer is None:
                    self.get_logger().info('Translate Failed using original answer')
                    generated = res.generated_text
                
                generated = translated_answer

        result.result = generated
        result.success = True
        return result

    def add_rbg_to_request(self, msg: Image):
        self.req.has_rgb_image = True
        self.req.rgb_image = msg

    def add_depth_to_request(self, msg: Image):
        self.req.has_d_image = True
        self.req.d_image = msg

    async def call_gemma(self, prompt, context : str = ""):
        self.req.prompt = prompt
        if context:
            self.get_logger().info("Add Context")
            self.req.context_json = context

        self.get_logger().info('request gemma service... \n{}'.format(self.req.prompt))
        res = await self.gemma.call_async(self.req)
        self.get_logger().info('request gemma service... Done')

        if not(res.success):
            self.get_logger().info('Gemma Response Failed')
            raise Exception("Gemma Response Failed")

        return res

    def do_action(self, action_json):
        is_valid, error_msg, action_dict = validate_custom_json(action_json)
        if not(is_valid):
            self.get_logger().info("Invalid generated action json : " + error_msg)
            raise Exception("Invalid generated action json : " + error_msg)
        
        # TODO: Loop 종료 상세하게 구현
        if action_dict["action"]["name"] == "end":
            return {
                "is_end" : True
            }

        # TODO: Control Node 호출

    def loop_by_human(self):
        # TODO: 사람이 작성한 답변 받아서
        # TODO: Action 저장
        # TODO: Action JSON String 반환
        pass

    def scene_data_to_websocket(self, prompt_dict):
        timestamp = datetime.fromtimestamp(
                    self.req.rgb_image.header.stamp.sec + self.req.rgb_image.header.stamp.nanosec / 1e9, 
                    tz=timezone.utc
                    ).isoformat(timespec='milliseconds').replace('+00:00', 'Z')

        scene_data = {
            "prompts" : {
                "cognition": prompt_dict["cognition"],
                "thinking": prompt_dict["reasoning"],
                "action": prompt_dict["action"] 
            },
            "vision" : {
                "rgbImage" : {
                    "data" : image_to_base64(self.req.rgb_image, self.get_logger(), ".jpg"),
                    "format" : "jpeg",
                    "width" : self.req.rgb_image.width,
                    "height" : self.req.rgb_image.height
                },
                "depthImage" : {
                    "data" : image_to_base64(self.req.d_image, self.get_logger(), ".png"),
                    "format" : "png",
                    "width" : self.req.d_image.width,
                    "height" : self.req.d_image.height,
                    "encoding" : "16UC1",
                    "unit" : "mm",
                    "scale" : "1.0"
                },
                "imageTimestamp" : timestamp
            }
        }

        self.websocket.broadcast_message("sceneData", scene_data)

        return timestamp

    def agent_response_to_websocket(self, response_dict, timestamp):
        response_data = {
            "responses" : {
                "cognition": response_dict["cognition"],
                "thinking": response_dict["reasoning"],
                "action": response_dict["action"] 
            },
            "associatedImageTimestamp" : timestamp
        }

        self.websocket.broadcast_message("agentResponse", response_data)

        return timestamp


    def translate_text_deepl(self, text_input: str, target_lang: str, api_key: Optional[str] = None) -> str:
        actual_api_key = api_key or os.getenv("DEEPL_API_KEY")
        if not actual_api_key:
            print("Error: DeepL API key not provided.")
            print("Please pass it as an argument or set the DEEPL_API_KEY environment variable.")
            return None


        try:
            translator = deepl.Translator(actual_api_key)
            result = translator.translate_text(text_input, target_lang=target_lang)

        except:
            self.get_logger().info('Translate Failed')
            result = None

        return result.text
    

def main(args=None):
    rclpy.init(args=args)

    agricsense_action_server = AgricsenseActionServer()

    rclpy.spin(agricsense_action_server)


if __name__ == '__main__':
    main()