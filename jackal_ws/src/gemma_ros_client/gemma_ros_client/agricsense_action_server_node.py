import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from sensor_msgs.msg import Image
from agricsense_action_interfaces.action import Agricsense
from gemma_ros2_interface.srv import GenerateGemma

import deepl
import os
from typing import Optional

class AgricsenseActionServer(Node):

    def __init__(self):
        super().__init__('agricsense_action_server')

        self.declare_parameter('use_translation', False)

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
            '/zed/depth/image_raw',
            self.add_depth_to_request,
            10)

        # Gemma Service Client
        self.gemma = self.create_client(GenerateGemma, 'generate_gemma')

        while not self.gemma.wait_for_service(timeout_sec=10.0):
            self.get_logger().info('service not available, waiting again...')
        
        self.req = GenerateGemma.Request()

        self.get_logger().info('Action server is ready.')


    async def execute_callback(self, goal_handle):
        use_translation = self.get_parameter('use_translation').get_parameter_value().bool_value

        result = Agricsense.Result()
        feedback = Agricsense.Feedback()

        real_prompt = goal_handle.request.prompt

        if use_translation:
            is_translated = True

            self.get_logger().info('Try translation')
            real_prompt = self.translate_text_deepl(goal_handle.request.prompt, "EN-US")

            if real_prompt is None:
                self.get_logger().info('Translate Failed using original prompt')
                
                real_prompt = goal_handle.request.prompt
                is_translated = False

        
        self.req.prompt = real_prompt

        self.get_logger().info('request gemma service...')
        res = await self.gemma.call_async(self.req)
        self.get_logger().info('request gemma service... Done')

        if not(res.success):
            self.get_logger().info('Gemma Response Failed')
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

    def add_rbg_to_request(self, msg):
        self.req.has_rgb_image = True
        self.req.rgb_image = msg

    def add_depth_to_request(self, msg):
        self.req.has_d_image = True
        self.req.d_image = msg


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