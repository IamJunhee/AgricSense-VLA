import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from sensor_msgs.msg import Image
from agricsense_action_interfaces.action import Agricsense
from gemma_ros2_interface.srv import GenerateGemma

import deepl
import json
import os
from typing import Optional

from jinja2 import Environment, FileSystemLoader

class AgricsenseControlPannel(Node):

    def __init__(self):
        super().__init__('agricsense_action_server')

        self.declare_parameter('use_translation', False)
        self.declare_parameter('prompt_template', "/root/AgricSense-VLA/jackal_ws/src/gemma_ros_client/resource/prompt_template_kor.txt")
        self.declare_parameter('tools_json', "/root/AgricSense-VLA/jackal_ws/src/gemma_ros_client/resource/tools_kor.json")

        # Tool list
        tools_json_path = self.get_parameter('tools_json').get_parameter_value().string_value

        with open(tools_json_path) as f:
            self.tools_json = json.load(f)

        # Prompt Template
        prompt_template_path = self.get_parameter('prompt_template').get_parameter_value().string_value

        self.env = Environment(
            loader=FileSystemLoader(os.path.dirname(prompt_template_path))
        )

        self.prompt_template = self.env.get_template(os.path.basename(prompt_template_path))

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

        while not self.gemma.wait_for_service(timeout_sec=10.0):
            self.get_logger().info('service not available, waiting again...')
        
        self.req = GenerateGemma.Request()

        self.get_logger().info('Action server is ready.')


def main(args=None):
    rclpy.init(args=args)

    agricsense_control_pannel = AgricsenseControlPannel()

    rclpy.spin(agricsense_control_pannel)


if __name__ == '__main__':
    main()