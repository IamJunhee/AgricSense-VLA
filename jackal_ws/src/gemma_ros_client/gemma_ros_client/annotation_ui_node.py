import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import tkinter as tk
from tkinter import simpledialog
import csv
import os
import math
import threading
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

class AnnotationUINode(Node):
    def __init__(self):
        super().__init__('annotation_ui_node')

        self.declare_parameter('csv_path', "/root/AgricSense-VLA/jackal_ws/src/gemma_ros_client/resource/farm_info/")
        self.csv_path_base = self.get_parameter('csv_path').get_parameter_value().string_value

        self.current_pose = None
        qos_profile = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )

        self.subscription = self.create_subscription(
            Odometry,
            '/odometry/filtered/global',
            self.pose_callback,
            qos_profile
        )

        self.csv_path = os.path.join(self.csv_path_base, "farm_info.csv")
        if not os.path.exists(self.csv_path):
            with open(self.csv_path, 'w', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                writer.writerow(['x', 'y', 'angle', 'label'])

        self.get_logger().info("📝 Annotation UI Node 시작됨")

    def pose_callback(self, msg: Odometry):
        self.current_pose = msg.pose.pose


    def save_annotation(self, label: str):
        if self.current_pose is None:
            self.get_logger().warn("❗ 현재 위치를 아직 수신하지 못했습니다.")
            return

        x = self.current_pose.position.x
        y = self.current_pose.position.y

        q = self.current_pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        angle = math.degrees(math.atan2(siny_cosp, cosy_cosp))

        with open(self.csv_path, 'a', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            writer.writerow([x, y, angle, label])
            self.get_logger().info(f"📍 저장됨: x={x:.2f}, y={y:.2f}, θ={angle:.2f}, label={label}")

def launch_ui(node: AnnotationUINode):
    window = tk.Tk()
    window.tk.eval('encoding system utf-8')
    window.title("라벨링 UI")

    def ask_and_save():
        label = simpledialog.askstring("입력", "지역 설명 : ")
        if label:
            node.save_annotation(label)

    tk.Button(window, text="지역 정보 저장", width=25, height=3, command=ask_and_save).pack(pady=20)

    window.mainloop()

def main(args=None):
    rclpy.init(args=args)
    node = AnnotationUINode()

    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    try:
        launch_ui(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

