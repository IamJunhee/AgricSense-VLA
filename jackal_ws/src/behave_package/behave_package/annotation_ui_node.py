import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry  # ✅ 변경된 메시지 타입
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

        self.current_pose = None
        qos_profile = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )

        # ✅ Odometry 토픽
        self.subscription = self.create_subscription(
            Odometry,
            '/odometry/global',
            self.pose_callback,
            qos_profile
        )

        self.csv_path = os.path.join(os.path.expanduser("~"), "annotations.csv")
        if not os.path.exists(self.csv_path):
            with open(self.csv_path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['x', 'y', 'theta', 'label'])

        self.get_logger().info("📝 Annotation UI Node 시작됨")

    def pose_callback(self, msg: Odometry):
        if self.current_pose is None:
            self.get_logger().info("📡 /odometry/global 수신")
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
        theta = math.atan2(siny_cosp, cosy_cosp)

        with open(self.csv_path, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([x, y, theta, label])
            self.get_logger().info(f"📍 저장됨: x={x:.2f}, y={y:.2f}, θ={theta:.2f}, label={label}")

def launch_ui(node: AnnotationUINode):
    window = tk.Tk()
    window.title("라벨링 UI")

    def ask_and_save():
        label = simpledialog.askstring("input", "description : ")
        if label:
            node.save_annotation(label)

    tk.Button(window, text="point save", width=25, height=3, command=ask_and_save).pack(pady=20)

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

