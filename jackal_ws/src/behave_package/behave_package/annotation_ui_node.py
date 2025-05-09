import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import tkinter as tk
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
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            qos_profile  
        )

        self.csv_path = os.path.join(os.path.expanduser("~"), "annotations.csv")
        if not os.path.exists(self.csv_path):
            with open(self.csv_path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['x', 'y', 'theta', 'label'])

        self.get_logger().info("📝 Annotation UI Node 시작됨")

    def pose_callback(self, msg: PoseWithCovarianceStamped):
        self.get_logger().info("📡 /amcl_pose 수신됨")
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
            self.get_logger().info(f"📍 {label} 저장됨: x={x:.2f}, y={y:.2f}, θ={theta:.2f}")

def launch_ui(node: AnnotationUINode):
    window = tk.Tk()
    window.title("밭 라벨링 UI")

    def make_button(label):
        return tk.Button(window, text=label, width=20, height=2,
                         command=lambda: node.save_annotation(label))
    make_button("tomato").pack(pady=10)
    make_button("potato").pack(pady=10)

    window.mainloop()

def main(args=None):
    rclpy.init(args=args)
    node = AnnotationUINode()

    # ROS 콜백 스레드 실행
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    try:
        launch_ui(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

