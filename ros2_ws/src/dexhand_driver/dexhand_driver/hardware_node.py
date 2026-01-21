import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
import math
import random

# TODO 需要重新解决文件结构
# cd src/pyzlg_dexhand
# python3 -m pip install -e .
from pyzlg_dexhand.dexhand_interface import RightDexHand

HAS_DRIVER = True


class DexHandNode(Node):
    def __init__(self):
        super().__init__("dexhand_hardware_node")

        self.mode = "CHECKING"
        self.hand = None
        self.start_time = time.time()

        self.status_publisher = self.create_publisher(String, "/dexhand/status", 10)

        self.connect_hardware()

        self.timer = self.create_timer(0.1, self.timer_callback)

    def connect_hardware(self):
        """判断是真机还是模拟"""
        if HAS_DRIVER:
            try:
                self.get_logger().info("正在连接真实硬件...")
                self.hand = RightDexHand(auto_init=True)
                if self.hand.get_board_firmware_version(0):
                    self.mode = "REAL"
                    self.get_logger().info("✅ 真实硬件连接成功")
                else:
                    raise Exception("无法读取固件")
            except Exception as e:
                self.get_logger().error(f"❌ 硬件连接失败: {e} -> 切换至模拟模式")
                self.mode = "MOCK"
        else:
            self.get_logger().warn("未找到驱动库，进入模拟模式")
            self.mode = "MOCK"

    def timer_callback(self):
        current_data = {}

        if self.mode == "REAL" and self.hand:
            try:
                feedback = self.hand.get_feedback()
                current_data = {"joints": {k: v.angle for k, v in feedback.joints.items()}, "touch": {k: v.normal_force for k, v in feedback.touch.items()}}
            except Exception:
                self.get_logger().warn("读取硬件失败，切换回模拟模式")
                self.mode = "MOCK"

        if self.mode == "MOCK":
            t = time.time() - self.start_time
            current_data = {
                "joints": {"th_rot": 30 + 10 * math.sin(t), "ff_mcp": 45 + 15 * math.cos(t), "lf_mcp": 10 + 5 * math.sin(t * 2)},
                "touch": {"th": abs(2 * math.sin(t)), "ff": abs(2 * math.cos(t))},
            }

        msg_payload = {
            "mode": self.mode,
            "timestamp": time.time(),
            "payload": current_data,
        }

        msg = String()
        msg.data = json.dumps(msg_payload)
        self.status_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DexHandNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
