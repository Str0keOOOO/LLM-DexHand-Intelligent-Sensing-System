import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
import math
import random
from pyzlg_dexhand.dexhand_interface import DexHand, RightDexHand


HAS_DRIVER = True
class DexHandNode(Node):
    def __init__(self):
        super().__init__("dexhand_hardware_node")

        self.mode = "CHECKING"
        self.hand = None

        # 1. 尝试连接硬件
        self.connect_hardware()

        # 2. 发布话题
        self.status_publisher = self.create_publisher(String, "/dexhand/status", 10)

        # 3. 定时器 10Hz
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.start_time = time.time()

    def connect_hardware(self):
        """判断是真机还是模拟"""
        if HAS_DRIVER:
            try:
                self.get_logger().info("正在连接真实硬件...")
                # 这里根据您的实际驱动类来实例化
                self.hand = RightDexHand(auto_init=True)
                # 简单验证：读取版本号
                if self.hand.get_board_firmware_version(0):
                    self.mode = "REAL"
                    self.get_logger().info("✅ 真实硬件连接成功")
                else:
                    raise Exception("无法读取固件")
            except Exception as e:
                self.get_logger().error(f"❌ 硬件连接失败: {e}")
                self.mode = "MOCK"
        else:
            self.get_logger().warn("未找到驱动库，进入模拟模式")
            self.mode = "MOCK"

    def timer_callback(self):
        current_data = {}

        # A. 获取数据
        if self.mode == "REAL" and self.hand:
            try:
                # 获取真实反馈
                feedback = self.hand.get_feedback()
                current_data = {"joints": {k: v.angle for k, v in feedback.joints.items()}, "touch": {k: v.normal_force for k, v in feedback.touch.items()}}
            except Exception:
                self.mode = "MOCK"  # 运行时掉线自动切换

        # B. 如果是模拟模式 (或掉线)，生成模拟信号
        if self.mode == "MOCK":
            t = time.time() - self.start_time
            # 模拟正弦波数据
            current_data = {
                "joints": {"th_rot": 30 + 10 * math.sin(t), "ff_mcp": 45 + 15 * math.cos(t), "lf_mcp": 10 + 5 * math.sin(t * 2)},
                "touch": {"th": abs(2 * math.sin(t)), "ff": abs(2 * math.cos(t))},
            }

        # C. 构造统一的数据包
        msg_payload = {
            "mode": self.mode,  # 告诉后端/前端当前是真机还是模拟
            "timestamp": time.time(),
            "payload": current_data,
        }

        # 发布 JSON 字符串
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
