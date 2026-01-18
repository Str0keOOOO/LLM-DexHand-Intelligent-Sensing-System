import threading
import json
import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class BackendBridgeNode(Node):
    def __init__(self):
        super().__init__("backend_bridge_listener")

        # 订阅 ROS 节点发出的状态
        self.status_sub = self.create_subscription(String, "/dexhand/status", self.on_status_received, 10)

        # 内存缓存：用于存最新的 ROS 数据，供 WebSocket 读取
        # 默认状态
        self.latest_state = {
            "mode": "DISCONNECTED",  # 还没收到 ROS 数据时
            "payload": None,
            "timestamp": 0,
        }

    def on_status_received(self, msg):
        """收到 ROS 数据，更新内存"""
        try:
            data = json.loads(msg.data)
            self.latest_state = data  # 直接覆盖，包含 mode 和 payload
        except json.JSONDecodeError:
            pass


# 全局单例
bridge_node = None


def start_ros_bridge():
    """在后台线程启动 ROS 节点"""
    global bridge_node
    if not rclpy.ok():
        rclpy.init()

    bridge_node = BackendBridgeNode()

    # 启动线程通过 spin 监听消息
    thread = threading.Thread(target=rclpy.spin, args=(bridge_node,), daemon=True)
    thread.start()
    return bridge_node
