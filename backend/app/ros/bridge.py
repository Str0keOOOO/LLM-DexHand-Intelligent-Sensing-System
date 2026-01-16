import threading
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
from app.config import settings

class DexHandNode(Node):
    def __init__(self):
        super().__init__('backend_bridge')
        
        # 发布控制指令
        self.cmd_pub = self.create_publisher(String, settings.ROS_TOPIC_COMMAND, 10)
        
        # 1. 订阅传感器数据 (兼容 virtual_sensor_pub.py)
        self.sensor_sub = self.create_subscription(
            Float32MultiArray, settings.ROS_TOPIC_SENSORS, self.on_sensor, 10
        )
        
        # 2. 新增：订阅硬件状态 (适配 hardware_node.py 的 JSON 格式)
        self.status_sub = self.create_subscription(
            String, '/dexhand/status', self.on_status, 10
        )

        # 内存存储最新状态
        self.latest_state = {
            "fingers": [0.0, 0.0, 0.0, 0.0],
            "timestamp": 0
        }

    def on_sensor(self, msg):
        """处理 Float32MultiArray 类型的传感器数据"""
        # 将数组转换为列表并更新
        data = list(msg.data)
        self.latest_state["fingers"] = data
        # 这里也可以顺便写入数据库

    def on_status(self, msg):
        """处理 String (JSON) 类型的硬件状态数据"""
        try:
            data = json.loads(msg.data)
            # 假设 hardware_node 发送格式为 {"fingers": [...], "timestamp": ...}
            if "fingers" in data:
                self.latest_state["fingers"] = data["fingers"]
            if "timestamp" in data:
                self.latest_state["timestamp"] = data["timestamp"]
        except json.JSONDecodeError:
            self.get_logger().error("后端收到无法解析的 JSON 状态数据")

    def send_action(self, action: dict):
        msg = String()
        msg.data = json.dumps(action)
        self.cmd_pub.publish(msg)

# 全局单例
ros_node = None

def start_ros():
    global ros_node
    if not rclpy.ok(): rclpy.init()
    ros_node = DexHandNode()
    threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True).start()
    return ros_node