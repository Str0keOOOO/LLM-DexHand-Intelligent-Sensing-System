import threading
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
from app.config import settings

class DexHandNode(Node):
    def __init__(self):
        super().__init__('backend_bridge')
        self.cmd_pub = self.create_publisher(String, settings.ROS_TOPIC_COMMAND, 10)
        self.sensor_sub = self.create_subscription(
            Float32MultiArray, settings.ROS_TOPIC_SENSORS, self.on_sensor, 10
        )
        self.latest_data = [0.0, 0.0, 0.0] # 简单的内存存储

    def on_sensor(self, msg):
        self.latest_data = list(msg.data)
        # 这里也可以顺便写入数据库，或者回调出去

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