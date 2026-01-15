import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

# 全局变量，用于存储最新的机器人状态
# 默认值，防止启动初期无数据报错
latest_robot_state = {
    "fingers": [0.0, 0.0, 0.0]
}

class RosSubscriber(Node):
    def __init__(self):
        super().__init__('fastapi_bridge_node')
        # 订阅 ROS2 话题
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/dexhand/sensors',
            self.listener_callback,
            10
        )
        self.get_logger().info("FastAPI ROS2 Bridge Started. Listening on /dexhand/sensors")

    def listener_callback(self, msg):
        global latest_robot_state
        # 将接收到的 ROS 消息更新到全局变量
        if len(msg.data) >= 3:
            latest_robot_state["fingers"] = [
                round(msg.data[0], 2),
                round(msg.data[1], 2),
                round(msg.data[2], 2)
            ]

# 启动 ROS 节点的辅助函数
def start_ros_node():
    # 检查 rclpy 是否已经初始化，避免多次初始化报错
    if not rclpy.ok():
        rclpy.init()
    
    ros_node = RosSubscriber()
    
    # 在独立线程中运行 spin，避免阻塞 FastAPI 主线程
    thread = threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True)
    thread.start()
    
    return ros_node