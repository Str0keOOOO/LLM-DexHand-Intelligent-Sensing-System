#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import random
import time

class VirtualSensorPublisher(Node):
    def __init__(self):
        super().__init__('virtual_sensor_pub')
        # 创建发布者，话题为 /dexhand/sensors
        self.publisher_ = self.create_publisher(Float32MultiArray, '/dexhand/sensors', 10)
        self.timer = self.create_timer(0.1, self.publish_data) # 10Hz 发布频率
        self.get_logger().info("虚拟传感器已启动，正在发布模拟力数据...")

    def publish_data(self):
        msg = Float32MultiArray()
        # 模拟生成三个手指的压力数据 [食指, 中指, 拇指]
        # 添加一些正弦波动模拟真实呼吸感
        t = time.time()
        f1 = 5.0 + 2.0 * random.random()  # 食指 5-7N
        f2 = 3.0 + 1.5 * random.random()  # 中指 3-4.5N
        f3 = 1.0 + 3.0 * random.random()  # 拇指 1-4N
        
        msg.data = [f1, f2, f3]
        self.publisher_.publish(msg)
        # self.get_logger().info(f'Published: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = VirtualSensorPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()