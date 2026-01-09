import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time

# --- 关键：导入本地集成的 DexHand 驱动 ---
# 根据你的目录结构，pyzlg_dexhand 文件夹里还有一个 pyzlg_dexhand 包
try:
    from .pyzlg_dexhand.pyzlg_dexhand.dexhand_interface import DexHand
except ImportError:
    # 备用导入：如果在某些 IDE 环境下路径不同
    from dexhand_driver.pyzlg_dexhand.pyzlg_dexhand.dexhand_interface import DexHand

class DexHandNode(Node):
    def __init__(self):
        super().__init__('dexhand_hardware_node')
        
        # 1. 初始化硬件
        self.get_logger().info('正在尝试连接 DexHand (CAN0)...')
        try:
            # 假设使用默认的 can0 接口，波特率通常在驱动内部配置或系统配置
            self.hand = DexHand(interface='can0') 
            self.device_connected = True
            self.get_logger().info('DexHand 连接成功！')
        except Exception as e:
            self.get_logger().error(f'DexHand 连接失败 (运行在模拟模式): {str(e)}')
            self.device_connected = False

        # 2. 创建发布者：发布灵巧手状态 (JSON 格式方便后端解析)
        self.status_publisher = self.create_publisher(String, '/dexhand/status', 10)
        
        # 3. 创建订阅者：接收后端指令
        self.cmd_subscription = self.create_subscription(
            String,
            '/dexhand/command',
            self.command_callback,
            10
        )

        # 4. 定时器：100Hz (0.01s) 频率读取数据
        self.timer = self.create_timer(0.01, self.timer_callback)

    def timer_callback(self):
        """周期性读取硬件状态并发布"""
        status_data = {}
        
        if self.device_connected:
            try:
                # 调用驱动获取真实状态 (假设驱动有 get_current_state 方法，需根据源码调整)
                # 这里根据 pyzlg_dexhand 的常见接口编写
                # status_data = self.hand.get_state() 
                pass
            except Exception as e:
                self.get_logger().warn(f'读取数据异常: {e}')
        
        # 为了演示和调试，如果没有硬件，我们构造一些模拟数据
        if not status_data:
            import random
            status_data = {
                "timestamp": time.time(),
                "fingers": [
                    random.uniform(0, 10), # 食指力/位置
                    random.uniform(0, 10), # 中指
                    random.uniform(0, 10), # 拇指
                    random.uniform(0, 10)  # 无名指
                ]
            }

        # 发布消息
        msg = String()
        msg.data = json.dumps(status_data)
        self.status_publisher.publish(msg)

    def command_callback(self, msg):
        """处理来自后端的控制指令"""
        try:
            command = json.loads(msg.data)
            self.get_logger().info(f'收到指令: {command}')
            
            if not self.device_connected:
                return

            # 解析动作并调用驱动
            # 示例：{"action": "grasp", "force": 5.0}
            if command.get("action") == "grasp":
                # self.hand.close_fingers(force=command.get("force", 1.0))
                pass
            elif command.get("action") == "open":
                # self.hand.open_fingers()
                pass
            elif command.get("action") == "reset":
                # self.hand.reset()
                pass
                
        except json.JSONDecodeError:
            self.get_logger().error('接收到的指令不是有效的 JSON')

def main(args=None):
    rclpy.init(args=args)
    node = DexHandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()