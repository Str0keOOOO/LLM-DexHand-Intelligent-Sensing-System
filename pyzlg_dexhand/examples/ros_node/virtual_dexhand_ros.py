#!/usr/bin/env python3
from ros_compat import ROSNode
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
from std_srvs.srv import Trigger
from sensor_msgs.msg import JointState
import numpy as np
from enum import Enum
import time
import yaml
import os.path
import random
import math
from types import SimpleNamespace
from filters import DampedVelocityKalmanFilter

# Joint names (Order is important for motor feedback)
joint_names = [
    "th_dip", "th_mcp", "th_rot",
    "ff_spr", "ff_dip", "ff_mcp",
    "mf_dip", "mf_mcp",
    "rf_dip", "rf_mcp",
    "lf_dip", "lf_mcp",
]

class HardwareMapping(Enum):
    th_dip = ("th_dip", ["f_joint1_3", "f_joint1_4"])
    th_mcp = ("th_mcp", ["f_joint1_2"])
    th_rot = ("th_rot", ["f_joint1_1"])
    ff_spr = ("ff_spr", ["f_joint2_1", "f_joint4_1", "f_joint5_1"])
    ff_dip = ("ff_dip", ["f_joint2_3", "f_joint2_4"])
    ff_mcp = ("ff_mcp", ["f_joint2_2"])
    mf_dip = ("mf_dip", ["f_joint3_3", "f_joint3_4"])
    mf_mcp = ("mf_mcp", ["f_joint3_2"])
    rf_dip = ("rf_dip", ["f_joint4_3", "f_joint4_4"])
    rc_mcp = ("rf_mcp", ["f_joint4_2"])
    lf_dip = ("lf_dip", ["f_joint5_3", "f_joint5_4"])
    lf_mcp = ("lf_mcp", ["f_joint5_2"])

class JointMapping:
    def __init__(self, prefix: str = "l"):
        self.prefix = prefix
        self.urdf_to_hw = {}
        for hw_mapping in HardwareMapping:
            dex_joint, urdf_joints = hw_mapping.value
            for urdf_joint in urdf_joints:
                full_name = f"{prefix}_{urdf_joint}"
                self.urdf_to_hw[full_name] = dex_joint
        self.joint_names = sorted(list(self.urdf_to_hw.keys()))

    def map_command(self, joint_values):
        hw_joint_values = {dex_joint: [] for dex_joint in joint_names}
        for name, value in joint_values.items():
            if name in self.urdf_to_hw:
                dex_joint = self.urdf_to_hw[name]
                hw_joint_values[dex_joint].append(value)
        command = {}
        for dex_joint in joint_names:
            values = hw_joint_values[dex_joint]
            if values:
                command[dex_joint] = float(np.rad2deg(sum(values) / len(values)))
        return command

    def map_feedback(self, hardware_values):
        joint_state_dict = {}
        for urdf_joint in self.joint_names:
            dex_joint = self.urdf_to_hw.get(urdf_joint)
            if dex_joint in hardware_values:
                joint_state_dict[urdf_joint] = float(np.deg2rad(hardware_values[dex_joint]))
        return joint_state_dict

class DexHandNode(ROSNode):
    def __init__(self, config: dict):
        super().__init__("dexhand")
        self.hands_list = config.get("hands", ["right"])
        send_rate = config.get("rate", 100.0)
        self.start_time = time.time()
        
        # 手动控制状态
        self.manual_override_until = 0.0
        self.last_manual_positions = {}
        for hand in self.hands_list:
            self.last_manual_positions[hand] = {j: 0.0 for j in joint_names}

        self.topic_config = {
            "left": {
                "command": "/left_hand/joint_commands",
                "joint_feedback": "/left_hand/joint_states",
                "touch_feedback": "/left_hand/touch_sensors",
                "motor_feedback": "/left_hand/motor_feedback"
            },
            "right": {
                "command": "/right_hand/joint_commands",
                "joint_feedback": "/right_hand/joint_states",
                "touch_feedback": "/right_hand/touch_sensors",
                "motor_feedback": "/right_hand/motor_feedback"
            }
        }
        
        self.joint_mappings = {}
        self.simulated_state = {}
        self.fingertip_mapping = {"th": 0, "ff": 1, "mf": 2, "rf": 3, "lf": 4}
        self.kalman_filters = {} # Dummy holder
        
        self.joint_state_pubs = {}
        self.touch_sensor_pubs = {}
        self.motor_feedback_pubs = {}

        for hand in self.hands_list:
            self.simulated_state[hand] = {joint: 0.0 for joint in joint_names}
            self.joint_mappings[hand] = JointMapping("l" if hand == "left" else "r")
            
            self.create_subscription(JointState, self.topic_config[hand]["command"], lambda msg, h=hand: self.command_callback(msg, h))
            self.joint_state_pubs[hand] = self.create_publisher(JointState, self.topic_config[hand]["joint_feedback"], 10)
            self.touch_sensor_pubs[hand] = self.create_publisher(Float64MultiArray, self.topic_config[hand]["touch_feedback"], 10)
            self.motor_feedback_pubs[hand] = self.create_publisher(Float64MultiArray, self.topic_config[hand]["motor_feedback"], 10)

        self.create_service(Trigger, "/reset_hands", self.reset_callback)
        self.timer = self.create_timer(1.0/send_rate, self.send_commands)
        self.logger.info("DexHand Asymmetric Demo Node Started")

    def command_callback(self, msg: JointState, hand: str):
        try:
            self.manual_override_until = time.time() + 5.0
            mapping = self.joint_mappings[hand]
            joint_values_urdf = {n: p for n, p in zip(msg.name, msg.position) if not math.isnan(p)}
            hw_commands = mapping.map_command(joint_values_urdf)
            for joint, val in hw_commands.items():
                if joint in self.last_manual_positions[hand]:
                    self.last_manual_positions[hand][joint] = val
        except Exception: pass

    def send_commands(self):
        for hand in self.hands_list:
            self.process_and_publish_feedback(hand)

    def process_and_publish_feedback(self, hand: str):
        try:
            is_manual = time.time() < self.manual_override_until
            
            if is_manual:
                target_state = self.last_manual_positions[hand]
                for k, v in target_state.items():
                    self.simulated_state[hand][k] = v
            else:
                # --- 核心修改：左右手使用不同的波形 ---
                t = time.time() - self.start_time
                
                if hand == 'left':
                    # 左手：快一点的节奏，相位 0
                    cycle_angle = (math.sin(t * 2.5) + 1) / 2
                    cycle_force = (math.sin(t * 1.5) + 1) / 2 # 力和角度节奏不同步
                    base_angle = 10.0 + 80.0 * cycle_angle # 10~90度
                    base_force = 0.5 + 3.0 * cycle_force   # 0.5~3.5N
                else:
                    # 右手：慢一点，且有相位差 (PI)
                    cycle_angle = (math.sin(t * 2.0 + math.pi) + 1) / 2
                    cycle_force = (math.cos(t * 2.0) + 1) / 2
                    base_angle = 20.0 + 60.0 * cycle_angle # 20~80度 (幅度略小)
                    base_force = 1.0 + 4.0 * cycle_force   # 1.0~5.0N (力更大)

                for k in self.simulated_state[hand]:
                    # 给每个关节加一点独立的随机偏移，看起来不那么机械
                    self.simulated_state[hand][k] = base_angle + random.uniform(-2, 2)

            # --- 构建数据 ---
            feedback = SimpleNamespace(joints={}, touch={})
            current_state = self.simulated_state[hand]
            
            # 计算平均角度用于估算力（如果不是手动模式，上面已经算了 base_force，这里做个兼容）
            avg_angle = sum(current_state.values()) / len(current_state)
            # 如果是手动模式，力根据角度算；自动模式用上面的 base_force
            if is_manual:
                force_val = (avg_angle / 90.0) * 3.0
            else:
                force_val = base_force if 'base_force' in locals() else 0.0

            # 填充 Joints
            for joint in joint_names:
                ang = current_state.get(joint, 0.0)
                jd = SimpleNamespace()
                jd.angle = ang 
                jd.encoder_position = int(ang * 10) + 2048
                jd.current = 50 + ang + random.uniform(-2, 2)
                jd.velocity = 0.0; jd.error_code = 0; jd.impedance = 1.0
                feedback.joints[joint] = jd

            # 填充 Touch
            for f in ["th", "ff", "mf", "rf", "lf"]:
                td = SimpleNamespace()
                td.timestamp = time.time_ns()
                td.normal_force = max(0, force_val + random.uniform(-0.2, 0.2))
                td.direction = -1; td.temperature = 25.0 + random.uniform(-0.5, 0.5)
                feedback.touch[f] = td

            # --- 发布消息 ---
            # 1. JointState (URDF names)
            pos_dict = {k: v.angle for k, v in feedback.joints.items()}
            pos_dict_urdf = self.joint_mappings[hand].map_feedback(pos_dict)
            js = JointState()
            js.header.stamp = self.get_ros_time().to_msg()
            js.name = self.joint_mappings[hand].joint_names
            js.position = [pos_dict_urdf.get(j, 0.0) for j in js.name]
            js.velocity = [0.0] * len(js.name)
            self.joint_state_pubs[hand].publish(js)

            # 2. Touch Array
            touch_msg = Float64MultiArray()
            touch_msg.data = [0.0] * 40
            for fname, tdata in feedback.touch.items():
                if fname in self.fingertip_mapping:
                    idx = self.fingertip_mapping[fname]
                    touch_msg.data[idx*8] = tdata.timestamp/1e9
                    touch_msg.data[idx*8+1] = tdata.normal_force
                    touch_msg.data[idx*8+7] = tdata.temperature
            self.touch_sensor_pubs[hand].publish(touch_msg)

            # 3. Motor Array (Fixed Order -> RobotChart Uses This!)
            motor_msg = Float64MultiArray()
            motor_msg.data = [0.0] * 84
            cur_time = time.time()
            for idx, name in enumerate(joint_names):
                jd = feedback.joints[name]
                base = idx * 7
                motor_msg.data[base] = cur_time
                motor_msg.data[base+1] = jd.angle # Angle in Degrees
                motor_msg.data[base+3] = jd.current
            self.motor_feedback_pubs[hand].publish(motor_msg)

        except Exception: pass

    def reset_callback(self, req, res):
        res.success = True; res.message = "Reset"; return res
    def on_shutdown(self): pass

if __name__ == "__main__":
    import os
    config_path = os.path.join(os.path.dirname(__file__), "../../config", "config.yaml")
    try:
        with open(config_path) as f: config = yaml.safe_load(f)
        DexHandNode(config["DexHand"]["ROS_Node"]).spin()
    except Exception as e: print(e)