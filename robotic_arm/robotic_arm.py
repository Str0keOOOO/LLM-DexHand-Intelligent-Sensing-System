import Robot


class RobotRPC:
    def __init__(self, robot_address: str):
        self.robot_address = robot_address
        self.client = None

    def connect(self):
        """连接机器人（带缓存）"""
        if self.client is not None:
            return self.client
        try:
            self.client = Robot.RPC(self.robot_address)
            print(f"Connected to 机器人, Address {self.robot_address}")
            # print(self.client)
            # TODO 这里没有连上需要验证
            return self.client
        except Exception as e:
            print(f"连接失败: {str(e)}")
            self.client = None
            return None

    def call_track_script(self, n: int):
        """调用第n段轨迹"""
        client = self.connect()
        if client is None:
            return None

        path = f"/usr/local/etc/controller/lua/UAV0{n}.lua"
        client.SetSysVarValue(4, 0)
        client.Mode(0)  # 机器人切入自动运行模式
        client.ProgramLoad(path)
        code = client.ProgramRun()
        return code

    def init_robot(self):
        """初始化：打开夹爪（你原逻辑就是这样）"""
        client = self.connect()
        if client is None:
            return False
        self.open_gripper()
        return True

    def check_program_state(self):
        """检测机器人程序运行状态"""
        client = self.connect()
        if client is None:
            return None

        state = client.GetProgramState()  # 1停止/无程序，2运行中，3暂停
        if state == 2:
            print("机器人程序运行中")
        elif state == 1:
            print("程序停止或无程序运行")
        elif state == 3:
            print("机器人程序已暂停")
        else:
            print("机器人程序状态未知")
        return state

    def get_current_line(self):
        """机器人当前程序行号"""
        client = self.connect()
        if client is None:
            return None
        res = client.GetCurrentLine()
        # 兼容：有的SDK返回int，有的可能返回(错误码, 行号)
        if isinstance(res, (tuple, list)) and len(res) >= 2:
            return res[1]
        return res

    def open_gripper(self):
        """通知PLC打开夹爪（松开）- 实际是写机器人系统变量"""
        client = self.connect()
        if client is None:
            return False
        client.SetSysVarValue(2, 1.0)
        return True

    def close_gripper(self):
        """通知PLC关闭夹爪（夹紧）"""
        client = self.connect()
        if client is None:
            return False
        client.SetSysVarValue(3, 1.0)
        return True

    def get_actual_joint_pos_degree(self):
        """获取机器人角度"""
        client = self.connect()
        if client is None:
            return None
        return client.GetActualJointPosDegree()

    def get_sys_var_value(self, idx: int):
        client = self.connect()
        if client is None:
            return None
        return client.GetSysVarValue(idx)
