interface FingerData {
    index: number;  // 食指
    middle: number; // 中指
    thumb: number;  // 拇指
}

// 获取机器人状态数据接口
export interface RobotStatusData {
    timestamp: string;
    fingers: FingerData;
}