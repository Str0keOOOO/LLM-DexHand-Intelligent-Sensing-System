export interface ConnectOptions {
    path?: string;
    onOpen?: (ev: Event) => void;
    onClose?: (ev: CloseEvent) => void;
    onError?: (ev: Event) => void;
}

export interface RosHealth {
    status: 'ok';
    ros_bridge?: 'connected' | 'initializing';
}

// frontend/src/composable/interfaces/Inter2Robot.ts

export interface JointMap {
    [key: string]: number;
}

export interface HandData {
    joints: JointMap;
    touch: number[]; // 40个浮点数: 5手指 * 8数据
    motor: number[]; // 84个浮点数: 12电机 * 7数据
}

export interface RobotState {
    left: HandData;
    right: HandData;
    timestamp: number;
}

// 触觉数据解析后的辅助接口（方便画图）
export interface ParsedTouchData {
    finger: string;
    normalForce: number;
    temperature: number;
}