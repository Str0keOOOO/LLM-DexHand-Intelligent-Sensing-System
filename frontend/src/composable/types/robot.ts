export type HandSide = 'left' | 'right'

// 具体的关节结构，用于 UI 绑定 (Manual Control)
export interface JointsState {
    th_dip: number;
    th_mcp: number;
    th_rot: number
    ff_spr: number;
    ff_dip: number;
    ff_mcp: number
    mf_dip: number;
    mf_mcp: number
    rf_dip: number;
    rf_mcp: number
    lf_dip: number;
    lf_mcp: number
}

// 通用关节映射，用于 API 传输
export type JointMap = Record<string, number>

// 机器人实时状态
export interface HandData {
    joints: JointMap
    touch: number[]
    motor: number[]
}

export interface RobotState {
    left: HandData
    right: HandData
    timestamp: number
}

// 历史图表数据
export interface HistoryData {
    time: string[]
    leftForce: number[]
    rightForce: number[]
}

// 传感器历史记录项
export interface SensorItem {
    time: string
    field: string
    value: number
}

// --- 控制相关 ---

export interface ControlForm {
    hand: HandSide
    joints: JointsState
}

export interface ControlCommand {
    hand: HandSide
    joints: JointMap
}

// --- 机械臂 (Arm) 相关控制类型 ---

export interface TrackScriptRequest {
    script_name: string;
    parameters?: Record<string, any>;
}

export interface SysVarRequest {
    var_name: string;
}

export interface ArmJointPositions {
    joints: number[]; // 假设返回的是关节角度数组
}

// --- 机械臂 (Arm) 控制相关类型 ---
export interface TrackScriptRequest {
    script_name: string;
    parameters?: Record<string, any>;
}

export interface SysVarRequest {
    var_name: string;
}

export interface ArmJointPositions {
    joints: number[]; // 假设后端返回关节角度数组
}

export interface ProgramState {
    is_running: boolean;
    mode: string;
}