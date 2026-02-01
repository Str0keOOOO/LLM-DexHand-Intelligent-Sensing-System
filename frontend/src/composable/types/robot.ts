export type HandSide = 'left' | 'right'

// 具体的关节结构，用于 UI 绑定 (Manual Control)
export interface JointsState {
    th_dip: number; th_mcp: number; th_rot: number
    ff_spr: number; ff_dip: number; ff_mcp: number
    mf_dip: number; mf_mcp: number
    rf_dip: number; rf_mcp: number
    lf_dip: number; lf_mcp: number
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