/**
 * 灵巧手 / 机械臂 相关类型定义
 *
 * 说明：字段多数为 snake_case，用于和后端/ROS 消息保持一致。
 */

// =========================
// DexHand (灵巧手) 基础类型
// =========================

/**
 * 灵巧手端侧定义
 */
export type HandSide = 'right'

/**
 * 具体的关节 UI 绑定结构 (用于手动控制面板 Manual Control)
 */
export interface JointsState {
    th_dip: number
    th_mcp: number
    th_rot: number
    ff_spr: number
    ff_dip: number
    ff_mcp: number
    mf_dip: number
    mf_mcp: number
    rf_dip: number
    rf_mcp: number
    lf_dip: number
    lf_mcp: number
}

/**
 * 12 个语义化关节名
 */
export type JointName = keyof JointsState

export type JointMap = Record<JointName, number>

export interface JointData {
    position: JointMap
    velocity: JointMap
}

/**
 * 结构化的触觉传感器数据 (5 根手指)
 */
export interface TouchData {
    normal_force: number[]
    normal_force_delta: number[]
    tangential_force: number[]
    tangential_force_delta: number[]
    direction: number[]
    proximity: number[]
    temperature: number[]
}

/**
 * 结构化的电机底层反馈数据 (12 个电机)
 */
export interface MotorData {
    angle: number[]
    encoder_position: number[]
    current: number[]
    velocity: number[]
    /** 错误代码 (0 为正常) */
    error_code: number[]
    impedance: number[]
}

/**
 * 灵巧手完整实时状态
 */
export interface HandData {
    joint: JointData
    touch: TouchData
    motor: MotorData
}

/**
 * 全局机器人状态 (WebSocket 传输主体)
 */
export interface RobotState {
    right: HandData
    timestamp: number
}

// =========================
// 历史数据 & 数据库查询
// =========================

/**
 * 传感器历史记录（原始结构化数据）
 *
 * 后端不同 measurement 类型的 data 结构不同：
 * - dexhand_joints / dexhand_velocities: Record<string, number>
 * - dexhand_touch: TouchData
 * - dexhand_motor: MotorData
 */
export interface HistoricalSensorData<TData = unknown> {
    time: string
    data: TData
}

export type MeasurementType =
    | 'dexhand_joints'
    | 'dexhand_velocities'
    | 'dexhand_touch'
    | 'dexhand_motor'

/**
 * 历史图表数据 (用于趋势分析)
 */
export interface HistoryData {
    time: string[]
    rightForce: number[]
}

/**
 * 数据库查询记录项
 */
export interface SensorItem {
    time: string
    field: string
    value: number
}

// =========================
// 控制指令相关
// =========================

export interface ControlForm {
    hand: HandSide
    joints: JointsState
}

export interface ControlCommand {
    hand: HandSide
    joints: JointMap
}

// =========================
// 机械臂 (Arm) API DTO
// =========================

export interface TrackScriptRequest {
    script_name: string
    parameters?: Record<string, unknown>
}

export interface SysVarRequest {
    var_name: string
}

export interface ArmJointPositions {
    joints: number[]
}

export interface ProgramState {
    is_running: boolean
    mode: string
}