/**
 * 灵巧手端侧定义
 */
export type HandSide = 'right'

/**
 * 12个语义化关节映射 (Key-Value)
 * 用于：
 * 1. 关节位置 (Positions/Joints)
 * 2. 关节速度 (Velocities)
 * 3. 关节力度/电流 (Efforts)
 */
export type JointMap = Record<string, number>

/**
 * 具体的关节 UI 绑定结构 (用于手动控制面板 Manual Control)
 */
export interface JointsState {
    th_dip: number;
    th_mcp: number;
    th_rot: number;
    ff_spr: number;
    ff_dip: number;
    ff_mcp: number;
    mf_dip: number;
    mf_mcp: number;
    rf_dip: number;
    rf_mcp: number;
    lf_dip: number;
    lf_mcp: number;
}

/**
 * 结构化的触觉传感器数据 (5根手指)
 */
export interface TouchData {
    normal_force: number[];           // 法向力
    normal_force_delta: number[];     // 法向力变化量
    tangential_force: number[];       // 切向力
    tangential_force_delta: number[]; // 切向力变化量
    direction: number[];              // 方向 (0-359°)
    proximity: number[];              // 接近度
    temperature: number[];            // 温度
}

/**
 * 结构化的电机底层反馈数据 (12个电机)
 */
export interface MotorData {
    angle: number[];            // 角度
    encoder_position: number[]; // 编码器位置
    current: number[];          // 电流
    velocity: number[];         // 速度
    error_code: number[];       // 错误代码 (0为正常)
    impedance: number[];        // 阻抗
}

/**
 * 灵巧手完整实时状态
 */
export interface HandData {
    joints: JointMap;      // 12个语义关节位置 (Deg)
    velocities: JointMap;  // 12个语义关节速度 (Deg/s)
    touch: TouchData;      // 7个维度的触觉数据
    motor: MotorData;      // 6个维度的电机原始反馈
}

/**
 * 全局机器人状态 (WebSocket 传输主体)
 */
export interface RobotState {
    right: HandData;
    timestamp: number;
}

export interface HistoricalSensorData {
    time: string;
    data: any; // 根据 measurement 类型不同，可能是 Record<string, number> 或结构化的数组对象
}

export type MeasurementType = 'dexhand_joints' | 'dexhand_velocities' | 'dexhand_touch' | 'dexhand_motor';


/**
 * 历史图表数据 (用于趋势分析)
 */
export interface HistoryData {
    time: string[];
    rightForce: number[];
}

/**
 * 数据库查询记录项
 */
export interface SensorItem {
    time: string;
    field: string;
    value: number;
}

// --- 控制指令相关 ---

export interface ControlForm {
    hand: HandSide;
    joints: JointsState;
}

export interface ControlCommand {
    hand: HandSide;
    joints: JointMap;
}

// --- 机械臂 (Arm) 控制相关 ---

export interface TrackScriptRequest {
    script_name: string;
    parameters?: Record<string, any>;
}

export interface SysVarRequest {
    var_name: string;
}

export interface ArmJointPositions {
    joints: number[];
}

export interface ProgramState {
    is_running: boolean;
    mode: string;
}