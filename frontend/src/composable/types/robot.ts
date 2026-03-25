// 基础
export interface SuccessResponse {
    success: boolean;
}

// 灵巧手
export interface HandCommand {
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

export interface HandState {
    joint: {
        position: Record<string, number>;
        velocity: Record<string, number>;
    };
    touch: {
        normal_force: number[];
        normal_force_delta: number[];
        tangential_force: number[];
        tangential_force_delta: number[];
        direction: number[];
        proximity: number[];
        temperature: number[];
    };
    motor: {
        angle: number[];
        encoder_position: number[];
        current: number[];
        velocity: number[];
        error_code: number[];
        impedance: number[];
    };
    timestamp: number;
}


// 机械臂
export interface ArmCommand {
    nb: 'x' | 'y' | 'z' | 'rx' | 'ry' | 'rz'
    dir: 'positive' | 'negative'
    vel: number
    acc: number
    max_dis: number
}

export interface ArmState {
    x: number;
    y: number;
    z: number;
    rx: number;
    ry: number;
    rz: number;
    timestamp: number;
}