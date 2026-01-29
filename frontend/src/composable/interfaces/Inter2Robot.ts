export interface JointMap {
    [key: string]: number;
}

export interface HandData {
    joints: JointMap;
    touch: number[];
    motor: number[];
}

export interface RobotState {
    left: HandData;
    right: HandData;
    timestamp: number;
}

export interface HistoryData {
    time: string[];
    leftForce: number[];
    rightForce: number[];
}

export interface ControlCommand {
    hand: 'left' | 'right';
    joints: Record<string, number>;
}