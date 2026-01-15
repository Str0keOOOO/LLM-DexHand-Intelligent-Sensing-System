import request from '@/composable/utils/request'
export interface RobotStatusData {
    timestamp: string
    fingers: number[] // [食指, 中指, 拇指]
}

// 获取机器人实时状态
export function getRobotStatus() {
    return request.get<any, RobotStatusData>('/robot/status')
}