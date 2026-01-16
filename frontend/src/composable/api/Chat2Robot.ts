import request from '@/composable/utils/request'
import type {RobotStatusData} from "@/composable/interfaces/Inter2Robot.ts";

// 获取机器人实时状态
export function getRobotStatus() {
    return request.get<any, RobotStatusData>('/robot/status')
}