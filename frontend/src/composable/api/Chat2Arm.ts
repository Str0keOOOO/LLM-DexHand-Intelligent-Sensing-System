import request from '@/composable/utils/request'
import type {ArmJointPositions} from '@/composable/types/robot'

// =========================
// Arm (机械臂) - Bridge API
// =========================

/** 后端返回通常为透传结构，这里保持宽松类型，避免阻塞 UI */
export type ApiAnyResponse = Record<string, unknown>

export type JogAxis = 'x' | 'y' | 'z' | 'Rx' | 'Ry' | 'Rz'
export type JogDirection = 'positive' | 'negative'

export interface StartJogRequest {
    /** 参考系：例如 4 */
    ref: number
    /** x, y, z, Rx, Ry, Rz */
    axis: JogAxis
    /** positive 或 negative */
    direction: JogDirection
    vel: number
    acc: number
    max_dist: number
}

/** 连接机械臂 */
export function connectArm() {
    // 后端 route: POST /connect -> _request('POST', '/api/connect')
    return request.post<ApiAnyResponse>('/robotic_arm/connect')
}

/** 获取当前关节角度（degree） */
export function getActualJointPosDegree() {
    return request.get<ArmJointPositions | ApiAnyResponse>('/robotic_arm/get-actual-joint-pos-degree')
}

/**
 * 点动（Jog）控制
 *
 * 注意：后端会把 axis/direction 映射为 axis_id/0|1。
 */
export function startJog(data: StartJogRequest) {
    // 后端 route: POST /start-jog -> _request('POST', '/api/start-jog')
    return request.post<ApiAnyResponse>('/robotic_arm/start-jog', data)
}

