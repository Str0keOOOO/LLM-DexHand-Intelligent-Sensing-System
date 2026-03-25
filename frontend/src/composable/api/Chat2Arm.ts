import request from '@/composable/utils/request'
import type {ArmCommand, SuccessResponse} from '@/composable/types/robot'

export function resetArm() {
    return request.post<SuccessResponse>('/arm/reset')
}

export function checkArm() {
    return request.post<SuccessResponse>('/arm/check')
}

export function moveArm(data: ArmCommand) {
    return request.post<SuccessResponse>('/arm/move', data)
}

