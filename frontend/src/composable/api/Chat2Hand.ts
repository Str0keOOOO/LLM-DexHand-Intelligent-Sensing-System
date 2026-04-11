import request from '@/composable/utils/request'
import type {HandCommand, SuccessResponse} from '@/composable/types/robot';


export function resetHand() {
    return request.get<SuccessResponse, SuccessResponse>('/hand/reset')
}

export function checkHand() {
    return request.get<SuccessResponse, SuccessResponse>('/hand/check')
}

export function moveHand(data: HandCommand) {
    return request.post<SuccessResponse>('/hand/move', data)
}

