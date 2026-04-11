import request from '@/composable/utils/request'
import type {ChatHistoryResponse, SensorHistoryResponse} from "@/composable/types/db.ts";

export function getChatHistory(limit: number = 20, skip: number = 0) {
    return request.get<ChatHistoryResponse>('/db/chat_history', {
        params: {limit, skip}
    })
}


export function getSensorHistory(target: string, minutes: number) {
    return request.get<SensorHistoryResponse>('/db/sensor_history', {
        params: {target, minutes}
    })
}