import request from '@/composable/utils/request'
import type {SensorItem} from "@/composable/types/robot";
import type {ChatLogItem} from "@/composable/types/llm";


export function getChatHistory(limit: number = 50, skip: number = 0) {
    return request<any, { data: ChatLogItem[] }>({
        url: '/data_base/chat_history',
        method: 'get',
        params: {limit, skip}
    })
}

export function getSensorHistory(minutes: number = 1, hand: string = 'right') {
    return request<any, { data: SensorItem[] }>({
        url: '/data_base/sensor_history',
        method: 'get',
        params: {minutes, hand}
    })
}