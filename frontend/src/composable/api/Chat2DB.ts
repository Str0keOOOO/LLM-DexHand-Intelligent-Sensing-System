import request from '@/composable/utils/request'
import type {MeasurementType} from "@/composable/types/robot";
import type {ChatLogItem} from "@/composable/types/llm";

export function getChatHistory(limit: number = 50, skip: number = 0) {
    return request<any, { data: ChatLogItem[] }>({
        url: '/data_base/chat_history',
        method: 'get',
        params: {limit, skip}
    })
}


export function getSensorHistory(
    measurement: MeasurementType = 'dexhand_joints',
    minutes: number = 1,
    hand: string = 'right'
) {
    return request<any, { data: any[] }>({
        url: '/data_base/sensor_history',
        method: 'get',
        params: {measurement, minutes, hand}
    })
}