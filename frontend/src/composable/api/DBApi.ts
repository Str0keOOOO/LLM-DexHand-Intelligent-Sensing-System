import request from '@/composable/utils/request'

export interface ChatLogItem {
    id: number
    role: string
    content: string
    model: string | null
    created_at: string
}

export interface SensorItem {
    time: string
    field: string
    value: number
}

// 获取聊天记录
export const getChatHistory = (limit: number = 50, skip: number = 0) => {
    return request<any, { data: ChatLogItem[] }>({
        url: '/data_base/chat_history',
        method: 'get',
        params: { limit, skip }
    })
}

// 获取传感器历史数据
export const getSensorHistory = (minutes: number = 1, hand: string = 'right') => {
    return request<any, { data: SensorItem[] }>({
        url: '/data_base/sensor_history',
        method: 'get',
        params: { minutes, hand }
    })
}