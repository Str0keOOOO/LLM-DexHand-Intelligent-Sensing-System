export interface ChatHistoryResponse {
    success: boolean
    data: Array<{
        id: number
        role: 'user' | 'assistant' | 'system' | (string & {})
        content: string
        model: string
        timestamp: number
    }>
}

export interface SensorHistoryResponse {
    success: boolean
    data: Array<{
        target: string
        data: {
            j_1: number
            j_2: number
            j_3: number
            j_4: number
            j_5: number
            j_6: number
        }
        /** Unix 时间戳（秒） */
        timestamp: number
    }>
}
