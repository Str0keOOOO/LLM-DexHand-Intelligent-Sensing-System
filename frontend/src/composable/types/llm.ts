// 基础
export interface SuccessResponse {
    success: boolean;
}

// 大模型聊天
export interface ModelListResponse {
    success: boolean
    data: {
        model: Array<{
            label: string
            value: string
        }>
        timestamp: number
    }
}

export interface ModelChatResponse {
    success: boolean
    data: {
        reply: string
        model_name: string
        timestamp: number
    }
}

export type ConnStatus = 'init' | 'checking' | 'success' | 'fail'

export interface ModelOption {
    label: string
    value: string
}

export interface ChatMsg {
    role: 'system' | 'user' | string
    content: string
    model?: string
}

export interface ControlRespPayload {
    sent_to?: string
    sentTo?: string
    result?: {
        sent_to?: string
        [key: string]: any
    }

    [key: string]: any
}
