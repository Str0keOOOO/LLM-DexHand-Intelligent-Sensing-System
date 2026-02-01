// 通用基础类型
export type Role = 'user' | 'system' | string
export type ConnStatus = 'init' | 'checking' | 'success' | 'fail'

// 模型选项
export interface ModelOption {
    label: string
    value: string
}

// 聊天消息实体
export interface ChatMsg {
    role: Role
    content: string
    model?: string
}

// 数据库日志实体
export interface ChatLogItem {
    id: number
    role: string
    content: string
    model: string | null
    created_at: string
}

// --- API 请求/响应接口 ---

export interface ChatRequest {
    message: string
    model?: string
}

export interface ChatResponse {
    reply: string
    model_name: string
    action_code?: string | null
    control_command?: any // 兼容后端可能返回的控制指令字段
    controlCommand?: any
}

export interface CheckModelRequest {
    model: string
}

export interface CheckModelResponse {
    success: boolean
    message: string
}

export interface ModelListResponse {
    models: ModelOption[]
}

export interface ControlRespPayload {
    sent_to?: string
    sentTo?: string
    result?: { sent_to?: string }
    message?: string
    error?: string
}