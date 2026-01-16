// 定义接口返回的数据类型 (对应后端的 ChatResponse)
export interface ChatResponseData {
    reply: string
    action_code?: string
}

// 定义发送消息的参数类型 (对应后端的 ChatRequest)
export interface SendMessageParams {
    message: string
}