import request from '@/composable/utils/request'
import type {
    ChatRequest,
    ChatResponse,
    CheckModelRequest,
    CheckModelResponse,
    ModelListResponse
} from '@/composable/interfaces/Inter2LLM.ts'

enum API {
    SEND_CHAT = '/chat/send',
    CHECK_MODEL = '/chat/check',
}

/**
 * 获取后端支持的模型列表
 */
export const getModels = () => {
    return request.get<ModelListResponse>('/chat/models')
}

/**
 * 检测某个模型是否能正常连接
 */
export const checkModelConnect = (modelName: string) => {
    const data: CheckModelRequest = { model: modelName }
    return request.post<CheckModelResponse>(API.CHECK_MODEL, data)
}

/**
 * 发送聊天消息
 */
export const sendChatMsg = (msg: string, model: string = "gpt-3.5-turbo") => {
    const data: ChatRequest = {
        message: msg,
        model: model
    }
    return request.post<ChatResponse>(API.SEND_CHAT, data)
}