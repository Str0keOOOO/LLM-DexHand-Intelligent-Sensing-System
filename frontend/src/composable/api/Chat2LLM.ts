import request from '@/composable/utils/request'
import type {
    ChatRequest,
    ChatResponse,
    CheckModelRequest,
    CheckModelResponse,
    ModelListResponse
} from '@/composable/types/llm'

export function getModels() {
    return request.get<ModelListResponse>('/chat/models')
}

export function checkModelConnect(modelName: string) {
    const data: CheckModelRequest = {model: modelName}
    return request.post<CheckModelResponse>('/chat/check', data)
}

export function sendChatMsg(msg: string, model: string) {
    const data: ChatRequest = {
        message: msg,
        model: model
    }
    return request.post<ChatResponse>('/chat/send', data)
}