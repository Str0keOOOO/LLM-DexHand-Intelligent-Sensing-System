import request from '@/composable/utils/request'
import type {
    SuccessResponse, ModelListResponse, ModelChatResponse
} from '@/composable/types/llm'

export function checkModelConnect(model: string) {
    const data = {model: model}
    return request.post<SuccessResponse>('/llm/check', data)
}


export function listModels() {
    return request.get<ModelListResponse>('/llm/models')
}


export function sendChatMsg(msg: string, model: string) {
    const data = {
        message: msg,
        model: model
    }
    return request.post<ModelChatResponse>('/llm/send', data)
}