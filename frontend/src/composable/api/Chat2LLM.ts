import request from "@/composable/utils/request.ts";
import type { SendMessageParams,ChatResponseData } from "@/composable/interfaces/Inter2Back";

// 发送自然语言指令的接口
export function sendChatCommand(data: SendMessageParams) {
    return request.post<any, ChatResponseData>('/chat/send', data)
}