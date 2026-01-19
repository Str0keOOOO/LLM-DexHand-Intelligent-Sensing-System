import {ref} from 'vue'
import type {Ref} from 'vue'
import {sendChatMsg} from '@/composable/api/Chat2LLM'
import type {ChatMsg} from "@/composable/interfaces/Inter2LLM.ts";

export function useSendCommand(params: {
    chatHistory: Ref<ChatMsg[]>
    inputCommand: Ref<string>
    selectedModel: Ref<any>
    connStatus: Ref<'init' | 'checking' | 'success' | 'fail'>
    isSending?: Ref<boolean>
}) {
    const {chatHistory, inputCommand, selectedModel, connStatus} = params
    const internalSending = params.isSending ?? ref(false)
    const isSending = internalSending

    const normalize = (res: any) => {
        // 兼容 AxiosResponse 或直接返回的数据
        return res?.data ?? res ?? {}
    }

    const sendCommand = async () => {
        if (!inputCommand.value) return

        const modelToUse = (typeof selectedModel.value === 'object' && selectedModel.value !== null)
            ? (selectedModel.value as any).value
            : selectedModel.value

        if (connStatus.value === 'fail') {
            chatHistory.value.push({
                role: 'system',
                content: `❌ 发送失败：模型 [${modelToUse}] 未连接成功。`
            })
            return
        }

        chatHistory.value.push({role: 'user', content: inputCommand.value})
        const textToSend = inputCommand.value

        inputCommand.value = ''
        isSending.value = true

        try {
            const res = await sendChatMsg(textToSend, modelToUse)
            const payload = normalize(res)

            const reply = payload.reply ?? payload.data?.reply ?? String(payload.reply ?? '')
            const modelName = payload.model_name ?? payload.model ?? ''
            const actionCode = payload.action_code ?? payload.actionCode ?? null

            chatHistory.value.push({
                role: 'system',
                content: reply,
                model: modelName
            })

            if (actionCode) {
                chatHistory.value.push({
                    role: 'system',
                    content: `[执行层] ${actionCode}`,
                    model: 'Robot Core'
                })
            }
        } catch (error) {
            chatHistory.value.push({role: 'system', content: '❌ 错误：请求超时或服务异常'})
        } finally {
            isSending.value = false
        }
    }

    return {
        sendCommand,
        isSending
    }
}