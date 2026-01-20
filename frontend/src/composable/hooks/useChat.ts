import { ref } from 'vue'
import type { ModelOption, ChatMsg, CheckModelResponse } from '@/composable/interfaces/Inter2LLM.ts'
import type { ModelVal, ConnStatus } from '@/composable/types/Type2LLM'
import { getModels, sendChatMsg, checkModelConnect } from '@/composable/api/Chat2LLM'

// TODO 思考是否需要拆分，不拆分的时候要简化一下
export function useChat(defaultModel: ModelOption = { label: '默认模型 (GPT-3.5)', value: 'gpt-3.5-turbo' }) {
    // 聊天相关（提前声明以便 initModels 使用）
    const chatHistory = ref<ChatMsg[]>([])
    const inputCommand = ref<string>('')
    const internalSending = ref<boolean>(false)
    const isSending = internalSending

    // 模型相关
    const modelOptions = ref<ModelOption[]>([])
    const selectedModel = ref<ModelVal>(defaultModel.value)
    const isLoadingModels = ref(false)

    function extractModels(res: any): ModelOption[] {
        const data = res?.data ?? res
        if (Array.isArray(data?.models)) return data.models
        return []
    }

    async function initModels(): Promise<void> {
        isLoadingModels.value = true
        try {
            const res = await getModels()
            const models = extractModels(res)
            if (models.length > 0) {
                modelOptions.value = models
                selectedModel.value = (models[0] as any).value ?? defaultModel.value
            } else {
                modelOptions.value = [defaultModel]
                selectedModel.value = defaultModel.value
            }
        } catch (error) {
            console.error('无法获取模型列表:', error)
            modelOptions.value = [{ label: '连接失败 (默认)', value: defaultModel.value }]
            selectedModel.value = defaultModel.value
        } finally {
            isLoadingModels.value = false
            // 若历史为空，推送开场白（不新增任何控制变量）
            if (chatHistory.value.length === 0) {
                chatHistory.value.push({
                    role: 'system',
                    content: '你好，我是机器人，请问有什么可以帮忙的？'
                })
            }
        }
    }

    // 模型连接检测
    const connStatus = ref<ConnStatus>('init')
    const connMessage = ref<string>('')

    function extractCheck(res: any): CheckModelResponse {
        const data = res?.data ?? res
        return {
            success: data?.success,
            message: data?.message
        }
    }

    async function handleModelCheck(modelVal: ModelVal): Promise<boolean> {
        if (!modelVal) return false
        const realModelName = (typeof modelVal === 'object' && modelVal !== null) ? (modelVal as any).value : modelVal

        connStatus.value = 'checking'
        connMessage.value = '连接检测中...'

        try {
            const res = await checkModelConnect(realModelName)
            const payload = extractCheck(res)
            if (payload.success) {
                connStatus.value = 'success'
                connMessage.value = '模型连接正常'
                return true
            } else {
                connStatus.value = 'fail'
                connMessage.value = payload.message || '连接失败'
                return false
            }
        } catch (e) {
            connStatus.value = 'fail'
            connMessage.value = '网络错误/后端不可达'
            return false
        }
    }

    const normalize = (res: any) => res?.data ?? res ?? {}

    const sendCommand = async () => {
        if (!inputCommand.value) return

        const modelToUse = (typeof selectedModel.value === 'object' && selectedModel.value !== null)
            ? (selectedModel.value as any).value
            : (selectedModel.value as any)

        if (connStatus.value === 'fail') {
            chatHistory.value.push({
                role: 'system',
                content: `❌ 发送失败：模型 [${modelToUse}] 未连接成功。`
            })
            return
        }

        chatHistory.value.push({ role: 'user', content: inputCommand.value })
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
            chatHistory.value.push({ role: 'system', content: '❌ 错误：请求超时或服务异常' })
        } finally {
            isSending.value = false
        }
    }

    return {
        // 模型
        modelOptions,
        selectedModel,
        isLoadingModels,
        initModels,
        // 连接检测
        connStatus,
        connMessage,
        handleModelCheck,
        // 聊天
        chatHistory,
        inputCommand,
        isSending,
        sendCommand
    }
}
