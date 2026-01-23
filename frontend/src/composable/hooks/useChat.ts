import {onMounted, ref, watch} from 'vue'
import { storeToRefs } from 'pinia'
import { useChatStore } from '@/composable/stores/Store2Chat'
import { getModels, sendChatMsg, checkModelConnect } from '@/composable/api/Chat2LLM'
import type { ModelOption,ChatMsg } from '@/composable/interfaces/Inter2LLM.ts'
import type { ConnStatus } from '@/composable/types/Type2LLM'

const chatWelcomeMessage: ChatMsg = {
    role: 'system',
    content: '你好，我是机器人，请问有什么可以帮忙的？'
}

export function useChat() {
    const chatStore = useChatStore()
    const { chatHistory, selectedModel, modelOptions, isLoadingModels } = storeToRefs(chatStore)

    const inputCommand = ref('')
    const isSending = ref(false)
    const connStatus = ref<ConnStatus>('init')
    const connMessage = ref('')

    function extractModels(res: any): ModelOption[] {
        const data = res?.data ?? res
        if (Array.isArray(data?.models)) return data.models
        return []
    }

    async function initModels() {
        isLoadingModels.value = true
        try {
            const res = await getModels()
            const models = extractModels(res)

            if (models.length > 0) {
                modelOptions.value = models

                const currentExists = models.some((m: any) => m.value === selectedModel.value)

                if (!selectedModel.value || !currentExists) {
                    selectedModel.value = (models[0] as any).value
                }
            } else {
                modelOptions.value = []
            }
        } catch (error) {
            console.error('无法获取模型列表:', error)
            modelOptions.value = [] // 出错时清空列表
        } finally {
            isLoadingModels.value = false
            if (chatHistory.value.length === 0) {
                chatHistory.value.push(chatWelcomeMessage)
            }
        }
    }

    function extractCheck(res: any) {
        const data = res?.data ?? res
        return { success: data?.success, message: data?.message }
    }

    async function handleModelCheck(modelVal: any): Promise<boolean> {
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

        if (!selectedModel.value) {
            chatHistory.value.push({ role: 'system', content: '❌ 请先选择一个模型' })
            return
        }

        const modelToUse = (typeof selectedModel.value === 'object')
            ? (selectedModel.value as any).value
            : selectedModel.value

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

    const clearChatHistory = (keepWelcome = true) => {
        chatStore.clearHistory()

        inputCommand.value = ''

        if (keepWelcome) {
            chatHistory.value.push(chatWelcomeMessage)
        }
    }

    onMounted(async () => {
        await initModels()
    })

    watch(selectedModel, async (newVal) => {
        await handleModelCheck(newVal)
    })

    return {
        modelOptions, // 模型下拉选项列表
        selectedModel, // 当前选中的模型值
        isLoadingModels, // 是否正在加载模型列表
        connStatus, // 当前模型连接检测状态：init/checking/success/fail
        connMessage, // 连接检测提示文案
        chatHistory, // 聊天记录列表
        inputCommand, // 输入框内容
        isSending, // 是否正在发送消息
        sendCommand, // 发送当前输入到后端并把回复写入 chatHistory
        clearChatHistory // 清空聊天记录
    }
}