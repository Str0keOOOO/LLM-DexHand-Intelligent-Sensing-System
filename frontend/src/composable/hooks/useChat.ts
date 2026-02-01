import {onMounted, ref, watch, computed} from 'vue'
import {storeToRefs} from 'pinia'
import {useChatStore} from '@/composable/stores/Store2Chat'
import {getModels, sendChatMsg, checkModelConnect} from '@/composable/api/Chat2LLM'
import {sendControlCommand as apiSendControl} from '@/composable/api/Chat2Robot'
import type {ModelOption, ChatMsg, ConnStatus, ControlRespPayload} from '@composable/types/llm'

const chatWelcomeMessage: ChatMsg = {
    role: 'system',
    content: '你好，我是机器人，请问有什么可以帮忙的？'
}

export function useChat() {
    const chatStore = useChatStore()
    const {
        chatHistory,
        selectedModel: selectedModelRaw,
        modelOptions,
        isLoadingModels
    } = storeToRefs(chatStore)

    const inputCommand = ref('')
    const isSending = ref(false)
    const connStatus = ref<ConnStatus>('init')
    const connMessage = ref('')

    const selectedModel = computed<string>({
        get() {
            const v = selectedModelRaw.value as unknown
            if (typeof v === 'string') return v
            if (v && typeof v === 'object' && 'value' in (v as any) && typeof (v as any).value === 'string') {
                return (v as any).value
            }
            return ''
        },
        set(next: string) {
            ;(selectedModelRaw as any).value = next
        }
    })

    function extractModels(res: any): ModelOption[] {
        const data = res?.data ?? res
        if (Array.isArray(data?.models)) return data.models
        if (Array.isArray(res?.models)) return res.models
        return []
    }

    async function initModels() {
        isLoadingModels.value = true
        try {
            const res = await getModels()
            const models = extractModels(res)

            if (models.length > 0) {
                modelOptions.value = models

                const current = selectedModel.value
                const currentExists = models.some((m: any) => m.value === current)
                if (!current || !currentExists) {
                    selectedModel.value = String((models[0] as any).value ?? '')
                }
            } else {
                modelOptions.value = []
            }
        } catch (error) {
            console.error('无法获取模型列表:', error)
            modelOptions.value = []
        } finally {
            isLoadingModels.value = false
            if (chatHistory.value.length === 0) chatHistory.value.push(chatWelcomeMessage)
        }
    }

    function extractCheck(res: any) {
        const data = res?.data ?? res
        return {success: data?.success, message: data?.message}
    }

    async function handleModelCheck(modelVal: any): Promise<boolean> {
        const realModelName =
            typeof modelVal === 'string'
                ? modelVal
                : (modelVal && typeof modelVal === 'object' ? (modelVal as any).value : '')

        if (!realModelName) return false

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
        } catch {
            connStatus.value = 'fail'
            connMessage.value = '网络错误/后端不可达'
            return false
        }
    }

    const normalize = (res: any) => res?.data ?? res ?? {}

    const sendCommand = async () => {
        if (!inputCommand.value) return

        const modelToUse = selectedModel.value
        if (!modelToUse) {
            chatHistory.value.push({role: 'system', content: '❌ 请先选择一个模型'})
            return
        }

        if (connStatus.value === 'fail') {
            chatHistory.value.push({
                role: 'system',
                content: `❌ 发送失败：模型 \[${modelToUse}\] 未连接成功。`
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

            const reply = String(payload?.reply ?? payload?.data?.reply ?? '')
            const modelName = String(payload?.model_name ?? payload?.model ?? '')
            const controlCmd = payload?.control_command ?? payload?.controlCommand ?? null

            chatHistory.value.push({
                role: 'system',
                content: reply,
                model: modelName
            })

            if (controlCmd) {
                const jointsObj = (controlCmd?.joints ?? {}) as Record<string, unknown>
                const cmdSummary = `手部: ${controlCmd.hand === 'left' ? '左手' : '右手'}, 关节数: ${Object.keys(jointsObj).length}`
                const cmdDetail = JSON.stringify(jointsObj, null, 2)

                chatHistory.value.push({
                    role: 'system',
                    content: `🤖 \[识别到指令\] ${cmdSummary}\n\`\`\`json\n${cmdDetail}\n\`\`\``,
                    model: 'Controller'
                })

                chatHistory.value.push({role: 'system', content: '⏳ 正在下发指令到硬件层...', model: 'System'})

                try {
                    const ctrlRes = await apiSendControl(controlCmd)
                    const ctrlPayload = normalize(ctrlRes) as ControlRespPayload

                    const sentTo =
                        ctrlPayload?.sent_to ?? ctrlPayload?.sentTo ?? ctrlPayload?.result?.sent_to ?? 'Unknown'

                    chatHistory.value.push({
                        role: 'system',
                        content: `✅ 指令执行成功 \(Sent to ${sentTo}\)`,
                        model: 'System'
                    })
                } catch (err: any) {
                    const msg =
                        err?.response?.data?.message ??
                        err?.response?.data?.error ??
                        err?.message ??
                        String(err ?? 'Unknown error')

                    chatHistory.value.push({
                        role: 'system',
                        content: `❌ 指令执行失败: ${msg}`,
                        model: 'System'
                    })
                }
            }
        } catch (error) {
            chatHistory.value.push({role: 'system', content: '❌ 错误：请求超时或服务异常'})
            console.error(error)
        } finally {
            isSending.value = false
        }
    }

    const clearChatHistory = (keepWelcome = true) => {
        chatStore.clearHistory()
        inputCommand.value = ''
        if (keepWelcome) chatHistory.value.push(chatWelcomeMessage)
    }

    onMounted(async () => {
        await initModels()
    })

    watch(selectedModel, async (newVal) => {
        await handleModelCheck(newVal)
    })

    return {
        modelOptions,
        selectedModel,
        isLoadingModels,
        connStatus,
        connMessage,
        chatHistory,
        inputCommand,
        isSending,
        sendCommand,
        clearChatHistory
    }
}