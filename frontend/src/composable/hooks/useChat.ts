import {onMounted, ref, watch, computed} from 'vue'
import {storeToRefs} from 'pinia'
import {useChatStore} from '@/composable/stores/Store2Chat'
import {listModels, sendChatMsg, checkModelConnect} from '@/composable/api/Chat2LLM'
import {sendControlCommand as apiSendControl} from '@/composable/api/Chat2Hand.ts'
import type {ModelOption, ChatMsg, ConnStatus, ControlRespPayload} from '@/composable/types/llm'

const chatWelcomeMessage: ChatMsg = {
    role: 'system',
    content: '你好，我是机器人，请问有什么可以帮忙的？'
}

export function useChat() {
    const chatStore = useChatStore()
    // 从 store 中解构状态，保持响应式
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

    // --- 语音识别 (STT) 相关逻辑 ---
    const isRecording = ref(false)
    let recognition: any = null

    function initSpeechRecognition() {
        const SpeechRecognition = (window as any).SpeechRecognition || (window as any).webkitSpeechRecognition
        if (!SpeechRecognition) return null

        const recognizer = new SpeechRecognition()
        recognizer.lang = 'zh-CN'
        recognizer.continuous = false
        recognizer.interimResults = true

        recognizer.onstart = () => {
            isRecording.value = true
        }
        recognizer.onend = () => {
            isRecording.value = false
        }
        recognizer.onerror = () => {
            isRecording.value = false
        }
        recognizer.onresult = (event: any) => {
            const transcript = Array.from(event.results)
                .map((result: any) => result[0])
                .map((result: any) => result.transcript)
                .join('')
            inputCommand.value = transcript
        }
        return recognizer
    }

    const toggleRecording = () => {
        if (!recognition) recognition = initSpeechRecognition()
        if (!recognition) {
            alert('当前浏览器不支持语音识别')
            return
        }
        isRecording.value ? recognition.stop() : recognition.start()
    }


    const selectedModel = computed<string>({
        get() {
            const v = selectedModelRaw.value as unknown
            if (typeof v === 'string') return v
            if (v && typeof v === 'object' && 'value' in (v as ModelOption)) {
                return (v as ModelOption).value
            }
            return ''
        },
        set(next: string) {
            ;(selectedModelRaw as any).value = next
        }
    })

    function extractModels(res: any): ModelOption[] {
        const data = res?.data ?? res
        const list = Array.isArray(data?.models) ? data.models : (Array.isArray(res?.models) ? res.models : [])
        return list as ModelOption[]
    }

    async function initModels() {
        isLoadingModels.value = true
        try {
            const res = await listModels()
            const models: ModelOption[] = extractModels(res) // 显式声明类型

            if (models.length > 0) {
                modelOptions.value = models

                const current = selectedModel.value
                const currentExists = models.some((m: ModelOption) => m.value === current)
                if (!current || !currentExists) {
                    selectedModel.value = models[0]?.value ?? ''
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

    async function handleModelCheck(modelVal: string): Promise<boolean> {
        if (!modelVal) return false

        connStatus.value = 'checking'
        connMessage.value = '连接检测中...'

        try {
            const res = await checkModelConnect(modelVal)
            const data = res?.data ?? res
            if (data?.success) {
                connStatus.value = 'success'
                connMessage.value = '模型连接正常'
                return true
            } else {
                connStatus.value = 'fail'
                connMessage.value = data?.message || '连接失败'
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
                    content: `🤖 [识别到指令] ${cmdSummary}\n\`\`\`json\n${cmdDetail}\n\`\`\``,
                    model: 'Controller'
                })

                chatHistory.value.push({role: 'system', content: '⏳ 正在下发指令到硬件层...', model: 'System'})

                try {
                    const ctrlRes = await apiSendControl(controlCmd)
                    const ctrlPayload = normalize(ctrlRes) as ControlRespPayload

                    const sentTo = ctrlPayload?.sent_to ?? ctrlPayload?.sentTo ?? ctrlPayload?.result?.sent_to ?? 'Unknown'

                    chatHistory.value.push({
                        role: 'system',
                        content: `✅ 指令执行成功 (Sent to ${sentTo})`,
                        model: 'System'
                    })
                } catch (err: any) {
                    const msg = err?.response?.data?.message ?? err?.response?.data?.error ?? err?.message ?? 'Unknown error'
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
        /** 可供选择的大语言模型列表 */
        modelOptions,
        /** 当前选中的模型名称 */
        selectedModel,
        /** 是否正在加载模型列表数据 */
        isLoadingModels,
        /** 大语言模型的连接检测状态（'init' | 'checking' | 'success' | 'fail'） */
        connStatus,
        /** 大语言模型连接检测后的具体提示信息 */
        connMessage,
        /** 对话历史记录数组 */
        chatHistory,
        /** 用户当前的输入框指令文本（支持手动输入和语音识别产出） */
        inputCommand,
        /** 当前是否正在发送消息并等待大语言模型响应中 */
        isSending,
        /** 当前是否正在进行语音识别（录音中） */
        isRecording,
        /** 发送聊天指令（将输入框文本发送给后端选定的 LLM）并尝试解析和下发附带的硬件控制命令 */
        sendCommand,
        /** 清除界面上的对话历史记录（可选择是否保留初始欢迎语） */
        clearChatHistory,
        /** 切换语音识别的状态：点击开启录制，再点停止录制并转写到输入框 */
        toggleRecording
    }
}