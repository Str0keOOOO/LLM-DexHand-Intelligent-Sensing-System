import {ref} from 'vue'
import type {ModelOption} from '@/composable/interfaces/Inter2LLM.ts'
import {getModels} from '@/composable/api/Chat2LLM'

export function useModels(defaultModel: ModelOption = {label: '默认模型 (GPT-3.5)', value: 'gpt-3.5-turbo'}) {
    const modelOptions = ref<ModelOption[]>([])
    const selectedModel = ref<string>(defaultModel.value)
    const isLoadingModels = ref(false)

    // 兼容接口可能直接返回数据或返回 AxiosResponse 的情况
    const extractModels = (res: any): ModelOption[] => {
        const data = res?.data ?? res
        if (Array.isArray(data?.models)) return data.models
        return []
    }

    const initModels = async () => {
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
            modelOptions.value = [{label: '连接失败 (默认)', value: defaultModel.value}]
            selectedModel.value = defaultModel.value
        } finally {
            isLoadingModels.value = false
        }
    }

    return {
        modelOptions,
        selectedModel,
        isLoadingModels,
        initModels
    }
}
