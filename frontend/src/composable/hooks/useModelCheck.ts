import {ref} from 'vue'
import {checkModelConnect} from '@/composable/api/Chat2LLM'

type ModelVal = any

interface CheckModelResponse {
    success?: boolean
    message?: string
}

export function useModelCheck() {
    const connStatus = ref<'init' | 'checking' | 'success' | 'fail'>('init')
    const connMessage = ref<string>('')

    const extract = (res: any): CheckModelResponse => {
        const data = res?.data ?? res
        return {
            success: data?.success,
            message: data?.message
        }
    }

    const handleModelCheck = async (modelVal: ModelVal): Promise<boolean> => {
        if (!modelVal) return false
        const realModelName = (typeof modelVal === 'object' && modelVal !== null)
            ? modelVal.value
            : modelVal

        connStatus.value = 'checking'
        connMessage.value = '连接检测中...'

        try {
            const res = await checkModelConnect(realModelName)
            const payload = extract(res)
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

    return {
        connStatus,
        connMessage,
        handleModelCheck
    }
}