import {ref} from 'vue';
import {getChatHistory, getSensorHistory} from '@/composable/api/Chat2DB';
import type {ChatHistoryResponse, SensorHistoryResponse} from '@/composable/types/db';
import {ElMessage} from 'element-plus';

export function useDB() {
    const chatHistoryData = ref<ChatHistoryResponse | null>(null);
    const sensorHistoryData = ref<SensorHistoryResponse | null>(null);

    const isLoadingChat = ref(false);
    const isLoadingSensor = ref(false);

    async function fetchChatHistory(limit: number = 20, skip: number = 0) {
        isLoadingChat.value = true;
        try {
            const res = await getChatHistory(limit, skip);
            chatHistoryData.value = res;
            return res;
        } catch (err: any) {
            const msg = err?.response?.data?.message ?? err?.message ?? '获取聊天记录失败';
            ElMessage.error(msg);
            throw err;
        } finally {
            isLoadingChat.value = false;
        }
    }

    async function fetchSensorHistory(target: string, minutes: number) {
        isLoadingSensor.value = true;
        try {
            const res = await getSensorHistory(target, minutes);
            sensorHistoryData.value = res;
            return res;
        } catch (err: any) {
            const msg = err?.response?.data?.message ?? err?.message ?? '获取传感器历史数据失败';
            ElMessage.error(msg);
            throw err;
        } finally {
            isLoadingSensor.value = false;
        }
    }

    return {
        /** 聊天历史数据结果 */
        chatHistoryData,
        /** 传感器历史数据结果 */
        sensorHistoryData,
        /** 是否正在加载聊天历史 */
        isLoadingChat,
        /** 是否正在加载传感器历史 */
        isLoadingSensor,
        /** 执行获取聊天历史记录 */
        fetchChatHistory,
        /** 执行获取传感器历史数据 */
        fetchSensorHistory
    };
}

