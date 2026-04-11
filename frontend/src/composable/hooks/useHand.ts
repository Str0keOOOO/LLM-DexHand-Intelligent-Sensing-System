import {ref, computed, onMounted, onUnmounted} from 'vue';
import {checkHand, resetHand, moveHand} from '@/composable/api/Chat2Hand';
import type {HandCommand, HandState} from '@/composable/types/robot';
import {ElMessage} from 'element-plus';

const handState = ref<HandState | null>(null);
const isConnecting = ref(false);
const isConnected = ref(false);

let ws: WebSocket | null = null;
let reconnectTimer: number | undefined = undefined;

export function useHand() {
    const connStatusText = computed(() => (isConnected.value ? '通讯成功' : '已断开'));
    const connStatusColor = computed(() => (isConnected.value ? '#22c55e' : '#f56c6c'));

    const formattedTime = computed(() => {
        const ts = handState.value?.timestamp;
        if (!ts) return '--:--:--';
        const date = new Date(ts * 1000);
        return date.toLocaleTimeString('en-GB');
    });

    const connectWebSocket = () => {
        if (ws) return;

        const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
        const wsUrl = `${protocol}//${window.location.host}/api/hand/data`;

        ws = new WebSocket(wsUrl);

        ws.onopen = () => {
            console.log('🔗 Hand WebSocket Channel Opened');
            clearTimeout(reconnectTimer);
            isConnected.value = true;
        };

        ws.onmessage = (event) => {
            try {
                const result = JSON.parse(event.data);
                if (result.success && result.data && Object.keys(result.data).length > 0) {
                    handState.value = result.data;
                } else if (result.success && Object.keys(result.data || {}).length === 0) {
                    // ROS不活跃，可以不做特殊处理，或者改变状态
                    if (handState.value) {
                        // 保持原来的，或者只更新时间戳
                    }
                }
            } catch (e) {
                console.error('Hand WebSocket Parse Error:', e);
            }
        };

        ws.onclose = () => {
            console.log('🔌 Hand WebSocket Disconnected, retrying in 3s...');
            isConnected.value = false;
            ws = null;
            reconnectTimer = window.setTimeout(connectWebSocket, 3000);
        };

        ws.onerror = (err) => {
            console.error('Hand WebSocket Error:', err);
            ws?.close();
        };
    };

    function closeWebSocket() {
        if (reconnectTimer) clearTimeout(reconnectTimer);
        if (ws) {
            ws.onclose = null;
            ws.close();
            ws = null;
        }
        isConnected.value = false;
    }

    async function check() {
        isConnecting.value = true;
        try {
            const res = await checkHand();
            if (res.success) {
                ElMessage.success('灵巧手状态检查成功');
                return res;
            } else {
                ElMessage.warning('灵巧手状态返回异常');
            }
        } catch (err: any) {
            const msg = err?.response?.data?.message ?? err?.response?.data?.error ?? err?.message ?? '检查状态失败，无法连接服务器';
            ElMessage.error(msg);
            throw err;
        } finally {
            isConnecting.value = false;
        }
    }

    async function reset() {
        try {
            const res = await resetHand();
            if (res.success) {
                ElMessage.success('手部重置序列已启动');
            } else {
                ElMessage.error('重置失败');
            }
            return res;
        } catch (err: any) {
            ElMessage.error('手部重置失败: ' + (err?.response?.data?.message ?? err.message));
            throw err;
        }
    }

    async function move(data: HandCommand) {
        try {
            const res = await moveHand(data);
            ElMessage.success('灵巧手动作指令已下发');
            return res;
        } catch (err: any) {
            const msg = err?.response?.data?.message ?? err?.response?.data?.error ?? err?.message ?? '灵巧手动作指令下发失败';
            ElMessage.error(msg);
            throw err;
        }
    }

    onMounted(() => {
        connectWebSocket();
    });

    // onUnmounted(() => {
    //     closeWebSocket();
    // });

    return {
        /** 当前是否成功通过 WebSocket 连接到了灵巧手的实时数据推送服务 */
        isConnected,
        /** 当前是否正在进行 API 等耗时请求 */
        isConnecting,
        /** 通过 WebSocket 实时接收到的灵巧手当前物理状态 (包含从文档中返回的各个参数) */
        handState,
        /** 连接状态的可读文本（'通讯成功' 或 '已断开'） */
        connStatusText,
        /** 连接状态的 UI 颜色代码（成功为绿色，断开为红色） */
        connStatusColor,
        /** 根据机械手最后更新时间戳格式化成的可读时间 */
        formattedTime,
        /** 主动建立到 /api/hand/data 的 WebSocket 连接 */
        connectWebSocket,
        /** 主动断开当前 WebSocket 连接 */
        closeWebSocket,
        /** 检查灵巧手是否就绪，调用后端 /hand/check 接口 */
        check,
        /** 将灵巧手恢复到初始/复位状态，调用后端 /hand/reset 接口 */
        reset,
        /** 控制灵巧手动作，调用后端 /hand/move 接口 */
        move
    };
}