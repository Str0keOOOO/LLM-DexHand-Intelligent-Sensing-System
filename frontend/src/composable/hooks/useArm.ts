import {ref, computed, onMounted, onUnmounted} from 'vue';
import {checkArm, resetArm, moveArm} from '@/composable/api/Chat2Arm';
import type {ArmCommand, ArmState} from '@/composable/types/robot';
import {ElMessage} from 'element-plus';

const isConnecting = ref(false);
const isConnected = ref(false);
const ArmState = ref<ArmState | null>(null);

let ws: WebSocket | null = null;
let reconnectTimer: number | undefined = undefined;

export function useArm() {
    const connStatusText = computed(() => (isConnected.value ? '通讯成功' : '已断开'));
    const connStatusColor = computed(() => (isConnected.value ? '#22c55e' : '#f56c6c'));

    const connectWebSocket = () => {
        if (ws) return;

        // 动态适配 HTTP/HTTPS 的 ws 协议
        const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
        const wsUrl = `${protocol}//${window.location.host}/api/arm/pos`;

        ws = new WebSocket(wsUrl);

        ws.onopen = () => {
            console.log('🔗 Arm WebSocket Channel Opened');
            clearTimeout(reconnectTimer);
            isConnected.value = true;
        };

        ws.onmessage = (event) => {
            try {
                const result = JSON.parse(event.data);
                // 校验文档指定格式 { success: true, data: { ... } }
                if (result.success && result.data) {
                    ArmState.value = result.data;
                }
            } catch (e) {
                console.error('WebSocket Parse Error:', e);
            }
        };

        ws.onclose = () => {
            console.log('🔌 Arm WebSocket Disconnected, retrying in 3s...');
            isConnected.value = false;
            ws = null;
            // 尝试自动重连
            reconnectTimer = window.setTimeout(connectWebSocket, 3000);
        };

        ws.onerror = (err) => {
            console.error('Arm WebSocket Error:', err);
            ws?.close(); // 触发 onclose 引起重连
        };
    };

    function closeWebSocket() {
        if (reconnectTimer) clearTimeout(reconnectTimer);
        if (ws) {
            ws.onclose = null; // 避免触发自动重连逻辑
            ws.close();
            ws = null;
        }
        isConnected.value = false;
    }

    // --- actions ---
    async function check() {
        isConnecting.value = true;
        try {
            const res = await checkArm();
            if (res.success) {
                ElMessage.success('机械臂状态检查成功');
                return res;
            } else {
                ElMessage.warning('机械臂状态返回异常');
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
            const res = await resetArm();
            ElMessage.success('机械臂复位指令已下发');
            return res;
        } catch (err: any) {
            ElMessage.error('机械臂复位失败: ' + (err?.response?.data?.message ?? err.message));
            throw err;
        }
    }

    async function move(data: ArmCommand) {
        try {
            const res = await moveArm(data);
            ElMessage.success('机械臂移动指令已下发');
            return res;
        } catch (err: any) {
            const msg = err?.response?.data?.message ?? err?.response?.data?.error ?? err?.message ?? '移动指令下发失败';
            ElMessage.error(msg);
            throw err;
        }
    }

    onMounted(() => {
        // 组件挂载时自动建立 Websocket 链接
        connectWebSocket();
    });

    // onUnmounted(() => {
    //     // 组件销毁时断开 Websocket
    //     closeWebSocket();
    // });

    return {
        /** 当前是否成功通过 WebSocket 连接到了机械臂的实时数据推送服务 */
        isConnected,
        /** 当前是否正在进行 API 等耗时请求（如加载中状态） */
        isConnecting,
        /** 通过 WebSocket 实时接收到的机械臂当前物理状态及姿态 (x, y, z, rx, ry, rz, timestamp) */
        ArmState,
        /** 连接状态的可读文本（'通讯成功' 或 '已断开'） */
        connStatusText,
        /** 连接状态的 UI 颜色代码（成功为绿色，断开为红色） */
        connStatusColor,
        /** 主动建立到 /api/arm/pos 的 WebSocket 连接，开始接收机械臂的实时数据 */
        connectWebSocket,
        /** 主动断开当前 WebSocket 连接，停止接收数据，并清除自动重连任务 */
        closeWebSocket,
        /** 检查机械臂是否就绪，调用后端 /arm/check 接口 */
        check,
        /** 将机械臂恢复到初始/复位状态，调用后端 /arm/reset 接口 */
        reset,
        /** 控制机械臂移动至指定的位置和姿态，调用后端 /arm/move 接口 */
        move
    };
}
