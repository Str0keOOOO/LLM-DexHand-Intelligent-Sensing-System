// frontend/src/composable/hooks/useRobot.ts
import { ref, onMounted, onUnmounted } from 'vue';
import type { RobotState } from '@/composable/interfaces/Inter2Robot';

// 全局单例状态，保证切换页面数据不丢失（可选，也可放在组件内）
const robotState = ref<RobotState>({
    left: { joints: {}, touch: [], motor: [] },
    right: { joints: {}, touch: [], motor: [] },
    timestamp: 0,
});

const isConnected = ref(false);
let ws: WebSocket | null = null;
let reconnectTimer: number | undefined = undefined;

export function useRobot() {

    const connectWebSocket = () => {
        if (ws) return;

        // 自动判断 WS 地址 (适配 Vite Proxy 或直连)
        const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
        // 如果你在 vite.config.ts 配置了 proxy '/api' -> 'http://localhost:8000'
        // 那么这里应该是 ws://localhost:5173/api/robot-data
        // 如果是直连后端，请改为 `ws://localhost:8000/api/robot-data`
        const wsUrl = `${protocol}//localhost:8000/api/ros_ws/robot-data`;

        ws = new WebSocket(wsUrl);

        ws.onopen = () => {
            console.log('🔗 Robot WebSocket Connected');
            isConnected.value = true;
            clearTimeout(reconnectTimer);
        };

        ws.onmessage = (event) => {
            try {
                const data = JSON.parse(event.data);
                // 简单校验数据完整性
                if (data.left && data.right) {
                    robotState.value = data;
                }
                console.log(data)
            } catch (e) {
                console.error('Parse Error:', e);
            }
        };

        ws.onclose = () => {
            console.log('🔌 Robot WebSocket Disconnected, retrying in 3s...');
            isConnected.value = false;
            ws = null;
            reconnectTimer = window.setTimeout(connectWebSocket, 3000);
        };

        ws.onerror = (err) => {
            console.error('WebSocket Error:', err);
            ws?.close();
        };
    };

    const closeWebSocket = () => {
        if (ws) {
            ws.close();
            ws = null;
        }
        clearTimeout(reconnectTimer);
    };

    onMounted(() => {
        connectWebSocket();
    });

    onUnmounted(() => {
        // 视需求决定是否在组件卸载时断开，
        // 如果希望后台一直保持连接，可以注释掉下面这行
        // closeWebSocket();
    });

    return {
        robotState,
        isConnected,
        connectWebSocket,
        closeWebSocket
    };
}