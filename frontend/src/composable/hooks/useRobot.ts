import {ref, computed, onMounted, onUnmounted} from 'vue';
import type {RobotState} from '@/composable/types/robot';
import {resetRobot} from "@/composable/api/Chat2Robot.ts";
import {ElMessage} from 'element-plus'

const robotState = ref<RobotState>({
    right: {
        joint: {
            position: {},
            velocity: {},
        },
        touch: {
            normal_force: [], normal_force_delta: [], tangential_force: [],
            tangential_force_delta: [], direction: [], proximity: [], temperature: []
        },
        motor: {
            angle: [], encoder_position: [], current: [],
            velocity: [], error_code: [], impedance: []
        }
    },
    timestamp: 0,
});

const isConnected = ref(false);
let ws: WebSocket | null = null;
let reconnectTimer: number | undefined = undefined;

export function useRobot() {
    const connStatusText = computed(() => (isConnected.value ? '通讯成功' : '已断开'));
    const connStatusColor = computed(() => (isConnected.value ? '#22c55e' : '#f56c6c'));

    const formattedTime = computed(() => {
        const ts = robotState.value.timestamp;
        if (!ts) return '--:--:--';
        const date = new Date(ts * 1000);
        return date.toLocaleTimeString('en-GB');
    });

    const connectWebSocket = () => {
        if (ws) return;

        const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
        const wsUrl = `${protocol}//${window.location.host}/api/ros_ws/robot-data`;

        ws = new WebSocket(wsUrl);

        ws.onopen = () => {
            console.log('🔗 WebSocket Channel Opened');
            clearTimeout(reconnectTimer);
        };

        ws.onmessage = (event) => {
            try {
                const data = JSON.parse(event.data);
                isConnected.value = !!data.ros_active;
                if (data.ros_active && data.right) {
                    robotState.value = data;
                } else if (!data.ros_active) {
                    robotState.value.timestamp = data.timestamp;
                }
            } catch (e) {
                console.error('Parse Error:', e);
            }
        };

        ws.onclose = () => {
            console.log('🔌 WebSocket Disconnected, retrying in 3s...');
            isConnected.value = false;
            ws = null;
            reconnectTimer = window.setTimeout(connectWebSocket, 3000);
        };

        ws.onerror = (err) => {
            console.error('WebSocket Error:', err);
            ws?.close();
        };
    };

    function closeWebSocket() {
        if (ws) {
            ws.close();
            ws = null;
        }
        clearTimeout(reconnectTimer);
        isConnected.value = false;
    }

    async function handleReset() {
        try {
            const res = await resetRobot()
            if (res.status === 'success') {
                ElMessage.success('手部重置序列已启动')
            } else {
                ElMessage.error('重置失败')
            }
        } catch (error) {
            ElMessage.error('无法连接到后端服务器')
        }
    }

    onMounted(() => {
        connectWebSocket();
    })

    onUnmounted(() => {
        closeWebSocket();
    });

    return {
        robotState,
        isConnected,
        connStatusText,
        connStatusColor,
        formattedTime,
        connectWebSocket,
        closeWebSocket,
        handleReset
    };
}