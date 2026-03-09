import {ref, computed, onMounted, onUnmounted} from 'vue';
import type {RobotState} from '@/composable/types/robot';
import {resetRobot} from "@/composable/api/Chat2Robot.ts";
import {ElMessage} from 'element-plus'

// 机器人数据状态
const robotState = ref<RobotState>({
    right: {
        joints: {}, velocities: {},
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

/**
 * 统一的连接状态
 * true:  WebSocket 已连接 且 ROS 桥接已就绪
 * false: WebSocket 断开 或 ROS 未启动
 */
const isConnected = ref(false);

let ws: WebSocket | null = null;
let reconnectTimer: number | undefined = undefined;

export function useRobot() {
    // 状态文字逻辑：仅在 isConnected 为 true 时显示通讯成功
    const connStatusText = computed(() => (isConnected.value ? '通讯成功' : '已断开'));

    // 状态颜色逻辑：只有端到端全通才显示绿色
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
            // 注意：此处不直接设 isConnected = true，因为要等后端传回 ros_active
            clearTimeout(reconnectTimer);
        };

        ws.onmessage = (event) => {
            try {
                const data = JSON.parse(event.data);

                // 更新连接状态
                isConnected.value = !!data.ros_active;

                if (data.ros_active && data.right) {
                    robotState.value = data;
                } else if (!data.ros_active) {
                    // 【关键修改】：ROS 断开时，可以选择重置状态或保持最后状态但变灰
                    // 如果希望界面彻底显示断开，可以重置部分关键数值
                    robotState.value.timestamp = data.timestamp;
                }
            } catch (e) {
                console.error('Parse Error:', e);
            }
        };

        ws.onclose = () => {
            console.log('🔌 WebSocket Disconnected, retrying in 3s...');
            // 连接断开，状态必为 false
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
    };

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
    });

    onUnmounted(() => {
        closeWebSocket();
    });

    return {
        robotState,
        isConnected, // 现在这一个变量就涵盖了所有连接逻辑
        connStatusText,
        connStatusColor,
        formattedTime,
        connectWebSocket,
        closeWebSocket,
        handleReset
    };
}