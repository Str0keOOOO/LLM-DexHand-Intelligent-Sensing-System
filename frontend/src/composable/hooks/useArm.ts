import { ref, computed, onMounted, onUnmounted } from 'vue';
import { connectArm, getActualJointPosDegree, startJog } from '@/composable/api/Chat2Arm';
import type { StartJogRequest } from '@/composable/api/Chat2Arm';
import { ElMessage } from 'element-plus';

// TODO加入reset
type AnyObj = Record<string, any>

function normalize<T = AnyObj>(res: any): T {
    return (res?.data ?? res ?? {}) as T
}

export function useArm() {
    // --- state ---
    const isConnecting = ref(false)
    const isConnected = ref(false)

    const isFetchingJoints = ref(false)
    const jointPositionsDeg = ref<number[]>([])

    const isJogging = ref(false)

    const connStatusText = computed(() => (isConnected.value ? '通讯成功' : '已断开'))
    const connStatusColor = computed(() => (isConnected.value ? '#22c55e' : '#f56c6c'))

    // --- polling ---
    const isPolling = ref(false)
    const pollIntervalMs = ref(500)
    let pollTimer: number | undefined

    function stopPolling() {
        if (pollTimer) {
            clearInterval(pollTimer)
            pollTimer = undefined
        }
        isPolling.value = false
    }

    function startPolling(intervalMs?: number) {
        if (typeof intervalMs === 'number' && intervalMs > 0) pollIntervalMs.value = intervalMs
        if (pollTimer) return
        isPolling.value = true

        // 先拉一次，避免 UI 空窗
        void refreshJoints()

        pollTimer = window.setInterval(() => {
            void refreshJoints()
        }, pollIntervalMs.value)
    }

    // --- actions ---
    async function connect() {
        isConnecting.value = true
        try {
            const res = await connectArm()
            const payload = normalize<any>(res)

            // 尽量兼容不同后端返回
            const ok =
                payload?.success === true ||
                payload?.connected === true ||
                payload?.status === 'success' ||
                payload?.status === true

            isConnected.value = !!ok
            ok ? ElMessage.success('机械臂连接成功') : ElMessage.warning('机械臂连接请求已发送')
            return payload
        } catch (err: any) {
            isConnected.value = false
            const msg = err?.response?.data?.message ?? err?.response?.data?.error ?? err?.message ?? '无法连接到后端服务器'
            ElMessage.error(msg)
            throw err
        } finally {
            isConnecting.value = false
        }
    }

    async function refreshJoints() {
        isFetchingJoints.value = true
        try {
            const res = await getActualJointPosDegree()
            const payload = normalize<any>(res[1])

            // 兼容：{ joints: number[] } 或直接 number[] 或其它透传
            const joints = Array.isArray(payload) ? payload : Array.isArray(payload?.joints) ? payload.joints : []
            jointPositionsDeg.value = joints

            // 如果能拿到 joints，就认为连接是 OK 的（可选）
            if (joints.length > 0) isConnected.value = true
            return payload
        } catch (err) {
            // 轮询时不要一直弹错误
            if (!isPolling.value) ElMessage.error('获取关节角度失败')
            throw err
        } finally {
            isFetchingJoints.value = false
        }
    }

    async function jog(data: StartJogRequest) {
        isJogging.value = true
        try {
            const res = await startJog(data)
            const payload = normalize<any>(res)
            ElMessage.success('已下发 Jog 指令')
            return payload
        } catch (err: any) {
            const msg = err?.response?.data?.message ?? err?.response?.data?.error ?? err?.message ?? 'Jog 指令下发失败'
            ElMessage.error(msg)
            throw err
        } finally {
            isJogging.value = false
        }
    }

    onMounted(() => {
        // 默认不自动连接；仅清理轮询资源
    })

    onUnmounted(() => {
        stopPolling()
    })

    return {
        // state
        isConnected,
        isConnecting,
        isFetchingJoints,
        isJogging,
        jointPositionsDeg,
        connStatusText,
        connStatusColor,

        // polling
        isPolling,
        pollIntervalMs,
        startPolling,
        stopPolling,

        // actions
        connect,
        refreshJoints,
        jog
    }
}
