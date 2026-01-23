import {ref, onMounted, onUnmounted, onBeforeUnmount, computed} from 'vue';
import * as echarts from 'echarts';
import type { ChartDataPoint, SeriesDataInterface } from '@/composable/interfaces/Inter2Chart';
import type { ConnectOptions } from '@/composable/interfaces/Inter2Robot.ts';
import { buildUrl, safeParse,rosHealth } from '@/composable/api/Chat2Robot';
import type { ConnStatus } from '@/composable/types/Type2LLM';

export function useRobotSocket(onData?: (data: any) => void, options?: ConnectOptions) {
    const socket = ref<WebSocket | null>(null);
    const isConnected = ref(false);

    const connect = () => {
        const url = buildUrl(options?.path);
        try {
            socket.value = new WebSocket(url);
        } catch (err) {
            console.error('WebSocket create error:', err);
            socket.value = null;
            isConnected.value = false;
            return socket.value;
        }

        socket.value.onopen = (ev) => {
            isConnected.value = true;
            options?.onOpen?.(ev);
        };

        socket.value.onclose = (ev) => {
            isConnected.value = false;
            options?.onClose?.(ev);
        };

        socket.value.onerror = (ev) => {
            console.error('WebSocket error:', ev);
            options?.onError?.(ev);
        };

        socket.value.onmessage = (event: MessageEvent) => {
            const res = safeParse(event.data);
            if (!res) return;
            if (res.mode === 'BACKEND_INIT') return;
            const dataContent = res.payload || res;
            if (res.timestamp && dataContent.timestamp === undefined) {
                dataContent.timestamp = res.timestamp;
            }
            onData?.(dataContent);
        };

        return socket.value;
    };

    const close = () => {
        if (!socket.value) return;
        try {
            socket.value.close();
        } catch (e) {
            console.error('WebSocket close error:', e);
        } finally {
            socket.value = null;
            isConnected.value = false;
        }
    };

    return {
        socket,
        connect,
        close,
        isConnected
    };
}

export function useRobot(options?: {
    autoInit?: boolean;
    autoStart?: boolean;
    maxPoints?: number;
    interval?: number;
}) {
    const RENDER_INTERVAL = options?.interval ?? 200;
    const MAX_POINTS = options?.maxPoints ?? 50;

    const chartRef = ref<HTMLElement | null>(null);
    let myChart: echarts.ECharts | null = null;
    let renderIntervalTimer: number | null = null;
    let dataBuffer: ChartDataPoint[] = [];

    const axisData: string[] = [];
    const seriesData: SeriesDataInterface = { ff: [], mf: [], th: [] };

    function formatTime(timestamp: number): string {
        if (!timestamp && timestamp !== 0) return '';
        const ts = Number(timestamp);
        const date = new Date(ts > 1e12 ? ts : ts * 1000);
        const h = date.getHours().toString().padStart(2, '0');
        const m = date.getMinutes().toString().padStart(2, '0');
        const s = date.getSeconds().toString().padStart(2, '0');
        const ms = date.getMilliseconds().toString().padStart(3, '0').slice(0, 2);
        return `${h}:${m}:${s}.${ms}`;
    }

    function initChart(): void {
        if (!chartRef.value) return;
        if (myChart) myChart.dispose();
        myChart = echarts.init(chartRef.value);

        const option: echarts.EChartsOption = {
            animation: true,
            animationDurationUpdate: RENDER_INTERVAL,
            animationEasingUpdate: 'linear',
            title: { text: 'DexHand 关节实时载荷' },
            tooltip: { trigger: 'axis' },
            legend: { data: ['食指', '中指', '拇指'], right: '5%', top: '5%' },
            grid: { left: '3%', right: '5%', bottom: '3%' },
            xAxis: {
                type: 'category',
                boundaryGap: false,
                data: axisData,
                axisLabel: { hideOverlap: true, interval: 'auto' }
            },
            yAxis: {
                type: 'value',
                min: 0,
                name: '力(N)',
                nameLocation: 'middle'
            },
            series: [
                {
                    name: '食指',
                    type: 'line',
                    data: seriesData.ff,
                    smooth: true,
                    showSymbol: false,
                    itemStyle: { color: '#5470C6' }
                },
                {
                    name: '中指',
                    type: 'line',
                    data: seriesData.mf,
                    smooth: true,
                    showSymbol: false,
                    itemStyle: { color: '#91CC75' }
                },
                {
                    name: '拇指',
                    type: 'line',
                    data: seriesData.th,
                    smooth: true,
                    showSymbol: false,
                    itemStyle: { color: '#FAC858' }
                }
            ]
        };

        myChart.setOption(option);
        window.addEventListener('resize', resizeChart);
    }

    function resizeChart(): void {
        myChart?.resize();
    }

    function handleDataUpdate(res: any): void {
        const timestamp = res.timestamp ?? res.time ?? Date.now() / 1000;
        let f0 = 0, f1 = 0, f2 = 0;

        if (res.joints) {
            f0 = Number(res.joints.ff_mcp ?? 0);
            f1 = Number(res.joints.lf_mcp ?? 0);
            f2 = Number(res.joints.th_rot ?? 0);
        } else if (res.touch) {
            f0 = Number(res.touch.ff ?? 0);
            f2 = Number(res.touch.th ?? 0);
        } else if (Array.isArray(res.fingers)) {
            f0 = Number(res.fingers[0] ?? 0);
            f1 = Number(res.fingers[1] ?? 0);
            f2 = Number(res.fingers[2] ?? 0);
        }

        dataBuffer.push({
            timeStr: formatTime(timestamp),
            ff: f0,
            mf: f1,
            th: f2
        });
    }

    function startRenderingLoop(): void {
        if (renderIntervalTimer) return;
        renderIntervalTimer = window.setInterval(() => {
            if (!myChart || dataBuffer.length === 0) return;

            for (const item of dataBuffer) {
                axisData.push(item.timeStr);
                seriesData.ff.push(item.ff);
                seriesData.mf.push(item.mf);
                seriesData.th.push(item.th);
            }
            dataBuffer = [];

            const removeCount = axisData.length - MAX_POINTS;
            if (removeCount > 0) {
                axisData.splice(0, removeCount);
                seriesData.ff.splice(0, removeCount);
                seriesData.mf.splice(0, removeCount);
                seriesData.th.splice(0, removeCount);
            }

            myChart.setOption({
                xAxis: { data: axisData },
                series: [
                    { data: seriesData.ff },
                    { data: seriesData.mf },
                    { data: seriesData.th }
                ]
            });
        }, RENDER_INTERVAL);
    }

    function stopRenderingLoop(): void {
        if (renderIntervalTimer) {
            clearInterval(renderIntervalTimer);
            renderIntervalTimer = null;
        }
    }

    function disposeChart(): void {
        stopRenderingLoop();
        window.removeEventListener('resize', resizeChart);
        myChart?.dispose();
        myChart = null;
        dataBuffer = [];
        axisData.length = 0;
        seriesData.ff.length = 0;
        seriesData.mf.length = 0;
        seriesData.th.length = 0;
    }

    function handleMounted(): void {
        if (options?.autoInit) initChart();
        if (options?.autoStart) startRenderingLoop();
    }

    function handleUnmounted(): void {
        disposeChart();
    }

    onMounted(handleMounted);
    onUnmounted(handleUnmounted);

    function isRunning(): boolean {
        return !!renderIntervalTimer;
    }

    function isInited(): boolean {
        return !!myChart;
    }

    return {
        chartRef,
        initChart,
        disposeChart,
        pushData: handleDataUpdate,
        startRenderingLoop,
        stopRenderingLoop,
        isRunning,
        isInited
    };
}

export function useRobotHealth(options?: { url?: string; interval?: number; autoStart?: boolean }) {
    const intervalMs = options?.interval ?? 5000;

    const robotStatus = ref<ConnStatus>('init');
    const robotMessage = ref<string>('');

    let timer: number | null = null;

    async function checkRobotHealth(): Promise<boolean> {
        robotStatus.value = 'checking';
        robotMessage.value = '检测中...';

        try {
            const res = await rosHealth();
            const data = (res as any).data ?? res;
            if (!data) {
                robotStatus.value = 'fail';
                robotMessage.value = '无返回数据';
                return false;
            }

            const ok = data.status === 'ok' && data.ros_bridge === 'connected';
            if (ok) {
                robotStatus.value = 'success';
                robotMessage.value = '机器人在线';
                return true;
            } else {
                robotStatus.value = 'fail';
                robotMessage.value = data.ros_bridge ? `状态：${data.ros_bridge}` : '机器人未就绪';
                return false;
            }
        } catch (err: any) {
            robotStatus.value = 'fail';
            robotMessage.value = err?.message ?? String(err) ?? '请求失败';
            return false;
        }
    }

    function startAutoPoll() {
        if (timer) return;
        checkRobotHealth().catch(() => {});
        timer = window.setInterval(() => {
            checkRobotHealth().catch(() => {});
        }, intervalMs);
    }

    function stopAutoPoll() {
        if (timer) {
            clearInterval(timer);
            timer = null;
        }
    }

    if (options?.autoStart) startAutoPoll();

    onBeforeUnmount(() => {
        stopAutoPoll();
    });

    const connStatusText = computed(() => {
        return robotStatus.value === 'checking'
            ? '检测中'
            : robotStatus.value === 'success'
                ? '正常'
                : robotStatus.value === 'fail'
                    ? '异常'
                    : '未知'
    })

    const connStatusColor = computed(() => {
        return robotStatus.value === 'checking'
            ? '#E6A23C'
            : robotStatus.value === 'success'
                ? '#67C23A'
                : robotStatus.value === 'fail'
                    ? '#F56C6C'
                    : '#909399'
    })

    onMounted(async () => {
        await checkRobotHealth()
    })

    return {
        robotStatus,
        robotMessage,
        connStatusText,
        connStatusColor,
        checkRobotHealth,
        startAutoPoll,
        stopAutoPoll
    };
}
