<script setup lang="ts">
import {onMounted, ref, onUnmounted} from 'vue';
import * as echarts from 'echarts';
import type {ChartDataPoint, SeriesDataInterface} from '@/composable/interfaces/Inter2Chart';
import { connectRobotWebSocket } from '@/composable/api/Chat2Robot';

const RENDER_INTERVAL: number = 200;
const MAX_POINTS: number = 50;

const chartRef = ref<HTMLElement | null>(null);
let myChart: echarts.ECharts | null = null;
let socket: WebSocket | null = null; // 可选保留用于调试
let wsClose: (() => void) | null = null; // 保存关闭函数
let dataBuffer: ChartDataPoint[] = [];
let renderIntervalTimer: number | null = null;

const axisData: string[] = [];
const seriesData: SeriesDataInterface = {
  ff: [],
  mf: [],
  th: []
};

const initChart = () => {
  if (chartRef.value) {
    if (myChart) myChart.dispose();
    myChart = echarts.init(chartRef.value);

    const option: echarts.EChartsOption = {
      animation: true,
      animationDurationUpdate: RENDER_INTERVAL,
      animationEasingUpdate: 'linear',

      title: {text: 'DexHand 关节实时载荷'},
      tooltip: {trigger: 'axis'},
      legend: {data: ['食指', '中指', '拇指'], right: '5%', top: '5%'},
      grid: {left: '3%', right: '5%', bottom: '3%'},
      xAxis: {
        type: 'category',
        boundaryGap: false,
        data: axisData,
        axisLabel: {
          hideOverlap: true,
          interval: 'auto'
        }
      },
      yAxis: {
        type: 'value',
        min: 0,
        name: '力(N)',
        nameLocation: 'middle',
      },
      series: [
        {
          name: '食指',
          type: 'line',
          data: seriesData.ff,
          smooth: true,
          showSymbol: false,
          itemStyle: {color: '#5470C6'}
        },
        {
          name: '中指',
          type: 'line',
          data: seriesData.mf,
          smooth: true,
          showSymbol: false,
          itemStyle: {color: '#91CC75'}
        },
        {
          name: '拇指',
          type: 'line',
          data: seriesData.th,
          smooth: true,
          showSymbol: false,
          itemStyle: {color: '#FAC858'}
        }
      ]
    };
    myChart.setOption(option);
  }
};

const formatTime = (timestamp: number) => {
  if (!timestamp && timestamp !== 0) return '';
  const ts = Number(timestamp);
  const date = new Date(ts > 1e12 ? ts : ts * 1000);
  const h = date.getHours().toString().padStart(2, '0');
  const m = date.getMinutes().toString().padStart(2, '0');
  const s = date.getSeconds().toString().padStart(2, '0');
  const ms = date.getMilliseconds().toString().padStart(3, '0').slice(0, 2);
  return `${h}:${m}:${s}.${ms}`;
};

const handleDataUpdate = (res: any) => {
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
};

const startRenderingLoop = () => {
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
      xAxis: {data: axisData},
      series: [
        {data: seriesData.ff},
        {data: seriesData.mf},
        {data: seriesData.th}
      ]
    });

  }, RENDER_INTERVAL);
};

onMounted(() => {
  initChart();
  const conn = connectRobotWebSocket(handleDataUpdate);
  socket = conn.socket;
  wsClose = conn.close;

  startRenderingLoop();

  window.addEventListener('resize', () => myChart?.resize());
});

onUnmounted(() => {
  if (renderIntervalTimer) clearInterval(renderIntervalTimer);
  wsClose?.();
  socket = null;
  myChart?.dispose();
});
</script>


<template>
  <div ref="chartRef" class="chart-container"></div>
</template>

<style scoped>
.chart-container {
  width: 100%;
  height: 400px;
}
</style>