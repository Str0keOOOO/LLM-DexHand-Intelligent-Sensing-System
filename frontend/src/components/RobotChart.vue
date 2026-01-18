<script setup lang="ts">
import { onMounted, ref, onUnmounted } from 'vue';
import * as echarts from 'echarts';

// 移除旧的 HTTP API 引用
// import { getRobotStatus } from '@/composable/api/Chat2Robot.ts';

const chartRef = ref<HTMLElement | null>(null);
let myChart: echarts.ECharts | null = null;
let socket: WebSocket | null = null; // 新增 socket 实例

const initChart = () => {
  if (chartRef.value) {
    myChart = echarts.init(chartRef.value);
    const option = {
      title: { text: 'DexHand 关节实时载荷' },
      tooltip: { trigger: 'axis' },
      legend: {
        data: ['食指', '中指', '拇指'],
        right: '5%',
        top: '5%',
      },
      grid: { left: '3%', right: '4%', bottom: '3%', containLabel: true },
      xAxis: {
        type: 'category',
        boundaryGap: false,
        data: [] as string[]
      },
      yAxis: {
        type: 'value',
        min: 0,
        max: 10,
        name: '力 (N)'
      },
      series: [
        { name: '食指', type: 'line', data: [] as number[], smooth: true, showSymbol: false },
        { name: '中指', type: 'line', data: [] as number[], smooth: true, showSymbol: false },
        { name: '拇指', type: 'line', data: [] as number[], smooth: true, showSymbol: false }
      ]
    };
    myChart.setOption(option);
  }
};

// 辅助函数：格式化时间戳 (秒 -> HH:mm:ss)
const formatTime = (timestamp: number) => {
  if (!timestamp) return '';
  const date = new Date(timestamp * 1000);
  const h = date.getHours().toString().padStart(2, '0');
  const m = date.getMinutes().toString().padStart(2, '0');
  const s = date.getSeconds().toString().padStart(2, '0');
  return `${h}:${m}:${s}`;
};

// 新增：WebSocket 连接逻辑
const connectWebSocket = () => {
  // 自动判断 ws 或 wss，并使用当前 host（配合 Vite proxy 转发 /api）
  // const protocol = location.protocol === 'https:' ? 'wss:' : 'ws:';
  // const url = `${protocol}//${location.host}/api/ws/robot-data`;

  const url = 'ws://127.0.0.1:8000/api/ws/robot-data';

  console.log("Attempting to connect to:", url); // 添加日志方便调试
  socket = new WebSocket(url);

  socket.onopen = () => {
    console.log("Robot WebSocket Connected");
  };

  socket.onmessage = (event) => {
    try {
      const res = JSON.parse(event.data);

      // 过滤掉后端初始化时的非数据消息
      if (res.mode === 'BACKEND_INIT') return;

      // 适配数据结构：后端可能将数据包裹在 payload 中
      // 根据你的 bridge.py，数据可能直接在 res 里，也可能在 res.payload 里
      const dataContent = res.payload || res;

      handleDataUpdate(dataContent);

    } catch (err) {
      console.error("WS Parse Error:", err);
    }
  };

  socket.onclose = () => {
    console.log("Robot WebSocket Disconnected");
  };

  socket.onerror = (err) => {
    console.error("Robot WebSocket Error:", err);
  };
};

// 将原来的数据更新逻辑提取出来
const handleDataUpdate = (res: any) => {
  if (myChart && res && res.fingers) {
    const option = myChart.getOption() as any;

    // 获取当前数据队列
    const data0 = (option.series?.[0]?.data as number[]) ?? [];
    const data1 = (option.series?.[1]?.data as number[]) ?? [];
    const data2 = (option.series?.[2]?.data as number[]) ?? [];
    const axisData = (option.xAxis?.[0]?.data as string[]) ?? [];

    // 保持最近 50 个点
    if (data0.length > 50) {
      data0.shift(); data1.shift(); data2.shift(); axisData.shift();
    }

    // 1. 处理时间轴
    const timeStr = formatTime(res.timestamp);
    axisData.push(timeStr);

    // 2. 处理传感器数据
    data0.push(res.fingers[0] ?? 0);
    data1.push(res.fingers[1] ?? 0);
    data2.push(res.fingers[2] ?? 0);

    // 3. 更新图表
    myChart.setOption({
      xAxis: { data: axisData },
      series: [
        { data: data0 },
        { data: data1 },
        { data: data2 }
      ]
    });
  }
};

onMounted(() => {
  initChart();
  // 启动 WebSocket 连接，替代原来的 setInterval
  connectWebSocket();
  window.addEventListener('resize', () => myChart?.resize());
});

onUnmounted(() => {
  // 组件销毁时关闭连接
  if (socket) {
    socket.close();
  }
  window.removeEventListener('resize', () => myChart?.resize());
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