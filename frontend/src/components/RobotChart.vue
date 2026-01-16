<script setup lang="ts">
import { onMounted, ref, onUnmounted } from 'vue';
import * as echarts from 'echarts';
import { getRobotStatus } from '@/composable/api/Chat2Robot.ts';

const chartRef = ref<HTMLElement | null>(null);
let myChart: echarts.ECharts | null = null;
let timer: any = null;

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
  // Python time.time() 返回的是秒，JS 需要毫秒
  const date = new Date(timestamp * 1000);
  const h = date.getHours().toString().padStart(2, '0');
  const m = date.getMinutes().toString().padStart(2, '0');
  const s = date.getSeconds().toString().padStart(2, '0');
  return `${h}:${m}:${s}`;
};

const fetchDataAndUpdate = async () => {
  try {
    const res = await getRobotStatus();

    if (myChart && res && res.fingers) {
      const option = myChart.getOption() as any;

      // 获取当前数据队列
      const data0 = (option.series?.[0]?.data as number[]) ?? [];
      const data1 = (option.series?.[1]?.data as number[]) ?? [];
      const data2 = (option.series?.[2]?.data as number[]) ?? [];
      const axisData = (option.xAxis?.[0]?.data as string[]) ?? [];

      // 保持最近 50 个点，防止内存溢出
      if (data0.length > 50) {
        data0.shift(); data1.shift(); data2.shift(); axisData.shift();
      }

      // 1. 处理时间轴
      const timeStr = formatTime(res.timestamp);
      axisData.push(timeStr);

      // 2. 处理传感器数据
      // 确保数据存在，如果某根手指没数据则补 0
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
  } catch (error) {
    // 偶尔的请求失败不打印 error，避免刷屏，除非是为了调试
    // console.error("Error fetching robot status:", error);
  }
};

onMounted(() => {
  initChart();
  // 每 100ms 刷新一次，动画更丝滑
  timer = setInterval(fetchDataAndUpdate, 100);
  window.addEventListener('resize', () => myChart?.resize());
});

onUnmounted(() => {
  clearInterval(timer);
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