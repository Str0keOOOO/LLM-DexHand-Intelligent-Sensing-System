<script setup lang="ts">
import { onMounted, ref, onUnmounted } from 'vue';
import * as echarts from 'echarts';
import { getRobotStatus } from '@/composable/api/robot';

const chartRef = ref<HTMLElement | null>(null);
let myChart: echarts.ECharts | null = null;
let timer: any = null;

const initChart = () => {
  if (chartRef.value) {
    myChart = echarts.init(chartRef.value);
    const option = {
      title: { text: 'DexHand 关节实时载荷 (ROS2 数据)' },
      tooltip: { trigger: 'axis' },
      legend: {
        data: ['食指', '中指', '拇指'],
        right: '5%',  // 距离右侧的距离
        top: '5%',     // 距离顶部的距离
        orient: 'vertical' // 如果想让三个标签垂直排列，可以加这一行；水平排列则删掉
      },
      grid: { left: '3%', right: '4%', bottom: '3%', containLabel: true },
      xAxis: { type: 'category', boundaryGap: false, data: [] as string[] },
      yAxis: { type: 'value', min: 0, max: 10 }, // 固定一下 Y 轴范围看波动更清晰
      series: [
        { name: '食指', type: 'line', data: [] as number[], smooth: true, showSymbol: false },
        { name: '中指', type: 'line', data: [] as number[], smooth: true, showSymbol: false },
        { name: '拇指', type: 'line', data: [] as number[], smooth: true, showSymbol: false }
      ]
    };
    myChart.setOption(option);
  }
};

// 从后端获取数据并更新图表
const fetchDataAndUpdate = async () => {
  try {
    const res = await getRobotStatus(); // 调用接口

    if (myChart) {
      const option = myChart.getOption() as any;
      const data0 = option.series[0].data; // 食指
      const data1 = option.series[1].data; // 中指
      const data2 = option.series[2].data; // 拇指
      const axisData = option.xAxis[0].data;

      // 保持最近 30 个点
      if (data0.length > 30) {
        data0.shift(); data1.shift(); data2.shift(); axisData.shift();
      }

      // 使用后端返回的时间戳和数据
      axisData.push(res.timestamp);

      // 注意：res.fingers 是 [食指, 中指, 拇指]
      data0.push(res.fingers[0]);
      data1.push(res.fingers[1]);
      data2.push(res.fingers[2]);

      myChart.setOption({
        xAxis: { data: axisData },
        series: [
          { data: data0 }, { data: data1 }, { data: data2 }
        ]
      });
    }
  } catch (error) {
    console.error("获取机器人状态失败:", error);
  }
};

onMounted(() => {
  initChart();
  // 每 200ms 轮询一次，实现准实时效果
  timer = setInterval(fetchDataAndUpdate, 200);
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
  height: 350px;
}
</style>