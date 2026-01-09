<script setup lang="ts">
import { onMounted, ref, onUnmounted } from 'vue';
import * as echarts from 'echarts';

const chartRef = ref<HTMLElement | null>(null);
let myChart: echarts.ECharts | null = null;

// 模拟实时数据更新
let timer: any = null;

const initChart = () => {
  if (chartRef.value) {
    myChart = echarts.init(chartRef.value);
    const option = {
      title: { text: '灵巧手关节实时载荷 (模拟数据)' },
      tooltip: { trigger: 'axis' },
      legend: { data: ['食指', '中指', '拇指'] },
      grid: { left: '3%', right: '4%', bottom: '3%', containLabel: true },
      xAxis: { type: 'category', boundaryGap: false, data: [] as string[] },
      yAxis: { type: 'value' },
      series: [
        { name: '食指', type: 'line', data: [] as number[], smooth: true },
        { name: '中指', type: 'line', data: [] as number[], smooth: true },
        { name: '拇指', type: 'line', data: [] as number[], smooth: true }
      ]
    };
    myChart.setOption(option);
  }
};

// 模拟数据生成
const updateData = () => {
  const now = new Date();
  const timeStr = [now.getHours(), now.getMinutes(), now.getSeconds()].join(':');

  if (myChart) {
    const option = myChart.getOption() as any;
    const data0 = option.series[0].data;
    const data1 = option.series[1].data;
    const data2 = option.series[2].data;
    const axisData = option.xAxis[0].data;

    if (data0.length > 20) {
      data0.shift(); data1.shift(); data2.shift(); axisData.shift();
    }

    axisData.push(timeStr);
    data0.push((Math.random() * 10 + 5).toFixed(2));
    data1.push((Math.random() * 10 + 2).toFixed(2));
    data2.push((Math.random() * 10).toFixed(2));

    myChart.setOption({
      xAxis: { data: axisData },
      series: [
        { data: data0 }, { data: data1 }, { data: data2 }
      ]
    });
  }
};

onMounted(() => {
  initChart();
  timer = setInterval(updateData, 1000); // 每秒刷新
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