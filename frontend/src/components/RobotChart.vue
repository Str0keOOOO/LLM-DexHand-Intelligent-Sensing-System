<script setup lang="ts">
import { ref, onMounted, onUnmounted, watch } from 'vue';
import * as echarts from 'echarts';
import { useRobot } from '@/composable/hooks/useRobot';
import type {HistoryData} from "@/composable/interfaces/Inter2Robot.ts";

const { robotState } = useRobot();

const postureChartRef = ref<HTMLElement | null>(null); // 姿态 (柱状)
const radarChartRef = ref<HTMLElement | null>(null);   // 受力分布 (雷达)
const trendChartRef = ref<HTMLElement | null>(null);   // 抓取力趋势 (折线)
const healthChartRef = ref<HTMLElement | null>(null);  // 电机负载 (热力/柱状)

let postureChart: echarts.ECharts | null = null;
let radarChart: echarts.ECharts | null = null;
let trendChart: echarts.ECharts | null = null;
let healthChart: echarts.ECharts | null = null;

const MAX_HISTORY_LEN = 50;

const historyData: HistoryData = {
  time: [],
  leftForce: [],
  rightForce: [],
};

const FINGER_NAMES = ['Thumb', 'Index', 'Middle', 'Ring', 'Pinky'];
const MOTOR_NAMES = [
  'TH_DIP', 'TH_MCP', 'TH_ROT',
  'FF_SPR', 'FF_DIP', 'FF_MCP',
  'MF_DIP', 'MF_MCP',
  'RF_DIP', 'RF_MCP',
  'LF_DIP', 'LF_MCP'
];

const extractMotorAngles = (motorArray: number[]) => {
  if (!motorArray || motorArray.length < 84) return new Array(12).fill(0);

  const angles: number[] = [];
  for (let i = 0; i < 12; i++) {
    const v = motorArray[i * 7 + 1] ?? 0;
    angles.push(Number(v.toFixed(1)));
  }
  return angles;
};

const extractMotorCurrents = (motorArray: number[]) => {
  if (!motorArray || motorArray.length < 84) return new Array(12).fill(0);

  const currents: number[] = [];
  for (let i = 0; i < 12; i++) {
    const v = motorArray[i * 7 + 3] ?? 0;
    currents.push(Math.abs(Number(v.toFixed(1))));
  }
  return currents;
};

const extractFingerForces = (touchArray: number[]) => {
  if (!touchArray || touchArray.length < 40) return [0, 0, 0, 0, 0];

  const forces: number[] = [];
  for (let i = 0; i < 5; i++) {
    const v = touchArray[i * 8 + 1] ?? 0;
    forces.push(Number(v.toFixed(2)));
  }
  return forces;
};

const initCharts = () => {
  if (postureChartRef.value) {
    postureChart = echarts.init(postureChartRef.value);
    postureChart.setOption({
      title: { text: 'Joint Angles (Deg)', left: 'center', textStyle: { fontSize: 14 } },
      tooltip: { trigger: 'axis', axisPointer: { type: 'shadow' } },
      legend: { bottom: 0, data: ['Left Hand', 'Right Hand'] },
      grid: { left: '3%', right: '4%', bottom: '10%', containLabel: true },
      xAxis: {
        type: 'category',
        data: MOTOR_NAMES,
        axisLabel: { interval: 0, rotate: 45, fontSize: 10 }
      },
      yAxis: { type: 'value', max: 120 }, // 固定最大值防止跳动
      series: [
        { name: 'Left Hand', type: 'bar', itemStyle: { color: '#3b82f6' }, data: [] },
        { name: 'Right Hand', type: 'bar', itemStyle: { color: '#8b5cf6' }, data: [] }
      ]
    });
  }

  if (radarChartRef.value) {
    radarChart = echarts.init(radarChartRef.value);
    radarChart.setOption({
      title: { text: 'Fingertip Forces (N)', left: 'center', textStyle: { fontSize: 14 } },
      tooltip: {},
      radar: {
        indicator: FINGER_NAMES.map(name => ({ name, max: 5.0 })), // 最大 5N
      },
      series: [{
        type: 'radar',
        data: [
          { value: [0,0,0,0,0], name: 'Left Hand', itemStyle: { color: '#3b82f6' } },
          { value: [0,0,0,0,0], name: 'Right Hand', itemStyle: { color: '#8b5cf6' } }
        ]
      }]
    });
  }

  if (trendChartRef.value) {
    trendChart = echarts.init(trendChartRef.value);
    trendChart.setOption({
      title: { text: 'Total Force Trend', left: 'center', textStyle: { fontSize: 14 } },
      tooltip: { trigger: 'axis' },
      legend: { bottom: 0, data: ['Left Total', 'Right Total'] },
      grid: { left: '3%', right: '4%', bottom: '10%', containLabel: true },
      xAxis: { type: 'category', boundaryGap: false, data: [] },
      yAxis: { type: 'value' },
      series: [
        { name: 'Left Total', type: 'line', smooth: true, areaStyle: { opacity: 0.2 }, data: [] },
        { name: 'Right Total', type: 'line', smooth: true, areaStyle: { opacity: 0.2 }, data: [] }
      ]
    });
  }

  if (healthChartRef.value) {
    healthChart = echarts.init(healthChartRef.value);
    healthChart.setOption({
      title: { text: 'Motor Load (mA)', left: 'center', textStyle: { fontSize: 14 } },
      tooltip: { trigger: 'axis' },
      legend: { bottom: 0, data: ['Left Hand', 'Right Hand'] },
      grid: { left: '3%', right: '4%', bottom: '10%', containLabel: true },
      xAxis: { type: 'value' },
      yAxis: { type: 'category', data: MOTOR_NAMES, inverse: true },
      series: [
        { name: 'Left Hand', type: 'bar', stack: null, itemStyle: { color: '#60a5fa' }, data: [] },
        { name: 'Right Hand', type: 'bar', stack: null, itemStyle: { color: '#a78bfa' }, data: [] }
      ]
    });
  }
};

watch(() => robotState.value, (newVal) => {
  if (!newVal || !postureChart) return;

  const leftAngles = extractMotorAngles(newVal.left.motor);
  const rightAngles = extractMotorAngles(newVal.right.motor);
  const leftForces = extractFingerForces(newVal.left.touch);
  const rightForces = extractFingerForces(newVal.right.touch);
  const leftCurrents = extractMotorCurrents(newVal.left.motor);
  const rightCurrents = extractMotorCurrents(newVal.right.motor);

  postureChart.setOption({ series: [{ data: leftAngles }, { data: rightAngles }] });

  radarChart?.setOption({
    series: [{ data: [
        { value: leftForces, name: 'Left Hand' },
        { value: rightForces, name: 'Right Hand' }
      ]}]
  });

  healthChart?.setOption({ series: [{ data: leftCurrents }, { data: rightCurrents }] });

  const nowStr = new Date().toLocaleTimeString();
  const leftTotal = leftForces.reduce((a, b) => a + b, 0);
  const rightTotal = rightForces.reduce((a, b) => a + b, 0);

  historyData.time.push(nowStr);
  historyData.leftForce.push(leftTotal);
  historyData.rightForce.push(rightTotal);

  if (historyData.time.length > MAX_HISTORY_LEN) {
    historyData.time.shift();
    historyData.leftForce.shift();
    historyData.rightForce.shift();
  }

  trendChart?.setOption({
    xAxis: { data: historyData.time },
    series: [{ data: historyData.leftForce }, { data: historyData.rightForce }]
  });

}, { deep: true });

const resizeHandler = () => {
  postureChart?.resize(); radarChart?.resize(); trendChart?.resize(); healthChart?.resize();
};
onMounted(() => { initCharts(); window.addEventListener('resize', resizeHandler); });
onUnmounted(() => { window.removeEventListener('resize', resizeHandler); postureChart?.dispose(); });
</script>

<template>
  <div class="charts-dashboard">
    <div class="chart-row">
      <div class="chart-card"><div ref="radarChartRef" class="chart-content"></div></div>
      <div class="chart-card"><div ref="postureChartRef" class="chart-content"></div></div>
    </div>
    <div class="chart-row">
      <div class="chart-card"><div ref="trendChartRef" class="chart-content"></div></div>
      <div class="chart-card"><div ref="healthChartRef" class="chart-content"></div></div>
    </div>
  </div>
</template>

<style scoped>
.charts-dashboard { display: flex; flex-direction: column; gap: 16px; width: 100%; height: 100%; }
.chart-row { display: flex; gap: 16px; flex: 1; min-height: 280px; }
.chart-card { flex: 1; background: #fff; border-radius: 12px; padding: 12px; box-shadow: 0 2px 8px rgba(0,0,0,0.04); border: 1px solid #f1f5f9; display: flex; flex-direction: column; }
.chart-content { flex: 1; width: 100%; min-height: 0; }
</style>