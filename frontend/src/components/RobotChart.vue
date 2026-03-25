<script setup lang="ts">
import {ref, onMounted, onUnmounted, watch} from 'vue';
import * as echarts from 'echarts';
import {useHand} from '@/composable/hooks/useHand.ts';

// 使用 hook 暴露的 handState
const {handState} = useHand();

const postureChartRef = ref<HTMLElement | null>(null);
const velocityChartRef = ref<HTMLElement | null>(null);
const mAngleRef = ref<HTMLElement | null>(null);
const mEncRef = ref<HTMLElement | null>(null);
const mCurRef = ref<HTMLElement | null>(null);
const mVelRef = ref<HTMLElement | null>(null);
const mErrRef = ref<HTMLElement | null>(null);
const mImpRef = ref<HTMLElement | null>(null);
const normForceRef = ref<HTMLElement | null>(null);
const normDeltaRef = ref<HTMLElement | null>(null);
const tangForceRef = ref<HTMLElement | null>(null);
const tangDeltaRef = ref<HTMLElement | null>(null);
const dirRef = ref<HTMLElement | null>(null);
const proxRef = ref<HTMLElement | null>(null);
const tempRef = ref<HTMLElement | null>(null);

const charts: Record<string, echarts.ECharts | null> = {};

const FINGER_NAMES = ['Thumb', 'Index', 'Middle', 'Ring', 'Pinky'];
const MOTOR_NAMES = ['TH_DIP', 'TH_MCP', 'TH_ROT', 'FF_SPR', 'FF_DIP', 'FF_MCP', 'MF_DIP', 'MF_MCP', 'RF_DIP', 'RF_MCP', 'LF_DIP', 'LF_MCP'];
const SEMANTIC_ORDER = ["th_dip", "th_mcp", "th_rot", "ff_spr", "ff_dip", "ff_mcp", "mf_dip", "mf_mcp", "rf_dip", "rf_mcp", "lf_dip", "lf_mcp"];

function getOrderedData(map: Record<string, number> = {}) {
  return SEMANTIC_ORDER.map((k) => Number((map[k] || 0).toFixed(1)));
}

function createBarChart(dom: HTMLElement | null, title: string, xAxisData: string[], color: string, min: number = 0, max: number = 100) {
  if (!dom) return null;
  const chart = echarts.init(dom);
  chart.setOption({
    animation: false,
    title: {text: title, left: 'center', textStyle: {fontSize: 13, color: '#475569'}},
    tooltip: {trigger: 'axis'},
    grid: {left: '8%', right: '4%', bottom: '15%', top: '25%', containLabel: true},
    xAxis: {
      type: 'category',
      data: xAxisData,
      axisLabel: {interval: 0, rotate: xAxisData.length > 5 ? 45 : 0, fontSize: 10}
    },
    yAxis: {type: 'value', min: min, max: max},
    series: [{type: 'bar', itemStyle: {color, borderRadius: [2, 2, 0, 0]}, data: [], barMaxWidth: 25}]
  });
  return chart;
}

function initCharts() {
  charts.posture = createBarChart(postureChartRef.value, 'Joint Positions (Deg)', MOTOR_NAMES, '#8b5cf6', 0, 90);
  // charts.velocity = createBarChart(velocityChartRef.value, 'Joint Velocities (Deg/s)', MOTOR_NAMES, '#10b981', -5, 5);
  charts.normF = createBarChart(normForceRef.value, 'Normal Force (N)', FINGER_NAMES, '#3b82f6', 0, 20);
  // charts.normD = createBarChart(normDeltaRef.value, 'Normal Force Delta', FINGER_NAMES, '#60a5fa', 0, 30000000);
  charts.tangF = createBarChart(tangForceRef.value, 'Tangential Force (N)', FINGER_NAMES, '#0ea5e9', 0, 20);
  // charts.tangD = createBarChart(tangDeltaRef.value, 'Tangential Force Delta', FINGER_NAMES, '#38bdf8', 0, 30000000);
  charts.dir = createBarChart(dirRef.value, 'Direction (0-359°)', FINGER_NAMES, '#8b5cf6', 0, 360);
  charts.prox = createBarChart(proxRef.value, 'Proximity', FINGER_NAMES, '#f97316', 0, 4060);
  charts.temp = createBarChart(tempRef.value, 'Temperature (°C)', FINGER_NAMES, '#ef4444', 0, 40);
  // charts.mAngle = createBarChart(mAngleRef.value, 'Motor Angle (Deg)', MOTOR_NAMES, '#6366f1',);
  // charts.mVel = createBarChart(mVelRef.value, 'Motor Velocity (rpm)', MOTOR_NAMES, '#14b8a6');
  // charts.mCur = createBarChart(mCurRef.value, 'Motor Current (mA)', MOTOR_NAMES, '#f59e0b');
  // charts.mEnc = createBarChart(mEncRef.value, 'Encoder Position', MOTOR_NAMES, '#3b82f6');
  // charts.mErr = createBarChart(mErrRef.value, 'Error Code (0=Normal)', MOTOR_NAMES, '#ef4444');
  // charts.mImp = createBarChart(mImpRef.value, 'Impedance', MOTOR_NAMES, '#a855f7');
}
watch(() => handState.value, (newVal) => {
  if (!newVal || !newVal.right || !newVal.right.motor || !newVal.right.touch) return;
  const r = newVal.right;
  const m = r.motor;
  const t = r.touch;
  console.log(handState)
  charts.posture?.setOption({series: [{data: getOrderedData(r.joint.position)}]});
  // charts.velocity?.setOption({series: [{data: getOrderedData(r.joint.velocity)}]});
  // charts.mAngle?.setOption({series: [{data: m.angle || []}]});
  // charts.mVel?.setOption({series: [{data: m.velocity || []}]});
  // charts.mCur?.setOption({series: [{data: m.current || []}]});
  // charts.mEnc?.setOption({series: [{data: m.encoder_position || []}]});
  // charts.mErr?.setOption({series: [{data: m.error_code || []}]});
  // charts.mImp?.setOption({series: [{data: m.impedance || []}]});
  charts.normF?.setOption({series: [{data: t.normal_force || []}]});
  // charts.normD?.setOption({series: [{data: t.normal_force_delta || []}]});
  charts.tangF?.setOption({series: [{data: t.tangential_force || []}]});
  // charts.tangD?.setOption({series: [{data: t.tangential_force_delta || []}]});
  charts.dir?.setOption({series: [{data: t.direction || []}]});
  charts.prox?.setOption({series: [{data: t.proximity || []}]});
  charts.temp?.setOption({series: [{data: t.temperature || []}]});
});

function resizeHandler() {
  Object.values(charts).forEach(c => c?.resize());
}

onMounted(() => {
  initCharts();
  window.addEventListener('resize', resizeHandler);
});
onUnmounted(() => {
  window.removeEventListener('resize', resizeHandler);
  Object.values(charts).forEach(c => c?.dispose());
});
</script>

<template>
  <div class="charts-dashboard">

    <div class="section-title">
      <span class="indicator semantic-indicator"></span> 语义关节状态 (Semantic Joints)
    </div>
    <div class="chart-row">
      <div class="chart-card">
        <div ref="postureChartRef" class="chart-content"></div>
      </div>
    </div>

    <div class="section-title" style="margin-top: 10px;">
      <span class="indicator touch-indicator"></span> 触觉传感反馈 (Touch Sensors)
    </div>
    <div class="chart-row">
      <div class="chart-card">
        <div ref="normForceRef" class="chart-content"></div>
      </div>
      <div class="chart-card">
        <div ref="tangForceRef" class="chart-content"></div>
      </div>
    </div>
    <div class="chart-row">
      <div class="chart-card">
        <div ref="dirRef" class="chart-content"></div>
      </div>
      <div class="chart-card">
        <div ref="proxRef" class="chart-content"></div>
      </div>
    </div>
    <div class="chart-row">
      <div class="chart-card" style="max-width: 33.33%;">
        <div ref="tempRef" class="chart-content"></div>
      </div>
    </div>

  </div>
</template>

<style scoped>
.charts-dashboard {
  display: flex;
  flex-direction: column;
  gap: 12px;
  width: 100%;
  padding-bottom: 30px;
}

.section-title {
  font-size: 16px;
  font-weight: 600;
  color: #1e293b;
  display: flex;
  align-items: center;
  gap: 8px;
  padding-bottom: 4px;
  border-bottom: 2px solid #e2e8f0;
}

.indicator {
  width: 8px;
  height: 16px;
  border-radius: 4px;
}

.semantic-indicator {
  background-color: #10b981;
}

.motor-indicator {
  background-color: #8b5cf6;
}

.touch-indicator {
  background-color: #3b82f6;
}

.chart-row {
  display: flex;
  gap: 16px;
  height: 220px;
}

.chart-card {
  flex: 1;
  background: #fff;
  border-radius: 12px;
  padding: 8px;
  box-shadow: 0 1px 4px rgba(0, 0, 0, 0.05);
  border: 1px solid #f1f5f9;
  display: flex;
  flex-direction: column;
}

.chart-content {
  flex: 1;
  width: 100%;
  height: 100%;
}
</style>