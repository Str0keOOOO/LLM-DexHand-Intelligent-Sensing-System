<script setup lang="ts">
import {onUnmounted} from 'vue';
import {useRobotChart} from '@/composable/hooks/useRobotChart';
import {connectRobotWebSocket} from '@/composable/api/Chat2Robot';

const {chartRef, pushData, disposeChart} = useRobotChart({
  autoInit: true,
  autoStart: true
});

const conn = connectRobotWebSocket((msg) => pushData(msg));

onUnmounted(() => {
  conn.close();
  disposeChart();
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