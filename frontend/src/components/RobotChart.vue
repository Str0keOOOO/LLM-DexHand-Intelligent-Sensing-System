<script setup lang="ts">
import { onUnmounted } from 'vue';
import {useRobotSocket, useRobot } from '@/composable/hooks/useRobot.ts';

const { chartRef, pushData, disposeChart } = useRobot({
  autoInit: true,
  autoStart: true
});

const { connect, close } = useRobotSocket((msg: any) => pushData(msg));

connect()

onUnmounted(() => {
  close();
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
