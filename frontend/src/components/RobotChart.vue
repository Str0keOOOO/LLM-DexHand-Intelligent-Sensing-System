<script setup lang="ts">
import { onUnmounted } from 'vue';
import {useRobotSocket, useRobotChart } from '@/composable/hooks/useRobotChart';

const { chartRef, pushData, disposeChart } = useRobotChart({
  autoInit: true,
  autoStart: true
});

// 显式类型 msg: any，使用 hook 返回的 connect/close 管理连接
const { connect, close } = useRobotSocket((msg: any) => pushData(msg));

// 建立连接（可选地保存返回的 WebSocket）
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
