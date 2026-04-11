<script setup lang="ts">
import HandChart from '@/components/HandChart.vue'
import ArmChart from '@/components/ArmChart.vue'
import {TrendCharts} from "@element-plus/icons-vue"
import {useHand} from '@/composable/hooks/useHand.ts'

const {isConnected} = useHand()
</script>

<template>
  <div class="chart-page-container">
    <el-card class="chart-card" shadow="never">
      <template #header>
        <div class="card-header">
          <div class="header-left">
            <div class="icon-box warning">
              <el-icon>s
                <TrendCharts/>
              </el-icon>
            </div>
            <span class="title">灵巧手传感器数据监控</span>
          </div>
          <el-tag size="small" :type="isConnected ? 'success' : 'danger'" effect="plain">
            {{ isConnected ? '实时数据流' : '离线' }}
          </el-tag>
        </div>
      </template>
      <div class="chart-container">
        <HandChart/>
        <ArmChart/>
      </div>
    </el-card>
  </div>
</template>

<style scoped lang="scss">
.chart-page-container {
  padding: 24px;
  background-color: #f5f7fa;
  min-height: calc(100vh - 84px);
  box-sizing: border-box;
}

.card-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  padding: 12px 0;

  .header-left {
    display: flex;
    align-items: center;
    gap: 10px;

    .icon-box {
      width: 32px;
      height: 32px;
      border-radius: 8px;
      display: flex;
      justify-content: center;
      align-items: center;
      background: linear-gradient(135deg, #fef3c7 0%, #fde68a 100%);
      color: #d97706;
    }

    .title {
      font-weight: 600;
      font-size: 16px;
      color: #1f2937;
    }
  }
}

.chart-card {
  border: none;
  border-radius: 16px;
  box-shadow: 0 4px 6px -1px rgba(0, 0, 0, 0.05);
  height: calc(100vh - 140px);
  display: flex;
  flex-direction: column;

  :deep(.el-card__body) {
    flex: 1;
    display: flex;
    flex-direction: column;
    padding: 20px;
  }
}

.chart-container {
  flex: 1;
  min-height: 500px;
}
</style>