<script setup lang="ts">
import { ref, onMounted, computed } from 'vue'
import { getSensorHistory, type SensorItem } from '@/composable/api/DBApi'
import { ElMessage } from 'element-plus'

// 响应式状态
const tableData = ref<SensorItem[]>([])
const loading = ref(false)
const selectedHand = ref('right')
const queryMinutes = ref(1)

/**
 * 获取后端数据
 */
const fetchData = async () => {
  loading.value = true
  try {
    // 调用 DBApi.ts 中的接口
    const res = await getSensorHistory(queryMinutes.value, selectedHand.value)

    if (res && res.data) {
      // 核心：按时间倒序排列（最新数据在最上面）
      tableData.value = res.data.sort((a, b) =>
          new Date(b.time).getTime() - new Date(a.time).getTime()
      )

      if (tableData.value.length === 0) {
        ElMessage.info('该时间段内暂无传感器数据')
      }
    }
  } catch (error) {
    console.error('Failed to fetch sensor history:', error)
    ElMessage.error('获取历史数据失败，请检查后端服务')
  } finally {
    loading.value = false
  }
}

/**
 * 导出 CSV 功能
 */
const downloadCSV = () => {
  if (tableData.value.length === 0) return

  const headers = ['Time', 'Hand', 'Field', 'Value']
  const rows = tableData.value.map(row => [
    new Date(row.time).toLocaleString(),
    selectedHand.value,
    row.field,
    row.value
  ])

  const csvContent = [
    headers.join(','),
    ...rows.map(r => r.join(','))
  ].join('\n')

  const blob = new Blob([csvContent], { type: 'text/csv;charset=utf-8;' })
  const url = URL.createObjectURL(blob)
  const link = document.createElement('a')
  link.href = url
  link.setAttribute('download', `sensor_${selectedHand.value}_${queryMinutes.value}min.csv`)
  document.body.appendChild(link)
  link.click()
  document.body.removeChild(link)
}

// 初始化加载
onMounted(() => {
  fetchData()
})
</script>

<template>
  <div class="history-panel">
    <div class="header">
      <div class="title-area">
        <h3>📈 Sensor History</h3>
        <span class="status-tag" :class="selectedHand">
          {{ selectedHand.toUpperCase() }} ({{ tableData.length }} rows)
        </span>
      </div>

      <div class="controls">
        <div class="select-group">
          <label>Hand:</label>
          <select v-model="selectedHand" @change="fetchData" :disabled="loading">
            <option value="right">Right Hand</option>
            <option value="left">Left Hand</option>
          </select>
        </div>

        <div class="select-group">
          <label>Range:</label>
          <select v-model="queryMinutes" @change="fetchData" :disabled="loading">
            <option :value="1">Last 1 Min</option>
            <option :value="5">Last 5 Mins</option>
            <option :value="10">Last 10 Mins</option>
            <option :value="30">Last 30 Mins</option>
          </select>
        </div>

        <button @click="fetchData" :disabled="loading" class="refresh-btn">
          {{ loading ? 'Loading...' : 'Refresh' }}
        </button>

        <button @click="downloadCSV" class="download-btn" :disabled="tableData.length === 0 || loading">
          ⬇ CSV
        </button>
      </div>
    </div>

    <div class="table-container" v-loading="loading">
      <table>
        <thead>
        <tr>
          <th>Time</th>
          <th>Side</th>
          <th>Sensor ID (Field)</th>
          <th>Value</th>
        </tr>
        </thead>
        <tbody>
        <tr v-for="(item, index) in tableData" :key="index">
          <td class="time">{{ new Date(item.time).toLocaleTimeString() }}</td>
          <td class="side-cell">
            <span :class="selectedHand">{{ selectedHand === 'left' ? 'L' : 'R' }}</span>
          </td>
          <td class="field">{{ item.field }}</td>
          <td class="value" :style="{ color: item.value > 0.5 ? '#fc8181' : '#63b3ed' }">
            {{ item.value.toFixed(4) }}
          </td>
        </tr>
        <tr v-if="tableData.length === 0 && !loading">
          <td colspan="4" class="no-data">No Data Found in InfluxDB</td>
        </tr>
        </tbody>
      </table>
    </div>
  </div>
</template>

<style scoped>
.history-panel {
  background: #1a1a1a;
  border-radius: 12px;
  padding: 1.2rem;
  color: #e0e0e0;
  border: 1px solid #333;
  box-shadow: 0 4px 20px rgba(0, 0, 0, 0.3);
}

.header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  margin-bottom: 1.2rem;
  flex-wrap: wrap;
  gap: 15px;
}

.title-area {
  display: flex;
  align-items: center;
  gap: 12px;
}

.status-tag {
  font-size: 0.75rem;
  padding: 2px 8px;
  border-radius: 4px;
  background: #333;
}
.status-tag.left { color: #63b3ed; border: 1px solid #2c5282; }
.status-tag.right { color: #f6ad55; border: 1px solid #9c4221; }

.controls {
  display: flex;
  gap: 12px;
  align-items: center;
}

.select-group {
  display: flex;
  align-items: center;
  gap: 6px;
  font-size: 0.85rem;
}

select, .refresh-btn {
  padding: 6px 10px;
  background: #2d2d2d;
  border: 1px solid #444;
  color: white;
  border-radius: 6px;
  cursor: pointer;
  transition: all 0.2s;
}

select:hover, .refresh-btn:hover {
  border-color: #666;
  background: #3d3d3d;
}

.download-btn {
  background: #2c5282 !important;
  border-color: #2b6cb0 !important;
  color: white;
  padding: 6px 12px;
  border-radius: 6px;
  cursor: pointer;
}

.download-btn:disabled {
  background: #444 !important;
  cursor: not-allowed;
  opacity: 0.6;
}

.table-container {
  max-height: 450px;
  overflow-y: auto;
  border: 1px solid #333;
  border-radius: 8px;
  background: #252525;
}

table {
  width: 100%;
  border-collapse: collapse;
  font-size: 0.85rem;
  text-align: left;
}

th {
  background: #333;
  padding: 12px 10px;
  position: sticky;
  top: 0;
  z-index: 1;
  font-weight: 600;
  color: #bbb;
}

td {
  padding: 10px;
  border-bottom: 1px solid #333;
}

.time {
  color: #888;
  font-family: 'Courier New', Courier, monospace;
}

.side-cell span.left { color: #63b3ed; }
.side-cell span.right { color: #f6ad55; }

.no-data {
  text-align: center;
  color: #666;
  padding: 40px 0;
}

/* 滚动条美化 */
.table-container::-webkit-scrollbar {
  width: 6px;
}
.table-container::-webkit-scrollbar-thumb {
  background: #444;
  border-radius: 3px;
}
</style>