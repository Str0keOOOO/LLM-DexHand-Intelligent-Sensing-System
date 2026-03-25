<script setup lang="ts">
import {ref, onMounted} from 'vue'
import {useDB} from '@/composable/hooks/useDB'
import type {MeasurementType, SensorItem} from '@/composable/types/robot'
import {ElMessage} from 'element-plus'

const {isLoadingSensor, fetchSensorHistory} = useDB()

const tableData = ref<SensorItem[]>([])
const selectedHand = ref('right')
const queryMinutes = ref(1)
const selectedCategory = ref<MeasurementType>('dexhand_joints')

async function fetchData() {
  try {
    const res = await fetchSensorHistory(
        selectedCategory.value,
        queryMinutes.value
    )
    if (res && res.data) {
      tableData.value = res.data // 后端已完成排序
      if (tableData.value.length === 0) ElMessage.info('该时间段内暂无数据')
    }
  } catch (error) {
    // Error 已经在 hook 中处理，这里只需空捕获即可
  }
}

function formatDataSummary(item: any) {
  if (selectedCategory.value.includes('joints')) {
    return Object.entries(item.data).map(([k, v]) => `${k}:${Number(v).toFixed(1)}°`).join(' | ')
  }
  if (selectedCategory.value === 'dexhand_touch') {
    return `Normal Force: [${item.data.normal_force.map((v: number) => v.toFixed(2)).join(', ')}]`
  }
  if (selectedCategory.value === 'dexhand_motor') {
    return `Angles: [${item.data.angle.map((v: number) => v.toFixed(1)).join(', ')}]`
  }
  return JSON.stringify(item.data)
}

function downloadCSV() {
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

  const blob = new Blob([csvContent], {type: 'text/csv;charset=utf-8;'})
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
  <div class="controls">
    <div class="select-group">
      <label>Category:</label>
      <select v-model="selectedCategory" @change="fetchData" :disabled="isLoadingSensor">
        <option value="dexhand_joints">Semantic Joints</option>
        <option value="dexhand_touch">Touch Sensors</option>
        <option value="dexhand_motor">Motor Feedback</option>
      </select>
    </div>
  </div>

  <div class="table-container">
    <table>
      <thead>
      <tr>
        <th>Time</th>
        <th>Category</th>
        <th>Data Summary (Structured)</th>
      </tr>
      </thead>
      <tbody>
      <tr v-for="(item, index) in tableData" :key="index">
        <td class="time">{{ new Date(item.time).toLocaleTimeString() }}</td>
        <td>
          <el-tag size="small">{{ selectedCategory }}</el-tag>
        </td>
        <td class="data-detail">{{ formatDataSummary(item) }}</td>
      </tr>
      </tbody>
    </table>
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

.status-tag.left {
  color: #63b3ed;
  border: 1px solid #2c5282;
}

.status-tag.right {
  color: #f6ad55;
  border: 1px solid #9c4221;
}

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

.side-cell span.left {
  color: #63b3ed;
}

.side-cell span.right {
  color: #f6ad55;
}

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