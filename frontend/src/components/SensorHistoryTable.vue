<script setup lang="ts">
import { ref, onMounted } from 'vue'
import { useDB } from '@/composable/hooks/useDB'
import { ElMessage } from 'element-plus'

const { isLoadingSensor, fetchSensorHistory } = useDB()

// 存储表格数据
const tableData = ref<any[]>([])
// 查询的设备目标：'hand' 或 'arm'
const selectedTarget = ref<'hand' | 'arm'>('hand')
const queryMinutes = ref(1)

async function fetchData() {
  try {
    // 根据新接口，传入 target 和 minutes
    const res = await fetchSensorHistory(
        selectedTarget.value,
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

// 格式化展示的字典数据
function formatDataSummary(item: any) {
  if (!item || !item.data) return '无数据'

  // 遍历 data 里的键值对，保留 2 位小数进行展示
  return Object.entries(item.data)
      .map(([k, v]) => `${k}: ${Number(v).toFixed(2)}`)
      .join(' | ')
}

function downloadCSV() {
  if (tableData.value.length === 0) return

  const headers = ['Time', 'Target', 'Data Details']
  const rows = tableData.value.map(row => {
    // 时间戳转换 (后端返回的是秒级时间戳，需要乘1000)
    const timeStr = new Date(row.timestamp * 1000).toLocaleString()
    // 提取格式化后的数据
    const dataStr = formatDataSummary(row)

    return [
      timeStr,
      row.target,
      `"${dataStr}"` // 用引号包裹，防止内部逗号或符号破坏 CSV 格式
    ]
  })

  const csvContent = [
    headers.join(','),
    ...rows.map(r => r.join(','))
  ].join('\n')

  const blob = new Blob(['\uFEFF' + csvContent], { type: 'text/csv;charset=utf-8;' })
  const url = URL.createObjectURL(blob)
  const link = document.createElement('a')
  link.href = url
  link.setAttribute('download', `history_${selectedTarget.value}_${queryMinutes.value}min.csv`)
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
  <div class="header-controls">
    <div class="controls">
      <div class="select-group">
        <label>设备目标 (Target):</label>
        <select v-model="selectedTarget" @change="fetchData" :disabled="isLoadingSensor">
          <option value="hand">灵巧手 (Hand)</option>
          <option value="arm">机械臂 (Arm)</option>
        </select>
      </div>

      <div class="select-group">
        <label>时间范围:</label>
        <select v-model="queryMinutes" @change="fetchData" :disabled="isLoadingSensor">
          <option :value="1">最近 1 分钟</option>
          <option :value="5">最近 5 分钟</option>
          <option :value="10">最近 10 分钟</option>
        </select>
      </div>
    </div>

    <button @click="downloadCSV" class="download-btn" :disabled="isLoadingSensor || tableData.length === 0">
      导出 CSV
    </button>
  </div>

  <div class="table-container">
    <table>
      <thead>
      <tr>
        <th>Time</th>
        <th>Target</th>
        <th>Data Details</th>
      </tr>
      </thead>
      <tbody>
      <tr v-for="(item, index) in tableData" :key="index">
        <td class="time">{{ new Date(item.timestamp * 1000).toLocaleTimeString() }}</td>
        <td>
          <el-tag size="small" :type="item.target === 'arm' ? 'success' : 'primary'">
            {{ item.target.toUpperCase() }}
          </el-tag>
        </td>
        <td class="data-detail">{{ formatDataSummary(item) }}</td>
      </tr>
      <tr v-if="tableData.length === 0">
        <td colspan="3" class="no-data">暂无数据记录</td>
      </tr>
      </tbody>
    </table>
  </div>
</template>

<style scoped>
.header-controls {
  display: flex;
  justify-content: space-between;
  align-items: center;
  margin-bottom: 1.2rem;
}

.controls {
  display: flex;
  gap: 20px;
  align-items: center;
}

.select-group {
  display: flex;
  align-items: center;
  gap: 8px;
  font-size: 0.85rem;
  color: #e0e0e0;
}

select {
  padding: 6px 10px;
  background: #2d2d2d;
  border: 1px solid #444;
  color: white;
  border-radius: 6px;
  cursor: pointer;
  transition: all 0.2s;
}

select:hover {
  border-color: #666;
  background: #3d3d3d;
}

select:disabled {
  opacity: 0.5;
  cursor: not-allowed;
}

.download-btn {
  background: #2c5282;
  border: 1px solid #2b6cb0;
  color: white;
  padding: 6px 16px;
  border-radius: 6px;
  cursor: pointer;
  font-size: 0.85rem;
}

.download-btn:disabled {
  background: #444;
  border-color: #444;
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
  color: #e0e0e0;
}

.time {
  color: #888;
  font-family: 'Courier New', Courier, monospace;
}

.data-detail {
  font-family: 'Courier New', Courier, monospace;
  font-size: 0.8rem;
  line-height: 1.4;
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