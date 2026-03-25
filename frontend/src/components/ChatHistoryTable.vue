<script setup lang="ts">
import {onMounted} from 'vue'
import {useDB} from '@/composable/hooks/useDB'
import type {ChatLogItem} from "@/composable/types/llm";

const {chatHistoryData, isLoadingChat, fetchChatHistory} = useDB()

async function fetchData() {
  await fetchChatHistory(100)
}

function downloadCSV() {
  const data = chatHistoryData.value?.data || []
  if (data.length === 0) return

  const headers = ['ID', 'Time', 'Role', 'Model', 'Content']
  const rows = data.map((row: ChatLogItem) => [
    row.id,
    row.created_at,
    row.role,
    row.model || 'N/A',
    `"${row.content.replace(/"/g, '""')}"` // 处理内容中的引号
  ])

  const csvContent = [
    headers.join(','),
    ...rows.map(r => r.join(','))
  ].join('\n')

  const blob = new Blob([csvContent], {type: 'text/csv;charset=utf-8;'})
  const url = URL.createObjectURL(blob)
  const link = document.createElement('a')
  link.href = url
  link.setAttribute('download', `chat_history_${new Date().toISOString().slice(0, 19)}.csv`)
  document.body.appendChild(link)
  link.click()
  document.body.removeChild(link)
}

onMounted(() => {
  fetchData()
})
</script>

<template>
  <div class="history-panel">
    <div class="header">
      <h3>💬 Chat History (MySQL)</h3>
      <div class="actions">
        <button @click="fetchData" :disabled="isLoadingChat">Refresh</button>
        <button @click="downloadCSV" class="download-btn" :disabled="!chatHistoryData?.data || chatHistoryData.data.length === 0">⬇ Download CSV</button>
      </div>
    </div>

    <div class="table-container">
      <table>
        <thead>
        <tr>
          <th>Time</th>
          <th>Role</th>
          <th>Model</th>
          <th>Content</th>
        </tr>
        </thead>
        <tbody>
        <tr v-for="item in chatHistoryData?.data || []" :key="item.id">
          <td class="time">{{ new Date(item.created_at).toLocaleString() }}</td>
          <td>
            <span :class="['tag', item.role]">{{ item.role }}</span>
          </td>
          <td>{{ item.model }}</td>
          <td class="content">{{ item.content }}</td>
        </tr>
        <tr v-if="(!chatHistoryData?.data || chatHistoryData.data.length === 0) && !isLoadingChat">
          <td colspan="4" style="text-align: center; color: #888;">No Data</td>
        </tr>
        </tbody>
      </table>
    </div>
  </div>
</template>

<style scoped>
.history-panel {
  background: #1e1e1e;
  border-radius: 8px;
  padding: 1rem;
  color: #fff;
  margin-bottom: 2rem;
  border: 1px solid #333;
}

.header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  margin-bottom: 1rem;
}

.actions button {
  margin-left: 10px;
  padding: 6px 12px;
  cursor: pointer;
  background: #333;
  border: 1px solid #555;
  color: white;
  border-radius: 4px;
}

.actions button:hover {
  background: #444;
}

.download-btn {
  background: #2c5282 !important;
  border-color: #2b6cb0 !important;
}

.table-container {
  max-height: 400px;
  overflow-y: auto;
  border: 1px solid #333;
}

table {
  width: 100%;
  border-collapse: collapse;
  font-size: 0.9rem;
}

th, td {
  padding: 8px;
  text-align: left;
  border-bottom: 1px solid #333;
}

th {
  background: #2d2d2d;
  position: sticky;
  top: 0;
}

.time {
  white-space: nowrap;
  color: #aaa;
  font-size: 0.8rem;
}

.content {
  max-width: 400px;
  overflow: hidden;
  text-overflow: ellipsis;
  white-space: nowrap;
}

.tag {
  padding: 2px 6px;
  border-radius: 4px;
  font-size: 0.8rem;
  text-transform: uppercase;
}

.tag.user {
  background: #2f855a;
}

.tag.assistant {
  background: #2b6cb0;
}
</style>