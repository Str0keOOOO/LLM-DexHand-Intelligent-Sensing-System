<script setup lang="ts">
import { ref } from 'vue'
import RobotChart from '../components/RobotChart.vue'
import axios from 'axios'

// 聊天相关数据
const inputCommand = ref('')
const chatHistory = ref([
  { role: 'system', content: 'LDISS 系统已就绪。请输入自然语言指令，例如：“抓取前方的红色方块”' }
])
const isSending = ref(false)

// 发送指令（目前仅前端模拟）
const sendCommand = async () => {
  if (!inputCommand.value) return

  // 1. 界面立即显示用户输入
  chatHistory.value.push({ role: 'user', content: inputCommand.value })
  const payload = { message: inputCommand.value } // 准备发给后端的数据
  inputCommand.value = ''
  isSending.value = true

  try {
    // 2. 向后端发送请求 (注意端口是 8000)
    const res = await axios.post('http://localhost:8000/api/chat/send', payload)

    // 3. 接收后端 LLM 的回复
    chatHistory.value.push({
      role: 'system',
      content: res.data.reply
    })

    // 如果有动作指令，也可以显示出来
    if (res.data.action_code) {
      chatHistory.value.push({
        role: 'system',
        content: `[执行层] ${res.data.action_code}`
      })
    }
  } catch (error) {
    chatHistory.value.push({ role: 'system', content: '❌ 错误：无法连接到 LDISS 后端服务' })
    console.error(error)
  } finally {
    isSending.value = false
  }
}
</script>

<template>
  <div class="dashboard">
    <el-row :gutter="20">
      <el-col :span="10">
        <el-card class="box-card chat-card">
          <template #header>
            <div class="card-header">
              <span><el-icon><ChatLineRound /></el-icon> 自然语言指令交互</span>
              <el-tag size="small">LLM Agent</el-tag>
              <!--TODO 可以变成某个大模型的名字-->
            </div>
          </template>

          <div class="chat-window">
            <div v-for="(msg, index) in chatHistory" :key="index" :class="['message', msg.role]">
              <div class="message-content">{{ msg.content }}</div>
            </div>
          </div>

          <div class="input-area">
            <el-input
                v-model="inputCommand"
                placeholder="请输入控制指令..."
                @keyup.enter="sendCommand"
                :disabled="isSending"
            >
              <template #append>
                <el-button @click="sendCommand" :loading="isSending">发送</el-button>
              </template>
            </el-input>
          </div>
        </el-card>
      </el-col>

      <el-col :span="14">
        <el-row :gutter="20" style="margin-bottom: 20px;">
          <el-col :span="8">
            <el-card shadow="hover" class="status-card">
              <div class="statistic">
                <div class="label">当前模式</div>
                <div class="value">自主采集</div>
              </div>
            </el-card>
          </el-col>
          <el-col :span="8">
            <el-card shadow="hover" class="status-card">
              <div class="statistic">
                <div class="label">运行时间</div>
                <div class="value">02:15:30</div>
              </div>
            </el-card>
          </el-col>
          <el-col :span="8">
            <el-card shadow="hover" class="status-card">
              <div class="statistic">
                <div class="label">通信状态</div>
                <div class="value" style="color: #67C23A">正常</div>
              </div>
            </el-card>
          </el-col>
        </el-row>

        <el-card class="box-card">
          <template #header>
            <div class="card-header">
              <span><el-icon><TrendCharts /></el-icon> 传感器数据监控</span>
            </div>
          </template>
          <RobotChart />
        </el-card>
      </el-col>
    </el-row>
  </div>
</template>

<style scoped>
.chat-card {
  height: calc(100vh - 120px);
  display: flex;
  flex-direction: column;
}

/* 强制让 Element Card body 填满剩余空间 */
:deep(.el-card__body) {
  flex: 1;
  display: flex;
  flex-direction: column;
  padding: 15px;
  overflow: hidden;
}

.chat-window {
  flex: 1;
  overflow-y: auto;
  margin-bottom: 15px;
  background-color: #f9f9f9;
  border-radius: 8px;
  padding: 10px;
  border: 1px solid #eee;
}

.message {
  margin-bottom: 10px;
  max-width: 85%;
}

.message.system {
  align-self: flex-start;
}

.message.user {
  align-self: flex-end;
  margin-left: auto;
  text-align: right;
}

.message-content {
  display: inline-block;
  padding: 8px 12px;
  border-radius: 8px;
  font-size: 14px;
}

.system .message-content {
  background-color: #e4e7ed;
  color: #303133;
}

.user .message-content {
  background-color: #409EFF;
  color: white;
}

.card-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
}

.status-card .value {
  font-size: 20px;
  font-weight: bold;
  margin-top: 5px;
  color: #303133;
}
.status-card .label {
  font-size: 12px;
  color: #909399;
}
</style>