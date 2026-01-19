<script setup lang="ts">
import {ref, watch, onMounted, computed} from 'vue'
import RobotChart from '@/components/RobotChart.vue'
import type {ChatMsg} from "@/composable/hooks/useSendCommand";
import {
  ChatLineRound,
  TrendCharts,
  Cpu,
  CircleCheck,
  CircleClose,
  Loading
} from "@element-plus/icons-vue"
import {useModels} from '@/composable/hooks/useModels'
import {useModelCheck} from '@/composable/hooks/useModelCheck'
import {useSendCommand} from '@/composable/hooks/useSendCommand'

const inputCommand = ref('')
const isSending = ref(false)
const chatHistory = ref<ChatMsg[]>([
  {role: 'system', content: 'LDISS 系统已就绪。请输入指令...'}
])

const {modelOptions, selectedModel, isLoadingModels, initModels} = useModels()
const {connStatus, connMessage, handleModelCheck} = useModelCheck()
const {sendCommand} = useSendCommand({
  chatHistory,
  inputCommand,
  selectedModel,
  connStatus,
  isSending
})


// TODO 思考要不要封装到useModelCheck里面
const connStatusText = computed(() => {
  return connStatus.value === 'checking'
      ? '检测中'
      : connStatus.value === 'success'
          ? '正常'
          : connStatus.value === 'fail'
              ? '异常'
              : '未知'
})

const connStatusColor = computed(() => {
  return connStatus.value === 'checking'
      ? '#E6A23C'
      : connStatus.value === 'success'
          ? '#67C23A'
          : connStatus.value === 'fail'
              ? '#F56C6C'
              : '#909399'
})

onMounted(() => {
  initModels()
})

watch(selectedModel, (newVal) => {
  handleModelCheck(newVal)
})

</script>

<template>
  <div class="dashboard">
    <el-row :gutter="20">
      <el-col :span="10">
        <el-card class="box-card chat-card">
          <template #header>
            <div class="card-header">
              <div class="header-left">
                <el-icon>
                  <ChatLineRound/>
                </el-icon>
                <span style="margin-left: 8px;">指令交互</span>
              </div>
              <div class="header-right">
                <el-select
                    v-model="selectedModel"
                    placeholder="加载中..."
                    size="small"
                    style="width: 140px"
                    :loading="isLoadingModels"
                    :disabled="isSending || isLoadingModels"
                >
                  <template #prefix>
                    <el-icon>
                      <Cpu/>
                    </el-icon>
                  </template>
                  <el-option
                      v-for="item in modelOptions"
                      :key="item.value"
                      :label="item.label"
                      :value="item.value"
                  />
                </el-select>

                <el-tooltip :content="connMessage" placement="top">
                  <div class="conn-status" :class="connStatus">
                    <el-icon v-if="connStatus === 'checking'" class="is-loading">
                      <Loading/>
                    </el-icon>
                    <el-icon v-else-if="connStatus === 'success'">
                      <CircleCheck/>
                    </el-icon>
                    <el-icon v-else-if="connStatus === 'fail'">
                      <CircleClose/>
                    </el-icon>
                  </div>
                </el-tooltip>
              </div>
            </div>
          </template>

          <div class="chat-window">
            <div v-for="(msg, index) in chatHistory" :key="index" :class="['message', msg.role]">
              <div class="message-content">
                {{ msg.content }}
                <div v-if="msg.role === 'system' && msg.model" class="model-tag">
                  Powered by: {{ msg.model }}
                </div>
              </div>
            </div>
          </div>

          <div class="input-area">
            <el-input v-model="inputCommand" placeholder="请输入控制指令..." @keyup.enter="sendCommand"
                      :disabled="isSending || connStatus === 'checking'">
              <template #append>
                <el-button @click="sendCommand" :loading="isSending" :disabled="connStatus === 'fail'">发送</el-button>
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
                <div class="value" :style="{ color: connStatusColor }">
                  {{ connStatusText }}
                </div>
              </div>
            </el-card>
          </el-col>
        </el-row>
        <el-card class="box-card">
          <template #header>
            <div class="card-header"><span><el-icon><TrendCharts/></el-icon>传感器数据监控</span></div>
          </template>
          <RobotChart/>
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
  display: flex;
  flex-direction: column;
}

.message.system {
  align-items: flex-start;
}

.message.user {
  align-items: flex-end;
  margin-left: auto;
}

.message-content {
  display: inline-block;
  padding: 8px 12px;
  border-radius: 8px;
  font-size: 14px;
  position: relative;
  min-width: 60px;
}

.system .message-content {
  background-color: #e4e7ed;
  color: #303133;
  border-bottom-left-radius: 2px;
}

.user .message-content {
  background-color: #409EFF;
  color: white;
  border-bottom-right-radius: 2px;
}

.model-tag {
  font-size: 10px;
  color: #909399;
  margin-top: 4px;
  text-align: right;
  border-top: 1px solid rgba(0, 0, 0, 0.05);
  padding-top: 2px;
  font-style: italic;
}

.user .model-tag {
  color: rgba(255, 255, 255, 0.7);
  border-top: 1px solid rgba(255, 255, 255, 0.2);
}

.card-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
}

.header-left {
  display: flex;
  align-items: center;
}

.header-right {
  display: flex;
  align-items: center;
  gap: 10px;
}

.conn-status {
  display: flex;
  align-items: center;
  font-size: 18px;
  cursor: help;
}

.conn-status.checking {
  color: #E6A23C;
}

.conn-status.success {
  color: #67C23A;
}

.conn-status.fail {
  color: #F56C6C;
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