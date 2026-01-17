<script setup lang="ts">
import {ref, watch, onMounted} from 'vue'
import RobotChart from '@/components/RobotChart.vue'
import {sendChatMsg, checkModelConnect, getModels} from '@/composable/api/Chat2LLM'
import {
  ChatLineRound,
  TrendCharts,
  Cpu,
  CircleCheck,
  CircleClose,
  Loading
} from "@element-plus/icons-vue"

// --- 1. 模型数据与状态 ---
const selectedModel = ref('')
const modelOptions = ref<Array<{ label: string, value: string }>>([])
const isLoadingModels = ref(false)
const connStatus = ref('init')
const connMessage = ref('')

// --- 2. 核心逻辑：获取列表 ---
const initModels = async () => {
  isLoadingModels.value = true
  try {
    const res = await getModels()
    // 这里的 res 已经被 request.ts 拦截器解包了，直接取属性
    // @ts-ignore
    if (res.models && res.models.length > 0) {
      // @ts-ignore
      modelOptions.value = res.models
      // [修复关键点] 确保赋值的是 .value (字符串)，而不是整个对象 item
      // @ts-ignore
      selectedModel.value = res.models[0].value
    } else {
      modelOptions.value = [{label: '默认模型 (GPT-3.5)', value: 'gpt-3.5-turbo'}]
      selectedModel.value = 'gpt-3.5-turbo'
    }
  } catch (error) {
    console.error("无法获取模型列表:", error)
    modelOptions.value = [{label: '连接失败 (默认)', value: 'gpt-3.5-turbo'}]
    selectedModel.value = 'gpt-3.5-turbo'
  } finally {
    isLoadingModels.value = false
  }
}

// --- 3. 逻辑：模型连接检测 (已修复对象类型问题) ---
const handleModelCheck = async (modelVal: any) => {
  if (!modelVal) return

  // [修复核心] 防御性编程：如果传进来的是对象，自动提取 value
  const realModelName = (typeof modelVal === 'object' && modelVal !== null)
      ? modelVal.value
      : modelVal

  console.log("Checking model:", realModelName) // 调试日志

  connStatus.value = 'checking'
  connMessage.value = '连接检测中...'

  try {
    const res = await checkModelConnect(realModelName)
    // @ts-ignore
    if (res.success) {
      connStatus.value = 'success'
      connMessage.value = '模型连接正常'
    } else {
      connStatus.value = 'fail'
      // @ts-ignore
      connMessage.value = res.message || '连接失败'
    }
  } catch (e) {
    connStatus.value = 'fail'
    connMessage.value = '网络错误/后端不可达'
  }
}

watch(selectedModel, (newVal) => {
  handleModelCheck(newVal)
})

onMounted(() => {
  initModels()
})

// --- 4. 聊天逻辑 ---
const inputCommand = ref('')
const isSending = ref(false)

interface ChatMsg {
  role: 'system' | 'user'
  content: string
  model?: string
}

const chatHistory = ref<ChatMsg[]>([
  {role: 'system', content: 'LDISS 系统已就绪。请输入指令...'}
])

const sendCommand = async () => {
  if (!inputCommand.value) return

  // 同样要做处理，防止 selectedModel 是对象
  const modelToUse = (typeof selectedModel.value === 'object' && selectedModel.value !== null)
      ? (selectedModel.value as any).value
      : selectedModel.value

  if (connStatus.value === 'fail') {
    chatHistory.value.push({
      role: 'system',
      content: `❌ 发送失败：模型 [${modelToUse}] 未连接成功。`
    })
    return
  }

  chatHistory.value.push({role: 'user', content: inputCommand.value})
  const textToSend = inputCommand.value

  inputCommand.value = ''
  isSending.value = true

  try {
    const res = await sendChatMsg(textToSend, modelToUse)

    chatHistory.value.push({
      role: 'system',
      // @ts-ignore
      content: res.reply,
      // @ts-ignore
      model: res.model_name
    })

    // @ts-ignore
    if (res.action_code) {
      chatHistory.value.push({
        role: 'system',
        // @ts-ignore
        content: `[执行层] ${res.action_code}`,
        model: 'Robot Core'
      })
    }
  } catch (error) {
    chatHistory.value.push({role: 'system', content: '❌ 错误：请求超时或服务异常'})
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
                <div class="value" style="color: #67C23A">正常</div>
              </div>
            </el-card>
          </el-col>
        </el-row>
        <el-card class="box-card">
          <template #header>
            <div class="card-header"><span><el-icon><TrendCharts/></el-icon> 传感器数据监控</span></div>
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