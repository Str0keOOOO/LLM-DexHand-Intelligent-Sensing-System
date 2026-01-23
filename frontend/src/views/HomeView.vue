<script setup lang="ts">
import RobotChart from '@/components/RobotChart.vue'
import {ChatLineRound, TrendCharts, Cpu, CircleCheck, CircleClose, Loading, Delete, UserFilled, Service, Timer, Odometer, Connection} from "@element-plus/icons-vue"
import {useChat} from '@/composable/hooks/useChat.ts'
import {useRobotHealth} from '@/composable/hooks/useRobot.ts';

const {modelOptions, selectedModel, isLoadingModels, connStatus, connMessage, chatHistory, inputCommand, isSending, sendCommand, clearChatHistory} = useChat()

const {connStatusText, connStatusColor} = useRobotHealth({autoStart: true, interval: 5000})

</script>

<template>
  <div class="dashboard-container">
    <el-row :gutter="24" class="main-row">
      <el-col :span="10" class="left-col">
        <el-card class="chat-card" :body-style="{ padding: '0', display: 'flex', flexDirection: 'column', height: '100%' }">
          <template #header>
            <div class="card-header">
              <div class="header-left">
                <div class="icon-box">
                  <el-icon :size="18"><ChatLineRound/></el-icon>
                </div>
                <span class="title">智能指令交互</span>
              </div>

              <div class="header-right">
                <el-select
                    v-model="selectedModel"
                    placeholder="选择模型"
                    size="small"
                    class="model-select"
                    :loading="isLoadingModels"
                    :disabled="isSending || isLoadingModels"
                >
                  <template #prefix>
                    <el-icon><Cpu/></el-icon>
                  </template>
                  <el-option
                      v-for="item in modelOptions"
                      :key="item.value"
                      :label="item.label"
                      :value="item.value"
                  />
                </el-select>

                <el-tooltip content="清空对话历史" placement="top">
                  <el-button
                      class="action-btn"
                      size="small"
                      circle
                      :disabled="isSending"
                      @click="clearChatHistory(true)"
                  >
                    <el-icon><Delete /></el-icon>
                  </el-button>
                </el-tooltip>

                <el-tooltip :content="connMessage" placement="top">
                  <div class="conn-badge" :class="connStatus">
                    <el-icon v-if="connStatus === 'checking'" class="is-loading"><Loading/></el-icon>
                    <el-icon v-else-if="connStatus === 'success'"><CircleCheck/></el-icon>
                    <el-icon v-else-if="connStatus === 'fail'"><CircleClose/></el-icon>
                  </div>
                </el-tooltip>
              </div>
            </div>
          </template>

          <div class="chat-window" id="chat-window">
            <div v-if="chatHistory.length === 0" class="empty-state">
              <el-icon :size="40" color="#E4E7ED"><Service /></el-icon>
              <p>暂无对话，请输入指令开始控制</p>
            </div>

            <div v-for="(msg, index) in chatHistory" :key="index" :class="['message-row', msg.role]">
              <el-avatar v-if="msg.role === 'system'" :size="36" class="avatar system-avatar" :icon="Service" />

              <div class="bubble-container">
                <div class="message-bubble">
                  {{ msg.content }}
                </div>
                <div v-if="msg.role === 'system' && msg.model" class="model-tag">
                  Powered by {{ msg.model }}
                </div>
              </div>

              <el-avatar v-if="msg.role === 'user'" :size="36" class="avatar user-avatar" :icon="UserFilled" />
            </div>
          </div>

          <div class="input-area">
            <el-input
                v-model="inputCommand"
                placeholder="请输入控制指令 (Enter发送)..."
                @keyup.enter="sendCommand"
                :disabled="isSending || connStatus === 'checking'"
                class="custom-input"
            >
              <template #suffix>
                <el-button
                    type="primary"
                    round
                    size="small"
                    @click="sendCommand"
                    :loading="isSending"
                    :disabled="connStatus === 'fail'"
                >
                  发送
                </el-button>
              </template>
            </el-input>
          </div>
        </el-card>
      </el-col>

      <el-col :span="14">
        <el-row :gutter="20" style="margin-bottom: 24px;">
          <el-col :span="8">
            <div class="stat-card">
              <div class="stat-icon blue-bg">
                <el-icon><Odometer /></el-icon>
              </div>
              <div class="stat-info">
                <div class="label">当前模式</div>
                <div class="value">自主采集</div>
              </div>
            </div>
          </el-col>
          <el-col :span="8">
            <div class="stat-card">
              <div class="stat-icon purple-bg">
                <el-icon><Timer /></el-icon>
              </div>
              <div class="stat-info">
                <div class="label">运行时间</div>
                <div class="value font-mono">02:15:30</div>
              </div>
            </div>
          </el-col>
          <el-col :span="8">
            <div class="stat-card">
              <div class="stat-icon green-bg" :style="{ backgroundColor: connStatus === 'fail' ? '#fee' : '' }">
                <el-icon :style="{ color: connStatusColor }"><Connection /></el-icon>
              </div>
              <div class="stat-info">
                <div class="label">通信状态</div>
                <div class="value" :style="{ color: connStatusColor }">
                  {{ connStatusText }}
                </div>
              </div>
            </div>
          </el-col>
        </el-row>

        <el-card class="chart-card" shadow="never">
          <template #header>
            <div class="card-header">
              <div class="header-left">
                <div class="icon-box warning">
                  <el-icon><TrendCharts/></el-icon>
                </div>
                <span class="title">传感器数据监控</span>
              </div>
              <el-tag size="small" effect="plain" type="info">实时</el-tag>
            </div>
          </template>
          <div class="chart-container">
            <RobotChart/>
          </div>
        </el-card>
      </el-col>
    </el-row>
  </div>
</template>

<style scoped lang="scss">
.dashboard-container {
  padding: 20px;
  background-color: #f5f7fa;
  min-height: calc(100vh - 84px);
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
      background: linear-gradient(135deg, #e0f2fe 0%, #bae6fd 100%);
      color: #0284c7;
      display: flex;
      justify-content: center;
      align-items: center;

      &.warning {
        background: linear-gradient(135deg, #fef3c7 0%, #fde68a 100%);
        color: #d97706;
      }
    }

    .title {
      font-weight: 600;
      font-size: 16px;
      color: #1f2937;
    }
  }

  .header-right {
    display: flex;
    align-items: center;
    gap: 12px;
  }
}

.chat-card {
  height: calc(100vh - 120px);
  border: none;
  box-shadow: 0 4px 6px -1px rgba(0, 0, 0, 0.05), 0 2px 4px -1px rgba(0, 0, 0, 0.03);
  border-radius: 16px;

  :deep(.el-card__header) {
    padding: 0 20px;
    border-bottom: 1px solid #f0f2f5;
  }
}

.model-select {
  width: 130px;
}

.conn-badge {
  display: flex;
  align-items: center;
  font-size: 20px;
  cursor: help;
  transition: transform 0.2s;

  &.checking { color: #E6A23C; }
  &.success { color: #67C23A; }
  &.fail { color: #F56C6C; }

  &:hover { transform: scale(1.1); }
}

.chat-window {
  flex: 1;
  background-color: #f8fafc; /* 极浅的蓝灰色 */
  padding: 20px;
  overflow-y: auto;
  display: flex;
  flex-direction: column;
  gap: 20px;

  &::-webkit-scrollbar { width: 6px; }
  &::-webkit-scrollbar-thumb { background: #cbd5e1; border-radius: 3px; }
  &::-webkit-scrollbar-track { background: transparent; }
}

.empty-state {
  height: 100%;
  display: flex;
  flex-direction: column;
  justify-content: center;
  align-items: center;
  color: #909399;
  gap: 10px;
  font-size: 14px;
}

.message-row {
  display: flex;
  align-items: flex-start;
  gap: 12px;
  max-width: 90%;

  &.user {
    flex-direction: row-reverse; // 用户消息反向排列
    align-self: flex-end; // 靠右对齐
  }

  &.system {
    align-self: flex-start; // 靠左对齐
  }
}

.avatar {
  background-color: #fff;
  border: 1px solid #e2e8f0;
  color: #64748b;
  flex-shrink: 0;

  &.user-avatar {
    background-color: #eff6ff;
    color: #3b82f6;
    border-color: #dbeafe;
  }
}

.bubble-container {
  display: flex;
  flex-direction: column;
}

.message-bubble {
  padding: 10px 16px;
  border-radius: 12px;
  font-size: 14px;
  line-height: 1.5;
  position: relative;
  box-shadow: 0 1px 2px rgba(0,0,0,0.05);
  word-break: break-word;
}

.system .message-bubble {
  background-color: #ffffff;
  color: #334155;
  border-top-left-radius: 2px; /* 小尖角效果 */
  border: 1px solid #f1f5f9;
}

.user .message-bubble {
  background: linear-gradient(135deg, #3b82f6 0%, #2563eb 100%);
  color: white;
  border-top-right-radius: 2px;
}

.model-tag {
  font-size: 10px;
  color: #94a3b8;
  margin-top: 4px;
  margin-left: 4px;
}

.input-area {
  padding: 15px 20px;
  background-color: #fff;
  border-top: 1px solid #f0f2f5;

  :deep(.el-input__wrapper) {
    box-shadow: none;
    background-color: #f1f5f9;
    border-radius: 20px;
    padding-left: 15px;

    &.is-focus {
      box-shadow: 0 0 0 1px #409eff inset;
      background-color: #fff;
    }
  }
}

.stat-card {
  background: white;
  padding: 20px;
  border-radius: 16px;
  display: flex;
  align-items: center;
  gap: 16px;
  box-shadow: 0 4px 6px -1px rgba(0, 0, 0, 0.05);
  transition: transform 0.2s, box-shadow 0.2s;
  height: 100%;
  border: 1px solid rgba(255,255,255,0); /* 预留边框位置 */

  &:hover {
    transform: translateY(-2px);
    box-shadow: 0 10px 15px -3px rgba(0, 0, 0, 0.1);
  }

  .stat-icon {
    width: 48px;
    height: 48px;
    border-radius: 12px;
    display: flex;
    justify-content: center;
    align-items: center;
    font-size: 24px;

    &.blue-bg { background-color: #eff6ff; color: #3b82f6; }
    &.purple-bg { background-color: #f3e8ff; color: #a855f7; }
    &.green-bg { background-color: #f0fdf4; color: #22c55e; }
  }

  .stat-info {
    display: flex;
    flex-direction: column;

    .label {
      font-size: 13px;
      color: #64748b;
      margin-bottom: 4px;
    }

    .value {
      font-size: 18px;
      font-weight: 700;
      color: #0f172a;

      &.font-mono {
        font-family: 'Monaco', 'Menlo', monospace; /* 时间用等宽字体更好看 */
      }
    }
  }
}

.chart-card {
  border: none;
  border-radius: 16px;
  box-shadow: 0 4px 6px -1px rgba(0, 0, 0, 0.05);

  :deep(.el-card__header) {
    padding: 15px 20px;
    border-bottom: 1px solid #f0f2f5;
  }
}

.chart-container {
  min-height: 350px;
  padding: 10px 0;
}
</style>