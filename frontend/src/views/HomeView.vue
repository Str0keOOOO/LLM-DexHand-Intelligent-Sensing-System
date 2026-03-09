<script setup lang="ts">
import {ref} from 'vue'
import ChatPanel from '@/components/ChatPanel.vue'

import {Timer, Odometer, Connection} from "@element-plus/icons-vue"

import {useChat} from '@/composable/hooks/useChat'
import {useRobot} from '@/composable/hooks/useRobot'

const {
  modelOptions, selectedModel, isLoadingModels,
  connStatus, connMessage, chatHistory,
  inputCommand, isSending, isRecording,
  sendCommand, clearChatHistory, toggleRecording
} = useChat()
const {isConnected, connStatusText, connStatusColor, formattedTime,} = useRobot()

const controlDialogVisible = ref(false)
</script>

<template>
  <div class="dashboard-container">
    <el-row :gutter="24" class="main-row">
      <el-col :span="10" class="left-col">
        <ChatPanel
            v-model:selectedModel="selectedModel"
            v-model:inputCommand="inputCommand"
            :model-options="modelOptions"
            :is-loading-models="isLoadingModels"
            :conn-status="connStatus"
            :conn-message="connMessage"
            :chat-history="chatHistory"
            :is-sending="isSending"
            :is-recording="isRecording"
            @send="sendCommand"
            @clear="clearChatHistory"
            @open-manual="controlDialogVisible = true"
            @toggle-recording="toggleRecording"
        />
      </el-col>

      <el-col :span="14">
        <el-row :gutter="20" style="margin-bottom: 24px;">
          <el-col :span="8">
            <div class="stat-card">
              <div class="stat-icon blue-bg">
                <el-icon>
                  <Odometer/>
                </el-icon>
              </div>
              <div class="stat-info">
                <div class="label">当前模式</div>
                <div class="value">虚拟/遥操作</div>
              </div>
            </div>
          </el-col>
          <el-col :span="8">
            <div class="stat-card">
              <div class="stat-icon purple-bg">
                <el-icon>
                  <Timer/>
                </el-icon>
              </div>
              <div class="stat-info">
                <div class="label">更新时间</div>
                <div class="value font-mono">{{ formattedTime }}</div>
              </div>
            </div>
          </el-col>
          <el-col :span="8">
            <div class="stat-card">
              <div class="stat-icon green-bg" :style="{ backgroundColor: isConnected ? '#f0fdf4' : '#fef2f2' }">
                <el-icon :style="{ color: connStatusColor }">
                  <Connection/>
                </el-icon>
              </div>
              <div class="stat-info">
                <div class="label">通信状态</div>
                <div class="value" :style="{ color: connStatusColor }">{{ connStatusText }}</div>
              </div>
            </div>
          </el-col>
        </el-row>
      </el-col>
    </el-row>
  </div>
</template>

<style scoped lang="scss">
/* --- 样式部分 --- */

.dashboard-container {
  padding: 20px;
  background-color: #f5f7fa;
  min-height: calc(100vh - 84px);
}

.main-row {
  margin-bottom: 20px;
}

.section-divider {
  margin: 30px 0 20px 0;

  .divider-text {
    display: flex;
    align-items: center;
    gap: 8px;
    font-size: 16px;
    font-weight: 600;
    color: #4b5563;
  }
}

.history-row {
  margin-bottom: 40px;
}

.stat-card {
  background: white;
  padding: 20px;
  border-radius: 16px;
  display: flex;
  align-items: center;
  gap: 16px;
  box-shadow: 0 4px 6px -1px rgba(0, 0, 0, 0.05);
  transition: transform 0.2s;
  height: 100%;
  border: 1px solid rgba(255, 255, 255, 0);

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

    &.blue-bg {
      background-color: #eff6ff;
      color: #3b82f6;
    }

    &.purple-bg {
      background-color: #f3e8ff;
      color: #a855f7;
    }

    &.green-bg {
      background-color: #f0fdf4;
      color: #22c55e;
    }
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
    }
  }
}

.left-col {
  /* 可以在这里给 ChatPanel 加上高度限制或者其他布局调整 */
}
</style>